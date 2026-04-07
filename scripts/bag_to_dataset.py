#!/usr/bin/env python3
"""
Convert rosbag episodes to xVLA-compatible dataset format.

Reads rosbag episodes from a source directory, extracts robot state,
actions, F/T, and camera images, and writes HDF5 + JSON + MP4 files
matching the sample dataset format.

Usage:
    from scripts.bag_to_dataset import convert_episode, create_dataset_meta
"""

import json
import os
import time
from bisect import bisect_left
from concurrent.futures import ThreadPoolExecutor, as_completed

import cv2
import h5py
import numpy as np

# Attempt ROS imports (only needed when actually converting)
try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import JointState, Image
    from geometry_msgs.msg import WrenchStamped
    from alicia_duo_leader_driver.msg import ArmJointState
except ImportError:
    pass

from ur15_dashboard.kinematics import forward_kinematics, fk_quaternion

JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]

# Message type string -> Python class
MSG_TYPES = {
    "sensor_msgs/msg/JointState": "sensor_msgs.msg.JointState",
    "geometry_msgs/msg/WrenchStamped": "geometry_msgs.msg.WrenchStamped",
    "alicia_duo_leader_driver/msg/ArmJointState": "alicia_duo_leader_driver.msg.ArmJointState",
    "sensor_msgs/msg/Image": "sensor_msgs.msg.Image",
}


def _get_msg_class(type_str):
    """Get message class from type string."""
    parts = type_str.split("/")
    if len(parts) == 3:
        mod = __import__(f"{parts[0]}.{parts[1]}", fromlist=[parts[2]])
        return getattr(mod, parts[2])
    raise ValueError(f"Unknown message type: {type_str}")


def _nearest_idx(timestamps, target):
    """Find index of nearest timestamp using binary search. O(log n)."""
    pos = bisect_left(timestamps, target)
    if pos == 0:
        return 0
    if pos >= len(timestamps):
        return len(timestamps) - 1
    if abs(timestamps[pos] - target) < abs(timestamps[pos - 1] - target):
        return pos
    return pos - 1


def _open_bag_reader(bag_path):
    """Open a rosbag SequentialReader."""
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader.open(storage_options, converter_options)
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    return reader, topic_types


def convert_episode(bag_path, dataset_dir, episode_idx, task_name="teleop"):
    """
    Convert a single rosbag episode to dataset format (memory-efficient).

    Uses a two-pass approach:
      Pass 1: Read lightweight topics (joints, FT, arm) + camera timestamps only.
      Pass 2: Stream camera images directly to VideoWriter without accumulation.

    Args:
        bag_path: Path to rosbag directory
        dataset_dir: Output dataset directory
        episode_idx: Episode index number
        task_name: Task description string

    Returns:
        dict: Episode metadata
    """
    cam_topic = "/ur15_camera/image_raw"
    joint_topic = "/joint_states"
    ft_topic = "/force_torque_sensor_broadcaster/wrench"
    arm_topic = "/arm_joint_state"

    # ── Pass 1: read lightweight data + camera timestamps ──
    print(f"  [{episode_idx}] Pass 1 (metadata): {bag_path}")
    reader, topic_types = _open_bag_reader(bag_path)

    # Lightweight storage: timestamps + extracted scalars only
    cam_timestamps = []          # just int64 timestamps
    joint_ts = []
    joint_data = []              # list of (q6, v6, e6) tuples
    ft_ts = []
    ft_data = []                 # list of 6-element tuples
    arm_ts = []
    arm_data = []                # list of (j1..j6, gripper) tuples

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic == cam_topic:
            cam_timestamps.append(timestamp)
        elif topic == joint_topic:
            msg = deserialize_message(data, _get_msg_class(topic_types[topic]))
            q = [0.0] * 6; v = [0.0] * 6; e = [0.0] * 6
            for i, name in enumerate(msg.name):
                if name in JOINT_NAMES:
                    idx = JOINT_NAMES.index(name)
                    q[idx] = msg.position[i] if i < len(msg.position) else 0.0
                    v[idx] = msg.velocity[i] if i < len(msg.velocity) else 0.0
                    e[idx] = msg.effort[i] if i < len(msg.effort) else 0.0
            joint_ts.append(timestamp)
            joint_data.append((q, v, e))
        elif topic == ft_topic:
            msg = deserialize_message(data, _get_msg_class(topic_types[topic]))
            ft_ts.append(timestamp)
            ft_data.append((
                msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
                msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z,
            ))
        elif topic == arm_topic:
            msg = deserialize_message(data, _get_msg_class(topic_types[topic]))
            arm_ts.append(timestamp)
            arm_data.append((
                msg.joint1, msg.joint2, msg.joint3,
                msg.joint4, msg.joint5, msg.joint6,
                msg.gripper,
            ))
    del reader  # free reader resources

    if not cam_timestamps:
        print(f"  [{episode_idx}] WARNING: No camera messages on {cam_topic}")

    # Determine reference timestamps
    if cam_timestamps:
        ref_timestamps = cam_timestamps
    elif joint_ts:
        ref_timestamps = joint_ts[::20]  # downsample to ~25 Hz
    else:
        print(f"  [{episode_idx}] ERROR: No reference timestamps")
        return None

    n_steps = len(ref_timestamps)
    t0 = ref_timestamps[0]
    timestamps_sec = [(t - t0) * 1e-9 for t in ref_timestamps]
    print(f"  [{episode_idx}] Steps: {n_steps}, Duration: {timestamps_sec[-1]:.1f}s")

    # Build output arrays using binary search (O(n log n) total)
    current_joint_position = np.zeros((n_steps, 6), dtype=np.float64)
    current_joint_velocity = np.zeros((n_steps, 6), dtype=np.float64)
    current_joint_effort = np.zeros((n_steps, 6), dtype=np.float64)
    target_joint_position = np.zeros((n_steps, 6), dtype=np.float64)
    target_gripper_position = np.zeros((n_steps, 1), dtype=np.float64)
    tcp_wrench = np.zeros((n_steps, 6), dtype=np.float64)

    for step, ref_ts in enumerate(ref_timestamps):
        if joint_ts:
            ji = _nearest_idx(joint_ts, ref_ts)
            current_joint_position[step] = joint_data[ji][0]
            current_joint_velocity[step] = joint_data[ji][1]
            current_joint_effort[step] = joint_data[ji][2]
        if ft_ts:
            fi = _nearest_idx(ft_ts, ref_ts)
            tcp_wrench[step] = ft_data[fi]
        if arm_ts:
            ai = _nearest_idx(arm_ts, ref_ts)
            target_joint_position[step] = arm_data[ai][:6]
            target_gripper_position[step, 0] = arm_data[ai][6]

    # Free lightweight data now that arrays are built
    del joint_ts, joint_data, ft_ts, ft_data, arm_ts, arm_data

    # ── Write HDF5 ──
    safe_task = task_name.replace(" ", "_")
    ep_name = f"{episode_idx:08d}-ur15_teleop__{safe_task}__episode_{episode_idx}"
    os.makedirs(os.path.join(dataset_dir, "data"), exist_ok=True)
    os.makedirs(os.path.join(dataset_dir, "episode"), exist_ok=True)
    os.makedirs(os.path.join(dataset_dir, "video_agentview"), exist_ok=True)

    h5_path = os.path.join(dataset_dir, "data", f"{ep_name}.h5")
    with h5py.File(h5_path, "w") as f:
        f.create_dataset("timestamps", data=np.array(timestamps_sec))
        f.create_dataset("current_joint_position", data=current_joint_position)
        f.create_dataset("current_joint_velocity", data=current_joint_velocity)
        f.create_dataset("current_joint_effort", data=current_joint_effort)
        f.create_dataset("target_joint_position", data=target_joint_position)
        f.create_dataset("target_gripper_position", data=target_gripper_position)
        f.create_dataset("TCP_wrench", data=tcp_wrench)
    del current_joint_position, current_joint_velocity, current_joint_effort
    del target_joint_position, target_gripper_position, tcp_wrench

    print(f"  [{episode_idx}] Wrote HDF5: {h5_path}")

    # ── Pass 2: stream camera images directly to video writer ──
    vid_path = os.path.join(dataset_dir, "video_agentview", f"{ep_name}.mp4")
    if cam_timestamps:
        print(f"  [{episode_idx}] Pass 2 (video): streaming {len(cam_timestamps)} frames")
        fps = n_steps / max(timestamps_sec[-1], 0.1)
        writer = None

        reader2, topic_types2 = _open_bag_reader(bag_path)
        filter_ = rosbag2_py.StorageFilter(topics=[cam_topic])
        reader2.set_filter(filter_)

        frame_count = 0
        while reader2.has_next():
            topic, data, timestamp = reader2.read_next()
            msg = deserialize_message(data, _get_msg_class(topic_types2[topic]))
            h, w = msg.height, msg.width
            if msg.encoding in ("rgb8", "bgr8"):
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
                if msg.encoding == "rgb8":
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            else:
                img = np.zeros((h, w, 3), dtype=np.uint8)
            if writer is None:
                fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                writer = cv2.VideoWriter(vid_path, fourcc, fps, (w, h))
            writer.write(img)
            frame_count += 1

        if writer is not None:
            writer.release()
        del reader2
        print(f"  [{episode_idx}] Wrote video: {vid_path} ({frame_count} frames, {fps:.1f} fps)")

    # Episode metadata
    ep_meta = {
        "episode_id": f"{episode_idx:08d}-0000",
        "filename": ep_name,
        "start_frame": 0,
        "end_frame": n_steps - 1,
        "instruction": [task_name],
        "label": ["ur15_teleop"],
        "source_bag": os.path.basename(bag_path),
    }

    # Write per-episode JSON
    ep_json_path = os.path.join(dataset_dir, "episode", f"{ep_name}.json")
    with open(ep_json_path, "w") as f:
        json.dump({"episodes": [ep_meta]}, f, indent=2)

    return ep_meta


def create_dataset_meta(dataset_dir, episodes_meta):
    """Create/update meta.json and episodes_all.json."""
    meta = {
        "dataset_name": "UR15 Teleop",
        "year": int(time.strftime("%Y")),
        "preprocessing": "xvla",
        "environment": {"type": "Real"},
        "robot": {
            "eef_type": "Gripper",
            "arm_type": "Single",
            "view_num": 1,
            "embodiment_count": 1,
        },
        "tasks": {"task_type": "Manipulation"},
        "scale": {"episodes": len(episodes_meta)},
    }
    with open(os.path.join(dataset_dir, "meta.json"), "w") as f:
        json.dump(meta, f, indent=2)

    with open(os.path.join(dataset_dir, "episodes_all.json"), "w") as f:
        json.dump({"episodes": episodes_meta}, f, indent=2)


def convert_all(rosbag_dir, dataset_dir, task_name="teleop", num_threads=1,
                status_callback=None):
    """Convert all episodes in rosbag_dir to dataset format.

    Args:
        rosbag_dir: Source directory with episode_* bags
        dataset_dir: Output dataset directory
        task_name: Task description string
        num_threads: Number of parallel conversion threads (default 1)
        status_callback: Optional callable(bag_name, status, detail) where
            status is 'converting', 'converted', or 'error'

    Returns:
        dict with conversion results
    """
    os.makedirs(dataset_dir, exist_ok=True)

    # Find all episode bags
    bags = sorted([
        d for d in os.listdir(rosbag_dir)
        if d.startswith("episode_") and os.path.isdir(os.path.join(rosbag_dir, d))
    ])

    if not bags:
        return {"success": False, "error": "No episodes found in " + rosbag_dir}

    # Check which are already converted
    existing = set()
    ep_dir = os.path.join(dataset_dir, "episode")
    if os.path.isdir(ep_dir):
        for f in os.listdir(ep_dir):
            if f.endswith(".json"):
                try:
                    with open(os.path.join(ep_dir, f)) as jf:
                        d = json.load(jf)
                    for ep in d.get("episodes", []):
                        if "source_bag" in ep:
                            existing.add(ep["source_bag"])
                except Exception:
                    pass

    # Separate into to-convert and skipped
    to_convert = []
    skipped = 0
    for i, bag_name in enumerate(bags):
        if bag_name in existing:
            print(f"  Skipping (already converted): {bag_name}")
            if status_callback:
                status_callback(bag_name, "skipped", None)
            skipped += 1
        else:
            to_convert.append((i, bag_name))

    all_meta = []
    converted = 0
    errors = 0

    def _convert_one(args):
        idx, bag_name = args
        bag_path = os.path.join(rosbag_dir, bag_name)
        if status_callback:
            status_callback(bag_name, "converting", None)
        try:
            meta = convert_episode(bag_path, dataset_dir, idx, task_name)
            if status_callback:
                status_callback(bag_name, "converted", meta)
            return bag_name, meta, None
        except Exception as e:
            if status_callback:
                status_callback(bag_name, "error", str(e))
            return bag_name, None, str(e)

    effective_threads = max(1, min(num_threads, len(to_convert))) if to_convert else 1
    print(f"  Converting {len(to_convert)} episodes with {effective_threads} threads")

    if effective_threads == 1:
        # Single-threaded (original behavior)
        for args in to_convert:
            bag_name, meta, err = _convert_one(args)
            if err:
                print(f"  ERROR converting {bag_name}: {err}")
                errors += 1
            elif meta:
                all_meta.append(meta)
                converted += 1
    else:
        with ThreadPoolExecutor(max_workers=effective_threads) as executor:
            futures = {executor.submit(_convert_one, args): args for args in to_convert}
            for future in as_completed(futures):
                bag_name, meta, err = future.result()
                if err:
                    print(f"  ERROR converting {bag_name}: {err}")
                    errors += 1
                elif meta:
                    all_meta.append(meta)
                    converted += 1

    # Rebuild full episode index
    all_episodes = []
    if os.path.isdir(ep_dir):
        for f in sorted(os.listdir(ep_dir)):
            if f.endswith(".json"):
                try:
                    with open(os.path.join(ep_dir, f)) as jf:
                        d = json.load(jf)
                    all_episodes.extend(d.get("episodes", []))
                except Exception:
                    pass

    create_dataset_meta(dataset_dir, all_episodes)

    result = {
        "success": True,
        "total_bags": len(bags),
        "converted": converted,
        "skipped": skipped,
        "errors": errors,
        "total_episodes": len(all_episodes),
    }
    print(f"  Done: {converted} converted, {skipped} skipped, {errors} errors, {len(all_episodes)} total")
    return result
