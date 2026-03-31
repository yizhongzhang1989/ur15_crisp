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


def read_bag_topics(bag_path):
    """Read all messages from a rosbag, grouped by topic.
    
    Returns:
        dict: {topic_name: [(timestamp_ns, deserialized_msg), ...]}
    """
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader.open(storage_options, converter_options)

    # Get topic types
    topic_types = {}
    for t in reader.get_all_topics_and_types():
        topic_types[t.name] = t.type

    messages = {}
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic not in messages:
            messages[topic] = []
        msg_class = _get_msg_class(topic_types[topic])
        msg = deserialize_message(data, msg_class)
        messages[topic].append((timestamp, msg))

    return messages


def convert_episode(bag_path, dataset_dir, episode_idx, task_name="teleop"):
    """
    Convert a single rosbag episode to dataset format.

    Args:
        bag_path: Path to rosbag directory
        dataset_dir: Output dataset directory
        episode_idx: Episode index number
        task_name: Task description string

    Returns:
        dict: Episode metadata
    """
    print(f"  Reading bag: {bag_path}")
    messages = read_bag_topics(bag_path)

    # Extract camera images (resampling reference clock)
    cam_topic = "/ur15_camera/image_raw"
    cam_msgs = messages.get(cam_topic, [])
    if not cam_msgs:
        print(f"  WARNING: No camera messages found on {cam_topic}")

    joint_topic = "/joint_states"
    joint_msgs = messages.get(joint_topic, [])
    ft_topic = "/force_torque_sensor_broadcaster/wrench"
    ft_msgs = messages.get(ft_topic, [])
    arm_topic = "/arm_joint_state"
    arm_msgs = messages.get(arm_topic, [])

    # Use camera timestamps as reference (lowest rate, defines steps)
    if cam_msgs:
        ref_timestamps = [t for t, _ in cam_msgs]
    elif joint_msgs:
        # Downsample joint states to ~25 Hz
        ref_timestamps = [t for t, _ in joint_msgs[::20]]
    else:
        print("  ERROR: No reference timestamps")
        return None

    n_steps = len(ref_timestamps)
    t0 = ref_timestamps[0]
    timestamps_sec = [(t - t0) * 1e-9 for t in ref_timestamps]

    print(f"  Steps: {n_steps}, Duration: {timestamps_sec[-1]:.1f}s")

    # Helper: find nearest message to a timestamp
    def nearest_msg(msgs, target_ts):
        if not msgs:
            return None
        idx = min(range(len(msgs)), key=lambda i: abs(msgs[i][0] - target_ts))
        return msgs[idx][1]

    # Extract data at each camera frame
    joint_positions = np.zeros((n_steps, 6), dtype=np.float64)
    joint_velocities = np.zeros((n_steps, 6), dtype=np.float64)
    joint_efforts = np.zeros((n_steps, 6), dtype=np.float64)
    tcp_poses = np.zeros((n_steps, 1, 7), dtype=np.float64)  # [x,y,z,qx,qy,qz,qw]
    forces = np.zeros((n_steps, 3), dtype=np.float64)
    torques_ft = np.zeros((n_steps, 3), dtype=np.float64)
    gripper_actions = np.zeros((n_steps, 1), dtype=np.float64)
    gripper_states = np.zeros((n_steps, 1), dtype=np.float64)
    action_poses = np.zeros((n_steps, 1, 7), dtype=np.float64)
    images = []

    for step, ref_ts in enumerate(ref_timestamps):
        # Joint state
        js = nearest_msg(joint_msgs, ref_ts)
        if js:
            q = [0.0] * 6
            v = [0.0] * 6
            e = [0.0] * 6
            for i, name in enumerate(js.name):
                if name in JOINT_NAMES:
                    idx = JOINT_NAMES.index(name)
                    q[idx] = js.position[i] if i < len(js.position) else 0.0
                    v[idx] = js.velocity[i] if i < len(js.velocity) else 0.0
                    e[idx] = js.effort[i] if i < len(js.effort) else 0.0
            joint_positions[step] = q
            joint_velocities[step] = v
            joint_efforts[step] = e
            # FK for TCP pose
            pos, quat = fk_quaternion(q)
            tcp_poses[step, 0, :3] = pos
            tcp_poses[step, 0, 3:] = quat

        # F/T
        ft = nearest_msg(ft_msgs, ref_ts)
        if ft:
            forces[step] = [ft.wrench.force.x, ft.wrench.force.y, ft.wrench.force.z]
            torques_ft[step] = [ft.wrench.torque.x, ft.wrench.torque.y, ft.wrench.torque.z]

        # Leader arm (action)
        arm = nearest_msg(arm_msgs, ref_ts)
        if arm:
            # Use leader arm as action source
            arm_q = [arm.joint1, arm.joint2, arm.joint3, arm.joint4, arm.joint5, arm.joint6]
            gripper_actions[step, 0] = arm.gripper / 1000.0  # normalize to [0, 1]
            gripper_states[step, 0] = arm.gripper / 1000.0
            # Compute target TCP pose from leader joints (approximate — uses same FK)
            # In practice, the target comes from the teleop mapping
            action_poses[step, 0, :3] = tcp_poses[step, 0, :3]  # use current as placeholder
            action_poses[step, 0, 3:] = tcp_poses[step, 0, 3:]

        # Camera image
        if cam_msgs:
            img_msg = nearest_msg(cam_msgs, ref_ts)
            if img_msg:
                h, w = img_msg.height, img_msg.width
                if img_msg.encoding in ("rgb8", "bgr8"):
                    img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(h, w, 3)
                    if img_msg.encoding == "rgb8":
                        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                else:
                    img = np.zeros((h, w, 3), dtype=np.uint8)
                images.append(img)

    # Create output directories
    ep_name = f"{episode_idx:08d}-ur15_teleop__{task_name}__episode_{episode_idx}"
    os.makedirs(os.path.join(dataset_dir, "data"), exist_ok=True)
    os.makedirs(os.path.join(dataset_dir, "episode"), exist_ok=True)
    os.makedirs(os.path.join(dataset_dir, "video_agentview"), exist_ok=True)

    # Write HDF5
    h5_path = os.path.join(dataset_dir, "data", f"{ep_name}.h5")
    with h5py.File(h5_path, "w") as f:
        f.create_dataset("timestamps", data=np.array(timestamps_sec))
        f.create_dataset("end_space/state/pose", data=tcp_poses)
        f.create_dataset("end_space/action/pose", data=action_poses)
        f.create_dataset("actuator_space/action/value", data=gripper_actions)
        f.create_dataset("actuator_space/state/value", data=gripper_states)

        # Meta
        f.create_dataset("meta/end_space/names", data=[b"arm_end"])
        f.create_dataset("meta/end_space/state_fields", data=[b"pose"])
        f.create_dataset("meta/end_space/action_fields", data=[b"pose"])
        f["meta/end_space/state_pose_reference"] = "world"
        f["meta/end_space/action_pose_reference"] = "world"
        f["meta/end_space/action_pose_type"] = "absolute"
        f.create_dataset("meta/end_space/roles/end", data=[0])
        f.create_dataset("meta/actuator_space/names", data=[b"gripper"])
        f.create_dataset("meta/actuator_space/action_fields", data=[b"value"])
        f.create_dataset("meta/actuator_space/state_fields", data=[b"value"])
        f.create_dataset("meta/actuator_space/types", data=[b"position"])
        f.create_dataset("meta/camera_space/names", data=[b"agentview"])
        f.create_dataset("meta/camera_space/state_fields", data=[b"image"])
        f.create_dataset("meta/camera_space/static_extrinsic", data=[0])
        f.create_dataset("meta/camera_space/static_intrinsic", data=[1])

        # Extra data not in sample but useful
        f.create_dataset("extra/joint_positions", data=joint_positions)
        f.create_dataset("extra/joint_velocities", data=joint_velocities)
        f.create_dataset("extra/joint_efforts", data=joint_efforts)
        f.create_dataset("extra/forces", data=forces)
        f.create_dataset("extra/torques", data=torques_ft)

    print(f"  Wrote HDF5: {h5_path}")

    # Write video
    if images:
        vid_path = os.path.join(dataset_dir, "video_agentview", f"{ep_name}.mp4")
        h, w = images[0].shape[:2]
        fps = n_steps / max(timestamps_sec[-1], 0.1)
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(vid_path, fourcc, fps, (w, h))
        for img in images:
            writer.write(img)
        writer.release()
        print(f"  Wrote video: {vid_path} ({len(images)} frames, {fps:.1f} fps)")

    # Episode metadata
    ep_meta = {
        "episode_id": f"{episode_idx:08d}-0000",
        "filename": ep_name,
        "start_frame": 0,
        "end_frame": n_steps,
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


def convert_all(rosbag_dir, dataset_dir, task_name="teleop"):
    """Convert all episodes in rosbag_dir to dataset format.
    
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

    all_meta = []
    converted = 0
    skipped = 0

    for i, bag_name in enumerate(bags):
        bag_path = os.path.join(rosbag_dir, bag_name)
        if bag_name in existing:
            print(f"  Skipping (already converted): {bag_name}")
            skipped += 1
            continue
        try:
            meta = convert_episode(bag_path, dataset_dir, i, task_name)
            if meta:
                all_meta.append(meta)
                converted += 1
        except Exception as e:
            print(f"  ERROR converting {bag_name}: {e}")

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
        "total_episodes": len(all_episodes),
    }
    print(f"  Done: {converted} converted, {skipped} skipped, {len(all_episodes)} total")
    return result
