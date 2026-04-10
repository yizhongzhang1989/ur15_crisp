#!/usr/bin/env python3
"""
Data Collection Dashboard — Web-based dashboard for UR15 data collection.

Subscribes to /joint_states and /force_torque_sensor_broadcaster/wrench,
serves a web page with a 3D URDF viewer sub-window and data collection controls.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import WrenchStamped

import http.server
import json
import math
import mimetypes
import os
import signal
import subprocess
import threading
import time
import numpy as np
import cv2
from functools import partial

mimetypes.add_type("application/octet-stream", ".dae")
mimetypes.add_type("application/octet-stream", ".stl")
mimetypes.add_type("image/jpeg", ".jpg")
mimetypes.add_type("application/javascript", ".js")

RAD_TO_DEG = 180.0 / math.pi

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


class DashboardHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, dashboard_node=None, viewer_html=b"", ur15_static_dir="", **kwargs):
        self._dashboard = dashboard_node
        self._viewer_html = viewer_html
        self._ur15_static_dir = ur15_static_dir
        super().__init__(*args, **kwargs)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(self._viewer_html)))
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            self.wfile.write(self._viewer_html)
        elif self.path == "/api/events":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self._dashboard.register_sse_client(self.wfile)
            try:
                while True:
                    time.sleep(1)
            except Exception:
                pass
            finally:
                self._dashboard.unregister_sse_client(self.wfile)
        elif self.path == "/api/state":
            state = self._dashboard.get_latest_state()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(state or {}).encode())
        elif self.path == "/api/camera_stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            try:
                while True:
                    frame = self._dashboard.get_camera_jpeg()
                    if frame is not None:
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")
                    time.sleep(0.04)  # ~25 fps
            except Exception:
                pass
        elif self.path == "/process":
            process_html = os.path.join(os.path.dirname(self._dashboard._static_dir), "static", "process.html")
            if os.path.isfile(process_html):
                with open(process_html, "rb") as f:
                    content = f.read()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(content)))
                self.end_headers()
                self.wfile.write(content)
            else:
                self.send_error(404)
        elif self.path == "/api/process/status":
            result = self._dashboard.get_process_status()
            self._json_response(result)
        elif self.path == "/api/process/status_lite":
            result = self._dashboard.get_process_status_lite()
            self._json_response(result)
        elif self.path == "/api/process/convert_status":
            result = self._dashboard.get_convert_status()
            self._json_response(result)
        elif self.path.startswith("/robot/") or self.path.startswith("/vendor/"):
            # Serve URDF, meshes, and JS vendor files from ur15_dashboard
            rel_path = self.path.lstrip("/")
            file_path = os.path.join(self._ur15_static_dir, rel_path)
            if os.path.isfile(file_path):
                content_type, _ = mimetypes.guess_type(file_path)
                with open(file_path, "rb") as f:
                    data = f.read()
                self.send_response(200)
                self.send_header("Content-Type", content_type or "application/octet-stream")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            else:
                self.send_error(404)
        else:
            super().do_GET()

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        body = json.loads(self.rfile.read(length)) if length else {}
        if self.path == "/api/recording/start":
            result = self._dashboard.start_recording(body.get("save_path", ""))
            self._json_response(result)
        elif self.path == "/api/recording/stop":
            result = self._dashboard.stop_recording()
            self._json_response(result)
        elif self.path == "/api/recording/discard":
            result = self._dashboard.discard_last()
            self._json_response(result)
        elif self.path == "/api/replay/start":
            result = self._dashboard.start_replay()
            self._json_response(result)
        elif self.path == "/api/replay/stop":
            result = self._dashboard.stop_replay()
            self._json_response(result)
        elif self.path == "/api/process/convert":
            result = self._dashboard.run_conversion(body)
            self._json_response(result)
        elif self.path == "/api/process/convert/stop":
            result = self._dashboard.stop_conversion()
            self._json_response(result)
        elif self.path == "/api/process/delete_all":
            result = self._dashboard.delete_all_converted()
            self._json_response(result)
        else:
            self.send_error(404)

    def _json_response(self, data):
        try:
            body = json.dumps(data).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, format, *args):
        if len(args) >= 2 and "404" in str(args[1]):
            super().log_message(format, *args)


class DataCollectionDashboard(Node):
    def __init__(self):
        super().__init__("data_collection_dashboard")
        self.declare_parameter("port", 8086)
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("camera_topic", "/ur15_camera/image_raw")

        web_port = self.get_parameter("port").value
        web_host = self.get_parameter("host").value
        camera_topic = self.get_parameter("camera_topic").value

        self._latest_state = None
        self._lock = threading.Lock()
        self._sse_clients = []
        self._sse_lock = threading.Lock()

        # Joint state
        self._joint_positions = {}
        self._joint_velocities = {}
        self._joint_efforts = {}

        # Target joint state
        self._target_positions = {}

        # F/T sensor
        self._ft_force = np.zeros(3)
        self._ft_torque = np.zeros(3)

        # Rate tracking
        self._msg_count = 0
        self._last_rate_time = time.time()
        self._rate = 0.0

        # SSE throttle
        self._last_sse_push = 0.0
        self._sse_interval = 1.0 / 30.0

        # Camera image
        self._camera_jpeg = None
        self._camera_lock = threading.Lock()

        # Replay state
        self._replaying = False
        self._replay_thread = None
        self._replay_stop_event = threading.Event()
        self._arm_pub = None

        # Conversion state
        self._converting = False
        self._convert_lock = threading.Lock()
        self._convert_episode_status = {}  # bag_name -> {status, detail, timestamp}
        self._convert_result = None
        self._convert_thread = None
        self._convert_cancel_event = threading.Event()
        self._convert_dataset_dir = None

        # Recording state
        self._recording = False
        self._record_proc = None
        self._record_start_time = None
        self._record_bag_path = None
        self._episode_count = 0
        self._last_bag_path = None
        self._default_save_path = ""
        self._record_topics = [
            "/ur15_camera/image_raw",
            "/arm_joint_state",
            "/force_torque_sensor_broadcaster/wrench",
            "/joint_states",
        ]

        # Subscriptions
        self.create_subscription(
            JointState, "/joint_states", self._joint_state_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            WrenchStamped,
            "/force_torque_sensor_broadcaster/wrench",
            self._ft_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState, "/target_joint", self._target_joint_cb, 10
        )
        self.create_subscription(
            Image, camera_topic, self._camera_cb, qos_profile_sensor_data
        )
        self.get_logger().info(f"Camera topic: {camera_topic}")

        # Web server
        from common.workspace import get_workspace_root

        ws_root = get_workspace_root()
        static_dir = os.path.join(ws_root, "src", "data_collection", "static")
        self._static_dir = static_dir
        os.chdir(static_dir)

        # Default save path
        self._default_save_path = os.path.join(ws_root, "tmp", "rosbag_data")

        # Count existing episodes in save dir
        if os.path.isdir(self._default_save_path):
            existing = [d for d in os.listdir(self._default_save_path)
                        if d.startswith("episode_") and os.path.isdir(os.path.join(self._default_save_path, d))]
            self._episode_count = len(existing)
            self.get_logger().info(f"Found {self._episode_count} existing episodes in {self._default_save_path}")

        # Resolve ur15_dashboard static dir for robot/ and vendor/ assets
        ur15_static_dir = os.path.join(ws_root, "src", "ur15_dashboard", "static")
        self.get_logger().info(f"Serving static from: {static_dir}")
        self.get_logger().info(f"UR15 assets from: {ur15_static_dir}")

        html_path = os.path.join(static_dir, "index.html")
        with open(html_path, "rb") as f:
            viewer_html = f.read()

        handler = partial(
            DashboardHandler, dashboard_node=self, viewer_html=viewer_html,
            ur15_static_dir=ur15_static_dir
        )
        self._httpd = http.server.ThreadingHTTPServer((web_host, web_port), handler)
        self._web_thread = threading.Thread(
            target=self._httpd.serve_forever, daemon=True
        )
        self._web_thread.start()

        self.get_logger().info(f"Data Collection Dashboard: http://localhost:{web_port}")

    def _joint_state_cb(self, msg):
        now = time.time()
        with self._lock:
            for i, name in enumerate(msg.name):
                if name in JOINT_NAMES:
                    self._joint_positions[name] = msg.position[i] if i < len(msg.position) else 0.0
                    self._joint_velocities[name] = msg.velocity[i] if i < len(msg.velocity) else 0.0
                    self._joint_efforts[name] = msg.effort[i] if i < len(msg.effort) else 0.0
            self._msg_count += 1
            dt = now - self._last_rate_time
            if dt >= 1.0:
                self._rate = self._msg_count / dt
                self._msg_count = 0
                self._last_rate_time = now

        if now - self._last_sse_push >= self._sse_interval:
            self._last_sse_push = now
            self._push_sse()

    def _ft_cb(self, msg):
        self._ft_force = np.array(
            [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]
        )
        self._ft_torque = np.array(
            [msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]
        )

    def _target_joint_cb(self, msg):
        with self._lock:
            for i, name in enumerate(msg.name):
                if name in JOINT_NAMES:
                    self._target_positions[name] = msg.position[i] if i < len(msg.position) else 0.0

    def _camera_cb(self, msg):
        try:
            # Convert ROS Image to JPEG
            if msg.encoding in ("rgb8", "bgr8"):
                h, w = msg.height, msg.width
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
                if msg.encoding == "rgb8":
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif msg.encoding == "mono8":
                h, w = msg.height, msg.width
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
            else:
                return
            _, jpeg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
            with self._camera_lock:
                self._camera_jpeg = jpeg.tobytes()
        except Exception:
            pass

    def get_camera_jpeg(self):
        with self._camera_lock:
            return self._camera_jpeg

    def _push_sse(self):
        with self._lock:
            positions = [round(self._joint_positions.get(n, 0.0), 4) for n in JOINT_NAMES]
            positions_deg = [round(self._joint_positions.get(n, 0.0) * RAD_TO_DEG, 1) for n in JOINT_NAMES]
            velocities = [round(self._joint_velocities.get(n, 0.0), 4) for n in JOINT_NAMES]
            efforts = [round(self._joint_efforts.get(n, 0.0), 2) for n in JOINT_NAMES]
            rate = round(self._rate, 0)
            has_data = len(self._joint_positions) > 0
            target_positions = [round(self._target_positions.get(n, 0.0), 4) for n in JOINT_NAMES] if self._target_positions else []

        force_mag = round(float(np.linalg.norm(self._ft_force)), 2)
        state = {
            "connected": has_data,
            "joints_rad": positions,
            "joints_deg": positions_deg,
            "velocity": velocities,
            "effort": efforts,
            "ft": {
                "force": {
                    "x": round(float(self._ft_force[0]), 2),
                    "y": round(float(self._ft_force[1]), 2),
                    "z": round(float(self._ft_force[2]), 2),
                },
                "torque": {
                    "x": round(float(self._ft_torque[0]), 3),
                    "y": round(float(self._ft_torque[1]), 3),
                    "z": round(float(self._ft_torque[2]), 3),
                },
                "force_mag": force_mag,
            },
            "rate": rate,
            "target_joints_rad": target_positions,
            "recording": self._recording,
            "episode_count": self._count_episodes(),
            "record_duration": round(time.time() - self._record_start_time, 1) if self._recording and self._record_start_time else 0,
            "save_path": self._default_save_path,
            "can_discard": self._last_bag_path is not None and not self._recording,
            "replaying": self._replaying,
        }

        with self._lock:
            self._latest_state = state

        data_str = json.dumps(state)
        self._broadcast_sse(data_str)

    def _broadcast_sse(self, data_str):
        with self._sse_lock:
            dead = []
            for wf in self._sse_clients:
                try:
                    wf.write(f"data: {data_str}\n\n".encode())
                    wf.flush()
                except Exception:
                    dead.append(wf)
            for d in dead:
                self._sse_clients.remove(d)

    def get_latest_state(self):
        with self._lock:
            state = self._latest_state or {}
        state["save_path"] = self._default_save_path
        state["episode_count"] = self._count_episodes()
        state["recording"] = self._recording
        state["replaying"] = self._replaying
        state["can_discard"] = self._last_bag_path is not None and not self._recording
        return state

    def _count_episodes(self):
        """Count episode directories in save path."""
        if not os.path.isdir(self._default_save_path):
            return 0
        return len([d for d in os.listdir(self._default_save_path)
                     if d.startswith("episode_") and os.path.isdir(
                         os.path.join(self._default_save_path, d))])

    def get_process_status_lite(self):
        """Fast status: bag names + sizes only, no YAML/JSON/H5 parsing."""
        rosbag_dir = self._default_save_path
        from common.workspace import get_workspace_root
        dataset_dir = os.path.join(get_workspace_root(), "tmp", "dataset")
        bags = []
        if os.path.isdir(rosbag_dir):
            for d in sorted(os.listdir(rosbag_dir)):
                bag_path = os.path.join(rosbag_dir, d)
                if not d.startswith("episode_") or not os.path.isdir(bag_path):
                    continue
                try:
                    size_mb = sum(
                        os.path.getsize(os.path.join(bag_path, f))
                        for f in os.listdir(bag_path)
                    ) / (1024 * 1024)
                except OSError:
                    size_mb = 0
                bags.append({
                    "name": d,
                    "size_mb": round(size_mb, 1),
                    "converted": False,
                    "topics": [],
                    "duration": 0,
                    "total_messages": 0,
                    "dataset": None,
                })
        return {
            "rosbag_dir": rosbag_dir,
            "dataset_dir": dataset_dir,
            "bags": bags,
            "dataset": {"exists": os.path.isdir(dataset_dir), "episodes": 0, "h5_files": 0, "videos": 0},
        }

    def get_process_status(self):
        """Get detailed status of rosbag episodes and dataset."""
        import yaml
        from common.workspace import get_workspace_root
        ws_root = get_workspace_root()
        rosbag_dir = self._default_save_path
        dataset_dir = os.path.join(ws_root, "tmp", "dataset")

        # Build map of converted bag names -> episode JSON data
        converted_map = {}  # bag_name -> episode metadata dict
        ep_json_dir = os.path.join(dataset_dir, "episode")
        if os.path.isdir(ep_json_dir):
            for f in os.listdir(ep_json_dir):
                if f.endswith(".json"):
                    try:
                        with open(os.path.join(ep_json_dir, f)) as jf:
                            d = json.load(jf)
                        for ep in d.get("episodes", []):
                            if "source_bag" in ep:
                                converted_map[ep["source_bag"]] = {
                                    "filename": ep.get("filename", ""),
                                    "start_frame": ep.get("start_frame", 0),
                                    "end_frame": ep.get("end_frame", 0),
                                    "instruction": ep.get("instruction", []),
                                }
                    except Exception:
                        pass

        # Scan rosbag episodes with detailed info
        bags = []
        if os.path.isdir(rosbag_dir):
            for d in sorted(os.listdir(rosbag_dir)):
                bag_path = os.path.join(rosbag_dir, d)
                if not d.startswith("episode_") or not os.path.isdir(bag_path):
                    continue
                size_mb = sum(
                    os.path.getsize(os.path.join(bag_path, f))
                    for f in os.listdir(bag_path)
                ) / (1024 * 1024)
                bag_info = {
                    "name": d,
                    "size_mb": round(size_mb, 1),
                    "converted": d in converted_map,
                    "topics": [],
                    "duration": 0,
                    "total_messages": 0,
                    "dataset": None,
                }
                # Add converted dataset details
                if d in converted_map:
                    ep_meta = converted_map[d]
                    ds_detail = dict(ep_meta)
                    ds_detail["frames"] = ep_meta.get("end_frame", 0) - ep_meta.get("start_frame", 0)
                    # Check H5 file details
                    h5_name = ep_meta.get("filename", "")
                    h5_path = os.path.join(dataset_dir, "data", h5_name + ".h5")
                    if os.path.isfile(h5_path):
                        ds_detail["h5_size_mb"] = round(os.path.getsize(h5_path) / (1024 * 1024), 2)
                        try:
                            import h5py
                            with h5py.File(h5_path, "r") as hf:
                                datasets = []
                                def _visit(name, obj):
                                    if hasattr(obj, "shape"):
                                        datasets.append({"name": name, "shape": list(obj.shape), "dtype": str(obj.dtype)})
                                hf.visititems(_visit)
                                ds_detail["h5_datasets"] = datasets
                        except Exception:
                            pass
                    # Check video
                    vid_path = os.path.join(dataset_dir, "video_agentview", h5_name + ".mp4")
                    ds_detail["has_video"] = os.path.isfile(vid_path)
                    if ds_detail["has_video"]:
                        ds_detail["video_size_mb"] = round(os.path.getsize(vid_path) / (1024 * 1024), 2)
                    bag_info["dataset"] = ds_detail
                # Read metadata.yaml for topic details
                meta_file = os.path.join(bag_path, "metadata.yaml")
                if os.path.isfile(meta_file):
                    try:
                        with open(meta_file) as f:
                            meta = yaml.safe_load(f)
                        info = meta.get("rosbag2_bagfile_information", {})
                        bag_info["duration"] = round(info.get("duration", {}).get("nanoseconds", 0) * 1e-9, 1)
                        bag_info["total_messages"] = info.get("message_count", 0)
                        for t in info.get("topics_with_message_count", []):
                            tm = t.get("topic_metadata", {})
                            bag_info["topics"].append({
                                "name": tm.get("name", ""),
                                "type": tm.get("type", ""),
                                "count": t.get("message_count", 0),
                            })
                    except Exception:
                        pass
                bags.append(bag_info)

        # Scan dataset
        dataset_info = {"exists": False, "episodes": 0, "h5_files": 0, "videos": 0}
        if os.path.isdir(dataset_dir):
            dataset_info["exists"] = True
            data_dir = os.path.join(dataset_dir, "data")
            vid_dir = os.path.join(dataset_dir, "video_agentview")
            if os.path.isdir(ep_json_dir):
                dataset_info["episodes"] = len([f for f in os.listdir(ep_json_dir) if f.endswith(".json")])
            if os.path.isdir(data_dir):
                dataset_info["h5_files"] = len([f for f in os.listdir(data_dir) if f.endswith(".h5")])
            if os.path.isdir(vid_dir):
                dataset_info["videos"] = len([f for f in os.listdir(vid_dir) if f.endswith(".mp4")])
            meta_path = os.path.join(dataset_dir, "meta.json")
            if os.path.isfile(meta_path):
                with open(meta_path) as f:
                    dataset_info["meta"] = json.load(f)

        return {
            "rosbag_dir": rosbag_dir,
            "dataset_dir": dataset_dir,
            "bags": bags,
            "dataset": dataset_info,
        }

    def run_conversion(self, params):
        """Start bag-to-dataset conversion in a background thread."""
        if self._converting:
            return {"success": False, "error": "Conversion already in progress"}
        try:
            import sys
            from common.workspace import get_workspace_root
            ws_root = get_workspace_root()
            scripts_dir = os.path.join(ws_root, "scripts")
            if scripts_dir not in sys.path:
                sys.path.insert(0, scripts_dir)

            rosbag_dir = params.get("rosbag_dir", self._default_save_path)
            dataset_dir = params.get("dataset_dir", os.path.join(ws_root, "tmp", "dataset"))
            task_name = params.get("task_name", "teleop")
            num_threads = int(params.get("num_threads", 16))

            with self._convert_lock:
                self._converting = True
                self._convert_episode_status = {}
                self._convert_result = None
                self._convert_cancel_event.clear()
                self._convert_dataset_dir = dataset_dir

            self._convert_thread = threading.Thread(
                target=self._conversion_worker,
                args=(rosbag_dir, dataset_dir, task_name, num_threads),
                daemon=True,
            )
            self._convert_thread.start()
            self.get_logger().info(
                f"Conversion started: {rosbag_dir} -> {dataset_dir} "
                f"(task={task_name}, threads={num_threads})"
            )
            return {"success": True, "message": "Conversion started", "num_threads": num_threads}
        except Exception as e:
            self._converting = False
            return {"success": False, "error": str(e)}

    def _conversion_worker(self, rosbag_dir, dataset_dir, task_name, num_threads):
        """Background worker that runs the conversion."""
        try:
            from bag_to_dataset import convert_all

            def _status_cb(bag_name, status, detail):
                with self._convert_lock:
                    entry = {
                        "status": status,
                        "timestamp": time.time(),
                    }
                    if isinstance(detail, dict) and "stage" in detail:
                        entry["stage"] = detail["stage"]
                        entry["total_stages"] = detail["total_stages"]
                        entry["percent"] = detail["percent"]
                    self._convert_episode_status[bag_name] = entry
                if status != "converting":
                    self.get_logger().info(f"Episode {bag_name}: {status}")

            result = convert_all(
                rosbag_dir, dataset_dir, task_name,
                num_threads=num_threads,
                status_callback=_status_cb,
                cancel_event=self._convert_cancel_event,
            )
            with self._convert_lock:
                self._convert_result = result
                self._converting = False
            if self._convert_cancel_event.is_set():
                self.get_logger().info("Conversion cancelled by user")
                self._cleanup_incomplete_conversions(dataset_dir)
            else:
                self.get_logger().info(
                    f"Conversion finished: {result.get('converted', 0)} converted, "
                    f"{result.get('skipped', 0)} skipped, {result.get('errors', 0)} errors"
                )
        except Exception as e:
            with self._convert_lock:
                self._convert_result = {"success": False, "error": str(e)}
                self._converting = False
            self.get_logger().error(f"Conversion failed: {e}")

    def get_convert_status(self):
        """Return current conversion progress for polling."""
        with self._convert_lock:
            episodes = {}
            for bag_name, info in self._convert_episode_status.items():
                ep = {
                    "status": info["status"],
                    "timestamp": info.get("timestamp", 0),
                }
                if "stage" in info:
                    ep["stage"] = info["stage"]
                    ep["total_stages"] = info["total_stages"]
                    ep["percent"] = info["percent"]
                episodes[bag_name] = ep
            return {
                "converting": self._converting,
                "episodes": episodes,
                "result": self._convert_result,
            }

    def stop_conversion(self):
        """Stop the running conversion and clean up incomplete files."""
        if not self._converting:
            return {"success": False, "error": "No conversion in progress"}
        self._convert_cancel_event.set()
        self.get_logger().info("Conversion cancel signal sent")
        return {"success": True, "message": "Conversion stopping"}

    def _cleanup_incomplete_conversions(self, dataset_dir):
        """Remove output files for episodes that were not fully converted."""
        with self._convert_lock:
            incomplete = [
                name for name, info in self._convert_episode_status.items()
                if info["status"] in ("converting", "cancelled")
            ]
        if not incomplete:
            return
        self.get_logger().info(f"Cleaning up {len(incomplete)} incomplete episodes")
        # Find and remove any partial output files for incomplete episodes
        for bag_name in incomplete:
            # Match output files that contain the bag_name's episode index
            for subdir in ("data", "episode", "video_agentview"):
                dirpath = os.path.join(dataset_dir, subdir)
                if not os.path.isdir(dirpath):
                    continue
                for fname in os.listdir(dirpath):
                    # Episode output filenames include source_bag info via episode index
                    # but safer to check all recently created files from this bag
                    # The source_bag is stored in the episode JSON, but for incomplete
                    # ones the JSON may not exist. Match by checking episode JSONs.
                    pass
            # Alternative: search episode JSONs for source_bag references
            ep_json_dir = os.path.join(dataset_dir, "episode")
            if os.path.isdir(ep_json_dir):
                for fname in list(os.listdir(ep_json_dir)):
                    if not fname.endswith(".json"):
                        continue
                    fpath = os.path.join(ep_json_dir, fname)
                    try:
                        with open(fpath) as jf:
                            d = json.load(jf)
                        for ep in d.get("episodes", []):
                            if ep.get("source_bag") == bag_name:
                                ep_name = ep.get("filename", "")
                                self._remove_episode_files(dataset_dir, ep_name, fpath)
                                self.get_logger().info(f"Cleaned up incomplete: {bag_name}")
                    except Exception:
                        pass
            # Also remove partial files that may exist without a complete JSON
            # (if conversion was interrupted before writing the JSON)
            self._remove_partial_files_for_bag(dataset_dir, bag_name)
        # Update episode status to reflect cleanup
        with self._convert_lock:
            for name in incomplete:
                if name in self._convert_episode_status:
                    self._convert_episode_status[name]["status"] = "cancelled"

    def _remove_episode_files(self, dataset_dir, ep_name, json_path):
        """Remove H5, video, and JSON for a given episode name."""
        for subdir, ext in [("data", ".h5"), ("video_agentview", ".mp4")]:
            fpath = os.path.join(dataset_dir, subdir, ep_name + ext)
            if os.path.isfile(fpath):
                os.remove(fpath)
        if os.path.isfile(json_path):
            os.remove(json_path)

    def _remove_partial_files_for_bag(self, dataset_dir, bag_name):
        """Remove any output files that might exist without a completed episode JSON."""
        # The episode index in the filename corresponds to the bag's sorted position.
        # We scan for any files that reference this bag by checking H5 files
        # that don't have a corresponding completed episode JSON.
        data_dir = os.path.join(dataset_dir, "data")
        ep_dir = os.path.join(dataset_dir, "episode")
        vid_dir = os.path.join(dataset_dir, "video_agentview")
        if os.path.isdir(data_dir):
            for fname in list(os.listdir(data_dir)):
                if not fname.endswith(".h5"):
                    continue
                ep_name = fname[:-3]
                json_path = os.path.join(ep_dir, ep_name + ".json") if os.path.isdir(ep_dir) else ""
                if not os.path.isfile(json_path):
                    # No JSON means this H5 is from an incomplete conversion
                    os.remove(os.path.join(data_dir, fname))
                    vid_path = os.path.join(vid_dir, ep_name + ".mp4")
                    if os.path.isfile(vid_path):
                        os.remove(vid_path)
                    self.get_logger().info(f"Removed partial files: {ep_name}")

    def delete_all_converted(self):
        """Delete all converted dataset files (keeps rosbags)."""
        import shutil
        try:
            from common.workspace import get_workspace_root
            dataset_dir = os.path.join(get_workspace_root(), "tmp", "dataset")
            if not os.path.isdir(dataset_dir):
                return {"success": False, "error": "Dataset directory does not exist"}
            shutil.rmtree(dataset_dir)
            self.get_logger().info(f"Deleted dataset: {dataset_dir}")
            return {"success": True, "message": f"Deleted {dataset_dir}"}
        except Exception as e:
            return {"success": False, "error": str(e)}

    def register_sse_client(self, wfile):
        with self._sse_lock:
            self._sse_clients.append(wfile)

    def unregister_sse_client(self, wfile):
        with self._sse_lock:
            if wfile in self._sse_clients:
                self._sse_clients.remove(wfile)

    def destroy_node(self):
        if self._recording:
            self.stop_recording()
        self._httpd.shutdown()
        super().destroy_node()

    def start_recording(self, save_path=""):
        if self._recording:
            return {"success": False, "error": "Already recording"}
        if save_path:
            self._default_save_path = save_path
        base_dir = self._default_save_path
        os.makedirs(base_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        bag_name = f"episode_{self._count_episodes():04d}_{ts}"
        bag_path = os.path.join(base_dir, bag_name)
        cmd = ["ros2", "bag", "record"] + self._record_topics + ["-o", bag_path]
        try:
            self._record_proc = subprocess.Popen(
                cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )
            self._recording = True
            self._record_start_time = time.time()
            self._record_bag_path = bag_path
            self.get_logger().info(f"Recording started: {bag_path}")
            return {"success": True, "bag_path": bag_path}
        except Exception as e:
            return {"success": False, "error": str(e)}

    def stop_recording(self):
        if not self._recording or not self._record_proc:
            return {"success": False, "error": "Not recording"}
        try:
            os.killpg(os.getpgid(self._record_proc.pid), signal.SIGINT)
            self._record_proc.wait(timeout=5)
        except Exception:
            try:
                self._record_proc.kill()
            except Exception:
                pass
        duration = round(time.time() - self._record_start_time, 1) if self._record_start_time else 0
        bag_path = self._record_bag_path
        self._recording = False
        self._record_proc = None
        self._record_start_time = None
        self._last_bag_path = bag_path
        self._episode_count += 1
        self.get_logger().info(f"Recording stopped: {bag_path} ({duration}s)")
        return {"success": True, "bag_path": bag_path, "duration": duration, "episode": self._episode_count}

    def discard_last(self):
        if self._recording:
            return {"success": False, "error": "Stop recording first"}
        if not self._last_bag_path:
            return {"success": False, "error": "No episode to discard"}
        import shutil
        path = self._last_bag_path
        try:
            if os.path.isdir(path):
                shutil.rmtree(path)
            self._episode_count = max(0, self._episode_count - 1)
            self.get_logger().info(f"Discarded: {path}")
            self._last_bag_path = None
            return {"success": True, "discarded": path, "episode": self._episode_count}
        except Exception as e:
            return {"success": False, "error": str(e)}

    def start_replay(self):
        """Replay the last episode by publishing /arm_joint_state from rosbag."""
        if self._replaying:
            return {"success": False, "error": "Already replaying"}
        if self._recording:
            return {"success": False, "error": "Cannot replay while recording"}
        if not self._last_bag_path or not os.path.isdir(self._last_bag_path):
            return {"success": False, "error": "No episode to replay"}

        if self._arm_pub is None:
            from alicia_duo_leader_driver.msg import ArmJointState
            self._arm_pub = self.create_publisher(ArmJointState, "/arm_joint_state", 10)

        self._replay_stop_event.clear()
        self._replay_thread = threading.Thread(
            target=self._replay_worker, args=(self._last_bag_path,), daemon=True
        )
        self._replay_thread.start()
        self._replaying = True
        self.get_logger().info(f"Replay started: {self._last_bag_path}")
        return {"success": True, "bag_path": self._last_bag_path}

    def stop_replay(self):
        if not self._replaying:
            return {"success": False, "error": "Not replaying"}
        self._replay_stop_event.set()
        if self._replay_thread:
            self._replay_thread.join(timeout=3)
        self._replaying = False
        self.get_logger().info("Replay stopped")
        return {"success": True}

    def _replay_worker(self, bag_path):
        """Read /arm_joint_state from bag and publish at original rate."""
        try:
            import rosbag2_py
            from rclpy.serialization import deserialize_message
            from alicia_duo_leader_driver.msg import ArmJointState

            reader = rosbag2_py.SequentialReader()
            storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
            converter_options = rosbag2_py.ConverterOptions(
                input_serialization_format="cdr", output_serialization_format="cdr"
            )
            reader.open(storage_options, converter_options)

            topic_filter = rosbag2_py.StorageFilter(topics=["/arm_joint_state"])
            reader.set_filter(topic_filter)

            messages = []
            while reader.has_next():
                topic, data, timestamp = reader.read_next()
                messages.append((timestamp, data))

            if not messages:
                self.get_logger().warn("No /arm_joint_state messages in bag")
                self._replaying = False
                return

            self.get_logger().info(f"Replaying {len(messages)} messages")

            for i, (timestamp, data) in enumerate(messages):
                if self._replay_stop_event.is_set():
                    break
                msg = deserialize_message(data, ArmJointState)
                msg.header.stamp = self.get_clock().now().to_msg()
                self._arm_pub.publish(msg)

                if i + 1 < len(messages):
                    dt = (messages[i + 1][0] - timestamp) * 1e-9
                    if 0 < dt < 1.0:
                        time.sleep(dt)

        except Exception as e:
            self.get_logger().error(f"Replay error: {e}")
        finally:
            self._replaying = False
            self.get_logger().info("Replay finished")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = DataCollectionDashboard()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
