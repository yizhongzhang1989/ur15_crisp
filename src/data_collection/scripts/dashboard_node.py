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
        else:
            self.send_error(404)

    def _json_response(self, data):
        body = json.dumps(data).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

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
        os.chdir(static_dir)

        # Default save path
        self._default_save_path = os.path.join(ws_root, "tmp", "rosbag_data")

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
            "episode_count": self._episode_count,
            "record_duration": round(time.time() - self._record_start_time, 1) if self._recording and self._record_start_time else 0,
            "save_path": self._default_save_path,
            "can_discard": self._last_bag_path is not None and not self._recording,
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
            return self._latest_state

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
        bag_name = f"episode_{self._episode_count:04d}_{ts}"
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
