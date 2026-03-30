#!/usr/bin/env python3
"""
Data Collection Dashboard — Web-based dashboard for UR15 data collection.

Subscribes to /joint_states and /force_torque_sensor_broadcaster/wrench,
serves a web page with a 3D URDF viewer sub-window and data collection controls.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped

import http.server
import json
import math
import mimetypes
import os
import threading
import time
import numpy as np
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

    def log_message(self, format, *args):
        if len(args) >= 2 and "404" in str(args[1]):
            super().log_message(format, *args)


class DataCollectionDashboard(Node):
    def __init__(self):
        super().__init__("data_collection_dashboard")
        self.declare_parameter("port", 8086)
        self.declare_parameter("host", "0.0.0.0")

        web_port = self.get_parameter("port").value
        web_host = self.get_parameter("host").value

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

        # Web server
        from common.workspace import get_workspace_root

        ws_root = get_workspace_root()
        static_dir = os.path.join(ws_root, "src", "data_collection", "static")
        os.chdir(static_dir)

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
        self._httpd.shutdown()
        super().destroy_node()


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
