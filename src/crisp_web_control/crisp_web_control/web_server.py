"""Web server for CRISP robot control.

Provides a REST API and web UI for switching controllers,
monitoring robot status, and commanding motion.
"""

import json
import os
import threading
import time

import numpy as np
import rclpy
from flask import Flask, Response, jsonify, request, send_from_directory
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import ReentrantCallbackGroup

from crisp_py.robot import make_robot
from crisp_py.control.parameters_client import ParametersClient


class WebControlServer:
    """Combines a crisp_py Robot with a Flask web server."""

    def __init__(self):
        rclpy.init()
        self.robot = make_robot("ur")
        self._ready = False
        self._error = None
        self._joint_velocity = None
        self._joint_effort = None
        self._joint_names_ordered = None
        self._ft_force = np.zeros(3)
        self._ft_torque = np.zeros(3)
        self._commanded_torques = None
        self._param_clients = {}  # cache: controller_name -> ParametersClient

        # Subscribe to /joint_states for velocity and effort
        self.robot.node.create_subscription(
            JointState,
            "joint_states",
            self._joint_state_cb,
            qos_profile_sensor_data,
            callback_group=ReentrantCallbackGroup(),
        )

        # Subscribe to /force_torque_sensor_broadcaster/wrench for TCP force/torque
        self.robot.node.create_subscription(
            WrenchStamped,
            "force_torque_sensor_broadcaster/wrench",
            self._ft_cb,
            qos_profile_sensor_data,
            callback_group=ReentrantCallbackGroup(),
        )

        # Subscribe to commanded torques from CRISP controllers
        self.robot.node.create_subscription(
            JointState,
            "commanded_torques",
            self._commanded_torques_cb,
            10,
            callback_group=ReentrantCallbackGroup(),
        )

        # Wait for robot in background so Flask can start immediately
        t = threading.Thread(target=self._wait_for_robot, daemon=True)
        t.start()

        self.app = self._create_app()

    def _joint_state_cb(self, msg: JointState):
        """Cache velocity and effort from /joint_states."""
        names = self.robot.config.joint_names
        vel = np.zeros(len(names))
        eff = np.zeros(len(names))
        for jname, v, e in zip(msg.name, msg.velocity, msg.effort):
            if jname in names:
                idx = names.index(jname)
                vel[idx] = v
                eff[idx] = e
        self._joint_velocity = vel
        self._joint_effort = eff

    def _ft_cb(self, msg: WrenchStamped):
        """Cache TCP force/torque from /ft_data."""
        self._ft_force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        self._ft_torque = np.array([msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])

    def _commanded_torques_cb(self, msg: JointState):
        """Cache commanded torques from CRISP controller."""
        names = self.robot.config.joint_names
        torques = np.zeros(len(names))
        for jname, eff in zip(msg.name, msg.effort):
            if jname in names:
                torques[names.index(jname)] = eff
        self._commanded_torques = torques

    def _wait_for_robot(self):
        try:
            self.robot.wait_until_ready(timeout=30.0)
            self._ready = True
        except Exception as e:
            self._error = str(e)

    def _create_app(self):
        static_dir = self._find_static_dir()
        app = Flask(__name__, static_folder=static_dir)

        @app.route("/")
        def index():
            return send_from_directory(static_dir, "index.html")

        @app.route("/api/status")
        def status():
            if not self._ready:
                return jsonify({"ready": False, "error": self._error})

            pose = self.robot.end_effector_pose
            joints = self.robot.joint_values
            joint_names = self.robot.config.joint_names

            data = {
                "ready": True,
                "pose": {
                    "position": {
                        "x": round(float(pose.position[0]), 5),
                        "y": round(float(pose.position[1]), 5),
                        "z": round(float(pose.position[2]), 5),
                    },
                    "orientation": {
                        "x": round(float(pose.orientation.as_quat()[0]), 5),
                        "y": round(float(pose.orientation.as_quat()[1]), 5),
                        "z": round(float(pose.orientation.as_quat()[2]), 5),
                        "w": round(float(pose.orientation.as_quat()[3]), 5),
                    },
                },
                "joints": {
                    name: round(float(val), 4)
                    for name, val in zip(joint_names, joints)
                },
            }
            return jsonify(data)

        @app.route("/api/controllers")
        def controllers():
            if not self._ready:
                return jsonify({"ready": False})
            try:
                ctrl_list = self.robot.controller_switcher_client.get_controller_list()
                result = []
                for c in ctrl_list:
                    result.append({
                        "name": c.name,
                        "type": c.type,
                        "state": c.state,
                    })
                return jsonify({"ready": True, "controllers": result})
            except Exception as e:
                return jsonify({"ready": True, "error": str(e)}), 500

        @app.route("/api/switch_controller", methods=["POST"])
        def switch_controller():
            if not self._ready:
                return jsonify({"error": "Robot not ready"}), 503
            data = request.get_json()
            controller_name = data.get("controller")
            if not controller_name:
                return jsonify({"error": "Missing 'controller' field"}), 400
            try:
                self.robot.controller_switcher_client.switch_controller(controller_name)
                return jsonify({"success": True, "controller": controller_name})
            except Exception as e:
                return jsonify({"error": str(e)}), 500

        @app.route("/api/params/<controller_name>")
        def get_params(controller_name):
            """Get all parameters for a controller."""
            if not self._ready:
                return jsonify({"error": "Robot not ready"}), 503
            try:
                client = self._get_param_client(controller_name)
                names = client.list_parameters()
                values = client.get_parameters(names)
                params = {}
                for name, val in zip(names, values):
                    params[name] = val
                return jsonify({"controller": controller_name, "params": params})
            except Exception as e:
                return jsonify({"error": str(e)}), 500

        @app.route("/api/params/<controller_name>", methods=["POST"])
        def set_params(controller_name):
            """Set parameters for a controller. Body: {"params": {"name": value, ...}}"""
            if not self._ready:
                return jsonify({"error": "Robot not ready"}), 503
            data = request.get_json()
            params_to_set = data.get("params", {})
            if not params_to_set:
                return jsonify({"error": "Missing 'params' field"}), 400
            try:
                client = self._get_param_client(controller_name)
                # Read current types to ensure we send the correct type
                # (ROS 2 rejects int where double is expected)
                current_names = list(params_to_set.keys())
                current_values = client.get_parameters(current_names)
                param_tuples = []
                for (k, new_val), cur_val in zip(params_to_set.items(), current_values):
                    if isinstance(cur_val, float) and isinstance(new_val, int):
                        new_val = float(new_val)
                    param_tuples.append((k, new_val))
                client.set_parameters(param_tuples)
                return jsonify({"success": True, "set": current_names})
            except Exception as e:
                return jsonify({"error": str(e)}), 500

        @app.route("/api/set_target", methods=["POST"])
        def set_target():
            if not self._ready:
                return jsonify({"error": "Robot not ready"}), 503
            data = request.get_json()
            try:
                from crisp_py.utils.geometry import Pose
                from scipy.spatial.transform import Rotation

                pos = np.array([data["x"], data["y"], data["z"]])
                quat = [data["qx"], data["qy"], data["qz"], data["qw"]]
                ori = Rotation.from_quat(quat)
                target = Pose(position=pos, orientation=ori)
                self.robot.set_target(pose=target)
                return jsonify({"success": True})
            except Exception as e:
                return jsonify({"error": str(e)}), 500

        @app.route("/api/move_delta", methods=["POST"])
        def move_delta():
            if not self._ready:
                return jsonify({"error": "Robot not ready"}), 503
            data = request.get_json()
            try:
                dx = float(data.get("dx", 0))
                dy = float(data.get("dy", 0))
                dz = float(data.get("dz", 0))
                current = self.robot.end_effector_pose
                from crisp_py.utils.geometry import Pose

                new_pos = current.position + np.array([dx, dy, dz])
                target = Pose(position=new_pos, orientation=current.orientation)
                self.robot.set_target(pose=target)
                return jsonify({
                    "success": True,
                    "new_position": {
                        "x": round(float(new_pos[0]), 5),
                        "y": round(float(new_pos[1]), 5),
                        "z": round(float(new_pos[2]), 5),
                    },
                })
            except Exception as e:
                return jsonify({"error": str(e)}), 500

        @app.route("/api/set_target_joint", methods=["POST"])
        def set_target_joint():
            """Set target joint positions. Body: {"joints": [q1, q2, ..., q6]}"""
            if not self._ready:
                return jsonify({"error": "Robot not ready"}), 503
            data = request.get_json()
            try:
                joints = np.array(data["joints"], dtype=float)
                self.robot.set_target_joint(joints)
                return jsonify({"success": True})
            except Exception as e:
                return jsonify({"error": str(e)}), 500

        @app.route("/api/move_joint_delta", methods=["POST"])
        def move_joint_delta():
            """Move a single joint by a delta. Body: {"index": 0, "delta": 0.05}"""
            if not self._ready:
                return jsonify({"error": "Robot not ready"}), 503
            data = request.get_json()
            try:
                idx = int(data["index"])
                delta = float(data["delta"])
                current = self.robot.joint_values.copy()
                current[idx] += delta
                self.robot.set_target_joint(current)
                return jsonify({"success": True, "joint": self.robot.config.joint_names[idx], "new_value": round(float(current[idx]), 4)})
            except Exception as e:
                return jsonify({"error": str(e)}), 500

        @app.route("/api/move_to", methods=["POST"])
        def move_to():
            """Linearly interpolate to a target pose using crisp_py's move_to."""
            if not self._ready:
                return jsonify({"error": "Robot not ready"}), 503
            data = request.get_json()
            try:
                from crisp_py.utils.geometry import Pose
                from scipy.spatial.transform import Rotation

                pos = np.array([data["x"], data["y"], data["z"]])
                quat = [data["qx"], data["qy"], data["qz"], data["qw"]]
                ori = Rotation.from_quat(quat)
                target = Pose(position=pos, orientation=ori)
                speed = float(data.get("speed", 0.05))

                # Run move_to in background so the API returns immediately
                def _move():
                    self.robot.move_to(pose=target, speed=speed)

                threading.Thread(target=_move, daemon=True).start()
                return jsonify({"success": True})
            except Exception as e:
                return jsonify({"error": str(e)}), 500

        @app.route("/api/stream")
        def stream():
            """Server-Sent Events stream of robot status at ~10 Hz."""
            def generate():
                while True:
                    if not self._ready:
                        yield f"data: {json.dumps({'ready': False})}\n\n"
                        time.sleep(1.0)
                        continue
                    try:
                        pose = self.robot.end_effector_pose
                        joints = self.robot.joint_values
                        vel = self._joint_velocity
                        eff = self._joint_effort
                        data = {
                            "ready": True,
                            "pose": {
                                "x": round(float(pose.position[0]), 5),
                                "y": round(float(pose.position[1]), 5),
                                "z": round(float(pose.position[2]), 5),
                                "qx": round(float(pose.orientation.as_quat()[0]), 5),
                                "qy": round(float(pose.orientation.as_quat()[1]), 5),
                                "qz": round(float(pose.orientation.as_quat()[2]), 5),
                                "qw": round(float(pose.orientation.as_quat()[3]), 5),
                            },
                            "joints": [round(float(v), 4) for v in joints],
                            "velocity": [round(float(v), 4) for v in vel] if vel is not None else [],
                            "effort": [round(float(v), 2) for v in eff] if eff is not None else [],
                            "ft": {
                                "force": {"x": round(float(self._ft_force[0]), 2), "y": round(float(self._ft_force[1]), 2), "z": round(float(self._ft_force[2]), 2)},
                                "torque": {"x": round(float(self._ft_torque[0]), 3), "y": round(float(self._ft_torque[1]), 3), "z": round(float(self._ft_torque[2]), 3)},
                                "force_mag": round(float(np.linalg.norm(self._ft_force)), 2),
                            },
                            "cmd_torques": [round(float(v), 3) for v in self._commanded_torques] if self._commanded_torques is not None else [],
                        }
                        yield f"data: {json.dumps(data)}\n\n"
                    except Exception:
                        yield f"data: {json.dumps({'ready': False})}\n\n"
                    time.sleep(0.1)

            return Response(generate(), mimetype="text/event-stream")

        return app

    def _get_param_client(self, controller_name):
        """Get or create a ParametersClient for a controller."""
        if controller_name not in self._param_clients:
            client = ParametersClient(self.robot.node, f"/{controller_name}")
            client.wait_until_ready(timeout_sec=5.0)
            self._param_clients[controller_name] = client
        return self._param_clients[controller_name]

    def _find_static_dir(self):
        """Find static files directory — works in both dev and installed modes."""
        # Installed via colcon
        try:
            from ament_index_python.packages import get_package_share_directory
            installed = os.path.join(get_package_share_directory("crisp_web_control"), "static")
            if os.path.isdir(installed):
                return installed
        except Exception:
            pass
        # Dev mode — relative to this file
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(here, "static")

    def run(self, host="0.0.0.0", port=8080):
        print(f"\n  CRISP Web Control: http://{host}:{port}\n")
        self.app.run(host=host, port=port, threaded=True)

    def shutdown(self):
        self.robot.shutdown()
        rclpy.shutdown()


def main():
    server = WebControlServer()
    try:
        server.run()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()


if __name__ == "__main__":
    main()
