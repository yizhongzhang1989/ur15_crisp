"""Web server for CRISP robot control.

Provides a REST API and web UI for switching controllers,
monitoring robot status, and commanding motion.
"""

import json
import math
import os
import shutil
import threading
import time
from collections import deque

import numpy as np
import rclpy
import yaml
from flask import Flask, Response, jsonify, request, send_from_directory
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt8, Float64MultiArray
from geometry_msgs.msg import WrenchStamped, PoseStamped
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import ReentrantCallbackGroup

from crisp_py.robot import make_robot
from crisp_py.control.parameters_client import ParametersClient
from common.workspace import get_config_path
from ur15_dashboard.kinematics import fk_quaternion, quaternion_to_rpy
from web_control.auto_opt import OptimizationManager
from web_control.ur_force_mode import URForceModeStreamer

try:
    from alicia_duo_leader_driver.msg import ArmJointState
    _HAS_ALICIA = True
except ImportError:
    _HAS_ALICIA = False

_CRISP_CONTROLLERS = ["gravity_compensation", "cartesian_impedance_controller", "joint_impedance_controller"]


def _find_workspace_config():
    """Find config/ur15_controllers.yaml in the workspace root."""
    try:
        path = get_config_path("ur15_controllers.yaml")
        return path
    except RuntimeError:
        return None


def _load_position_control_config():
    """Load position control config, creating from template if needed."""
    try:
        user_config = get_config_path("position_control.yaml")
    except RuntimeError:
        return {}
    if not os.path.isfile(user_config):
        try:
            from ament_index_python.packages import get_package_share_directory
            template = os.path.join(
                get_package_share_directory("web_control"), "config", "position_control_template.yaml"
            )
            if os.path.isfile(template):
                shutil.copy2(template, user_config)
                print(f"[web_control] Created position_control.yaml from template: {user_config}")
        except Exception:
            pass
    if os.path.isfile(user_config):
        with open(user_config, "r") as f:
            return yaml.safe_load(f) or {}
    return {}


def _set_nested(d: dict, keys: list, value):
    """Set a value in a nested dict from dotted key parts."""
    for key in keys[:-1]:
        d = d.setdefault(key, {})
    d[keys[-1]] = value


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
        self._target_joints = None
        self._alicia_joints = None  # latest Alicia leader arm joints (6,)
        self._alicia_teleop = False  # teleop mode: forward Alicia joints as target
        self._alicia_scale = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        self._alicia_offset = np.zeros(6)
        self._gripper_pub = None  # publisher for /gripper/target_position
        self._gripper_level = None  # current gripper level (0-9)
        self._gripper_raw = None  # latest leader gripper value
        self._plot_buffer = deque(maxlen=5000)  # high-rate plot samples
        self._plot_lock = threading.Lock()
        self._plot_downsample = 0  # counter for downsampling 500Hz to ~100Hz
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

        # Subscribe to /target_joint for target joint positions
        self.robot.node.create_subscription(
            JointState,
            "target_joint",
            self._target_joint_cb,
            10,
            callback_group=ReentrantCallbackGroup(),
        )

        # Subscribe to Alicia leader arm (optional — only if message type available)
        if _HAS_ALICIA:
            self.robot.node.create_subscription(
                ArmJointState,
                "/arm_joint_state",
                self._alicia_cb,
                10,
                callback_group=ReentrantCallbackGroup(),
            )

        # Joint command target pipeline: raw → controllers
        self._cmd_target_pub = self.robot.node.create_publisher(
            JointState, "/cmd_target_joint", 10
        )
        self._cmd_target_pose_pub = self.robot.node.create_publisher(
            PoseStamped, "/cmd_target_pose", 10
        )
        self._fwd_pos_pub = self.robot.node.create_publisher(
            Float64MultiArray, "/forward_position_controller/commands", 10
        )
        self._cmd_target_raw = None     # latest raw target (set by API/teleop)
        self._cmd_target_pose = None    # cached FK pose: {"x","y","z","qx","qy","qz","qw"}
        # Load position control config for forward_position_controller trapezoidal profile
        pos_cfg = _load_position_control_config()
        joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
        ]
        vel_cfg = pos_cfg.get("fwd_pos_max_vel", {})
        accel_cfg = pos_cfg.get("fwd_pos_max_accel", {})
        decel_cfg = pos_cfg.get("fwd_pos_max_decel", {})
        default_vel = [2.0, 2.0, 2.0, 4.0, 4.0, 4.0]
        default_accel = [6.0, 6.0, 6.0, 12.0, 12.0, 12.0]
        default_decel = [4.0, 4.0, 4.0, 8.0, 8.0, 8.0]
        self._fwd_pos_max_vel = np.array([
            vel_cfg.get(jn, dv) for jn, dv in zip(joint_names, default_vel)
        ])
        self._fwd_pos_max_accel = np.array([
            accel_cfg.get(jn, da) for jn, da in zip(joint_names, default_accel)
        ])
        self._fwd_pos_max_decel = np.array([
            decel_cfg.get(jn, dd) for jn, dd in zip(joint_names, default_decel)
        ])
        self._fwd_pos_prev = None    # previous position command
        self._fwd_pos_vel = None     # current velocity state (rad/s)
        self._fwd_pos_time = None    # timestamp of last tick
        # 250 Hz timer to continuously publish raw and feed controllers
        self._cmd_pub_timer = self.robot.node.create_timer(
            0.004, self._cmd_pub_timer_cb, callback_group=ReentrantCallbackGroup()
        )

        # Publisher for gripper servo control
        self._gripper_pub = self.robot.node.create_publisher(UInt8, "/gripper/target_position", 10)
        # Timer at 5 Hz for gripper level control
        self._gripper_timer = self.robot.node.create_timer(
            0.2, self._gripper_timer_cb, callback_group=ReentrantCallbackGroup()
        )

        # Auto optimization manager (recording + replay, optimization later)
        self._opt_mgr = OptimizationManager(
            robot=self.robot,
            node=self.robot.node,
            set_cmd_target_fn=self._set_cmd_target,
            owner=self,
        )

        # UR native force-mode streamer (joystick-style direct wrench control).
        # We pass hooks so that on disable the controller state is reset
        # cleanly around the resend of the External Control program. Without
        # this, the scaled_joint_trajectory_controller retains its last
        # latched target from before force_mode was enabled and drives the
        # robot back to that pose as soon as URCap reconnects.
        #
        # The JTC-reset strategy:
        #   BEFORE resend: deactivate the active joint trajectory controller
        #                  (so there is no command source when URCap comes
        #                  back), and snap _cmd_target_raw to current joints
        #                  so CRISP doesn't immediately re-publish a stale
        #                  target either.
        #   AFTER  resend: reactivate the joint trajectory controller.
        #                  ros2_control reads the current hardware position
        #                  on reactivation, so the controller's hold-point
        #                  becomes the CURRENT pose instead of the stale one.
        self._jtc_deactivated_by_ur_force: list = []

        def _sync_before_resend():
            # 1. Snap CRISP/raw target to current actual joints.
            try:
                joints = self.robot.joint_values
                if joints is not None:
                    self._cmd_target_raw = [float(j) for j in joints]
                    self._fwd_pos_prev = np.array(joints, dtype=float)
                    self._fwd_pos_vel = np.zeros(6)
                    self._fwd_pos_time = time.monotonic()
            except Exception:
                pass
            # 2. Deactivate the joint trajectory controller that holds a
            #    stale target. Remember which ones we deactivated so we
            #    reactivate exactly those afterwards.
            self._jtc_deactivated_by_ur_force = []
            try:
                controllers = self.robot.controller_switcher_client.get_controller_list()
                # Match any active *trajectory_controller* on the joint
                # command interface (covers both scaled_joint_trajectory_controller
                # and joint_trajectory_controller).
                to_deactivate = [
                    c.name for c in controllers
                    if c.state == "active" and "trajectory_controller" in c.name
                ]
                if to_deactivate:
                    self.robot.controller_switcher_client._switch_controller(
                        to_deactivate, []
                    )
                    self._jtc_deactivated_by_ur_force = to_deactivate
            except Exception as e:
                print(f"[ur_force] failed to deactivate JTC: {e}")

        def _restore_after_resend():
            # Reactivate whatever we deactivated. ros2_control will read the
            # current hardware position on reactivation, so the controller's
            # hold-point becomes the current pose (no snap-back).
            if not self._jtc_deactivated_by_ur_force:
                return
            try:
                self.robot.controller_switcher_client._switch_controller(
                    [], list(self._jtc_deactivated_by_ur_force)
                )
            except Exception as e:
                print(f"[ur_force] failed to reactivate JTC: {e}")
            finally:
                self._jtc_deactivated_by_ur_force = []

        self._ur_force = URForceModeStreamer(
            self.robot.node,
            robot_ip="192.168.1.15",
            on_before_resend=_sync_before_resend,
            on_after_resend=_restore_after_resend,
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
        pos = np.zeros(len(names))
        for jname, v, e, p in zip(msg.name, msg.velocity, msg.effort, msg.position):
            if jname in names:
                idx = names.index(jname)
                vel[idx] = v
                eff[idx] = e
                pos[idx] = p
        self._joint_velocity = vel
        self._joint_effort = eff

        # Accumulate plot samples at ~100 Hz (every 5th callback from 500 Hz)
        self._plot_downsample += 1
        if self._plot_downsample >= 5:
            self._plot_downsample = 0
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            sample = {
                "t": round(stamp, 4),
                "pos": [round(float(v), 5) for v in pos],
                "eff": [round(float(v), 3) for v in eff],
                "cmd": [round(float(v), 4) for v in self._commanded_torques] if self._commanded_torques is not None else [],
                "tgt_raw": [round(float(v), 5) for v in self._cmd_target_raw] if self._cmd_target_raw is not None else [],
            }
            with self._plot_lock:
                self._plot_buffer.append(sample)

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

    def _target_joint_cb(self, msg: JointState):
        """Cache target joint positions."""
        names = self.robot.config.joint_names
        targets = np.zeros(len(names))
        for jname, pos in zip(msg.name, msg.position):
            if jname in names:
                targets[names.index(jname)] = pos
        self._target_joints = targets

    def _alicia_cb(self, msg):
        """Cache Alicia leader arm joints and gripper, optionally forward as target."""
        self._alicia_joints = np.array([
            msg.joint1, msg.joint2, msg.joint3,
            msg.joint4, msg.joint5, msg.joint6,
        ], dtype=np.float64)
        self._gripper_raw = float(msg.gripper)
        # Suppress teleop writes while auto-opt replay or optimization is running
        if self._alicia_teleop and self._ready and not self._opt_mgr.is_replaying and not self._opt_mgr.is_optimizing:
            target = self._alicia_joints * self._alicia_scale + self._alicia_offset
            self._set_cmd_target(target)

    def _gripper_timer_cb(self):
        """Send gripper command at 5 Hz when teleop is active, with 25-level hysteresis."""
        if not self._alicia_teleop or self._gripper_raw is None:
            return
        raw = self._gripper_raw
        # Leader: 1000 = open, 1 = close. Invert so 0 = open, 999 = close.
        raw = 1000.0 - raw
        # 25 levels: chunk size = 40, chunk i covers [i*40, (i+1)*40)
        chunk = 1000.0 / 25.0  # 40.0
        hysteresis = chunk * 0.2  # 8.0
        naive_level = max(0, min(24, int(raw / chunk)))
        if self._gripper_level is not None:
            boundary = self._gripper_level * chunk
            if naive_level > self._gripper_level:
                upper = (self._gripper_level + 1) * chunk
                if raw < upper + hysteresis:
                    return
            elif naive_level < self._gripper_level:
                if raw > boundary - hysteresis:
                    return
            else:
                return  # same level
        self._gripper_level = naive_level
        pos = int(naive_level * 255.0 / 24.0)
        pos = max(0, min(255, pos))
        gripper_msg = UInt8()
        gripper_msg.data = pos
        self._gripper_pub.publish(gripper_msg)

    def _cmd_target_raw_cb(self, msg: JointState):
        """Receive raw target joints from /cmd_target_joint."""
        names = self.robot.config.joint_names
        targets = [0.0] * len(names)
        for jname, pos in zip(msg.name, msg.position):
            if jname in names:
                targets[names.index(jname)] = pos
        self._cmd_target_raw = targets

    def _cmd_pub_timer_cb(self):
        """Continuously publish raw target and feed controllers at 250 Hz."""
        if not rclpy.ok():
            return
        stamp = self.robot.node.get_clock().now().to_msg()
        raw = self._cmd_target_raw
        if raw is None:
            return
        raw_msg = JointState()
        raw_msg.header.stamp = stamp
        raw_msg.name = self.robot.config.joint_names
        raw_msg.position = [float(j) for j in raw]
        self._cmd_target_pub.publish(raw_msg)
        # Compute and publish /cmd_target_pose via FK
        try:
            pos, quat = fk_quaternion(list(raw))
            pose_msg = PoseStamped()
            pose_msg.header.stamp = stamp
            pose_msg.header.frame_id = "base_link"
            pose_msg.pose.position.x = float(pos[0])
            pose_msg.pose.position.y = float(pos[1])
            pose_msg.pose.position.z = float(pos[2])
            pose_msg.pose.orientation.x = float(quat[0])
            pose_msg.pose.orientation.y = float(quat[1])
            pose_msg.pose.orientation.z = float(quat[2])
            pose_msg.pose.orientation.w = float(quat[3])
            self._cmd_target_pose_pub.publish(pose_msg)
            rpy = quaternion_to_rpy(quat)
            self._cmd_target_pose = {
                "x": round(float(pos[0]), 5), "y": round(float(pos[1]), 5), "z": round(float(pos[2]), 5),
                "qx": round(float(quat[0]), 5), "qy": round(float(quat[1]), 5),
                "qz": round(float(quat[2]), 5), "qw": round(float(quat[3]), 5),
                "roll": round(float(np.degrees(rpy[0])), 2),
                "pitch": round(float(np.degrees(rpy[1])), 2),
                "yaw": round(float(np.degrees(rpy[2])), 2),
            }
            # Feed FK pose to cartesian impedance controller
            from crisp_py.utils.geometry import Pose
            from scipy.spatial.transform import Rotation
            target_pose = Pose(position=pos, orientation=Rotation.from_quat(quat))
            self.robot.set_target(pose=target_pose)
        except Exception:
            pass
        # Feed to joint impedance controller
        self.robot.set_target_joint(np.array(raw))
        # Feed to forward position controller with trapezoidal velocity profile
        now = time.monotonic()
        if self._fwd_pos_prev is None:
            joints = self.robot.joint_values
            self._fwd_pos_prev = np.array(joints if joints is not None else raw, dtype=float)
            self._fwd_pos_vel = np.zeros(6)
            self._fwd_pos_time = now
        dt = now - self._fwd_pos_time
        dt = max(dt, 1e-6)  # avoid division by zero
        self._fwd_pos_time = now
        target = np.array(raw, dtype=float)
        pos_err = target - self._fwd_pos_prev
        # Compute braking velocity using deceleration limit
        # v_brake = sqrt(2 * decel * |distance|)
        dist = np.abs(pos_err)
        v_brake = np.sqrt(2.0 * self._fwd_pos_max_decel * dist)
        # Desired velocity: min of v_max and v_brake, with correct sign
        v_limit = np.minimum(self._fwd_pos_max_vel, v_brake)
        v_desired = np.sign(pos_err) * v_limit
        # For very small errors, set desired to zero to avoid jitter
        v_desired = np.where(dist < 1e-6, 0.0, v_desired)
        # Limit velocity change: use accel when speeding up, decel when slowing down
        dv = v_desired - self._fwd_pos_vel
        # Per-joint: speeding up if |v_desired| > |v_prev|, else slowing down
        speeding_up = np.abs(v_desired) > np.abs(self._fwd_pos_vel)
        dv_max = np.where(speeding_up, self._fwd_pos_max_accel * dt, self._fwd_pos_max_decel * dt)
        dv = np.clip(dv, -dv_max, dv_max)
        self._fwd_pos_vel = self._fwd_pos_vel + dv
        # Clamp velocity by v_max
        self._fwd_pos_vel = np.clip(self._fwd_pos_vel, -self._fwd_pos_max_vel, self._fwd_pos_max_vel)
        # Apply position step
        pos_cmd = self._fwd_pos_prev + self._fwd_pos_vel * dt
        # Don't overshoot target
        for i in range(6):
            if self._fwd_pos_vel[i] > 0 and pos_cmd[i] > target[i]:
                pos_cmd[i] = target[i]
                self._fwd_pos_vel[i] = 0.0
            elif self._fwd_pos_vel[i] < 0 and pos_cmd[i] < target[i]:
                pos_cmd[i] = target[i]
                self._fwd_pos_vel[i] = 0.0
        self._fwd_pos_prev = pos_cmd.copy()
        fwd_msg = Float64MultiArray()
        fwd_msg.data = pos_cmd.tolist()
        self._fwd_pos_pub.publish(fwd_msg)

    def _set_cmd_target(self, joints):
        """Set raw joint target."""
        self._cmd_target_raw = [float(j) for j in joints]

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

        @app.route("/api/set_target_pose_rpy", methods=["POST"])
        def set_target_pose_rpy():
            """Set target pose from position + RPY (degrees). Body: {"x","y","z","roll","pitch","yaw"}"""
            if not self._ready:
                return jsonify({"error": "Robot not ready"}), 503
            data = request.get_json()
            try:
                from crisp_py.utils.geometry import Pose
                from scipy.spatial.transform import Rotation

                pos = np.array([float(data["x"]), float(data["y"]), float(data["z"])])
                rpy_deg = [float(data["roll"]), float(data["pitch"]), float(data["yaw"])]
                ori = Rotation.from_euler('xyz', np.radians(rpy_deg))
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
                self._set_cmd_target(joints)
                return jsonify({"success": True})
            except Exception as e:
                return jsonify({"error": str(e)}), 500

        @app.route("/api/set_position_joint", methods=["POST"])
        def set_position_joint():
            """Alias for set_target_joint for backward compatibility."""
            return set_target_joint()

        @app.route("/api/move_joint_delta", methods=["POST"])
        def move_joint_delta():
            """Move a single joint by a delta. Body: {"index": 0, "delta": 0.05}"""
            if not self._ready:
                return jsonify({"error": "Robot not ready"}), 503
            data = request.get_json()
            try:
                idx = int(data["index"])
                delta = float(data["delta"])
                current = list(self._cmd_target_raw) if self._cmd_target_raw is not None else list(self.robot.joint_values)
                current[idx] += delta
                self._set_cmd_target(current)
                return jsonify({"success": True, "joint": self.robot.config.joint_names[idx], "new_value": round(float(current[idx]), 4)})
            except Exception as e:
                return jsonify({"error": str(e)}), 500

        @app.route("/api/alicia_teleop", methods=["POST"])
        def alicia_teleop():
            """Toggle Alicia teleop mode. Body: {"active": true/false, "scale": [...], "offset": [...]}"""
            if not self._ready:
                return jsonify({"error": "Robot not ready"}), 503
            data = request.get_json()
            self._alicia_teleop = bool(data.get("active", False))
            if "scale" in data:
                self._alicia_scale = np.array(data["scale"], dtype=float)
            if "offset" in data:
                self._alicia_offset = np.array(data["offset"], dtype=float)
            return jsonify({"success": True, "active": self._alicia_teleop})

        @app.route("/api/save_config", methods=["POST"])
        def save_config():
            """Save current runtime parameters back into config/ur15_controllers.yaml,
            preserving the file structure and only updating values."""
            if not self._ready:
                return jsonify({"error": "Robot not ready"}), 503
            config_path = _find_workspace_config()
            if not config_path or not os.path.isfile(config_path):
                return jsonify({"error": "Config file not found. Launch once first to create it from template."}), 500
            try:
                with open(config_path, "r") as f:
                    config = yaml.safe_load(f) or {}

                changed = 0
                for ctrl_name in _CRISP_CONTROLLERS:
                    if ctrl_name not in config:
                        continue
                    ros_params = config[ctrl_name].get("ros__parameters", {})
                    if not ros_params:
                        continue
                    try:
                        client = self._get_param_client(ctrl_name)
                        names = client.list_parameters()
                        values = client.get_parameters(names)
                        for n, v in zip(names, values):
                            if n == "use_sim_time":
                                continue
                            # Walk the nested dict to update the value in-place
                            keys = n.split(".")
                            d = ros_params
                            found = True
                            for k in keys[:-1]:
                                if isinstance(d, dict) and k in d:
                                    d = d[k]
                                else:
                                    found = False
                                    break
                            if found and isinstance(d, dict) and keys[-1] in d:
                                old = d[keys[-1]]
                                # Preserve type: convert numpy/etc to native Python
                                new_val = v
                                if isinstance(v, (np.floating, np.integer)):
                                    new_val = float(v) if isinstance(v, np.floating) else int(v)
                                if old != new_val:
                                    d[keys[-1]] = new_val
                                    changed += 1
                    except Exception:
                        continue

                with open(config_path, "w") as f:
                    yaml.dump(config, f, default_flow_style=False, sort_keys=False)

                return jsonify({"success": True, "path": config_path, "changed": changed})
            except Exception as e:
                return jsonify({"error": str(e)}), 500

        @app.route("/api/reset_config", methods=["POST"])
        def reset_config():
            """Delete user config to revert to template defaults on next launch."""
            config_path = _find_workspace_config()
            if not config_path:
                return jsonify({"error": "Cannot find workspace config/ directory"}), 500
            try:
                if os.path.isfile(config_path):
                    os.remove(config_path)
                    return jsonify({"success": True, "message": "Config deleted. Template will be used on next launch."})
                else:
                    return jsonify({"success": True, "message": "No user config exists."})
            except Exception as e:
                return jsonify({"error": str(e)}), 500

        @app.route("/api/config_status")
        def config_status():
            config_path = _find_workspace_config()
            exists = config_path is not None and os.path.isfile(config_path)
            return jsonify({"user_config_exists": exists, "path": config_path or ""})

        @app.route("/api/pos_control_params")
        def get_pos_control_params():
            """Get current position control parameters."""
            jn = self.robot.config.joint_names
            return jsonify({
                "fwd_pos_max_vel": {n: round(float(v), 4) for n, v in zip(jn, self._fwd_pos_max_vel)},
                "fwd_pos_max_accel": {n: round(float(v), 4) for n, v in zip(jn, self._fwd_pos_max_accel)},
                "fwd_pos_max_decel": {n: round(float(v), 4) for n, v in zip(jn, self._fwd_pos_max_decel)},
            })

        @app.route("/api/pos_control_params", methods=["POST"])
        def set_pos_control_params():
            """Update position control parameters at runtime. Body: {"fwd_pos_max_vel": {...}, ...}"""
            data = request.get_json()
            jn = self.robot.config.joint_names
            if "fwd_pos_max_vel" in data:
                for n, v in data["fwd_pos_max_vel"].items():
                    if n in jn:
                        self._fwd_pos_max_vel[jn.index(n)] = float(v)
            if "fwd_pos_max_accel" in data:
                for n, v in data["fwd_pos_max_accel"].items():
                    if n in jn:
                        self._fwd_pos_max_accel[jn.index(n)] = float(v)
            if "fwd_pos_max_decel" in data:
                for n, v in data["fwd_pos_max_decel"].items():
                    if n in jn:
                        self._fwd_pos_max_decel[jn.index(n)] = float(v)
            return jsonify({"success": True})

        @app.route("/api/pos_control_params/save", methods=["POST"])
        def save_pos_control_params():
            """Save current position control parameters to config/position_control.yaml."""
            try:
                config_path = get_config_path("position_control.yaml")
            except RuntimeError:
                return jsonify({"error": "Cannot find workspace config/ directory"}), 500
            jn = self.robot.config.joint_names
            cfg = {
                "fwd_pos_max_vel": {n: round(float(v), 4) for n, v in zip(jn, self._fwd_pos_max_vel)},
                "fwd_pos_max_accel": {n: round(float(v), 4) for n, v in zip(jn, self._fwd_pos_max_accel)},
                "fwd_pos_max_decel": {n: round(float(v), 4) for n, v in zip(jn, self._fwd_pos_max_decel)},
            }
            with open(config_path, "w") as f:
                yaml.dump(cfg, f, default_flow_style=False, sort_keys=False)
            return jsonify({"success": True, "path": config_path})

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

        @app.route("/api/opt/status")
        def opt_status():
            return jsonify(self._opt_mgr.status())

        @app.route("/api/opt/record", methods=["POST"])
        def opt_record():
            if not self._ready:
                return jsonify({"error": "Robot not ready"}), 503
            try:
                self._opt_mgr.start_recording()
                return jsonify({"success": True})
            except Exception as e:
                return jsonify({"error": str(e)}), 400

        @app.route("/api/opt/stop_record", methods=["POST"])
        def opt_stop_record():
            try:
                stopped = self._opt_mgr.stop_recording()
                return jsonify({"success": True, "stopped": stopped})
            except Exception as e:
                return jsonify({"error": str(e)}), 500

        @app.route("/api/opt/save", methods=["POST"])
        def opt_save():
            try:
                path = self._opt_mgr.save()
                return jsonify({"success": True, "path": path})
            except Exception as e:
                return jsonify({"error": str(e)}), 400

        @app.route("/api/opt/delete", methods=["POST"])
        def opt_delete():
            try:
                removed = self._opt_mgr.delete()
                return jsonify({"success": True, "removed": removed})
            except Exception as e:
                return jsonify({"error": str(e)}), 400

        @app.route("/api/opt/replay", methods=["POST"])
        def opt_replay():
            if not self._ready:
                return jsonify({"error": "Robot not ready"}), 503
            data = request.get_json() or {}
            try:
                alignment = float(data.get("alignment_time_sec", 2.0))
                settle = float(data.get("settle_time_sec", 0.5))
                self._opt_mgr.start_replay(alignment, settle)
                return jsonify({"success": True})
            except Exception as e:
                return jsonify({"error": str(e)}), 400

        @app.route("/api/opt/stop_replay", methods=["POST"])
        def opt_stop_replay():
            try:
                self._opt_mgr.abort_replay()
                return jsonify({"success": True})
            except Exception as e:
                return jsonify({"error": str(e)}), 500

        @app.route("/api/opt/start", methods=["POST"])
        def opt_start():
            if not self._ready:
                return jsonify({"error": "Robot not ready"}), 503
            data = request.get_json() or {}
            try:
                resume_exp = data.pop("resume_exp", None)
                self._opt_mgr.start_optimization(data, self._get_param_client, resume_exp=resume_exp)
                return jsonify({"success": True})
            except Exception as e:
                return jsonify({"error": str(e)}), 400

        @app.route("/api/opt/stop", methods=["POST"])
        def opt_stop():
            try:
                self._opt_mgr.stop_optimization()
                return jsonify({"success": True})
            except Exception as e:
                return jsonify({"error": str(e)}), 500

        @app.route("/api/opt/history")
        def opt_history():
            return jsonify({"history": self._opt_mgr.opt_history()})

        @app.route("/api/opt/apply_best", methods=["POST"])
        def opt_apply_best():
            if not self._ready:
                return jsonify({"error": "Robot not ready"}), 503
            try:
                best = self._opt_mgr.apply_best(self._get_param_client)
                return jsonify({"success": True, "params": best})
            except Exception as e:
                return jsonify({"error": str(e)}), 400

        @app.route("/api/opt/report")
        def opt_report():
            """Serve report for the latest experiment, or by name query param."""
            name = request.args.get("name", "")
            if name:
                try:
                    path = self._opt_mgr.get_experiment_report_path(name)
                    return send_from_directory(os.path.dirname(path), "report.html")
                except FileNotFoundError as e:
                    return jsonify({"error": str(e)}), 404
            # Fallback: latest experiment
            exps = self._opt_mgr.list_experiments()
            for exp in exps:
                if exp.get("has_report"):
                    return send_from_directory(exp["path"], "report.html")
            return jsonify({"error": "No report available."}), 404

        @app.route("/api/opt/experiments")
        def opt_experiments():
            return jsonify({"experiments": self._opt_mgr.list_experiments()})

        @app.route("/api/opt/experiments/<name>", methods=["DELETE"])
        def opt_delete_experiment(name):
            try:
                self._opt_mgr.delete_experiment(name)
                return jsonify({"success": True})
            except (FileNotFoundError, ValueError) as e:
                return jsonify({"error": str(e)}), 400

        # ---------- UR Native Force Mode (joystick-style wrench control) ----------
        @app.route("/api/ur_force/status")
        def ur_force_status():
            return jsonify(self._ur_force.status())

        @app.route("/api/ur_force/enable", methods=["POST"])
        def ur_force_enable():
            if not self._ready:
                return jsonify({"error": "Robot not ready"}), 503
            result = self._ur_force.enable()
            if not result.get("success"):
                return jsonify(result), 500
            return jsonify(result)

        @app.route("/api/ur_force/disable", methods=["POST"])
        def ur_force_disable():
            result = self._ur_force.disable()
            return jsonify(result)

        @app.route("/api/ur_force/wrench", methods=["POST"])
        def ur_force_wrench():
            if not self._ur_force.enabled:
                return jsonify({"error": "Force mode not enabled"}), 409
            data = request.get_json() or {}
            try:
                clamped = self._ur_force.set_wrench(
                    float(data.get("fx", 0.0)),
                    float(data.get("fy", 0.0)),
                    float(data.get("fz", 0.0)),
                    float(data.get("tx", 0.0)),
                    float(data.get("ty", 0.0)),
                    float(data.get("tz", 0.0)),
                )
                return jsonify({"success": True, "wrench": clamped})
            except (TypeError, ValueError) as e:
                return jsonify({"error": f"invalid input: {e}"}), 400

        @app.route("/api/ur_force/params", methods=["POST"])
        def ur_force_set_params():
            """Update tunable force-mode parameters at runtime.

            JSON body accepts any subset of:
              damping, gain_scaling, speed_limit_m_s, speed_limit_rad_s,
              max_force_n, max_torque_nm
            """
            data = request.get_json() or {}
            try:
                def _f(k):
                    v = data.get(k)
                    return float(v) if v is not None else None
                updated = self._ur_force.set_params(
                    damping=_f("damping"),
                    gain_scaling=_f("gain_scaling"),
                    speed_limit_m_s=_f("speed_limit_m_s"),
                    speed_limit_rad_s=_f("speed_limit_rad_s"),
                    max_force_n=_f("max_force_n"),
                    max_torque_nm=_f("max_torque_nm"),
                    payload_kg=_f("payload_kg"),
                    payload_cog_z=_f("payload_cog_z"),
                )
                return jsonify({"success": True, "params": updated})
            except (TypeError, ValueError) as e:
                return jsonify({"error": f"invalid input: {e}"}), 400

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
                                "roll": round(float(np.degrees(pose.orientation.as_euler('xyz')[0])), 2),
                                "pitch": round(float(np.degrees(pose.orientation.as_euler('xyz')[1])), 2),
                                "yaw": round(float(np.degrees(pose.orientation.as_euler('xyz')[2])), 2),
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
                            "cmd_target_raw": [round(float(v), 4) for v in self._cmd_target_raw] if self._cmd_target_raw is not None else [],
                            "cmd_target_pose": self._cmd_target_pose,
                            "alicia_joints": [round(float(v), 4) for v in self._alicia_joints] if self._alicia_joints is not None else [],
                            "alicia_teleop": self._alicia_teleop,
                            "gripper_raw": round(float(self._gripper_raw), 1) if self._gripper_raw is not None else None,
                            "gripper_level": self._gripper_level,
                            "ur_force": self._ur_force.status(),
                        }
                        # Drain plot buffer and attach batch
                        with self._plot_lock:
                            if self._plot_buffer:
                                data["plot_batch"] = list(self._plot_buffer)
                                self._plot_buffer.clear()
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
            installed = os.path.join(get_package_share_directory("web_control"), "static")
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
