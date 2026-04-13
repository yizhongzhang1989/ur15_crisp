"""VLA (Visual Language Action) joint control server for UR15.

Provides:
- ROS 2 node that subscribes to camera image and joint states
- VLA inference client (HTTP to external model server)
- Web dashboard at :8090 for instruction editing, visualization, and control
- Publishes predicted actions to /cmd_target_joint pipeline
"""

import io
import json
import os
import threading
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data
from flask import Flask, Response, jsonify, request, send_from_directory
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge

try:
    from alicia_duo_leader_driver.msg import ArmJointState
    _HAS_ALICIA_MSG = True
except ImportError:
    _HAS_ALICIA_MSG = False

# ---------------------------------------------------------------------------
# VLA HTTP client (from scripts/ur15_client_dummy.py)
# ---------------------------------------------------------------------------
try:
    import requests
    from PIL import Image as PILImage
    _HAS_REQUESTS = True
except ImportError:
    _HAS_REQUESTS = False


def vla_predict(url, images, state, instruction):
    """Call external VLA model server and return predicted action chunk."""
    files = []
    for i, img_np in enumerate(images):
        pil = PILImage.fromarray(img_np)
        buf = io.BytesIO()
        pil.save(buf, format="JPEG")
        buf.seek(0)
        files.append((f"image_{i}", (f"image_{i}.jpg", buf, "image/jpeg")))

    state_ser = {k: np.asarray(v).tolist() for k, v in state.items()}
    json_payload = {
        "task_description": instruction,
        "state": state_ser,
        "use_state": True,
        "has_left": False,
        "has_right": True,
        "has_progress": False,
    }
    json_buf = io.BytesIO(json.dumps(json_payload).encode("utf-8"))
    files.append(("json", ("json", json_buf, "application/json")))

    resp = requests.post(url, files=files, timeout=30)
    resp.raise_for_status()
    result = resp.json()
    action = {}
    for key, value in result.items():
        action[key] = np.array(value, dtype=np.float32)
    return action


# ---------------------------------------------------------------------------
# Main server
# ---------------------------------------------------------------------------
class VLAControlServer:
    JOINT_NAMES = [
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
    ]

    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("joint_vla_control")
        self.bridge = CvBridge()
        self._lock = threading.Lock()

        # --- State ---
        self._latest_image = None       # (H, W, 3) uint8 BGR
        self._latest_joints = None      # np.ndarray (6,)
        self._joint_names = self.JOINT_NAMES

        # --- VLA config ---
        self._vla_url = "http://10.190.172.212:4500/api/inference"
        self._instruction = "pick up the blue whiteboard eraser, erase all the drawings on the whiteboard, and then put the eraser back in its place"
        self._chunk_size = 30           # frames per prediction
        self._fps = 30                  # predicted frames per second

        # --- Prediction state ---
        self._predicted_joints = None   # (chunk_size, 6) or None
        self._predicted_gripper = None  # (chunk_size, 1) or None
        self._pred_step = 0             # current step index in chunk
        self._pred_timestamp = None     # when prediction was made
        self._predicting = False        # inference in progress

        # --- Execution state ---
        self._exec_mode = "stopped"     # "stopped", "step", "continuous"
        self._exec_step_requested = False

        # --- Plot buffer ---
        self._plot_buffer = deque(maxlen=3000)
        self._plot_lock = threading.Lock()
        self._plot_downsample = 0

        # --- Publishers ---
        if _HAS_ALICIA_MSG:
            self._arm_joint_pub = self.node.create_publisher(
                ArmJointState, "/arm_joint_state", 10
            )
        else:
            self._arm_joint_pub = None

        # --- Subscribers ---
        self.node.create_subscription(
            Image, "/ur15_camera/image_raw", self._image_cb,
            qos_profile_sensor_data, callback_group=ReentrantCallbackGroup(),
        )
        self.node.create_subscription(
            JointState, "joint_states", self._joint_state_cb,
            qos_profile_sensor_data, callback_group=ReentrantCallbackGroup(),
        )

        # --- Execution timer at 30 Hz ---
        self._exec_timer = self.node.create_timer(
            1.0 / self._fps, self._exec_timer_cb,
            callback_group=ReentrantCallbackGroup(),
        )

        # --- ROS spin in background ---
        self._spin_thread = threading.Thread(
            target=lambda: rclpy.spin(self.node), daemon=True
        )
        self._spin_thread.start()

        # --- Flask app ---
        self.app = self._create_app()

    # -----------------------------------------------------------------------
    # ROS callbacks
    # -----------------------------------------------------------------------
    def _image_cb(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self._latest_image = cv_img
        except Exception:
            pass

    def _joint_state_cb(self, msg):
        joints = np.zeros(6)
        for name, pos in zip(msg.name, msg.position):
            if name in self._joint_names:
                joints[self._joint_names.index(name)] = pos
        self._latest_joints = joints

        # Plot sample at ~30 Hz (every 16th callback from ~500 Hz)
        self._plot_downsample += 1
        if self._plot_downsample >= 16:
            self._plot_downsample = 0
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            sample = {
                "t": round(stamp, 4),
                "pos": [round(float(v), 5) for v in joints],
            }
            # Add predicted trajectory if available
            pred = self._predicted_joints
            grip = self._predicted_gripper
            step = self._pred_step
            if pred is not None and step < len(pred):
                sample["pred"] = [round(float(v), 5) for v in pred[step]]
                sample["pred_step"] = step
            if grip is not None and step < len(grip):
                sample["pred_grip"] = round(float(grip[step][0]), 4)
            with self._plot_lock:
                self._plot_buffer.append(sample)

    # -----------------------------------------------------------------------
    # Execution timer — sends one action frame per tick (30 Hz)
    # -----------------------------------------------------------------------
    def _exec_timer_cb(self):
        pred = self._predicted_joints
        if pred is None:
            return
        step = self._pred_step
        if step >= len(pred):
            if self._exec_mode == "continuous":
                self._exec_mode = "stopped"
            return

        should_send = False
        if self._exec_mode == "continuous":
            should_send = True
        elif self._exec_mode == "step" and self._exec_step_requested:
            should_send = True
            self._exec_step_requested = False

        if should_send:
            target = pred[step]
            grip = self._predicted_gripper
            if self._arm_joint_pub is not None:
                msg = ArmJointState()
                msg.header.stamp = self.node.get_clock().now().to_msg()
                msg.joint1 = float(target[0])
                msg.joint2 = float(target[1])
                msg.joint3 = float(target[2])
                msg.joint4 = float(target[3])
                msg.joint5 = float(target[4])
                msg.joint6 = float(target[5])
                msg.gripper = float(grip[step][0]) if grip is not None and step < len(grip) else 0.0
                self._arm_joint_pub.publish(msg)
            self._pred_step = step + 1

    # -----------------------------------------------------------------------
    # VLA inference (runs in background thread)
    # -----------------------------------------------------------------------
    def _run_inference(self):
        if not _HAS_REQUESTS:
            return {"error": "requests/Pillow not installed"}
        img = self._latest_image
        joints = self._latest_joints
        if img is None:
            return {"error": "No camera image available"}
        if joints is None:
            return {"error": "No joint state available"}

        # Convert BGR → RGB
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        state = {
            "ROBOT_RIGHT_JOINTS": joints.astype(np.float32),
        }
        try:
            self._predicting = True
            action = vla_predict(self._vla_url, [img_rgb], state, self._instruction)
            self._predicted_joints = action.get("ROBOT_RIGHT_JOINTS", None)
            self._predicted_gripper = action.get("ROBOT_RIGHT_GRIPPER", None)
            self._pred_step = 0
            self._pred_timestamp = time.time()
            self._predicting = False
            return {"success": True, "chunk_size": len(self._predicted_joints) if self._predicted_joints is not None else 0}
        except Exception as e:
            self._predicting = False
            return {"error": str(e)}

    # -----------------------------------------------------------------------
    # Flask app
    # -----------------------------------------------------------------------
    def _create_app(self):
        static_dir = self._find_static_dir()
        app = Flask(__name__, static_folder=static_dir)

        @app.route("/")
        def index():
            return send_from_directory(static_dir, "index.html")

        @app.route("/api/status")
        def status():
            pred = self._predicted_joints
            return jsonify({
                "has_image": self._latest_image is not None,
                "has_joints": self._latest_joints is not None,
                "joints": [round(float(v), 4) for v in self._latest_joints] if self._latest_joints is not None else [],
                "instruction": self._instruction,
                "vla_url": self._vla_url,
                "exec_mode": self._exec_mode,
                "predicting": self._predicting,
                "pred_available": pred is not None,
                "pred_chunk_size": len(pred) if pred is not None else 0,
                "pred_step": self._pred_step,
                "pred_timestamp": self._pred_timestamp,
            })

        @app.route("/api/config", methods=["POST"])
        def set_config():
            data = request.get_json()
            if "instruction" in data:
                self._instruction = data["instruction"]
            if "vla_url" in data:
                self._vla_url = data["vla_url"]
            return jsonify({"success": True})

        @app.route("/api/predict", methods=["POST"])
        def predict():
            if self._predicting:
                return jsonify({"error": "Inference already in progress"}), 409
            result = [None]
            def _run():
                result[0] = self._run_inference()
            t = threading.Thread(target=_run, daemon=True)
            t.start()
            t.join(timeout=35)
            if result[0] is None:
                return jsonify({"error": "Inference timeout"}), 504
            if "error" in result[0]:
                return jsonify(result[0]), 500
            return jsonify(result[0])

        @app.route("/api/exec", methods=["POST"])
        def exec_control():
            """Control execution: {"mode": "stopped"|"step"|"continuous"}"""
            data = request.get_json()
            mode = data.get("mode", "stopped")
            if mode not in ("stopped", "step", "continuous"):
                return jsonify({"error": "Invalid mode"}), 400
            self._exec_mode = mode
            if mode == "step":
                self._exec_step_requested = True
            return jsonify({"success": True, "mode": mode})

        @app.route("/api/step", methods=["POST"])
        def step_once():
            """Send one step in step mode."""
            self._exec_mode = "step"
            self._exec_step_requested = True
            return jsonify({"success": True, "step": self._pred_step})

        @app.route("/api/reset_step", methods=["POST"])
        def reset_step():
            """Reset step counter to replay from beginning."""
            self._pred_step = 0
            return jsonify({"success": True})

        @app.route("/api/image")
        def get_image():
            """Return latest camera image as JPEG."""
            img = self._latest_image
            if img is None:
                return Response(status=204)
            _, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
            return Response(buf.tobytes(), mimetype="image/jpeg")

        @app.route("/api/prediction")
        def get_prediction():
            """Return full predicted trajectory."""
            pred = self._predicted_joints
            grip = self._predicted_gripper
            if pred is None:
                return jsonify({"available": False})
            result = {
                "available": True,
                "joints": pred.tolist(),
                "step": self._pred_step,
            }
            if grip is not None:
                result["gripper"] = grip.tolist()
            return jsonify(result)

        @app.route("/api/stream")
        def stream():
            """SSE stream of status + plot data at ~10 Hz."""
            def generate():
                while True:
                    pred = self._predicted_joints
                    data = {
                        "joints": [round(float(v), 4) for v in self._latest_joints] if self._latest_joints is not None else [],
                        "exec_mode": self._exec_mode,
                        "pred_step": self._pred_step,
                        "pred_chunk_size": len(pred) if pred is not None else 0,
                        "predicting": self._predicting,
                    }
                    # Predicted trajectory for overlay
                    if pred is not None:
                        data["pred_current"] = [round(float(v), 4) for v in pred[min(self._pred_step, len(pred)-1)]]
                    # Drain plot buffer
                    with self._plot_lock:
                        if self._plot_buffer:
                            data["plot_batch"] = list(self._plot_buffer)
                            self._plot_buffer.clear()
                    yield f"data: {json.dumps(data)}\n\n"
                    time.sleep(0.1)
            return Response(generate(), mimetype="text/event-stream")

        return app

    def _find_static_dir(self):
        try:
            from ament_index_python.packages import get_package_share_directory
            installed = os.path.join(get_package_share_directory("joint_vla_control"), "static")
            if os.path.isdir(installed):
                return installed
        except Exception:
            pass
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(here, "static")

    def run(self, host="0.0.0.0", port=8091):
        print(f"\n  VLA Joint Control: http://{host}:{port}\n")
        self.app.run(host=host, port=port, threaded=True)

    def shutdown(self):
        self.node.destroy_node()
        rclpy.shutdown()


def main():
    server = VLAControlServer()
    try:
        server.run()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()


if __name__ == "__main__":
    main()
