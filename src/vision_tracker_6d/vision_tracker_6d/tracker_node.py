"""Vision Tracker 6D — backend processing engine.

Subscribes to camera image topics (sensor_msgs/Image), detects chessboard
patterns, computes 6D poses, applies temporal filtering, and publishes
results on ROS topics.  Camera nodes (e.g. usb_cam) run separately and
publish images — this node is purely a consumer.

Shared state (latest frames, detection results, pose) is exposed via
thread-safe properties so that the web server can read them at any time.
"""

import json
import logging
import os
import shutil
import threading
import time
from typing import Dict, List, Optional

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from common.workspace import get_workspace_root
from .calibration_loader import load_calibration, CameraCalibration
from .pattern_detector import PatternDetector, DetectionResult
from .pose_filter import PoseFilter

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Config helpers
# ---------------------------------------------------------------------------

_TEMPLATE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "config", "vision_tracker_config_template.yaml",
)
# Installed location (colcon share)
try:
    from ament_index_python.packages import get_package_share_directory
    _TEMPLATE_INSTALLED = os.path.join(
        get_package_share_directory("vision_tracker_6d"),
        "config", "vision_tracker_config_template.yaml",
    )
except Exception:
    _TEMPLATE_INSTALLED = None


def _find_config() -> str:
    """Return path to config/vision_tracker.yaml, copying template if needed."""
    ws = get_workspace_root()
    cfg_path = os.path.join(ws, "config", "vision_tracker.yaml")
    if not os.path.isfile(cfg_path):
        # Copy template
        template = _TEMPLATE
        if not os.path.isfile(template) and _TEMPLATE_INSTALLED:
            template = _TEMPLATE_INSTALLED
        if os.path.isfile(template):
            os.makedirs(os.path.dirname(cfg_path), exist_ok=True)
            shutil.copy2(template, cfg_path)
            logger.info("Copied config template to %s", cfg_path)
        else:
            raise FileNotFoundError(
                f"Cannot find config template. Expected at {_TEMPLATE}"
            )
    return cfg_path


def _load_config(path: str) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


# ---------------------------------------------------------------------------
# Tracker engine
# ---------------------------------------------------------------------------

class VisionTracker:
    """Backend engine: subscribe to image topics, detect chessboard, publish pose."""

    def __init__(self):
        rclpy.init()
        self._node = Node("vision_tracker_6d")

        # Load config
        self._config_path = _find_config()
        self._cfg = _load_config(self._config_path)

        # --- Camera config (topics) ---
        cam_cfgs: List[dict] = self._cfg.get("cameras", [])
        self._camera_names: List[str] = []
        for cam_cfg in cam_cfgs:
            if not cam_cfg.get("enabled", True):
                continue
            self._camera_names.append(cam_cfg["name"])

        # --- Load intrinsic calibration ---
        ws = get_workspace_root()
        intrinsic_path = os.path.join(ws, "config", "intrinsic_calibration_result.json")
        self._intrinsic: Optional[CameraCalibration] = None
        if os.path.isfile(intrinsic_path):
            try:
                self._intrinsic = load_calibration(intrinsic_path)
                self._node.get_logger().info(
                    f"Loaded intrinsic calibration: fx={self._intrinsic.camera_matrix[0,0]:.1f} "
                    f"fy={self._intrinsic.camera_matrix[1,1]:.1f} "
                    f"rms={self._intrinsic.rms_error or 0:.4f}"
                )
            except Exception:
                self._node.get_logger().error(
                    f"Failed to load intrinsic calibration from {intrinsic_path}"
                )
        else:
            self._node.get_logger().warn(
                f"No intrinsic calibration found at {intrinsic_path} — "
                "run calibration first. Tracking will show video only."
            )

        # --- Load calibration pattern from JSON ---
        pattern_path = os.path.join(ws, "config", "calibration_pattern.json")
        if not os.path.isfile(pattern_path):
            tmpl = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                "..", "config", "calibration_pattern_template.json",
            )
            if not os.path.isfile(tmpl):
                try:
                    from ament_index_python.packages import get_package_share_directory
                    tmpl = os.path.join(
                        get_package_share_directory("vision_tracker_6d"),
                        "config", "calibration_pattern_template.json",
                    )
                except Exception:
                    pass
            if os.path.isfile(tmpl):
                os.makedirs(os.path.dirname(pattern_path), exist_ok=True)
                shutil.copy2(tmpl, pattern_path)
                self._node.get_logger().info(f"Copied pattern template to {pattern_path}")

        self._pattern_info: dict = {}
        if os.path.isfile(pattern_path):
            with open(pattern_path, "r") as f:
                self._pattern_info = json.load(f)
            self._node.get_logger().info(
                f"Loaded pattern: {self._pattern_info.get('pattern_id', '?')} "
                f"{self._pattern_info.get('parameters', {}).get('width', '?')}x"
                f"{self._pattern_info.get('parameters', {}).get('height', '?')}"
            )
        else:
            self._node.get_logger().warn(f"Pattern config not found: {pattern_path}")

        # --- Create detector (shared for all cameras — same intrinsics) ---
        self._detector: Optional[PatternDetector] = None
        if self._intrinsic is not None and self._pattern_info:
            try:
                self._detector = PatternDetector(
                    pattern_config=self._pattern_info,
                    camera_matrix=self._intrinsic.camera_matrix,
                    dist_coeffs=self._intrinsic.dist_coeffs,
                )
            except Exception:
                self._node.get_logger().error("Failed to create PatternDetector", exc_info=True)

        # --- Pose filter ---
        flt = self._cfg.get("filter", {})
        self._pose_filter = PoseFilter(
            alpha=flt.get("alpha", 0.5),
            enabled=flt.get("enabled", True),
        )

        # --- ROS publishers ---
        topics = self._cfg.get("topics", {})
        self._pose_pub = self._node.create_publisher(
            PoseStamped,
            topics.get("pose_output", "/vision_tracker/chessboard_pose"),
            10,
        )
        self._det_pub = self._node.create_publisher(
            Bool,
            topics.get("detection_status", "/vision_tracker/detection_active"),
            10,
        )

        # --- Shared state (thread-safe via lock) ---
        self._lock = threading.Lock()
        self._tracking_enabled = True
        self._latest_frames: Dict[str, np.ndarray] = {}
        self._latest_annotated: Dict[str, np.ndarray] = {}
        self._latest_results: Dict[str, DetectionResult] = {}
        self._latest_pose: Optional[dict] = None
        self._detection_active = False
        self._fps = 0.0
        self._frame_count = 0
        self._fps_t0 = time.monotonic()
        self._cameras_receiving: Dict[str, bool] = {
            n: False for n in self._camera_names
        }

        # --- Subscribe to image topics ---
        cb_group = ReentrantCallbackGroup()
        for cam_cfg in cam_cfgs:
            if not cam_cfg.get("enabled", True):
                continue
            name = cam_cfg["name"]
            topic = cam_cfg.get("image_topic", f"/{name}/image_raw")
            self._node.get_logger().info(
                f"Subscribing to image topic: {topic} (camera: {name})"
            )
            # Use a default-argument closure to capture `name` per iteration
            self._node.create_subscription(
                Image,
                topic,
                lambda msg, cam_name=name: self._image_cb(msg, cam_name),
                qos_profile_sensor_data,
                callback_group=cb_group,
            )

    # ------------------------------------------------------------------
    # ROS Image callback
    # ------------------------------------------------------------------

    def _image_cb(self, msg: Image, camera_name: str):
        """Called when a new image arrives on a camera topic."""
        # Convert sensor_msgs/Image to numpy BGR
        frame = self._imgmsg_to_cv2(msg)
        if frame is None:
            return

        with self._lock:
            self._cameras_receiving[camera_name] = True
            self._latest_frames[camera_name] = frame
            enabled = self._tracking_enabled

        if not enabled:
            annotated = frame.copy()
            cv2.putText(
                annotated, "Tracking paused", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (128, 128, 128), 2,
            )
            with self._lock:
                self._latest_annotated[camera_name] = annotated
            return

        # Detect pattern
        if self._detector is None:
            annotated = frame.copy()
            msg_text = "No calibration" if self._intrinsic is None else "No pattern"
            cv2.putText(
                annotated, msg_text, (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2,
            )
            with self._lock:
                self._latest_annotated[camera_name] = annotated
            return

        result = self._detector.detect(frame, draw=True)
        with self._lock:
            self._latest_results[camera_name] = result
            if result.image_with_corners is not None:
                self._latest_annotated[camera_name] = result.image_with_corners

        # Pose estimation
        if result.found and result.rvec is not None:
            T = PatternDetector.rvec_tvec_to_matrix(result.rvec, result.tvec)
            pos, quat = PatternDetector.matrix_to_quaternion(T)
            pos, quat = self._pose_filter.update(pos, quat)

            pose_dict = {
                "x": round(float(pos[0]), 5),
                "y": round(float(pos[1]), 5),
                "z": round(float(pos[2]), 5),
                "qx": round(float(quat[0]), 5),
                "qy": round(float(quat[1]), 5),
                "qz": round(float(quat[2]), 5),
                "qw": round(float(quat[3]), 5),
                "camera": camera_name,
                "reproj_error": round(result.reprojection_error, 4),
            }
            with self._lock:
                self._latest_pose = pose_dict
                self._detection_active = True

            self._publish_pose(pos, quat)
            self._publish_detection(True)
        else:
            with self._lock:
                self._detection_active = False
            self._publish_detection(False)

        # FPS tracking
        self._frame_count += 1
        now = time.monotonic()
        dt = now - self._fps_t0
        if dt >= 1.0:
            with self._lock:
                self._fps = self._frame_count / dt
            self._frame_count = 0
            self._fps_t0 = now

    # ------------------------------------------------------------------
    # Image conversion
    # ------------------------------------------------------------------

    @staticmethod
    def _imgmsg_to_cv2(msg: Image) -> Optional[np.ndarray]:
        """Convert sensor_msgs/Image to a BGR numpy array (no cv_bridge)."""
        encoding = msg.encoding
        h, w = msg.height, msg.width
        data = np.frombuffer(msg.data, dtype=np.uint8)

        if encoding in ("bgr8", "8UC3"):
            return data.reshape((h, w, 3))
        elif encoding == "rgb8":
            return cv2.cvtColor(data.reshape((h, w, 3)), cv2.COLOR_RGB2BGR)
        elif encoding in ("mono8", "8UC1"):
            gray = data.reshape((h, w))
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        elif encoding == "bgra8":
            return cv2.cvtColor(data.reshape((h, w, 4)), cv2.COLOR_BGRA2BGR)
        elif encoding == "rgba8":
            return cv2.cvtColor(data.reshape((h, w, 4)), cv2.COLOR_RGBA2BGR)
        elif encoding in ("yuyv", "yuv422_yuy2"):
            yuv = data.reshape((h, w, 2))
            return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_YUYV)
        else:
            logger.warning("Unsupported image encoding: %s", encoding)
            return None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self):
        """Nothing to open — cameras are external ROS nodes."""
        self._node.get_logger().info(
            "VisionTracker started — waiting for image topics…"
        )

    def spin(self):
        """Block on ROS spin (call from main thread)."""
        rclpy.spin(self._node)

    def shutdown(self):
        self._node.destroy_node()
        rclpy.shutdown()

    # ------------------------------------------------------------------
    # Thread-safe accessors (for web server)
    # ------------------------------------------------------------------

    @property
    def node(self) -> Node:
        return self._node

    @property
    def config(self) -> dict:
        return self._cfg

    @property
    def config_path(self) -> str:
        return self._config_path

    def get_status(self) -> dict:
        """Return a snapshot of current tracker state."""
        with self._lock:
            return {
                "tracking_enabled": self._tracking_enabled,
                "detection_active": self._detection_active,
                "fps": round(self._fps, 1),
                "cameras": {
                    name: {
                        "receiving": self._cameras_receiving.get(name, False),
                    }
                    for name in self._camera_names
                },
                "has_intrinsic": self._intrinsic is not None,
                "has_detector": self._detector is not None,
                "pose": self._latest_pose,
                "pattern": self._pattern_info,
            }

    def get_latest_annotated_frame(self, camera_name: str) -> Optional[np.ndarray]:
        """Return the latest annotated frame for MJPEG streaming."""
        with self._lock:
            return self._latest_annotated.get(camera_name)

    def get_latest_raw_frame(self, camera_name: str) -> Optional[np.ndarray]:
        """Return the latest raw (unannotated) frame for calibration capture."""
        with self._lock:
            frame = self._latest_frames.get(camera_name)
            return frame.copy() if frame is not None else None

    def get_camera_names(self) -> List[str]:
        return list(self._camera_names)

    def set_tracking_enabled(self, enabled: bool):
        with self._lock:
            self._tracking_enabled = enabled
            if not enabled:
                self._detection_active = False
                self._pose_filter.reset()

    def is_tracking_enabled(self) -> bool:
        with self._lock:
            return self._tracking_enabled

    # ------------------------------------------------------------------
    # ROS publishing
    # ------------------------------------------------------------------

    def _publish_pose(self, position: np.ndarray, quaternion: np.ndarray):
        msg = PoseStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = float(position[2])
        msg.pose.orientation.x = float(quaternion[0])
        msg.pose.orientation.y = float(quaternion[1])
        msg.pose.orientation.z = float(quaternion[2])
        msg.pose.orientation.w = float(quaternion[3])
        self._pose_pub.publish(msg)

    def _publish_detection(self, active: bool):
        msg = Bool()
        msg.data = active
        self._det_pub.publish(msg)


def main():
    tracker = VisionTracker()
    try:
        tracker.start()
        tracker.spin()
    except KeyboardInterrupt:
        pass
    finally:
        tracker.shutdown()


if __name__ == "__main__":
    main()
