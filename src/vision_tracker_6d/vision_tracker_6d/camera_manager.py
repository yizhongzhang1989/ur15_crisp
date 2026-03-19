"""Multi-camera frame acquisition manager.

Manages one or more OpenCV VideoCapture sources (USB cameras, RTSP streams,
or video files) and provides synchronised frame grabbing.
"""

import logging
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import cv2
import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class CameraInfo:
    """Configuration and runtime state for a single camera."""
    name: str
    source: object  # int (device index) or str (URL / file path)
    enabled: bool = True
    calibration_file: str = ""
    cap: Optional[cv2.VideoCapture] = field(default=None, repr=False)


class CameraManager:
    """Open, manage, and grab frames from multiple cameras."""

    def __init__(self, camera_configs: List[dict]):
        """Initialise cameras from a list of config dicts.

        Each dict should contain keys matching the YAML camera entry:
          name, source, calibration_file, enabled
        """
        self._cameras: Dict[str, CameraInfo] = {}
        for cfg in camera_configs:
            if not cfg.get("enabled", True):
                continue
            name = cfg["name"]
            source = cfg["source"]
            self._cameras[name] = CameraInfo(
                name=name,
                source=source,
                enabled=True,
                calibration_file=cfg.get("calibration_file", ""),
            )

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def open_all(self) -> Dict[str, bool]:
        """Open all enabled cameras. Returns {name: success}."""
        results = {}
        for name, cam in self._cameras.items():
            results[name] = self._open(cam)
        return results

    def close_all(self) -> None:
        """Release all camera captures."""
        for cam in self._cameras.values():
            if cam.cap is not None and cam.cap.isOpened():
                cam.cap.release()
                cam.cap = None

    # ------------------------------------------------------------------
    # Frame grabbing
    # ------------------------------------------------------------------

    def grab_frames(self) -> Dict[str, np.ndarray]:
        """Grab one frame from every open camera.

        Returns:
            Dictionary mapping camera name → BGR frame (numpy array).
            Cameras that fail to grab are omitted.
        """
        frames: Dict[str, np.ndarray] = {}
        for name, cam in self._cameras.items():
            if cam.cap is None or not cam.cap.isOpened():
                continue
            ret, frame = cam.cap.read()
            if ret and frame is not None:
                frames[name] = frame
            else:
                logger.warning("Failed to grab frame from %s", name)
        return frames

    def grab_frame(self, camera_name: str) -> Optional[np.ndarray]:
        """Grab a single frame from the named camera."""
        cam = self._cameras.get(camera_name)
        if cam is None or cam.cap is None or not cam.cap.isOpened():
            return None
        ret, frame = cam.cap.read()
        return frame if ret else None

    # ------------------------------------------------------------------
    # Queries
    # ------------------------------------------------------------------

    @property
    def camera_names(self) -> List[str]:
        return list(self._cameras.keys())

    def get_camera_info(self, name: str) -> Optional[CameraInfo]:
        return self._cameras.get(name)

    def is_open(self, name: str) -> bool:
        cam = self._cameras.get(name)
        return cam is not None and cam.cap is not None and cam.cap.isOpened()

    def status(self) -> Dict[str, bool]:
        """Return {name: is_open} for all cameras."""
        return {name: self.is_open(name) for name in self._cameras}

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _open(self, cam: CameraInfo) -> bool:
        """Open a single camera capture."""
        try:
            src = cam.source
            if isinstance(src, str) and src.isdigit():
                src = int(src)
            cap = cv2.VideoCapture(src)
            if cap.isOpened():
                cam.cap = cap
                logger.info("Opened camera %s (source=%s)", cam.name, cam.source)
                return True
            else:
                logger.error("Cannot open camera %s (source=%s)", cam.name, cam.source)
                cap.release()
                return False
        except Exception:
            logger.exception("Error opening camera %s", cam.name)
            return False
