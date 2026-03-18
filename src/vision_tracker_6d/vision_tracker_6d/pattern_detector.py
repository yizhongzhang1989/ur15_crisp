"""Generic pattern detector for 6D pose estimation.

Uses the camera_calibration_toolkit's pattern system to detect calibration
patterns (standard chessboard, ChArUco, grid board) and computes the
board's 6D pose via solvePnP.
"""

import logging
import sys
import os
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np

logger = logging.getLogger(__name__)

# Ensure toolkit is importable
_TOOLKIT_DIR = None
def _ensure_toolkit():
    global _TOOLKIT_DIR
    if _TOOLKIT_DIR is not None:
        return
    candidates = [
        os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      "..", "ThirdParty", "camera_calibration_toolkit"),
    ]
    try:
        from common.workspace import get_workspace_root
        candidates.append(os.path.join(
            get_workspace_root(), "src", "vision_tracker_6d",
            "ThirdParty", "camera_calibration_toolkit",
        ))
    except Exception:
        pass
    for d in candidates:
        d = os.path.normpath(d)
        if os.path.isdir(d):
            if d not in sys.path:
                sys.path.insert(0, d)
            _TOOLKIT_DIR = d
            return


@dataclass
class DetectionResult:
    """Result of a single pattern detection + pose estimation."""
    found: bool
    corners_2d: Optional[np.ndarray] = None
    point_ids: Optional[np.ndarray] = None
    obj_points: Optional[np.ndarray] = None
    rvec: Optional[np.ndarray] = None
    tvec: Optional[np.ndarray] = None
    reprojection_error: Optional[float] = None
    image_with_corners: Optional[np.ndarray] = None


class PatternDetector:
    """Detect any supported calibration pattern and estimate 6D pose."""

    def __init__(
        self,
        pattern_config: dict,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
    ):
        _ensure_toolkit()
        from core.calibration_patterns import load_pattern_from_json

        self._pattern = load_pattern_from_json(pattern_config)
        self._camera_matrix = camera_matrix
        self._dist_coeffs = dist_coeffs
        self._pattern_id = pattern_config.get("pattern_id", "unknown")

        logger.info(
            "PatternDetector initialised: %s (K: fx=%.1f fy=%.1f)",
            self._pattern_id,
            camera_matrix[0, 0],
            camera_matrix[1, 1],
        )

    def detect(self, frame: np.ndarray, draw: bool = True) -> DetectionResult:
        """Detect pattern and compute 6D pose.

        Args:
            frame: BGR image.
            draw: If True, annotate a copy of the frame.

        Returns:
            DetectionResult with pose if found.
        """
        success, corners, point_ids = self._pattern.detect_corners(frame)

        if not success or corners is None or len(corners) < 4:
            annotated = None
            if draw:
                annotated = frame.copy()
                cv2.putText(
                    annotated, "No pattern detected", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2,
                )
            return DetectionResult(found=False, image_with_corners=annotated)

        # Generate matching 3D object points
        obj_points = self._pattern.generate_object_points(point_ids)

        # Ensure correct shapes for solvePnP
        img_pts = corners.reshape(-1, 1, 2).astype(np.float64)
        obj_pts = obj_points.reshape(-1, 1, 3).astype(np.float64)
        n_pts = img_pts.shape[0]

        # Choose solvePnP method based on point count
        if n_pts < 4:
            return DetectionResult(found=False, image_with_corners=frame.copy() if draw else None)
        elif n_pts < 6:
            method = cv2.SOLVEPNP_P3P  # works with exactly 4 points
            ok, rvec, tvec = cv2.solvePnP(
                obj_pts[:4], img_pts[:4], self._camera_matrix, self._dist_coeffs,
                flags=method,
            )
        else:
            ok, rvec, tvec = cv2.solvePnP(
                obj_pts, img_pts, self._camera_matrix, self._dist_coeffs,
            )

        if not ok:
            annotated = None
            if draw:
                annotated = self._pattern.draw_corners(frame.copy(), corners, point_ids)
                cv2.putText(
                    annotated, "solvePnP failed", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2,
                )
            return DetectionResult(found=False, image_with_corners=annotated)

        # Reprojection error
        reproj_pts, _ = cv2.projectPoints(
            obj_pts, rvec, tvec, self._camera_matrix, self._dist_coeffs
        )
        error = float(np.mean(np.linalg.norm(
            img_pts.reshape(-1, 2) - reproj_pts.reshape(-1, 2), axis=1
        )))

        # Annotate
        annotated = None
        if draw:
            annotated = self._pattern.draw_corners(frame.copy(), corners, point_ids)
            axis_len = float(np.max(obj_points)) * 0.5
            if axis_len < 0.001:
                axis_len = 0.03
            cv2.drawFrameAxes(
                annotated, self._camera_matrix, self._dist_coeffs,
                rvec, tvec, axis_len, 3,
            )
            cv2.putText(
                annotated, f"reproj: {error:.3f}px", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2,
            )

        return DetectionResult(
            found=True,
            corners_2d=corners,
            point_ids=point_ids,
            obj_points=obj_points,
            rvec=rvec,
            tvec=tvec,
            reprojection_error=error,
            image_with_corners=annotated,
        )

    # ------------------------------------------------------------------
    # Pose conversion utilities
    # ------------------------------------------------------------------

    @staticmethod
    def rvec_tvec_to_matrix(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
        """Convert rotation vector + translation to a 4x4 homogeneous matrix."""
        R, _ = cv2.Rodrigues(rvec)
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3, 3] = tvec.flatten()
        return T

    @staticmethod
    def matrix_to_quaternion(T: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Extract position and quaternion (xyzw) from a 4x4 matrix."""
        R = T[:3, :3]
        pos = T[:3, 3]

        trace = np.trace(R)
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s

        q = np.array([x, y, z, w])
        q /= np.linalg.norm(q)
        return pos, q
