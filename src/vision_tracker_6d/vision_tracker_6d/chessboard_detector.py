"""Chessboard detection and 6D pose estimation via OpenCV solvePnP.

Given a camera frame, detects the inner corners of a chessboard pattern and
computes the board's 6D pose (rotation + translation) relative to the camera,
using the camera's intrinsic calibration parameters.
"""

import logging
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class DetectionResult:
    """Result of a single chessboard detection attempt."""
    found: bool
    corners_2d: Optional[np.ndarray] = None   # (N, 1, 2) sub-pixel corners
    rvec: Optional[np.ndarray] = None          # (3, 1) Rodrigues rotation vector
    tvec: Optional[np.ndarray] = None          # (3, 1) translation vector
    reprojection_error: Optional[float] = None
    image_with_corners: Optional[np.ndarray] = None  # annotated frame (BGR)


class ChessboardDetector:
    """Detect a chessboard and estimate its 6D pose."""

    def __init__(
        self,
        rows: int,
        cols: int,
        square_size: float,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        use_subpix: bool = True,
        adaptive_threshold: bool = True,
        normalize_image: bool = True,
        fast_check: bool = True,
    ):
        self.rows = rows
        self.cols = cols
        self.square_size = square_size
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.use_subpix = use_subpix

        # Build findChessboardCorners flags
        self._flags = 0
        if adaptive_threshold:
            self._flags |= cv2.CALIB_CB_ADAPTIVE_THRESH
        if normalize_image:
            self._flags |= cv2.CALIB_CB_NORMALIZE_IMAGE
        if fast_check:
            self._flags |= cv2.CALIB_CB_FAST_CHECK

        # Sub-pixel refinement termination criteria
        self._subpix_criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.001,
        )

        # Build 3D object points for the board (Z = 0 plane)
        self._obj_points = np.zeros((rows * cols, 3), dtype=np.float32)
        self._obj_points[:, :2] = (
            np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_size
        )

    def detect(self, frame: np.ndarray, draw: bool = True) -> DetectionResult:
        """Detect chessboard corners and compute 6D pose.

        Args:
            frame: BGR image (numpy array).
            draw: If True, annotate a copy of the frame with detected corners
                  and the coordinate axes.

        Returns:
            DetectionResult with detection status and pose if found.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        found, corners = cv2.findChessboardCorners(
            gray, (self.cols, self.rows), self._flags
        )

        if not found or corners is None:
            annotated = frame.copy() if draw else None
            if draw:
                # Show "No detection" text
                cv2.putText(
                    annotated, "No chessboard detected", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2,
                )
            return DetectionResult(found=False, image_with_corners=annotated)

        # Sub-pixel refinement
        if self.use_subpix:
            corners = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), self._subpix_criteria
            )

        # Solve PnP
        success, rvec, tvec = cv2.solvePnP(
            self._obj_points, corners, self.camera_matrix, self.dist_coeffs
        )

        if not success:
            return DetectionResult(found=False)

        # Compute reprojection error
        reproj_pts, _ = cv2.projectPoints(
            self._obj_points, rvec, tvec, self.camera_matrix, self.dist_coeffs
        )
        error = float(np.mean(np.linalg.norm(
            corners.reshape(-1, 2) - reproj_pts.reshape(-1, 2), axis=1
        )))

        # Annotate
        annotated = None
        if draw:
            annotated = frame.copy()
            cv2.drawChessboardCorners(
                annotated, (self.cols, self.rows), corners, found
            )
            # Draw coordinate axes (length = 3 squares)
            axis_len = self.square_size * 3
            cv2.drawFrameAxes(
                annotated, self.camera_matrix, self.dist_coeffs,
                rvec, tvec, axis_len, 3,
            )
            # Show reprojection error
            cv2.putText(
                annotated, f"reproj err: {error:.3f}px", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2,
            )

        return DetectionResult(
            found=True,
            corners_2d=corners,
            rvec=rvec,
            tvec=tvec,
            reprojection_error=error,
            image_with_corners=annotated,
        )

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
        """Extract position (xyz) and quaternion (xyzw) from a 4x4 matrix.

        Returns:
            (position, quaternion) — both as 1-D numpy arrays.
            Quaternion is in (x, y, z, w) order (ROS convention).
        """
        R = T[:3, :3]
        pos = T[:3, 3]

        # Rotation matrix to quaternion (Shepperd's method)
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
