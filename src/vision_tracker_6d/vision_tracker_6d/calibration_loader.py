"""Load camera calibration data from camera_calibration_toolkit JSON output.

Supports loading intrinsic parameters (camera matrix, distortion coefficients)
from the ``calibration_data.json`` files produced by the toolkit.
"""

import json
import logging
from dataclasses import dataclass
from typing import Optional

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class CameraCalibration:
    """Intrinsic calibration data for a single camera."""
    camera_matrix: np.ndarray       # 3x3 intrinsic matrix
    dist_coeffs: np.ndarray         # 1-D distortion coefficients
    image_size: tuple               # (width, height)
    rms_error: Optional[float] = None
    distortion_model: str = "standard"


def load_calibration(json_path: str) -> CameraCalibration:
    """Load a CameraCalibration from a calibration_data.json file.

    Args:
        json_path: Path to the JSON file produced by camera_calibration_toolkit.

    Returns:
        CameraCalibration with intrinsic parameters populated.

    Raises:
        FileNotFoundError: If json_path does not exist.
        KeyError: If required fields are missing from the JSON.
    """
    with open(json_path, "r") as f:
        data = json.load(f)

    camera_matrix = np.array(data["camera_matrix"], dtype=np.float64)
    dist_coeffs = np.array(data["distortion_coefficients"], dtype=np.float64).flatten()
    image_size = tuple(data.get("image_size", (0, 0)))
    rms_error = data.get("rms_error")
    distortion_model = data.get("distortion_model", "standard")

    logger.info(
        "Loaded calibration from %s  (fx=%.1f fy=%.1f cx=%.1f cy=%.1f rms=%.4f)",
        json_path,
        camera_matrix[0, 0],
        camera_matrix[1, 1],
        camera_matrix[0, 2],
        camera_matrix[1, 2],
        rms_error or 0.0,
    )
    return CameraCalibration(
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
        image_size=image_size,
        rms_error=rms_error,
        distortion_model=distortion_model,
    )
