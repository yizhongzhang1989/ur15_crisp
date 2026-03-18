"""Temporal pose filtering for smoother 6D pose estimates.

Provides exponential moving average (EMA) smoothing of position and
quaternion (using SLERP for rotations).
"""

import numpy as np
from typing import Optional, Tuple


def _slerp(q0: np.ndarray, q1: np.ndarray, alpha: float) -> np.ndarray:
    """Spherical linear interpolation between two unit quaternions (xyzw)."""
    dot = np.dot(q0, q1)
    # Ensure shortest path
    if dot < 0:
        q1 = -q1
        dot = -dot
    dot = min(dot, 1.0)

    if dot > 0.9995:
        # Very close — use linear interpolation
        result = q0 + alpha * (q1 - q0)
        return result / np.linalg.norm(result)

    theta_0 = np.arccos(dot)
    theta = theta_0 * alpha
    sin_theta = np.sin(theta)
    sin_theta_0 = np.sin(theta_0)

    s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    result = s0 * q0 + s1 * q1
    return result / np.linalg.norm(result)


class PoseFilter:
    """Exponential moving average filter for 6D poses."""

    def __init__(self, alpha: float = 0.5, enabled: bool = True):
        """
        Args:
            alpha: Smoothing factor in (0, 1]. Higher = more responsive,
                   lower = smoother. 1.0 means no filtering.
            enabled: If False, update() returns the raw input unchanged.
        """
        self.alpha = np.clip(alpha, 0.01, 1.0)
        self.enabled = enabled
        self._pos: Optional[np.ndarray] = None
        self._quat: Optional[np.ndarray] = None

    def update(
        self, position: np.ndarray, quaternion: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Feed a new measurement and return the filtered pose.

        Args:
            position: (3,) array [x, y, z].
            quaternion: (4,) array [x, y, z, w] (ROS convention).

        Returns:
            (filtered_position, filtered_quaternion)
        """
        if not self.enabled:
            self._pos = position.copy()
            self._quat = quaternion.copy()
            return position, quaternion

        if self._pos is None:
            # First measurement — initialise
            self._pos = position.copy()
            self._quat = quaternion.copy()
            return position, quaternion

        # EMA on position
        self._pos = self._pos + self.alpha * (position - self._pos)

        # SLERP on quaternion
        self._quat = _slerp(self._quat, quaternion, self.alpha)

        return self._pos.copy(), self._quat.copy()

    def reset(self):
        """Clear filter state."""
        self._pos = None
        self._quat = None
