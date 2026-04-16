"""
UR15 Forward Kinematics

Computes TCP (tool0) pose from joint angles using the URDF transform chain.
Chain: base_link → shoulder → upper_arm → forearm → wrist_1 → wrist_2 → wrist_3 → flange → tool0

Usage:
    from ur15_dashboard.kinematics import forward_kinematics, fk_6dof

    q = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]  # 6 joint angles in radians
    T = forward_kinematics(q)                   # 4x4 numpy matrix
    pose = fk_6dof(q)                           # [x, y, z, rx, ry, rz] (meters + axis-angle)
"""

import numpy as np
from scipy.spatial.transform import Rotation
from typing import List, Tuple


def _rot_z(angle: float) -> np.ndarray:
    """Rotation matrix around Z axis."""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([
        [c, -s, 0, 0],
        [s,  c, 0, 0],
        [0,  0, 1, 0],
        [0,  0, 0, 1],
    ])


def _transform(rpy: Tuple[float, float, float], xyz: Tuple[float, float, float]) -> np.ndarray:
    """Build 4x4 homogeneous transform from RPY (roll, pitch, yaw) and XYZ translation.
    Uses extrinsic XYZ convention (same as URDF fixed-axis RPY)."""
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    T[0, 3], T[1, 3], T[2, 3] = xyz
    return T


# Fixed transform: base_link → base_link_inertia (Rz(pi))
_T_BASE_INERTIA = _transform((0, 0, np.pi), (0, 0, 0))

# UR15 URDF joint origins (from model.urdf)
# Chain: base_link_inertia → shoulder_pan → shoulder_lift → elbow → wrist_1 → wrist_2 → wrist_3
# Each entry: (rpy, xyz) for the joint origin transform
_JOINT_ORIGINS = [
    # shoulder_pan_joint
    ((0, 0, 0), (0, 0, 0.2186)),
    # shoulder_lift_joint
    ((1.570796327, 0, 0), (0, 0, 0)),
    # elbow_joint
    ((0, 0, 0), (-0.6475, 0, 0)),
    # wrist_1_joint
    ((0, 0, 0), (-0.5164, 0, 0.1824)),
    # wrist_2_joint
    ((1.570796327, 0, 0), (0, -0.1361, 0)),
    # wrist_3_joint
    ((1.570796326589793, np.pi, np.pi), (0, 0.1434, 0)),
]

# Fixed transforms after wrist_3
# wrist_3 → flange
_T_WRIST3_FLANGE = _transform((0, -np.pi/2, -np.pi/2), (0, 0, 0))
# flange → tool0
_T_FLANGE_TOOL0 = _transform((np.pi/2, 0, np.pi/2), (0, 0, 0))
# Combined fixed tail
_T_FIXED_TAIL = _T_WRIST3_FLANGE @ _T_FLANGE_TOOL0


def forward_kinematics(joint_angles: List[float]) -> np.ndarray:
    """
    Compute the 4x4 homogeneous transform from base_link to tool0.

    Args:
        joint_angles: List of 6 joint angles in radians
            [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]

    Returns:
        4x4 numpy array: homogeneous transform matrix (base_link → tool0)
    """
    if len(joint_angles) != 6:
        raise ValueError(f"Expected 6 joint angles, got {len(joint_angles)}")

    T = _T_BASE_INERTIA.copy()
    for i in range(6):
        rpy, xyz = _JOINT_ORIGINS[i]
        T = T @ _transform(rpy, xyz) @ _rot_z(joint_angles[i])

    T = T @ _T_FIXED_TAIL
    return T


def rotation_to_axis_angle(R: np.ndarray) -> np.ndarray:
    """Convert 3x3 rotation matrix to axis-angle representation (rx, ry, rz)."""
    return Rotation.from_matrix(R).as_rotvec()


def rotation_to_quaternion(R: np.ndarray) -> np.ndarray:
    """Convert 3x3 rotation matrix to quaternion [x, y, z, w]."""
    return Rotation.from_matrix(R).as_quat()


def quaternion_to_rpy(quat: np.ndarray) -> np.ndarray:
    """Convert quaternion [x, y, z, w] to RPY (roll, pitch, yaw) in radians.
    Uses extrinsic XYZ convention (same as URDF)."""
    return Rotation.from_quat(quat).as_euler('xyz')


def fk_6dof(joint_angles: List[float]) -> np.ndarray:
    """
    Compute TCP pose as 6-DOF vector [x, y, z, rx, ry, rz].

    Position in meters, orientation as axis-angle (radians).

    Args:
        joint_angles: List of 6 joint angles in radians

    Returns:
        numpy array [x, y, z, rx, ry, rz]
    """
    T = forward_kinematics(joint_angles)
    pos = T[:3, 3]
    aa = rotation_to_axis_angle(T[:3, :3])
    return np.concatenate([pos, aa])


def fk_quaternion(joint_angles: List[float]) -> Tuple[np.ndarray, np.ndarray]:
    """
    Compute TCP pose as position + quaternion.

    Args:
        joint_angles: List of 6 joint angles in radians

    Returns:
        Tuple of (position[3], quaternion[4] as [x, y, z, w])
    """
    T = forward_kinematics(joint_angles)
    pos = T[:3, 3]
    quat = rotation_to_quaternion(T[:3, :3])
    return pos, quat
