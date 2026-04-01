#!/usr/bin/env python3
"""
Tool Weight Calibration — Computation functions.

Estimates tool mass and center of gravity from F/T sensor readings
at multiple robot orientations using least squares.

The UR actual_TCP_force (published on /force_torque_sensor_broadcaster/wrench)
is in the TOOL frame (frame_id: tool0). It is compensated for set_payload
and may be zeroed by zero_ftsensor.

With set_payload(0), the model is:
    F_measured = m * R^T @ [0,0,-g] - B_force
    T_measured = p_cog x (m * R^T @ [0,0,-g]) - B_torque

where R is tool rotation in base frame, so R^T @ g rotates gravity to tool frame.

Usage:
    from calibrate_tool_weight import solve_mass_and_cog
    # samples: list of (force[3], torque[3], R_tool[3x3])
    mass, cog, f_err, t_err = solve_mass_and_cog(samples)
"""

import numpy as np

GRAVITY = 9.80665  # m/s^2


def solve_mass_and_cog(samples):
    """
    Solve for tool mass and center of gravity from F/T samples.

    The F/T data is in TOOL frame (actual_TCP_force from RTDE, frame_id: tool0).

    Args:
        samples: list of tuples (force[3], torque[3], R_tool[3x3])
            force: F/T force reading in TOOL frame (N)
            torque: F/T torque reading in TOOL frame (Nm)
            R_tool: 3x3 rotation matrix of tool in base frame

    Returns:
        mass: tool mass (kg)
        cog: center of gravity [cx, cy, cz] in tool frame (m)
        force_residual: average force fitting error (N)
        torque_residual: average torque fitting error (Nm)
    """
    n = len(samples)
    if n < 4:
        raise ValueError(f"Need at least 4 samples, got {n}")

    samples = [(np.asarray(f), np.asarray(t), np.asarray(R)) for f, t, R in samples]

    # --- Force: solve for mass + bias ---
    # F_measured = m * g_tool - B_force
    # where g_tool = R^T @ [0, 0, -g] (gravity rotated to tool frame)
    A = np.zeros((3 * n, 4))
    b_f = np.zeros(3 * n)
    for i, (force, torque, R) in enumerate(samples):
        g_tool = R.T @ np.array([0, 0, -GRAVITY])
        row = 3 * i
        A[row+0] = [g_tool[0], -1,  0,  0]
        A[row+1] = [g_tool[1],  0, -1,  0]
        A[row+2] = [g_tool[2],  0,  0, -1]
        b_f[row:row+3] = force

    x_f = np.linalg.lstsq(A, b_f, rcond=None)[0]
    mass = x_f[0]
    force_bias = x_f[1:4]

    # --- Torque: solve for CoG + bias ---
    # T_measured = p_cog x F_gravity_tool - B_torque
    # where F_gravity_tool = m * g_tool (the gravity force in tool frame)
    # and p_cog x F = [cy*Fz - cz*Fy, cz*Fx - cx*Fz, cx*Fy - cy*Fx]
    C = np.zeros((3 * n, 6))
    d_t = np.zeros(3 * n)
    for i, (force, torque, R) in enumerate(samples):
        g_tool = R.T @ np.array([0, 0, -GRAVITY])
        Fg = mass * g_tool  # gravity force in tool frame
        Fx, Fy, Fz = Fg
        row = 3 * i
        C[row+0] = [   0,  Fz, -Fy, -1,  0,  0]
        C[row+1] = [ -Fz,   0,  Fx,  0, -1,  0]
        C[row+2] = [  Fy, -Fx,   0,  0,  0, -1]
        d_t[row:row+3] = torque

    x_t = np.linalg.lstsq(C, d_t, rcond=None)[0]
    cog = x_t[0:3]

    # Residuals
    force_residual = np.linalg.norm(A @ x_f - b_f) / n
    torque_residual = np.linalg.norm(C @ x_t - d_t) / n

    return float(mass), cog.tolist(), float(force_residual), float(torque_residual)
