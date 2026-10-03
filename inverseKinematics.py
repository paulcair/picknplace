"""
inverseKinematics.py

This is a script to control leArm 6DOF robot via serial connection. This script
is written to perform inverse kinematics: given a tool-frame (x, y, z) target
and a width, it computes joint angles. tool_x is set to width / 2.

The solver uses the analytical (geometric) Jacobian of the 5 revolute joints
and damped least squares. Inputs are target (x, y, z) and gripper/object
width. Internally tool_x = width / 2, which sets T6 and servo 1.

Author: Paul Cairns
Date: Oct 3rd, 2026
"""
import math
import numpy as np
import commandRobot
from dhMatrices import get_dh_matrices

# Joint limits matching the 0-180 deg servo mapping used in forwardKinematics.py
JOINT_MIN = 0.0
JOINT_MAX = math.pi

# Default seed matches the forward-kinematics example pose
DEFAULT_SEED_DEG = [90.0, 90.0, 0.0, 0.0, 90.0]


def tool_x_to_s1(tool_x):
    """Map tool_x to servo 1 command (1500 to 2500)."""
    return int((30 - tool_x) * 2000 / 60 + 1500)


def clamp_width(width):
    return float(np.clip(width, 0.0, 60.0))


def width_to_tool_x(width):
    """Half the gripper/object width, used as the tool-frame parameter."""
    return clamp_width(width)/2.0


def angles_to_servos(theta_deg, tool_x):
    """
    Convert joint angles (degrees) and claw width to servo commands.

    Servos are numbered from the tool (S_1) back to the base (S_6).
    """
    S_1 = tool_x_to_s1(tool_x)
    S_2 = int(theta_deg[4] / 180 * 2000 + 500)
    S_3 = int(theta_deg[3] / 180 * 2000 + 500)
    S_4 = int(theta_deg[2] / 180 * 2000 + 500)
    S_5 = int(theta_deg[1] / 180 * 2000 + 500)
    S_6 = int(theta_deg[0] / 180 * 2000 + 500)
    return [S_1, S_2, S_3, S_4, S_5, S_6]


def cumulative_transforms(theta, S_1):
    """
    Return T0_1 ... T0_6 for the current joints and gripper command.

    T0_6 is the tool frame: the five DH joints followed by the T6 offset.
    """
    dh_matrices = get_dh_matrices(theta[0], theta[1], theta[2], theta[3], theta[4], S_1)
    transforms = []
    T = np.eye(4)
    for A in dh_matrices:
        T = T @ A
        transforms.append(T)
    return transforms


def tool_position(theta, S_1):
    """Cartesian origin of the tool frame in the base frame."""
    T0_6 = cumulative_transforms(theta, S_1)[-1]
    return T0_6[:3, 3].copy()


def analytical_jacobian(transforms):
    """
    3x5 analytical position Jacobian of the tool-frame origin.

    Column i is z_{i-1} × (p_tool - p_{i-1}) for revolute joint i.
    Frame 0 is the base (z = [0, 0, 1], origin at 0). Frames 1-4 come from
    T0_1 ... T0_4. p_tool is taken from T0_6 so the gripper offset is included.
    """
    p_tool = transforms[5][:3, 3]
    origins = [np.zeros(3)] + [transforms[i][:3, 3] for i in range(4)]
    z_axes = [np.array([0.0, 0.0, 1.0])] + [transforms[i][:3, 2] for i in range(4)]

    J = np.zeros((3, 5))
    for i in range(5):
        J[:, i] = np.cross(z_axes[i], p_tool - origins[i])
    return J


def _solve_from_seed(
    target,
    S_1,
    seed_deg,
    max_iters,
    position_tol,
    damping,
    max_step,
    nullspace_gain,
):
    q_seed = np.clip(np.radians(np.asarray(seed_deg, dtype=float).reshape(5)), JOINT_MIN, JOINT_MAX)
    q = q_seed.copy()
    residual = np.inf
    success = False
    it = 0
    I3 = np.eye(3)
    I5 = np.eye(5)

    for it in range(1, max_iters + 1):
        transforms = cumulative_transforms(q, S_1)
        error = target - transforms[5][:3, 3]
        residual = np.linalg.norm(error)
        if residual < position_tol:
            success = True
            break

        J = analytical_jacobian(transforms)
        JJT = J @ J.T
        damped = JJT + (damping ** 2) * I3
        dq = J.T @ np.linalg.solve(damped, error)
        if nullspace_gain:
            J_pinv_J = J.T @ np.linalg.solve(damped, J)
            dq += nullspace_gain * ((I5 - J_pinv_J) @ (q_seed - q))

        step_norm = np.linalg.norm(dq)
        if step_norm > max_step:
            dq *= max_step / step_norm

        q = np.clip(q + dq, JOINT_MIN, JOINT_MAX)

    return q, success, it, residual


def _default_seeds(target):
    yaw_deg = math.degrees(math.atan2(target[1], target[0]))
    if yaw_deg < 0:
        yaw_deg += 360.0
    yaw_clipped = float(np.clip(yaw_deg, 0.0, 180.0))
    return [
        DEFAULT_SEED_DEG,
        [yaw_clipped, 90.0, 45.0, 45.0, 90.0],
        [yaw_clipped, 120.0, 40.0, 20.0, 90.0],
        [yaw_clipped, 60.0, 80.0, 40.0, 90.0],
        [0.0, 90.0, 45.0, 45.0, 90.0],
        [180.0, 90.0, 45.0, 45.0, 90.0],
    ]


def inverse_kinematics(
    target_xyz,
    width,
    seed_deg=None,
    max_iters=200,
    position_tol=0.5,
    damping=1e-2,
    max_step=0.2,
    nullspace_gain=0.0,
):
    """
    Solve for joint angles that place the tool-frame origin at target_xyz.

    Parameters
    ----------
    target_xyz : iterable of 3 floats
        Desired tool-frame origin (mm), same units as the DH parameters.
    width : float
        Full gripper/object width, 0 to 30. Internally tool_x = width / 2.
    seed_deg : list of 5 floats, optional
        Initial joint guess in degrees. If omitted, several seeds are tried.
    max_iters, position_tol, damping, max_step, nullspace_gain
        Newton / damped-least-squares settings.

    Returns
    -------
    theta_deg : ndarray, shape (5,)
        Joint angles in degrees (theta_1 ... theta_5).
    tool_x : float
        width / 2, used for T6 and servo 1.
    info : dict
        Convergence metadata (success, iterations, residual).
    """
    width = clamp_width(width)
    tool_x = width_to_tool_x(width)
    S_1 = tool_x_to_s1(tool_x)
    target = np.asarray(target_xyz, dtype=float).reshape(3)

    if seed_deg is None:
        seeds = _default_seeds(target)
    else:
        seeds = [seed_deg]

    best = None
    for seed in seeds:
        q, success, it, residual = _solve_from_seed(
            target,
            S_1,
            seed,
            max_iters,
            position_tol,
            damping,
            max_step,
            nullspace_gain,
        )
        candidate = (residual, q, success, it, seed)
        if best is None or residual < best[0]:
            best = candidate
        if success:
            break

    residual, q, success, it, seed = best
    theta_deg = np.degrees(q)
    info = {
        "success": success,
        "iterations": it,
        "residual_mm": residual,
        "position": tool_position(q, S_1),
        "S_1": S_1,
        "seed_deg": seed,
        "width": width,
        "tool_x": tool_x,
    }
    return theta_deg, tool_x, info


def main():
    target_x = 50
    target_y = 150
    target_z = 150.0
    width = 0

    theta_deg, tool_x, ik_info = inverse_kinematics([target_x, target_y, target_z], width)
    theta_1_deg, theta_2_deg, theta_3_deg, theta_4_deg, theta_5_deg = theta_deg
    angles = angles_to_servos(theta_deg, tool_x)

    print("IK success:", ik_info["success"])
    print("Iterations:", ik_info["iterations"])
    print("Position residual (mm):", round(ik_info["residual_mm"], 4))
    print("Solved tool position (mm):", np.round(ik_info["position"], 4))
    print("theta_1 ... theta_5 (deg):", np.round(theta_deg, 4))
    print("width:", width)
    print("tool_x (width/2):", tool_x)
    print("Servo commands [S_1 ... S_6]:", angles)

    robot = commandRobot.RobotController()

    if robot.ser is None:
        print("Failed to establish serial connection. Exiting.")
        return

    try:
        print("Moving Robot...")
        robot.move(angles)
    except Exception as e:
        print(f"Error during movement: {e}")
    finally:
        robot.close()


if __name__ == "__main__":
    main()
