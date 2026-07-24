"""Small self-contained kinematics helper for the RASCL robot.

The first WP3 milestone uses the calibrated ``tcp_link`` as the tool center
point (TCP).  The geometry constants below are copied from
rascl_description/urdf/rascl.urdf.  This avoids adding a heavy kinematics
dependency while still making Cartesian targets consistent with URDF and TF.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, List, Sequence, Tuple

Vector3 = Tuple[float, float, float]
Matrix4 = List[List[float]]

JOINT_NAMES = ["shoulder_joint", "upperarm_joint", "lowerarm_joint", "spur_gear_joint"]
ARM_JOINT_NAMES = JOINT_NAMES[:3]

# Joint limits are intentionally kept identical to the URDF ros2_control block.
ARM_LIMITS = [
    (-1.570796327, 1.570796327),
    (-3.141592654, 3.141592654),
    (-3.141592654, 3.141592654),
]
SPUR_GEAR_LIMIT = (-6.283185307, 6.283185307)

# Global XY calibration. A joint pose reported at model XY [0.12, 0.12] was
# measured at physical XY [0.16, 0.16]. Relative to the previous base origin,
# the complete arm model is therefore translated +40 mm in both base_link X
# and Y. The Z origin and all rotating/local geometry remain unchanged.
BASE_TO_SHOULDER_ORIGIN = (0.040, 0.020, 0.057441)

# Fixed Cartesian TCP at the spur_gear_joint center in the lowerarm frame.
# Keeping a separate fixed link prevents gripper rotation from moving the
# planning frame while matching the historical spur-gear-center reference.
TCP_ORIGIN_IN_LOWERARM = (0.13916, 0.0, 0.0179)

# Nominal position of the TCP in base_link when q=[0,0,0].  This is useful when
# calibrating the real robot: at the physical URDF zero pose, the hardware
# count offsets must make FK([0,0,0]) describe the real TCP pose. Automatic
# reference-switch Homing itself finishes at a different, non-zero model pose.
NOMINAL_ZERO_TCP_IN_BASE_LINK = (0.33756, 0.01823, 0.043001)


@dataclass
class IKResult:
    """Result returned by the numerical inverse kinematics solver."""

    success: bool
    q: List[float]
    position: Vector3
    error_norm: float
    iterations: int
    message: str


def _identity() -> Matrix4:
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _matmul(a: Matrix4, b: Matrix4) -> Matrix4:
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _translation(x: float, y: float, z: float) -> Matrix4:
    t = _identity()
    t[0][3] = x
    t[1][3] = y
    t[2][3] = z
    return t


def _rot_x(angle: float) -> Matrix4:
    c = math.cos(angle)
    s = math.sin(angle)
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, c, -s, 0.0],
        [0.0, s, c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _rot_y(angle: float) -> Matrix4:
    c = math.cos(angle)
    s = math.sin(angle)
    return [
        [c, 0.0, s, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [-s, 0.0, c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _rot_z(angle: float) -> Matrix4:
    c = math.cos(angle)
    s = math.sin(angle)
    return [
        [c, -s, 0.0, 0.0],
        [s, c, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _rpy(roll: float, pitch: float, yaw: float) -> Matrix4:
    # URDF convention: fixed-axis roll, pitch, yaw equals Rz(yaw) * Ry(pitch) * Rx(roll).
    return _matmul(_matmul(_rot_z(yaw), _rot_y(pitch)), _rot_x(roll))


def _origin(xyz: Vector3, rpy: Vector3) -> Matrix4:
    return _matmul(_translation(*xyz), _rpy(*rpy))


def _axis_rotation(axis: Vector3, angle: float) -> Matrix4:
    x, y, z = axis
    norm = math.sqrt(x * x + y * y + z * z)
    if norm <= 0.0:
        raise ValueError("Joint axis must be non-zero")
    x /= norm
    y /= norm
    z /= norm
    c = math.cos(angle)
    s = math.sin(angle)
    one_minus_c = 1.0 - c
    return [
        [x * x * one_minus_c + c, x * y * one_minus_c - z * s, x * z * one_minus_c + y * s, 0.0],
        [y * x * one_minus_c + z * s, y * y * one_minus_c + c, y * z * one_minus_c - x * s, 0.0],
        [z * x * one_minus_c - y * s, z * y * one_minus_c + x * s, z * z * one_minus_c + c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _position(t: Matrix4) -> Vector3:
    return (t[0][3], t[1][3], t[2][3])


def _vec_sub(a: Sequence[float], b: Sequence[float]) -> List[float]:
    return [a[i] - b[i] for i in range(3)]


def _norm(v: Sequence[float]) -> float:
    return math.sqrt(sum(x * x for x in v))


def clamp_to_limits(q: Sequence[float], limits: Sequence[Tuple[float, float]] = ARM_LIMITS) -> List[float]:
    return [min(max(float(value), limits[i][0]), limits[i][1]) for i, value in enumerate(q)]


def forward_tcp(q_arm: Sequence[float]) -> Vector3:
    """Return the calibrated ``tcp_link`` origin in base_link coordinates.

    Args:
        q_arm: [shoulder_joint, upperarm_joint, lowerarm_joint] in radians.

    Returns:
        TCP position [x, y, z] in meters, expressed in base_link.
    """

    if len(q_arm) != 3:
        raise ValueError("forward_tcp expects exactly three arm joint values")

    q1, q2, q3 = q_arm
    transform = _identity()

    # shoulder_joint, base_link -> shoulder
    transform = _matmul(transform, _origin(BASE_TO_SHOULDER_ORIGIN, (0.0, 0.0, 0.0)))
    transform = _matmul(transform, _axis_rotation((0.0, 0.0, -1.0), q1))

    # upperarm_joint, shoulder -> upperarm
    transform = _matmul(transform, _origin((-0.0116, -0.0057, 0.06556), (-math.pi / 2.0, 0.0, 0.0)))
    transform = _matmul(transform, _axis_rotation((0.0, 0.0, -1.0), q2))

    # lowerarm_joint, upperarm -> lowerarm
    transform = _matmul(transform, _origin((0.17, 0.08, 0.02183), (-math.pi, 0.0, 0.0)))
    transform = _matmul(transform, _axis_rotation((0.0, 0.0, -1.0), q3))

    # Match the dedicated fixed tcp_link in rascl.urdf.  TCP must not depend on
    # the gripper's spur_gear_joint angle.
    transform = _matmul(transform, _origin(TCP_ORIGIN_IN_LOWERARM, (0.0, -math.pi / 2.0, 0.0)))
    return _position(transform)


def _solve_3x3(a: Sequence[Sequence[float]], b: Sequence[float]) -> List[float]:
    """Solve a 3x3 linear system with Gaussian elimination."""

    matrix = [[float(a[i][j]) for j in range(3)] + [float(b[i])] for i in range(3)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda row: abs(matrix[row][col]))
        if abs(matrix[pivot][col]) < 1e-12:
            raise ValueError("Singular linear system")
        if pivot != col:
            matrix[col], matrix[pivot] = matrix[pivot], matrix[col]
        pivot_value = matrix[col][col]
        for j in range(col, 4):
            matrix[col][j] /= pivot_value
        for row in range(3):
            if row == col:
                continue
            factor = matrix[row][col]
            for j in range(col, 4):
                matrix[row][j] -= factor * matrix[col][j]
    return [matrix[i][3] for i in range(3)]


def _jacobian_numeric(q: Sequence[float], step: float = 1e-5) -> List[List[float]]:
    base = forward_tcp(q)
    jacobian = [[0.0 for _ in range(3)] for _ in range(3)]
    for joint_index in range(3):
        perturbed = list(q)
        perturbed[joint_index] += step
        shifted = forward_tcp(perturbed)
        for axis_index in range(3):
            jacobian[axis_index][joint_index] = (shifted[axis_index] - base[axis_index]) / step
    return jacobian


def _damped_least_squares_step(jacobian: Sequence[Sequence[float]], error: Sequence[float], damping: float) -> List[float]:
    # Solve (J^T J + lambda^2 I) dq = J^T e.
    lhs = [[0.0 for _ in range(3)] for _ in range(3)]
    rhs = [0.0 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            lhs[i][j] = sum(jacobian[k][i] * jacobian[k][j] for k in range(3))
        lhs[i][i] += damping * damping
        rhs[i] = sum(jacobian[k][i] * error[k] for k in range(3))
    return _solve_3x3(lhs, rhs)


def inverse_tcp(
    target: Vector3,
    seed: Sequence[float] | None = None,
    tolerance: float = 0.002,
    max_iterations: int = 160,
) -> IKResult:
    """Find arm joint angles that place the TCP near a target position.

    This numerical IK controls only the TCP position.  It deliberately does not
    constrain the end-effector orientation because the RASCL arm has only three
    arm joints in this first WP3 step.
    """

    if seed is None:
        seed = [0.0, 0.0, 0.0]

    # Try several seeds.  The current joint state is still tried first, which
    # reduces unnecessary configuration flips during real robot debugging.
    seed_candidates = [
        list(seed),
        [0.0, 0.0, 0.0],
        [0.0, 0.35, -0.35],
        [0.0, -0.35, 0.35],
        [0.5, 0.25, -0.25],
        [-0.5, 0.25, -0.25],
        [0.5, -0.25, 0.25],
        [-0.5, -0.25, 0.25],
    ]

    best_result: IKResult | None = None
    for candidate in seed_candidates:
        q = clamp_to_limits(candidate)
        for iteration in range(max_iterations):
            current = forward_tcp(q)
            error = _vec_sub(target, current)
            error_norm = _norm(error)
            if error_norm <= tolerance:
                return IKResult(True, q, current, error_norm, iteration, "IK converged")

            jacobian = _jacobian_numeric(q)
            try:
                delta_q = _damped_least_squares_step(jacobian, error, damping=0.025)
            except ValueError:
                break

            # Limit the per-iteration step to avoid jumping across joint limits.
            max_step = max(abs(value) for value in delta_q)
            if max_step > 0.15:
                scale = 0.15 / max_step
                delta_q = [value * scale for value in delta_q]

            q = clamp_to_limits([q[i] + delta_q[i] for i in range(3)])

        final_position = forward_tcp(q)
        final_error = _norm(_vec_sub(target, final_position))
        result = IKResult(False, q, final_position, final_error, max_iterations, "IK did not reach tolerance")
        if best_result is None or result.error_norm < best_result.error_norm:
            best_result = result

    assert best_result is not None
    if best_result.error_norm <= tolerance:
        best_result.success = True
        best_result.message = "IK converged after seed search"
    return best_result


def within_limits(q: Iterable[float], limits: Sequence[Tuple[float, float]] = ARM_LIMITS) -> bool:
    return all(limits[i][0] <= value <= limits[i][1] for i, value in enumerate(q))
