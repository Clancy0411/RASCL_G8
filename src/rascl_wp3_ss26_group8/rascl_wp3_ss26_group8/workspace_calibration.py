"""Task-space calibration for the fixed RASCL cube board.

The coefficients were fitted from real-hardware centring corrections measured
across the board on 2026-07-29.  This is deliberately separate from the URDF
and joint Home calibration: it corrects board targets before inverse
kinematics, without moving the shoulder axis or redefining an encoder zero.
"""

from __future__ import annotations

from typing import Sequence, Tuple


Vector3 = Tuple[float, float, float]

# Corrected board target, in metres:
#   x' = 1.0098577586*x - 0.0114794948*y + 0.0006327808
#   y' = 0.0041074758*x + 1.0252229617*y - 0.0033871656
DEFAULT_BOARD_XY_MATRIX = (
    1.0098577586235102,
    -0.011479494849091436,
    0.004107475770149662,
    1.025222961738071,
)
DEFAULT_BOARD_XY_OFFSET_M = (
    0.0006327807577263632,
    -0.003387165557392895,
)

# Axis-aligned bounds of the measured data.  The compensation is not clamped
# at these limits; callers use them to warn about extrapolation.
BOARD_XY_MEASURED_BOUNDS_M = (-0.230, 0.250, 0.030, 0.250)


def compensate_board_xy(
    target: Sequence[float],
    matrix: Sequence[float] = DEFAULT_BOARD_XY_MATRIX,
    offset_m: Sequence[float] = DEFAULT_BOARD_XY_OFFSET_M,
) -> Vector3:
    """Apply the fitted board-plane affine correction to an XYZ target."""

    if len(target) != 3:
        raise ValueError("board XY compensation expects an XYZ target")
    if len(matrix) != 4:
        raise ValueError("board_xy_compensation_matrix must contain four values")
    if len(offset_m) != 2:
        raise ValueError("board_xy_compensation_offset_m must contain two values")

    x, y, z = (float(value) for value in target)
    m00, m01, m10, m11 = (float(value) for value in matrix)
    offset_x, offset_y = (float(value) for value in offset_m)
    return (
        m00 * x + m01 * y + offset_x,
        m10 * x + m11 * y + offset_y,
        z,
    )


def board_xy_is_within_measured_bounds(target: Sequence[float]) -> bool:
    """Return whether a target lies inside the fitted data's XY bounds."""

    if len(target) < 2:
        raise ValueError("board XY bounds check expects at least X and Y")
    x = float(target[0])
    y = float(target[1])
    x_min, x_max, y_min, y_max = BOARD_XY_MEASURED_BOUNDS_M
    return x_min <= x <= x_max and y_min <= y <= y_max
