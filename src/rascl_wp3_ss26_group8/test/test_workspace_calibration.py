"""Regression tests for the fitted cube-board XY correction."""

import math

import pytest

from rascl_wp3_ss26_group8.workspace_calibration import (
    BOARD_XY_MEASURED_BOUNDS_M,
    DEFAULT_BOARD_XY_MATRIX,
    DEFAULT_BOARD_XY_OFFSET_M,
    board_xy_is_within_measured_bounds,
    compensate_board_xy,
)


def test_board_xy_compensation_matches_fitted_equation_and_preserves_z():
    corrected = compensate_board_xy((-0.020, 0.250, 0.080))

    assert corrected[0] == pytest.approx(-0.02243425, abs=1e-8)
    assert corrected[1] == pytest.approx(0.25283643, abs=1e-8)
    assert corrected[2] == pytest.approx(0.080, abs=0.0)


def test_board_xy_compensation_defaults_preserve_the_approved_fit():
    assert DEFAULT_BOARD_XY_MATRIX == pytest.approx(
        (1.0098577586235102, -0.011479494849091436,
         0.004107475770149662, 1.025222961738071)
    )
    assert DEFAULT_BOARD_XY_OFFSET_M == pytest.approx(
        (0.0006327807577263632, -0.003387165557392895)
    )


def test_board_xy_compensation_accepts_runtime_coefficients():
    corrected = compensate_board_xy(
        (0.1, -0.2, 0.3),
        matrix=(1.0, 0.0, 0.0, 1.0),
        offset_m=(0.01, -0.02),
    )

    assert corrected == pytest.approx((0.11, -0.22, 0.3))


@pytest.mark.parametrize(
    ("target", "expected"),
    [
        ((-0.230, 0.030, 0.0), True),
        ((0.250, 0.250, 0.0), True),
        ((-0.231, 0.100, 0.0), False),
        ((0.100, 0.251, 0.0), False),
    ],
)
def test_board_xy_measured_bounds(target, expected):
    assert board_xy_is_within_measured_bounds(target) is expected


def test_board_xy_bounds_constant_remains_the_measured_rectangle():
    assert BOARD_XY_MEASURED_BOUNDS_M == (-0.230, 0.250, 0.030, 0.250)


@pytest.mark.parametrize(
    ("target", "matrix", "offset"),
    [
        ((0.0, 0.0), (1.0, 0.0, 0.0, 1.0), (0.0, 0.0)),
        ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0)),
        ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 1.0), (0.0,)),
    ],
)
def test_board_xy_compensation_rejects_malformed_values(target, matrix, offset):
    with pytest.raises(ValueError):
        compensate_board_xy(target, matrix=matrix, offset_m=offset)


def test_board_xy_compensation_coefficients_are_finite():
    corrected = compensate_board_xy((0.250, 0.030, 0.080))
    assert all(math.isfinite(value) for value in corrected)
