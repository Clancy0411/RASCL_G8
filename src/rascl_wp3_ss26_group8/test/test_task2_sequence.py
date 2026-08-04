"""Tests for the submission-facing WP3 Task 2 online sequence."""

import math
from pathlib import Path

import pytest

from rascl_wp3_ss26_group8.task2_sequence import (
    DEFAULT_GOAL_X_M,
    DEFAULT_GOAL_Y_M,
    DEFAULT_MAX_FEASIBLE_RADIUS_M,
    CartesianStep,
    GripperStep,
    build_pick_and_place_sequence,
    validate_cube_center,
    validate_task2_configuration,
)


def _package_root() -> Path:
    return Path(__file__).resolve().parents[1]


def test_default_goal_is_at_the_declared_maximum_feasible_radius():
    validate_task2_configuration(
        goal_x=DEFAULT_GOAL_X_M,
        goal_y=DEFAULT_GOAL_Y_M,
        min_radius=0.10,
        max_radius=DEFAULT_MAX_FEASIBLE_RADIUS_M,
    )
    assert DEFAULT_MAX_FEASIBLE_RADIUS_M == pytest.approx(
        math.hypot(DEFAULT_GOAL_X_M, DEFAULT_GOAL_Y_M)
    )


def test_configuration_rejects_a_goal_that_is_not_at_the_maximum_radius():
    with pytest.raises(ValueError, match="goal radius must equal"):
        validate_task2_configuration(
            goal_x=0.18,
            goal_y=-0.04,
            min_radius=0.10,
            max_radius=0.20,
        )


def test_cube_centre_is_checked_against_radius_and_half_plane():
    radius, angle = validate_cube_center(
        0.16,
        0.08,
        min_radius=0.10,
        max_radius=DEFAULT_MAX_FEASIBLE_RADIUS_M,
    )
    assert radius == pytest.approx(math.hypot(0.16, 0.08))
    assert angle == pytest.approx(math.atan2(0.08, 0.16))

    with pytest.raises(ValueError, match="outside the declared feasible region"):
        validate_cube_center(
            0.25,
            0.0,
            min_radius=0.10,
            max_radius=DEFAULT_MAX_FEASIBLE_RADIUS_M,
        )
    with pytest.raises(ValueError, match="outside the allowed shoulder range"):
        validate_cube_center(
            -0.16,
            0.0,
            min_radius=0.10,
            max_radius=DEFAULT_MAX_FEASIBLE_RADIUS_M,
        )


def test_online_sequence_uses_safe_vertical_pick_and_place_transfers():
    sequence = build_pick_and_place_sequence(
        0.16,
        0.08,
        goal_x=DEFAULT_GOAL_X_M,
        goal_y=DEFAULT_GOAL_Y_M,
        travel_z=0.10,
        pick_z=0.045,
        place_z=0.045,
        motion_duration_s=5.0,
        gripper_duration_s=5.0,
    )

    assert [step.label for step in sequence] == [
        "approach_pick",
        "descend_pick",
        "close_gripper",
        "lift_pick",
        "approach_goal",
        "descend_goal",
        "open_gripper",
        "retreat_goal",
    ]
    assert isinstance(sequence[0], CartesianStep)
    assert isinstance(sequence[2], GripperStep)
    assert isinstance(sequence[4], CartesianStep)
    assert sequence[0].target == (0.16, 0.08, 0.10)
    assert sequence[3].target == (0.16, 0.08, 0.10)
    assert sequence[4].target == (DEFAULT_GOAL_X_M, DEFAULT_GOAL_Y_M, 0.10)
    assert sequence[5].target == (DEFAULT_GOAL_X_M, DEFAULT_GOAL_Y_M, 0.045)
    assert sequence[7].target == (DEFAULT_GOAL_X_M, DEFAULT_GOAL_Y_M, 0.10)


def test_submission_installs_wp3_tsk2_and_provides_the_required_launch_file():
    package_root = _package_root()
    setup_source = (package_root / "setup.py").read_text(encoding="utf-8")
    node_source = (
        package_root / "rascl_wp3_ss26_group8" / "wp3_tsk2.py"
    ).read_text(encoding="utf-8")
    launch_source = (
        package_root / "launch" / "wp3_tsk2.launch.py"
    ).read_text(encoding="utf-8")

    assert "wp3_tsk2 = rascl_wp3_ss26_group8.wp3_tsk2:main" in setup_source
    assert 'super().__init__("wp3_tsk2")' in node_source
    assert "self.create_subscription(" in node_source
    assert "Point, goal_topic, self._goal_callback" in node_source
    assert 'executable="wp3_tsk2"' in launch_source
