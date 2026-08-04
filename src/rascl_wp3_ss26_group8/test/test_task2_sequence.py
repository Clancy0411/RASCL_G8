"""Tests for the submission-facing WP3 Task 2 online sequence."""

import math
from pathlib import Path

import pytest

from rascl_wp3_ss26_group8.task2_sequence import (
    DEFAULT_GOAL_X_M,
    DEFAULT_GOAL_Y_M,
    DEFAULT_INNER_ROUTE_X_M,
    DEFAULT_INNER_ROUTE_Y_M,
    DEFAULT_INNER_ROUTE_MAX_RADIUS_M,
    DEFAULT_MIDDLE_ROUTE_MAX_RADIUS_M,
    DEFAULT_OUTER_ROUTE_X_M,
    DEFAULT_OUTER_ROUTE_Y_M,
    INNER_ROUTE,
    MIDDLE_ROUTE,
    OUTER_ROUTE,
    CartesianStep,
    GripperStep,
    build_pick_and_place_sequence,
    classify_radial_route,
    validate_cube_center,
    validate_task2_configuration,
)


def _package_root() -> Path:
    return Path(__file__).resolve().parents[1]


def test_configuration_preserves_the_fixed_goal_and_accepts_finite_coordinates():
    validate_task2_configuration(
        goal_x=DEFAULT_GOAL_X_M,
        goal_y=DEFAULT_GOAL_Y_M,
    )
    validate_task2_configuration(goal_x=0.30, goal_y=0.0)


def test_default_internal_goal_preserves_the_group_29_target_correction():
    assert (DEFAULT_GOAL_X_M, DEFAULT_GOAL_Y_M) == pytest.approx(
        (0.1812, -0.0336)
    )


def test_legacy_group_29_radius_boundaries_select_inner_middle_and_outer():
    assert classify_radial_route(0.169999) == INNER_ROUTE
    assert classify_radial_route(DEFAULT_INNER_ROUTE_MAX_RADIUS_M) == MIDDLE_ROUTE
    assert classify_radial_route(DEFAULT_MIDDLE_ROUTE_MAX_RADIUS_M) == MIDDLE_ROUTE
    assert classify_radial_route(0.200001) == OUTER_ROUTE


def test_cube_centre_is_classified_by_radius_and_checked_by_shoulder_range():
    radius, angle = validate_cube_center(
        0.250,
        0.060,
    )
    assert radius == pytest.approx(math.hypot(0.250, 0.060))
    assert angle == pytest.approx(math.atan2(0.060, 0.250))

    radius, _ = validate_cube_center(0.30, 0.0)
    assert radius == pytest.approx(0.30)
    with pytest.raises(ValueError, match="outside the allowed shoulder range"):
        validate_cube_center(-0.16, 0.0)


def test_middle_route_uses_direct_vertical_pick_and_place_transfer():
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


def test_inner_route_descends_inside_and_pushes_outward_to_the_goal():
    sequence = build_pick_and_place_sequence(
        0.12,
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
        "approach_inner",
        "descend_inner",
        "push_outward_to_goal",
        "open_gripper",
        "retreat_goal",
    ]
    assert sequence[4].target == (
        DEFAULT_INNER_ROUTE_X_M,
        DEFAULT_INNER_ROUTE_Y_M,
        0.10,
    )
    assert sequence[5].target == (
        DEFAULT_INNER_ROUTE_X_M,
        DEFAULT_INNER_ROUTE_Y_M,
        0.045,
    )
    assert sequence[6].target == (DEFAULT_GOAL_X_M, DEFAULT_GOAL_Y_M, 0.045)


def test_outer_route_descends_outside_and_pulls_inward_to_the_goal():
    sequence = build_pick_and_place_sequence(
        0.23,
        0.10,
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
        "approach_outer",
        "descend_outer",
        "pull_inward_to_goal",
        "open_gripper",
        "retreat_goal",
    ]
    assert sequence[4].target == (
        DEFAULT_OUTER_ROUTE_X_M,
        DEFAULT_OUTER_ROUTE_Y_M,
        0.10,
    )
    assert sequence[5].target == (
        DEFAULT_OUTER_ROUTE_X_M,
        DEFAULT_OUTER_ROUTE_Y_M,
        0.045,
    )
    assert sequence[6].target == (DEFAULT_GOAL_X_M, DEFAULT_GOAL_Y_M, 0.045)


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
    assert "classify_radial_route" in node_source
    assert "duration = float(step.duration_s)" in node_source
    assert 'self.declare_parameter("gripper_settle_duration_s", 1.0)' in node_source
    assert "verify_endpoint=False" in node_source
    assert "gripper_final_tolerance_counts" not in node_source
    assert 'default_value="0.1812"' in launch_source
    assert 'default_value="-0.0336"' in launch_source
    assert 'default_value="0.17"' in launch_source
    assert 'default_value="0.20"' in launch_source
    assert 'default_value="0.1517"' in launch_source
    assert 'default_value="-0.0282"' in launch_source
    assert 'default_value="0.2107"' in launch_source
    assert 'default_value="-0.0391"' in launch_source
    assert 'executable="wp3_tsk2"' in launch_source
