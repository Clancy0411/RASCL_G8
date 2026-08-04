"""Pure Task 2 validation and pick-and-place sequence generation."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple, Union


Vector3 = Tuple[float, float, float]

DEFAULT_GOAL_X_M = 0.18
DEFAULT_GOAL_Y_M = -0.04
DEFAULT_MIN_FEASIBLE_RADIUS_M = 0.10
DEFAULT_MAX_FEASIBLE_RADIUS_M = math.hypot(DEFAULT_GOAL_X_M, DEFAULT_GOAL_Y_M)
DEFAULT_SHOULDER_ANGLE_LIMIT_RAD = 0.5 * math.pi


@dataclass(frozen=True)
class CartesianStep:
    """One online-planned Cartesian waypoint."""

    label: str
    target: Vector3
    duration_s: float


@dataclass(frozen=True)
class GripperStep:
    """One relative Drive 3 gripper action."""

    label: str
    action: str
    duration_s: float


Task2Step = Union[CartesianStep, GripperStep]


def radial_distance(x: float, y: float) -> float:
    """Return the planar distance from the robot base."""

    return math.hypot(float(x), float(y))


def validate_task2_configuration(
    *,
    goal_x: float,
    goal_y: float,
    min_radius: float,
    max_radius: float,
    radius_tolerance: float = 1.0e-6,
) -> None:
    """Validate the declared feasible annulus and fixed goal.

    The WP3 task sheet requires the fixed goal to lie at the maximum feasible
    radius. The default maximum is therefore derived from the documented raw
    goal rather than from a second, corrected coordinate.
    """

    values = (goal_x, goal_y, min_radius, max_radius, radius_tolerance)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("Task 2 configuration values must be finite")
    if min_radius < 0.0:
        raise ValueError("minimum feasible radius cannot be negative")
    if max_radius <= min_radius:
        raise ValueError("maximum feasible radius must exceed the minimum")
    if radius_tolerance < 0.0:
        raise ValueError("radius tolerance cannot be negative")

    goal_radius = radial_distance(goal_x, goal_y)
    if abs(goal_radius - max_radius) > radius_tolerance:
        raise ValueError(
            "the Task 2 goal radius must equal the declared maximum feasible "
            f"radius: goal={goal_radius:.9f} m, maximum={max_radius:.9f} m"
        )


def validate_cube_center(
    x: float,
    y: float,
    *,
    min_radius: float,
    max_radius: float,
    shoulder_angle_limit_rad: float = DEFAULT_SHOULDER_ANGLE_LIMIT_RAD,
    tolerance: float = 1.0e-9,
) -> Tuple[float, float]:
    """Validate one runtime cube centre against the declared feasible region."""

    x = float(x)
    y = float(y)
    values = (x, y, min_radius, max_radius, shoulder_angle_limit_rad, tolerance)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("cube coordinates and feasible-region values must be finite")
    if min_radius < 0.0 or max_radius <= min_radius:
        raise ValueError("invalid feasible-radius interval")
    if shoulder_angle_limit_rad <= 0.0 or shoulder_angle_limit_rad > math.pi:
        raise ValueError("invalid shoulder-angle limit")

    radius = radial_distance(x, y)
    if radius < min_radius - tolerance or radius > max_radius + tolerance:
        raise ValueError(
            f"cube radius {radius:.6f} m is outside the declared feasible region "
            f"[{min_radius:.6f}, {max_radius:.6f}] m"
        )

    angle = math.atan2(y, x)
    if abs(angle) > shoulder_angle_limit_rad + tolerance:
        raise ValueError(
            f"cube angle {angle:.6f} rad is outside the allowed shoulder range "
            f"[-{shoulder_angle_limit_rad:.6f}, +{shoulder_angle_limit_rad:.6f}] rad"
        )
    return radius, angle


def build_pick_and_place_sequence(
    start_x: float,
    start_y: float,
    *,
    goal_x: float,
    goal_y: float,
    travel_z: float,
    pick_z: float,
    place_z: float,
    motion_duration_s: float,
    gripper_duration_s: float,
) -> Tuple[Task2Step, ...]:
    """Build the online Task 2 sequence for one published cube centre."""

    numeric_values = (
        start_x,
        start_y,
        goal_x,
        goal_y,
        travel_z,
        pick_z,
        place_z,
        motion_duration_s,
        gripper_duration_s,
    )
    if not all(math.isfinite(float(value)) for value in numeric_values):
        raise ValueError("Task 2 sequence values must be finite")
    if motion_duration_s <= 0.0 or gripper_duration_s <= 0.0:
        raise ValueError("Task 2 durations must be positive")
    if travel_z <= pick_z or travel_z <= place_z:
        raise ValueError("travel_z must be above both pick_z and place_z")

    start_x = float(start_x)
    start_y = float(start_y)
    goal_x = float(goal_x)
    goal_y = float(goal_y)
    motion_duration_s = float(motion_duration_s)
    gripper_duration_s = float(gripper_duration_s)

    return (
        CartesianStep("approach_pick", (start_x, start_y, travel_z), motion_duration_s),
        CartesianStep("descend_pick", (start_x, start_y, pick_z), motion_duration_s),
        GripperStep("close_gripper", "close", gripper_duration_s),
        CartesianStep("lift_pick", (start_x, start_y, travel_z), motion_duration_s),
        CartesianStep("approach_goal", (goal_x, goal_y, travel_z), motion_duration_s),
        CartesianStep("descend_goal", (goal_x, goal_y, place_z), motion_duration_s),
        GripperStep("open_gripper", "open", gripper_duration_s),
        CartesianStep("retreat_goal", (goal_x, goal_y, travel_z), motion_duration_s),
    )
