"""Pure Task 2 validation and pick-and-place sequence generation."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple, Union


Vector3 = Tuple[float, float, float]

# Fixed placement input used by the original group-29 implementation.  Board
# compensation is still applied once inside wp3_tsk2 before IK.
DEFAULT_GOAL_X_M = 0.1812
DEFAULT_GOAL_Y_M = -0.0336
DEFAULT_MIN_FEASIBLE_RADIUS_M = 0.10
# Farthest labelled cube centre across the two physical box plates: (250, 60) mm.
DEFAULT_MAX_FEASIBLE_RADIUS_M = math.hypot(0.250, 0.060)
DEFAULT_SHOULDER_ANGLE_LIMIT_RAD = 0.5 * math.pi
DEFAULT_INNER_ROUTE_MAX_RADIUS_M = 0.17
DEFAULT_MIDDLE_ROUTE_MAX_RADIUS_M = 0.20
DEFAULT_INNER_ROUTE_X_M = 0.1517
DEFAULT_INNER_ROUTE_Y_M = -0.0282
DEFAULT_OUTER_ROUTE_X_M = 0.2107
DEFAULT_OUTER_ROUTE_Y_M = -0.0391

INNER_ROUTE = "inner"
MIDDLE_ROUTE = "middle"
OUTER_ROUTE = "outer"


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


def classify_radial_route(
    radius: float,
    *,
    inner_route_max_radius: float = DEFAULT_INNER_ROUTE_MAX_RADIUS_M,
    middle_route_max_radius: float = DEFAULT_MIDDLE_ROUTE_MAX_RADIUS_M,
) -> str:
    """Select the legacy group-29 route from the cube's planar radius."""

    radius = float(radius)
    inner_route_max_radius = float(inner_route_max_radius)
    middle_route_max_radius = float(middle_route_max_radius)
    values = (radius, inner_route_max_radius, middle_route_max_radius)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Task 2 route radii must be finite")
    if radius < 0.0:
        raise ValueError("cube radius cannot be negative")
    if inner_route_max_radius <= 0.0:
        raise ValueError("inner-route maximum radius must be positive")
    if middle_route_max_radius <= inner_route_max_radius:
        raise ValueError(
            "middle-route maximum radius must exceed the inner-route maximum"
        )

    if radius < inner_route_max_radius:
        return INNER_ROUTE
    if radius <= middle_route_max_radius:
        return MIDDLE_ROUTE
    return OUTER_ROUTE


def validate_task2_configuration(
    *,
    goal_x: float,
    goal_y: float,
    min_radius: float,
    max_radius: float,
    radius_tolerance: float = 1.0e-6,
    inner_route_max_radius: float = DEFAULT_INNER_ROUTE_MAX_RADIUS_M,
    middle_route_max_radius: float = DEFAULT_MIDDLE_ROUTE_MAX_RADIUS_M,
    inner_route_x: float = DEFAULT_INNER_ROUTE_X_M,
    inner_route_y: float = DEFAULT_INNER_ROUTE_Y_M,
    outer_route_x: float = DEFAULT_OUTER_ROUTE_X_M,
    outer_route_y: float = DEFAULT_OUTER_ROUTE_Y_M,
) -> None:
    """Validate the declared feasible annulus and fixed goal.

    The configured annulus follows the labelled cube positions on the physical
    box plates.  The fixed goal must lie inside that annulus.
    """

    values = (
        goal_x,
        goal_y,
        min_radius,
        max_radius,
        radius_tolerance,
        inner_route_x,
        inner_route_y,
        outer_route_x,
        outer_route_y,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("Task 2 configuration values must be finite")
    if min_radius < 0.0:
        raise ValueError("minimum feasible radius cannot be negative")
    if max_radius <= min_radius:
        raise ValueError("maximum feasible radius must exceed the minimum")
    if radius_tolerance < 0.0:
        raise ValueError("radius tolerance cannot be negative")

    classify_radial_route(
        0.0,
        inner_route_max_radius=inner_route_max_radius,
        middle_route_max_radius=middle_route_max_radius,
    )
    if inner_route_max_radius <= min_radius:
        raise ValueError(
            "inner-route maximum radius must exceed the feasible minimum"
        )
    if middle_route_max_radius >= max_radius:
        raise ValueError(
            "middle-route maximum radius must be below the feasible maximum"
        )
    goal_radius = radial_distance(goal_x, goal_y)
    if (
        goal_radius < min_radius - radius_tolerance
        or goal_radius > max_radius + radius_tolerance
    ):
        raise ValueError(
            "the Task 2 goal radius must lie inside the declared feasible region: "
            f"goal={goal_radius:.9f} m, region=[{min_radius:.9f}, {max_radius:.9f}] m"
        )
    for label, route_x, route_y in (
        (INNER_ROUTE, inner_route_x, inner_route_y),
        (OUTER_ROUTE, outer_route_x, outer_route_y),
    ):
        route_radius = radial_distance(route_x, route_y)
        if (
            route_radius < min_radius - radius_tolerance
            or route_radius > max_radius + radius_tolerance
        ):
            raise ValueError(
                f"{label}-route waypoint radius {route_radius:.9f} m lies outside "
                f"the feasible region [{min_radius:.9f}, {max_radius:.9f}] m"
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
    inner_route_max_radius: float = DEFAULT_INNER_ROUTE_MAX_RADIUS_M,
    middle_route_max_radius: float = DEFAULT_MIDDLE_ROUTE_MAX_RADIUS_M,
    inner_route_x: float = DEFAULT_INNER_ROUTE_X_M,
    inner_route_y: float = DEFAULT_INNER_ROUTE_Y_M,
    outer_route_x: float = DEFAULT_OUTER_ROUTE_X_M,
    outer_route_y: float = DEFAULT_OUTER_ROUTE_Y_M,
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
        inner_route_max_radius,
        middle_route_max_radius,
        inner_route_x,
        inner_route_y,
        outer_route_x,
        outer_route_y,
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
    inner_route_x = float(inner_route_x)
    inner_route_y = float(inner_route_y)
    outer_route_x = float(outer_route_x)
    outer_route_y = float(outer_route_y)
    motion_duration_s = float(motion_duration_s)
    gripper_duration_s = float(gripper_duration_s)
    route = classify_radial_route(
        radial_distance(start_x, start_y),
        inner_route_max_radius=inner_route_max_radius,
        middle_route_max_radius=middle_route_max_radius,
    )

    pickup_steps: Tuple[Task2Step, ...] = (
        CartesianStep("approach_pick", (start_x, start_y, travel_z), motion_duration_s),
        CartesianStep("descend_pick", (start_x, start_y, pick_z), motion_duration_s),
        GripperStep("close_gripper", "close", gripper_duration_s),
        CartesianStep("lift_pick", (start_x, start_y, travel_z), motion_duration_s),
    )

    if route == INNER_ROUTE:
        placement_steps: Tuple[Task2Step, ...] = (
            CartesianStep(
                "approach_inner",
                (inner_route_x, inner_route_y, travel_z),
                motion_duration_s,
            ),
            CartesianStep(
                "descend_inner",
                (inner_route_x, inner_route_y, place_z),
                motion_duration_s,
            ),
            CartesianStep(
                "push_outward_to_goal", (goal_x, goal_y, place_z), motion_duration_s
            ),
        )
    elif route == OUTER_ROUTE:
        placement_steps = (
            CartesianStep(
                "approach_outer",
                (outer_route_x, outer_route_y, travel_z),
                motion_duration_s,
            ),
            CartesianStep(
                "descend_outer",
                (outer_route_x, outer_route_y, place_z),
                motion_duration_s,
            ),
            CartesianStep(
                "pull_inward_to_goal", (goal_x, goal_y, place_z), motion_duration_s
            ),
        )
    else:
        placement_steps = (
            CartesianStep(
                "approach_goal", (goal_x, goal_y, travel_z), motion_duration_s
            ),
            CartesianStep(
                "descend_goal", (goal_x, goal_y, place_z), motion_duration_s
            ),
        )

    return pickup_steps + placement_steps + (
        GripperStep("open_gripper", "open", gripper_duration_s),
        CartesianStep("retreat_goal", (goal_x, goal_y, travel_z), motion_duration_s),
    )
