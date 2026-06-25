"""Minimum-jerk trajectory generation for WP3."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from typing import Iterable, List, Sequence

from .kinematics import JOINT_NAMES


@dataclass
class TrajectoryPoint:
    """One joint-space trajectory sample."""

    time_from_start: float
    positions: List[float]


def minimum_jerk_scalar(tau: float) -> float:
    """Return the normalized minimum-jerk interpolation value.

    tau must be in [0, 1].  The polynomial has zero velocity and zero
    acceleration at both endpoints, which is why it is suitable for gentle
    pick-and-place motions.
    """

    tau = min(max(float(tau), 0.0), 1.0)
    return 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5


def generate_joint_trajectory(
    q_start: Sequence[float],
    q_goal: Sequence[float],
    duration: float,
    rate_hz: float,
) -> List[TrajectoryPoint]:
    """Generate a sampled joint-space minimum-jerk trajectory."""

    if len(q_start) != len(q_goal):
        raise ValueError("q_start and q_goal must have the same length")
    if duration <= 0.0:
        raise ValueError("duration must be positive")
    if rate_hz <= 0.0:
        raise ValueError("rate_hz must be positive")

    sample_count = max(2, int(round(duration * rate_hz)) + 1)
    points: List[TrajectoryPoint] = []
    for sample_index in range(sample_count):
        t = min(sample_index / rate_hz, duration)
        tau = t / duration
        s = minimum_jerk_scalar(tau)
        q = [float(q_start[i]) + (float(q_goal[i]) - float(q_start[i])) * s for i in range(len(q_start))]
        points.append(TrajectoryPoint(t, q))

    # Ensure the final sample is exactly the requested goal even after rounding.
    points[-1] = TrajectoryPoint(duration, [float(value) for value in q_goal])
    return points


def write_csv(path: str, trajectory: Iterable[TrajectoryPoint], joint_names: Sequence[str] = JOINT_NAMES) -> None:
    """Write trajectory samples to CSV for documentation and debugging."""

    with open(path, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["time_from_start"] + list(joint_names))
        for point in trajectory:
            writer.writerow([f"{point.time_from_start:.6f}"] + [f"{value:.9f}" for value in point.positions])
