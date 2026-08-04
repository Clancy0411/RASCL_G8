"""Minimum-jerk trajectory generation for WP3."""

from __future__ import annotations

import csv
import math
import os
from dataclasses import dataclass
from typing import Iterable, List, Sequence

from .kinematics import JOINT_NAMES


TIME_COLUMN = "time_from_start"
SEGMENT_COLUMN = "segment"


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
        q = [
            float(q_start[i]) + (float(q_goal[i]) - float(q_start[i])) * s
            for i in range(len(q_start))
        ]
        points.append(TrajectoryPoint(t, q))

    # Ensure the final sample is exactly the requested goal even after rounding.
    points[-1] = TrajectoryPoint(duration, [float(value) for value in q_goal])
    return points


def write_csv(
    path: str,
    trajectory: Iterable[TrajectoryPoint],
    joint_names: Sequence[str] = JOINT_NAMES,
) -> None:
    """Write trajectory samples to CSV for documentation and debugging."""

    with open(path, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow([TIME_COLUMN] + list(joint_names))
        for point in trajectory:
            writer.writerow(
                [f"{point.time_from_start:.6f}"]
                + [f"{value:.9f}" for value in point.positions]
            )


def read_csv(
    path: str,
    joint_names: Sequence[str] = JOINT_NAMES,
) -> List[TrajectoryPoint]:
    """Load and validate an offline joint-space trajectory CSV."""

    expected_header = [TIME_COLUMN] + list(joint_names)
    points: List[TrajectoryPoint] = []
    with open(path, "r", newline="", encoding="utf-8") as csv_file:
        reader = csv.reader(csv_file)
        try:
            header = next(reader)
        except StopIteration as exc:
            raise ValueError(f"trajectory CSV is empty: {path}") from exc
        if header != expected_header:
            raise ValueError(
                "trajectory CSV header does not match the controller joint order: "
                f"expected {expected_header}, got {header}"
            )

        previous_time = -math.inf
        for row_number, row in enumerate(reader, start=2):
            if len(row) != len(expected_header):
                raise ValueError(
                    f"trajectory CSV row {row_number} has {len(row)} columns; "
                    f"expected {len(expected_header)}"
                )
            try:
                time_from_start = float(row[0])
                positions = [float(value) for value in row[1:]]
            except ValueError as exc:
                raise ValueError(
                    f"trajectory CSV row {row_number} contains a non-numeric value"
                ) from exc
            if not math.isfinite(time_from_start) or not all(
                math.isfinite(value) for value in positions
            ):
                raise ValueError(
                    f"trajectory CSV row {row_number} contains nan or infinity"
                )
            if not points and abs(time_from_start) > 1.0e-9:
                raise ValueError("trajectory CSV must start at time_from_start=0")
            if points and time_from_start <= previous_time:
                raise ValueError(
                    "trajectory CSV time_from_start values must be strictly increasing"
                )
            if time_from_start < 0.0:
                raise ValueError("trajectory CSV time_from_start cannot be negative")
            points.append(TrajectoryPoint(time_from_start, positions))
            previous_time = time_from_start

    if len(points) < 2:
        raise ValueError("trajectory CSV must contain at least two samples")
    return points


def write_segment_csv(
    path: str,
    segment: str,
    trajectory: Iterable[TrajectoryPoint],
    append: bool = True,
    joint_names: Sequence[str] = JOINT_NAMES,
) -> None:
    """Write one named trajectory into a multi-segment CSV.

    Every segment keeps its own ``time_from_start`` axis.  This lets a caller
    append all Task 1 arm legs to one file while loading and executing exactly
    one authorized leg at a time.
    """

    segment = str(segment).strip()
    if not segment:
        raise ValueError("trajectory segment name cannot be empty")

    points = list(trajectory)
    if len(points) < 2:
        raise ValueError("trajectory segment must contain at least two samples")

    expected_header = [SEGMENT_COLUMN, TIME_COLUMN] + list(joint_names)
    file_has_rows = append and os.path.exists(path) and os.path.getsize(path) > 0
    if file_has_rows:
        with open(path, "r", newline="", encoding="utf-8") as csv_file:
            reader = csv.reader(csv_file)
            header = next(reader)
            if header != expected_header:
                raise ValueError(
                    "segmented trajectory CSV header does not match the controller "
                    f"joint order: expected {expected_header}, got {header}"
                )
            if any(row and row[0] == segment for row in reader):
                raise ValueError(
                    f"trajectory segment already exists in CSV: {segment}"
                )

    mode = "a" if file_has_rows else "w"
    with open(path, mode, newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        if not file_has_rows:
            writer.writerow(expected_header)
        for point in points:
            writer.writerow(
                [segment, f"{point.time_from_start:.6f}"]
                + [f"{value:.9f}" for value in point.positions]
            )


def read_segment_csv(
    path: str,
    segment: str,
    joint_names: Sequence[str] = JOINT_NAMES,
) -> List[TrajectoryPoint]:
    """Load one named trajectory from a validated multi-segment CSV."""

    segment = str(segment).strip()
    if not segment:
        raise ValueError("trajectory segment name cannot be empty")

    expected_header = [SEGMENT_COLUMN, TIME_COLUMN] + list(joint_names)
    points: List[TrajectoryPoint] = []
    segment_names = set()
    previous_times = {}
    sample_counts = {}
    with open(path, "r", newline="", encoding="utf-8") as csv_file:
        reader = csv.reader(csv_file)
        try:
            header = next(reader)
        except StopIteration as exc:
            raise ValueError(f"trajectory CSV is empty: {path}") from exc
        if header != expected_header:
            raise ValueError(
                "segmented trajectory CSV header does not match the controller "
                f"joint order: expected {expected_header}, got {header}"
            )

        for row_number, row in enumerate(reader, start=2):
            if len(row) != len(expected_header):
                raise ValueError(
                    f"trajectory CSV row {row_number} has {len(row)} columns; "
                    f"expected {len(expected_header)}"
                )
            row_segment = row[0].strip()
            if not row_segment:
                raise ValueError(
                    f"trajectory CSV row {row_number} has an empty segment name"
                )
            try:
                time_from_start = float(row[1])
                positions = [float(value) for value in row[2:]]
            except ValueError as exc:
                raise ValueError(
                    f"trajectory CSV row {row_number} contains a non-numeric value"
                ) from exc
            if not math.isfinite(time_from_start) or not all(
                math.isfinite(value) for value in positions
            ):
                raise ValueError(
                    f"trajectory CSV row {row_number} contains nan or infinity"
                )
            if time_from_start < 0.0:
                raise ValueError("trajectory CSV time_from_start cannot be negative")

            count = sample_counts.get(row_segment, 0)
            if count == 0 and abs(time_from_start) > 1.0e-9:
                raise ValueError(
                    f"trajectory segment {row_segment!r} must start at "
                    "time_from_start=0"
                )
            if count and time_from_start <= previous_times[row_segment]:
                raise ValueError(
                    f"trajectory segment {row_segment!r} time_from_start values "
                    "must be strictly increasing"
                )

            segment_names.add(row_segment)
            previous_times[row_segment] = time_from_start
            sample_counts[row_segment] = count + 1
            if row_segment == segment:
                points.append(TrajectoryPoint(time_from_start, positions))

    if segment not in segment_names:
        available = ", ".join(sorted(segment_names)) or "none"
        raise ValueError(
            f"trajectory segment {segment!r} was not found; available segments: "
            f"{available}"
        )
    incomplete = sorted(name for name, count in sample_counts.items() if count < 2)
    if incomplete:
        raise ValueError(
            "each trajectory segment must contain at least two samples; invalid "
            f"segments: {incomplete}"
        )
    return points
