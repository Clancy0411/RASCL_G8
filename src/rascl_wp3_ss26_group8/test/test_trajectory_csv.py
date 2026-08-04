"""Tests for Task 1 offline trajectory CSV input and output."""

from pathlib import Path

import pytest

from rascl_wp3_ss26_group8.kinematics import JOINT_NAMES
from rascl_wp3_ss26_group8.trajectory import (
    generate_joint_trajectory,
    read_csv,
    write_csv,
)


def test_offline_trajectory_round_trip(tmp_path: Path):
    trajectory = generate_joint_trajectory(
        [0.0, 0.1, 0.2, 0.3],
        [0.4, 0.5, 0.6, 0.3],
        duration=1.0,
        rate_hz=10.0,
    )
    path = tmp_path / "task1_output.csv"
    write_csv(str(path), trajectory)

    loaded = read_csv(str(path))
    assert [point.time_from_start for point in loaded] == pytest.approx(
        [point.time_from_start for point in trajectory]
    )
    assert loaded[0].positions == pytest.approx(trajectory[0].positions)
    assert loaded[-1].positions == pytest.approx(trajectory[-1].positions)


def test_offline_trajectory_rejects_wrong_joint_order(tmp_path: Path):
    path = tmp_path / "wrong.csv"
    path.write_text(
        "time_from_start," + ",".join(reversed(JOINT_NAMES)) + "\n"
        "0.0,0,0,0,0\n"
        "1.0,0,0,0,0\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="header does not match"):
        read_csv(str(path))


def test_offline_trajectory_rejects_non_increasing_time(tmp_path: Path):
    path = tmp_path / "bad_time.csv"
    path.write_text(
        "time_from_start," + ",".join(JOINT_NAMES) + "\n"
        "0.0,0,0,0,0\n"
        "0.0,0,0,0,0\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="strictly increasing"):
        read_csv(str(path))


def test_wp3_task1_exposes_and_loads_offline_csv():
    package_root = Path(__file__).resolve().parents[1]
    node_source = (
        package_root / "rascl_wp3_ss26_group8" / "wp3_tsk1.py"
    ).read_text(encoding="utf-8")
    launch_source = (package_root / "launch" / "wp3_tsk1.launch.py").read_text(
        encoding="utf-8"
    )
    setup_source = (package_root / "setup.py").read_text(encoding="utf-8")
    script_source = (package_root.parents[1] / "rascl_debug.sh").read_text(
        encoding="utf-8"
    )

    assert 'self.declare_parameter("input_csv", "")' in node_source
    assert "trajectory = read_csv(input_csv)" in node_source
    assert '"input_csv": input_csv' in launch_source
    assert "glob('trajectories/*')" in setup_source
    assert '-p input_csv:="$TASK1_OUTPUT_CSV"' in script_source
    assert (
        'TASK1_OUTPUT_CSV="${RASCL_TASK1_OUTPUT_CSV:-$TRAJECTORY_DIR/'
        'task1_output.csv}"' in script_source
    )
