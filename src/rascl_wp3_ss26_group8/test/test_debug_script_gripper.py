"""Source-level regression checks for debug group 15 gripper presets."""

from pathlib import Path
import re


def _find_debug_script() -> Path:
    search_roots = (Path.cwd(), *Path.cwd().parents, *Path(__file__).resolve().parents)
    for root in search_roots:
        candidate = root / "rascl_debug.sh"
        if candidate.is_file():
            return candidate
    raise AssertionError("Could not locate rascl_debug.sh")


def _group_15_source() -> str:
    script = _find_debug_script().read_text(encoding="utf-8")
    start = script.index("group_gripper_action() {")
    end = script.index("\ngroup_set_target() {", start)
    return script[start:end]


def test_group_15_close_and_open_use_tested_exact_counts():
    script = _find_debug_script().read_text(encoding="utf-8")

    assert "GRIPPER_GRIP_DELTA_COUNTS=-150000" in script
    assert "GRIPPER_RELEASE_DELTA_COUNTS=150000" in script


def test_group_15_has_no_contact_lag_early_stop_path():
    group_15 = _group_15_source()

    assert "stop_on_contact" not in group_15
    assert "GRIPPER_CONTACT_" not in group_15
    assert "enable_spur_close_guard" not in group_15
    assert "enable_spur_hold_guard" not in group_15
    assert "capture_spur_contact_snapshot" not in group_15
    assert "/rascl_faulhaber_bridge/restore_spur_torque" in group_15


def test_group_23_reuses_target_plan_execute_without_changing_manual_groups():
    script = _find_debug_script().read_text(encoding="utf-8")
    start = script.index("group_target_plan_execute() {")
    end = script.index("\nprint_menu() {", start)
    group_23 = script[start:end]

    assert "group_set_target" in group_23
    assert "group_real_plan" in group_23
    assert "group_real_execute" in group_23
    assert '[[ ! -s "$PLAN_STATE_FILE" ]]' in group_23
    assert "23) group_target_plan_execute ;;" in script
    assert "9) group_real_plan ;;" in script
    assert "10) group_real_execute ;;" in script
    assert "14) group_set_target ;;" in script


def test_task1_stage_groups_preserve_all_fixed_actions_and_timing():
    script = _find_debug_script().read_text(encoding="utf-8")

    expected_stages = {
        "1": (
            ("0.16", "0.16", "0.10", "5"),
            ("0.16", "0.16", "0.05", "5"),
            ("0.16", "0.16", "0.15", "5"),
            ("0.0929", "-0.1327", "0.15", "5"),
            ("0.0929", "-0.1327", "0.05", "10"),
            ("0.07", "-0.10", "0.05", "5"),
            ("0.07", "-0.10", "0.15", "5"),
        ),
        "2": (
            ("0.17", "0.03", "0.15", "5"),
            ("0.17", "0.03", "0.085", "5"),
            ("0.17", "0.03", "0.15", "5"),
            ("0.18", "-0.04", "0.15", "5"),
            ("0.18", "-0.04", "0.05", "10"),
            ("0.18", "-0.04", "0.15", "5"),
        ),
        "3": (
            ("0.17", "0.03", "0.15", "5"),
            ("0.17", "0.03", "0.045", "5"),
            ("0.17", "0.03", "0.15", "5"),
            ("0.0642", "-0.0918", "0.15", "5"),
            ("0.0642", "-0.0918", "0.085", "10"),
            ("0.0642", "-0.0918", "0.15", "5"),
        ),
        "4": (
            ("0.18", "-0.04", "0.15", "5"),
            ("0.18", "-0.04", "0.045", "5"),
            ("0.18", "-0.04", "0.15", "5"),
            ("0.0642", "-0.0918", "0.15", "5"),
            ("0.0642", "-0.0918", "0.125", "10"),
        ),
    }

    for stage, waypoints in expected_stages.items():
        start = script.index(f"group_task1_stage_{stage}() {{")
        end = script.index("\n}\n", start) + 2
        stage_source = script[start:end]
        assert f"{24 + int(stage) - 1}) group_task1_stage_{stage} ;;" in script
        moves = re.findall(
            r'^\s*task1_move_to "[^"]+" ([^ ]+) ([^ ]+) ([^ ]+) ([^ ]+)$',
            stage_source,
            flags=re.MULTILINE,
        )
        assert moves == list(waypoints)
        assert re.findall(r"task1_gripper_preset (close|open) 5", stage_source) == ["close", "open"]
        assert "task1_wait_between_actions" not in stage_source


def test_task1_group_28_runs_all_stages_without_added_waits():
    script = _find_debug_script().read_text(encoding="utf-8")
    start = script.index("group_task1_all_stages() {")
    end = script.index("\n}\n", start) + 2
    group_28 = script[start:end]

    assert "28) group_task1_all_stages ;;" in script
    assert [line.strip() for line in group_28.splitlines() if line.strip().startswith("group_task1_stage_")] == [
        "group_task1_stage_1",
        "group_task1_stage_2",
        "group_task1_stage_3",
        "group_task1_stage_4",
    ]
    assert "sleep " not in group_28
    assert "task1_wait_between_actions" not in script


def test_task2_group_29_starts_the_required_online_ros_node():
    script = _find_debug_script().read_text(encoding="utf-8")
    start = script.index("group_task2_pick_and_place() {")
    end = script.index("\n}\n", start) + 2
    group_29 = script[start:end]

    assert "29) group_task2_pick_and_place ;;" in script
    assert "require_wp3_package" in group_29
    assert "require_csp_session" in group_29
    assert "require_active_controllers" in group_29
    assert "ros2 run rascl_wp3_ss26_group8 wp3_tsk2" in group_29
    assert "/goal_poses geometry_msgs/msg/Point" in group_29
    assert "-p execute:=true" in group_29
    assert "-p require_torque_service:=true" in group_29
    assert '-p output_directory:="$TASK2_OUTPUT_DIR"' in group_29
    assert 'read -r -p "Task 2 start x [m]: " x' not in group_29
