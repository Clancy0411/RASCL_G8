"""Source-level regression checks for debug group 15 gripper presets."""

from pathlib import Path


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

    assert "GRIPPER_GRIP_DELTA_COUNTS=-200000" in script
    assert "GRIPPER_RELEASE_DELTA_COUNTS=200000" in script


def test_group_15_has_no_contact_lag_early_stop_path():
    group_15 = _group_15_source()

    assert "stop_on_contact" not in group_15
    assert "GRIPPER_CONTACT_" not in group_15
    assert "enable_spur_close_guard" not in group_15
    assert "enable_spur_hold_guard" not in group_15
    assert "capture_spur_contact_snapshot" not in group_15
    assert "/rascl_faulhaber_bridge/restore_spur_torque" in group_15
