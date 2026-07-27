"""Regression checks for xacro-generated robot_description parameters."""

import ast
from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILES = (
    PACKAGE_ROOT / "launch" / "display.launch.py",
    PACKAGE_ROOT / "launch" / "ros2_control.launch.py",
)
ROS2_CONTROL_LAUNCH = PACKAGE_ROOT / "launch" / "ros2_control.launch.py"
HOMING_LAUNCH = PACKAGE_ROOT / "launch" / "homing.launch.py"
ROBOT_URDF = PACKAGE_ROOT / "urdf" / "rascl.urdf"


def _robot_description_value(path):
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue
        if not any(
            isinstance(target, ast.Name) and target.id == "robot_description"
            for target in node.targets
        ):
            continue
        assert isinstance(node.value, ast.Dict)
        for key, value in zip(node.value.keys, node.value.values):
            if isinstance(key, ast.Constant) and key.value == "robot_description":
                return value
    raise AssertionError(f"robot_description assignment not found in {path}")


def test_xacro_output_is_explicitly_typed_as_string():
    for path in LAUNCH_FILES:
        value = _robot_description_value(path)
        assert isinstance(value, ast.Call), path
        assert isinstance(value.func, ast.Name), path
        assert value.func.id == "ParameterValue", path
        value_type = next(
            (keyword.value for keyword in value.keywords if keyword.arg == "value_type"),
            None,
        )
        assert isinstance(value_type, ast.Name), path
        assert value_type.id == "str", path


def test_spur_gear_direction_defaults_are_consistently_reversed():
    tree = ast.parse(
        ROS2_CONTROL_LAUNCH.read_text(encoding="utf-8"),
        filename=str(ROS2_CONTROL_LAUNCH),
    )
    launch_default = None
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Name) or node.func.id != "DeclareLaunchArgument":
            continue
        if not node.args or not isinstance(node.args[0], ast.Constant):
            continue
        if node.args[0].value != "spur_gear_direction":
            continue
        launch_default = next(
            (
                keyword.value.value
                for keyword in node.keywords
                if keyword.arg == "default_value"
                and isinstance(keyword.value, ast.Constant)
            ),
            None,
        )
        break

    urdf_root = ET.parse(ROBOT_URDF).getroot()
    urdf_default = next(
        element.get("default")
        for element in urdf_root
        if element.tag.endswith("}arg") and element.get("name") == "gripper_direction"
    )
    assert launch_default == "-1"
    assert urdf_default == "-1"


def test_homing_motion_defaults_limit_transient_lag_and_interval_travel():
    tree = ast.parse(
        HOMING_LAUNCH.read_text(encoding="utf-8"),
        filename=str(HOMING_LAUNCH),
    )
    defaults = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Name) or node.func.id != "DeclareLaunchArgument":
            continue
        if not node.args or not isinstance(node.args[0], ast.Constant):
            continue
        default = next(
            (
                keyword.value.value
                for keyword in node.keywords
                if keyword.arg == "default_value"
                and isinstance(keyword.value, ast.Constant)
            ),
            None,
        )
        defaults[node.args[0].value] = default

    assert defaults["spur_gear_reference_timeout_s"] == "30.0"
    assert defaults["spur_gear_reference_profile_velocity"] == "3000"
    assert defaults["spur_gear_reference_profile_acceleration"] == "1000"
    assert defaults["spur_gear_reference_profile_deceleration"] == "1000"
    assert defaults["spur_gear_reference_following_error_confirm_s"] == "0.30"
    assert defaults["csp_torque_limit_per_mille"] == "1000"
    assert defaults["spur_close_torque_limit_per_mille"] == "100"
    assert defaults["homing_interval_max_travel_drive0_counts"] == "100000"
    assert defaults["homing_interval_max_travel_drive1_counts"] == "300000"
    assert defaults["homing_interval_max_travel_drive2_counts"] == "300000"
    assert defaults["homing_interval_timeout_s"] == "120.0"
    assert '"homing_midpoint_tolerance_counts": 500' in HOMING_LAUNCH.read_text(
        encoding="utf-8"
    )
