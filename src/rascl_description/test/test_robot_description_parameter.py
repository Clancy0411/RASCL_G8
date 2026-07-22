"""Regression checks for xacro-generated robot_description parameters."""

import ast
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILES = (
    PACKAGE_ROOT / "launch" / "display.launch.py",
    PACKAGE_ROOT / "launch" / "ros2_control.launch.py",
)


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
