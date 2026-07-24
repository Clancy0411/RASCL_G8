"""Regression checks for the spur-gear-center TCP definition."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

from rascl_wp3_ss26_group8.kinematics import (
    BASE_TO_SHOULDER_ORIGIN,
    SPUR_GEAR_LIMIT,
    TCP_ORIGIN_IN_LOWERARM,
    forward_tcp,
    inverse_tcp,
)


def _assert_vector_close(actual, expected, tolerance=1e-8):
    assert len(actual) == len(expected)
    for actual_value, expected_value in zip(actual, expected):
        assert math.isclose(
            actual_value,
            expected_value,
            rel_tol=0.0,
            abs_tol=tolerance,
        )


def _find_urdf():
    search_roots = (
        Path.cwd(),
        *Path.cwd().parents,
        *Path(__file__).resolve().parents,
    )
    relative_paths = (
        Path("src/rascl_description/urdf/rascl.urdf"),
        Path("rascl_description/urdf/rascl.urdf"),
    )
    for root in search_roots:
        for relative_path in relative_paths:
            candidate = root / relative_path
            if candidate.is_file():
                return candidate
    raise AssertionError("Could not locate rascl_description/urdf/rascl.urdf")


def test_urdf_and_python_use_the_same_tcp_origin():
    root = ET.parse(_find_urdf()).getroot()
    tcp_joint = next(
        joint
        for joint in root.findall("joint")
        if joint.get("name") == "tcp_fixed_joint"
    )
    spur_joint = next(
        joint
        for joint in root.findall("joint")
        if joint.get("name") == "spur_gear_joint"
    )
    tcp_origin = tcp_joint.find("origin")
    spur_origin = spur_joint.find("origin")
    origin_text = tcp_origin.get("xyz")
    urdf_origin = tuple(
        float(value)
        for value in origin_text.split()
    )
    _assert_vector_close(urdf_origin, TCP_ORIGIN_IN_LOWERARM)
    _assert_vector_close(
        urdf_origin,
        tuple(float(value) for value in spur_origin.get("xyz").split()),
    )
    _assert_vector_close(
        tuple(float(value) for value in tcp_origin.get("rpy").split()),
        tuple(float(value) for value in spur_origin.get("rpy").split()),
    )


def test_urdf_and_python_use_the_same_base_calibration():
    _assert_vector_close(
        BASE_TO_SHOULDER_ORIGIN,
        (0.040, 0.020, 0.057441),
    )
    root = ET.parse(_find_urdf()).getroot()
    shoulder_joint = next(
        joint
        for joint in root.findall("joint")
        if joint.get("name") == "shoulder_joint"
    )
    origin_text = shoulder_joint.find("origin").get("xyz")
    urdf_origin = tuple(float(value) for value in origin_text.split())
    _assert_vector_close(urdf_origin, BASE_TO_SHOULDER_ORIGIN)


def test_spur_gear_limits_match_urdf_and_ros2_control():
    root = ET.parse(_find_urdf()).getroot()
    spur_joint = next(
        joint
        for joint in root.findall("joint")
        if joint.get("name") == "spur_gear_joint"
    )
    urdf_limit = spur_joint.find("limit")
    _assert_vector_close(
        (float(urdf_limit.get("lower")), float(urdf_limit.get("upper"))),
        SPUR_GEAR_LIMIT,
    )

    control_joint = next(
        joint
        for joint in root.find("ros2_control").findall("joint")
        if joint.get("name") == "spur_gear_joint"
    )
    control_params = {
        param.get("name"): float(param.text)
        for param in control_joint.findall("param")
        if param.get("name") in ("min_position", "max_position")
    }
    _assert_vector_close(
        (control_params["min_position"], control_params["max_position"]),
        SPUR_GEAR_LIMIT,
    )


def test_reference_tcp_positions_match_calibrated_geometry():
    _assert_vector_close(
        forward_tcp((0.0, 0.0, 0.0)),
        (0.33756, 0.01823, 0.043001),
    )
    _assert_vector_close(
        forward_tcp((0.0, math.pi / 2.0, math.pi / 2.0)),
        (0.24756, 0.01823, 0.293001),
    )


def test_spur_gear_center_tcp_at_measured_joint_pose():
    measured_joint_pose = (0.777575714851, 0.646147976068, 2.120655279445)
    _assert_vector_close(
        forward_tcp(measured_joint_pose),
        (0.1710751938715591, -0.1115242161906357, 0.022972507874931877),
    )


def test_inverse_kinematics_replans_the_original_cartesian_target():
    target = (0.16, -0.16, 0.05)
    result = inverse_tcp(
        target,
        seed=(0.777575714851, 0.646147976068, 2.120655279445),
        tolerance=1e-8,
        max_iterations=500,
    )
    assert result.success
    _assert_vector_close(result.position, target)
