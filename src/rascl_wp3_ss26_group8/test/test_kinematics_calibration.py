"""Regression checks for the current measured TCP calibration."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

from rascl_wp3_ss26_group8.kinematics import (
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
    origin_text = tcp_joint.find("origin").get("xyz")
    urdf_origin = tuple(
        float(value)
        for value in origin_text.split()
    )
    _assert_vector_close(urdf_origin, TCP_ORIGIN_IN_LOWERARM)


def test_reference_tcp_positions_match_calibrated_geometry():
    _assert_vector_close(
        forward_tcp((0.0, 0.0, 0.0)),
        (0.29318978, -0.01580108, 0.07181469),
    )
    _assert_vector_close(
        forward_tcp((0.0, math.pi / 2.0, math.pi / 2.0)),
        (0.20318978, -0.01580108, 0.32181469),
    )


def test_grasp_center_extends_twenty_mm_from_measured_surface_point():
    # At this measured pose the calibrated gear-surface point was
    # [0.14, -0.16, 0.05] m.  Moving 20 mm along the local jaw direction gives
    # the fixed grasp center below.
    measured_joint_pose = (0.777575714851, 0.646147976068, 2.120655279445)
    _assert_vector_close(
        forward_tcp(measured_joint_pose),
        (0.14137022054036882, -0.16134895242506456, 0.030092640926086274),
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
