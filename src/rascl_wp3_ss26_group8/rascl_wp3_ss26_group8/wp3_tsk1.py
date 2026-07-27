#!/usr/bin/env python3
"""WP3 Task 1 first milestone: Cartesian target -> IK -> minimum-jerk motion.

This node is intentionally limited to one Cartesian target and does not control
an arbitrary end-effector orientation yet.  The first milestone is meant for
step-by-step validation in fake hardware/RViz before the full cube stacking
sequence is assembled.
"""

from __future__ import annotations

import os
import time
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from .kinematics import (
    ARM_JOINT_NAMES,
    JOINT_NAMES,
    NOMINAL_ZERO_TCP_IN_BASE_LINK,
    forward_tcp,
    inverse_tcp,
)
from .trajectory import generate_joint_trajectory, write_csv


class WP3Task1SingleTarget(Node):
    """Execute one minimum-jerk motion toward a base_link Cartesian target."""

    def __init__(self) -> None:
        super().__init__("wp3_tsk1")

        # Target position of the calibrated tcp_link in the base_link frame.
        self.declare_parameter("target_x", NOMINAL_ZERO_TCP_IN_BASE_LINK[0])
        self.declare_parameter("target_y", NOMINAL_ZERO_TCP_IN_BASE_LINK[1])
        self.declare_parameter("target_z", NOMINAL_ZERO_TCP_IN_BASE_LINK[2])

        # Match the default 20 ms ros2_control/CSP cycle so each offline sample
        # can become one cyclic position target without Profile Position motion
        # generation in the drive.
        self.declare_parameter("duration", 4.0)
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("position_tolerance", 0.002)

        # Safety/debug parameters.  execute=false performs IK and trajectory
        # generation only; it does not publish robot commands.
        self.declare_parameter("execute", False)
        self.declare_parameter("save_csv", True)
        self.declare_parameter("output_csv", "/tmp/rascl_wp3_tsk1_last_trajectory.csv")
        self.declare_parameter("final_hold_s", 0.5)
        self.declare_parameter("final_joint_tolerance_rad", 0.03)
        self.declare_parameter("final_tcp_tolerance_m", 0.01)
        self.declare_parameter("final_feedback_timeout_s", 2.0)
        self.declare_parameter("command_topic", "/rascl_position_controller/commands")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("joint_state_timeout_s", 5.0)

        self._latest_joint_state: Optional[JointState] = None
        self._latest_joint_state_received_at: Optional[float] = None
        joint_state_topic = self.get_parameter("joint_state_topic").value
        command_topic = self.get_parameter("command_topic").value

        self._command_publisher = self.create_publisher(Float64MultiArray, command_topic, 10)
        self._joint_state_subscription = self.create_subscription(
            JointState,
            joint_state_topic,
            self._joint_state_callback,
            10,
        )

        self.get_logger().info("WP3 Task 1 single-target minimum-jerk node started.")
        self.get_logger().info("Target frame: base_link, unit: meter.")
        self.get_logger().info(
            "TCP definition: fixed measured ideal tcp_link, 170 mm along "
            "lowerarm +X; independent of the physical spur_gear_joint origin."
        )
        self.get_logger().info(
            "Calibration convention: URDF q=[0,0,0,0] remains the physical model-zero pose "
            f"with nominal TCP {NOMINAL_ZERO_TCP_IN_BASE_LINK} m. The calibrated automatic-Home "
            "switch pose is nominally q=[0,+pi/2,+pi/2,0], not four zeros."
        )

    def _joint_state_callback(self, msg: JointState) -> None:
        self._latest_joint_state = msg
        self._latest_joint_state_received_at = time.monotonic()

    def _wait_for_joint_state(self) -> JointState:
        timeout_s = float(self.get_parameter("joint_state_timeout_s").value)
        deadline = time.monotonic() + timeout_s
        self.get_logger().info("Waiting for /joint_states ...")
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._latest_joint_state is not None:
                return self._latest_joint_state
        raise RuntimeError(f"No joint state received within {timeout_s:.1f} s")

    @staticmethod
    def _joint_positions_by_name(msg: JointState) -> Dict[str, float]:
        return {name: float(position) for name, position in zip(msg.name, msg.position)}

    def _extract_ordered_joint_positions(self, msg: JointState) -> List[float]:
        positions_by_name = self._joint_positions_by_name(msg)
        missing = [name for name in JOINT_NAMES if name not in positions_by_name]
        if missing:
            raise RuntimeError(f"/joint_states is missing required joint(s): {missing}")
        return [positions_by_name[name] for name in JOINT_NAMES]

    def plan(self) -> List[Float64MultiArray]:
        """Create trajectory commands from current joint state to Cartesian target."""

        target = (
            float(self.get_parameter("target_x").value),
            float(self.get_parameter("target_y").value),
            float(self.get_parameter("target_z").value),
        )
        duration = float(self.get_parameter("duration").value)
        rate_hz = float(self.get_parameter("rate_hz").value)
        tolerance = float(self.get_parameter("position_tolerance").value)

        joint_state = self._wait_for_joint_state()
        q_current = self._extract_ordered_joint_positions(joint_state)
        q_arm_current = q_current[:3]
        q_spur_current = q_current[3]
        tcp_current = forward_tcp(q_arm_current)

        self.get_logger().info(
            "Current joints [shoulder, upperarm, lowerarm, spur_gear] = "
            f"{[round(value, 5) for value in q_current]} rad"
        )
        self.get_logger().info(
            "Current TCP in base_link = "
            f"({tcp_current[0]:.4f}, {tcp_current[1]:.4f}, {tcp_current[2]:.4f}) m"
        )
        self.get_logger().info(
            f"Requested target TCP in base_link = ({target[0]:.4f}, {target[1]:.4f}, {target[2]:.4f}) m"
        )

        ik_result = inverse_tcp(target, seed=q_arm_current, tolerance=tolerance)
        self.get_logger().info(
            "IK result: "
            f"success={ik_result.success}, error={ik_result.error_norm:.5f} m, "
            f"q_arm={[round(value, 5) for value in ik_result.q]}, "
            f"fk=({ik_result.position[0]:.4f}, {ik_result.position[1]:.4f}, {ik_result.position[2]:.4f}) m"
        )
        if not ik_result.success:
            raise RuntimeError(
                "IK failed. The requested Cartesian target is probably outside the reachable workspace, "
                "too close to a singularity, or blocked by the current joint limits. "
                f"Best error was {ik_result.error_norm:.4f} m."
            )

        q_goal = [ik_result.q[0], ik_result.q[1], ik_result.q[2], q_spur_current]
        trajectory = generate_joint_trajectory(q_current, q_goal, duration, rate_hz)

        if bool(self.get_parameter("save_csv").value):
            output_csv = str(self.get_parameter("output_csv").value)
            os.makedirs(os.path.dirname(output_csv) or ".", exist_ok=True)
            write_csv(output_csv, trajectory)
            self.get_logger().info(f"Saved generated minimum-jerk trajectory to: {output_csv}")

        commands = []
        for point in trajectory:
            msg = Float64MultiArray()
            msg.data = point.positions
            commands.append(msg)
        self.get_logger().info(
            f"Generated {len(commands)} trajectory samples, duration={duration:.2f}s, rate={rate_hz:.2f}Hz."
        )
        return commands

    def execute(self, commands: List[Float64MultiArray]) -> None:
        """Publish the generated joint commands at the configured sample rate."""

        execute_motion = bool(self.get_parameter("execute").value)
        if not execute_motion:
            self.get_logger().warn(
                "execute=false: trajectory was generated but no command was published. "
                "Set execute:=true after checking the target and generated CSV."
            )
            return

        rate_hz = float(self.get_parameter("rate_hz").value)
        final_hold_s = float(self.get_parameter("final_hold_s").value)
        final_joint_tolerance = float(
            self.get_parameter("final_joint_tolerance_rad").value
        )
        final_tcp_tolerance = float(
            self.get_parameter("final_tcp_tolerance_m").value
        )
        final_feedback_timeout = float(
            self.get_parameter("final_feedback_timeout_s").value
        )
        if not commands:
            raise RuntimeError("Generated trajectory contains no commands")
        if final_joint_tolerance <= 0.0 or final_tcp_tolerance <= 0.0:
            raise ValueError("Final joint/TCP tolerances must be positive")
        if final_feedback_timeout <= 0.0:
            raise ValueError("final_feedback_timeout_s must be positive")
        dt = 1.0 / rate_hz

        self.get_logger().warn(
            "Publishing joint-space minimum-jerk command trajectory. "
            "Make sure fake hardware is active, or the real robot is calibrated and the area is clear."
        )

        next_time = time.monotonic()
        for command in commands:
            self._command_publisher.publish(command)
            rclpy.spin_once(self, timeout_sec=0.0)
            next_time += dt
            sleep_time = next_time - time.monotonic()
            if sleep_time > 0.0:
                time.sleep(sleep_time)

        if commands and final_hold_s > 0.0:
            end_time = time.monotonic() + final_hold_s
            while time.monotonic() < end_time:
                self._command_publisher.publish(commands[-1])
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(dt)

        # A completed publish loop is not proof that the drives reached the
        # endpoint. Require feedback newer than the final command and compare
        # both joint space and Cartesian TCP before reporting success.
        goal = [float(value) for value in commands[-1].data]
        target_tcp = (
            float(self.get_parameter("target_x").value),
            float(self.get_parameter("target_y").value),
            float(self.get_parameter("target_z").value),
        )
        fresh_after = time.monotonic()
        self._command_publisher.publish(commands[-1])
        feedback_deadline = fresh_after + final_feedback_timeout
        actual = None
        actual_tcp = None
        joint_errors = None
        tcp_error = None
        reached = False
        while rclpy.ok() and time.monotonic() < feedback_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (
                self._latest_joint_state is not None
                and self._latest_joint_state_received_at is not None
                and self._latest_joint_state_received_at >= fresh_after
            ):
                actual = self._extract_ordered_joint_positions(
                    self._latest_joint_state
                )
                joint_errors = [
                    goal_value - actual_value
                    for goal_value, actual_value in zip(goal, actual)
                ]
                actual_tcp = forward_tcp(actual[:3])
                tcp_error = sum(
                    (target_value - actual_value) ** 2
                    for target_value, actual_value in zip(target_tcp, actual_tcp)
                ) ** 0.5
                joint_ok = all(
                    abs(error) <= final_joint_tolerance for error in joint_errors
                )
                tcp_ok = tcp_error <= final_tcp_tolerance
                if joint_ok and tcp_ok:
                    reached = True
                    break
            # Continue holding the endpoint while waiting for convergence.
            self._command_publisher.publish(commands[-1])

        if actual is None:
            raise RuntimeError(
                "MOTION_RESULT reached=false reason=NO_FRESH_FINAL_JOINT_STATE"
            )

        result = (
            f"MOTION_RESULT reached={str(reached).lower()}; "
            f"goal_q={[round(value, 6) for value in goal]}; "
            f"actual_q={[round(value, 6) for value in actual]}; "
            f"joint_error_rad={[round(value, 6) for value in joint_errors]}; "
            f"target_tcp={[round(value, 6) for value in target_tcp]}; "
            f"actual_tcp={[round(value, 6) for value in actual_tcp]}; "
            f"tcp_error_m={tcp_error:.6f}; "
            f"limits(joint_rad/tcp_m)={final_joint_tolerance:.6f}/{final_tcp_tolerance:.6f}"
        )
        if not reached:
            self.get_logger().error(result)
            raise RuntimeError(
                "Final feedback did not reach the planned endpoint. "
                "The bridge will retain the CSP_STALL_SNAPSHOT for diagnosis."
            )

        self.get_logger().info(result)
        self.get_logger().info("Minimum-jerk motion reached the verified endpoint.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = WP3Task1SingleTarget()
        commands = node.plan()
        node.execute(commands)
    except Exception as exc:  # pylint: disable=broad-except
        exit_code = 1
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(f"wp3_tsk1 initialization failed: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)
