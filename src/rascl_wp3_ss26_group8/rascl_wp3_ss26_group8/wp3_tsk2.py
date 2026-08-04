"""WP3 Task 2 online pick-and-place node.

The node remains active, receives cube centres on ``/goal_poses`` as
``geometry_msgs/msg/Point``, plans each Cartesian waypoint from live feedback,
and processes published cubes sequentially.
"""

from __future__ import annotations

import csv
import math
import os
import queue
import threading
import time
from typing import List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from .kinematics import JOINT_NAMES, forward_tcp, inverse_tcp
from .task2_sequence import (
    DEFAULT_GOAL_X_M,
    DEFAULT_GOAL_Y_M,
    DEFAULT_MAX_FEASIBLE_RADIUS_M,
    DEFAULT_MIN_FEASIBLE_RADIUS_M,
    DEFAULT_SHOULDER_ANGLE_LIMIT_RAD,
    CartesianStep,
    GripperStep,
    build_pick_and_place_sequence,
    validate_cube_center,
    validate_task2_configuration,
)
from .trajectory import TrajectoryPoint, generate_joint_trajectory, write_csv
from .workspace_calibration import (
    DEFAULT_BOARD_XY_MATRIX,
    DEFAULT_BOARD_XY_OFFSET_M,
    board_xy_is_within_measured_bounds,
    compensate_board_xy,
)


JointVector = Tuple[float, float, float, float]
QueuedGoal = Optional[Tuple[int, float, float]]


class WP3Task2Online(Node):
    """Process runtime cube positions with online IK and minimum-jerk motion."""

    def __init__(self) -> None:
        super().__init__("wp3_tsk2")

        self.declare_parameter("goal_topic", "/goal_poses")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("command_topic", "/rascl_position_controller/commands")

        self.declare_parameter("goal_x", DEFAULT_GOAL_X_M)
        self.declare_parameter("goal_y", DEFAULT_GOAL_Y_M)
        self.declare_parameter("min_feasible_radius", DEFAULT_MIN_FEASIBLE_RADIUS_M)
        self.declare_parameter("max_feasible_radius", DEFAULT_MAX_FEASIBLE_RADIUS_M)
        self.declare_parameter(
            "shoulder_angle_limit_rad", DEFAULT_SHOULDER_ANGLE_LIMIT_RAD
        )

        self.declare_parameter("travel_z", 0.10)
        self.declare_parameter("pick_z", 0.045)
        self.declare_parameter("place_z", 0.045)
        self.declare_parameter("motion_duration", 5.0)
        self.declare_parameter("gripper_duration", 5.0)
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("position_tolerance", 0.002)
        self.declare_parameter("final_hold_s", 0.5)
        self.declare_parameter("final_joint_tolerance_rad", 0.03)
        self.declare_parameter("final_tcp_tolerance_m", 0.01)
        self.declare_parameter("joint_state_timeout_s", 5.0)
        self.declare_parameter("feedback_stale_timeout_s", 0.5)
        self.declare_parameter("final_feedback_timeout_s", 2.0)

        self.declare_parameter("apply_board_xy_compensation", True)
        self.declare_parameter(
            "board_xy_compensation_matrix", list(DEFAULT_BOARD_XY_MATRIX)
        )
        self.declare_parameter(
            "board_xy_compensation_offset_m", list(DEFAULT_BOARD_XY_OFFSET_M)
        )

        self.declare_parameter("gripper_close_counts", -150000)
        self.declare_parameter("gripper_open_counts", 150000)
        self.declare_parameter("spur_gear_direction", -1.0)
        self.declare_parameter("spur_gear_counts_per_revolution", 1323008.0)
        self.declare_parameter("spur_gear_min_position_rad", -2.0 * math.pi)
        self.declare_parameter("spur_gear_max_position_rad", 2.0 * math.pi)
        self.declare_parameter("spur_gear_speed_counts_per_s", 20000.0)
        self.declare_parameter("gripper_min_motion_duration_s", 0.5)
        self.declare_parameter("gripper_final_tolerance_counts", 5000.0)
        self.declare_parameter(
            "torque_service", "/rascl_faulhaber_bridge/restore_spur_torque"
        )
        self.declare_parameter("torque_service_timeout_s", 5.0)
        self.declare_parameter("require_torque_service", False)

        self.declare_parameter("execute", True)
        self.declare_parameter("save_csv", True)
        self.declare_parameter("output_directory", "/tmp/rascl_wp3_tsk2")
        self.declare_parameter("goal_queue_size", 10)

        self._validate_parameters()

        queue_size = int(self.get_parameter("goal_queue_size").value)
        self._goal_queue: queue.Queue[QueuedGoal] = queue.Queue(maxsize=queue_size)
        self._goal_counter = 0
        self._faulted = False
        self._stop_event = threading.Event()

        self._feedback_condition = threading.Condition()
        self._latest_positions: Optional[JointVector] = None
        self._latest_feedback_time: Optional[float] = None

        command_topic = str(self.get_parameter("command_topic").value)
        joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        goal_topic = str(self.get_parameter("goal_topic").value)
        torque_service = str(self.get_parameter("torque_service").value)

        self._command_publisher = self.create_publisher(
            Float64MultiArray, command_topic, 10
        )
        self._joint_state_subscription = self.create_subscription(
            JointState, joint_state_topic, self._joint_state_callback, 10
        )
        self._goal_subscription = self.create_subscription(
            Point, goal_topic, self._goal_callback, 10
        )
        self._torque_client = self.create_client(Trigger, torque_service)

        self._worker = threading.Thread(
            target=self._worker_main,
            name="wp3_tsk2_worker",
            daemon=True,
        )
        self._worker.start()

        min_radius = float(self.get_parameter("min_feasible_radius").value)
        max_radius = float(self.get_parameter("max_feasible_radius").value)
        goal_x = float(self.get_parameter("goal_x").value)
        goal_y = float(self.get_parameter("goal_y").value)
        execute = bool(self.get_parameter("execute").value)
        self.get_logger().info(
            "WP3 Task 2 online node ready: "
            f"topic={goal_topic}, feasible_radius=[{min_radius:.6f}, "
            f"{max_radius:.6f}] m, fixed_goal=({goal_x:.6f}, {goal_y:.6f}) m, "
            f"execute={execute}."
        )

    def _validate_parameters(self) -> None:
        goal_x = float(self.get_parameter("goal_x").value)
        goal_y = float(self.get_parameter("goal_y").value)
        min_radius = float(self.get_parameter("min_feasible_radius").value)
        max_radius = float(self.get_parameter("max_feasible_radius").value)
        validate_task2_configuration(
            goal_x=goal_x,
            goal_y=goal_y,
            min_radius=min_radius,
            max_radius=max_radius,
        )

        travel_z = float(self.get_parameter("travel_z").value)
        pick_z = float(self.get_parameter("pick_z").value)
        place_z = float(self.get_parameter("place_z").value)
        motion_duration = float(self.get_parameter("motion_duration").value)
        gripper_duration = float(self.get_parameter("gripper_duration").value)
        build_pick_and_place_sequence(
            max_radius,
            0.0,
            goal_x=goal_x,
            goal_y=goal_y,
            travel_z=travel_z,
            pick_z=pick_z,
            place_z=place_z,
            motion_duration_s=motion_duration,
            gripper_duration_s=gripper_duration,
        )

        positive_parameters = (
            "rate_hz",
            "position_tolerance",
            "final_joint_tolerance_rad",
            "final_tcp_tolerance_m",
            "joint_state_timeout_s",
            "feedback_stale_timeout_s",
            "final_feedback_timeout_s",
            "spur_gear_counts_per_revolution",
            "spur_gear_speed_counts_per_s",
            "gripper_min_motion_duration_s",
            "gripper_final_tolerance_counts",
            "torque_service_timeout_s",
        )
        for name in positive_parameters:
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be a finite number greater than zero")

        spur_direction = float(self.get_parameter("spur_gear_direction").value)
        spur_min = float(self.get_parameter("spur_gear_min_position_rad").value)
        spur_max = float(self.get_parameter("spur_gear_max_position_rad").value)
        if not math.isfinite(spur_direction) or spur_direction == 0.0:
            raise ValueError("spur_gear_direction must be finite and nonzero")
        if not math.isfinite(spur_min) or not math.isfinite(spur_max) or spur_min >= spur_max:
            raise ValueError("invalid Drive 3 position limits")

        queue_size = int(self.get_parameter("goal_queue_size").value)
        if queue_size <= 0:
            raise ValueError("goal_queue_size must be positive")

    @staticmethod
    def _positions_by_name(message: JointState) -> dict[str, float]:
        return dict(zip(message.name, message.position))

    def _joint_state_callback(self, message: JointState) -> None:
        positions_by_name = self._positions_by_name(message)
        if any(name not in positions_by_name for name in JOINT_NAMES):
            return
        positions = tuple(float(positions_by_name[name]) for name in JOINT_NAMES)
        if not all(math.isfinite(value) for value in positions):
            return
        with self._feedback_condition:
            self._latest_positions = positions  # type: ignore[assignment]
            self._latest_feedback_time = time.monotonic()
            self._feedback_condition.notify_all()

    def _goal_callback(self, message: Point) -> None:
        if self._faulted:
            self.get_logger().error(
                "Task 2 is faulted after an incomplete motion. Restart the node and "
                "the complete physical session before publishing another cube."
            )
            return

        x = float(message.x)
        y = float(message.y)
        min_radius = float(self.get_parameter("min_feasible_radius").value)
        max_radius = float(self.get_parameter("max_feasible_radius").value)
        angle_limit = float(self.get_parameter("shoulder_angle_limit_rad").value)
        try:
            radius, angle = validate_cube_center(
                x,
                y,
                min_radius=min_radius,
                max_radius=max_radius,
                shoulder_angle_limit_rad=angle_limit,
            )
        except ValueError as exc:
            self.get_logger().error(f"Rejected /goal_poses cube centre: {exc}")
            return

        self._goal_counter += 1
        goal = (self._goal_counter, x, y)
        try:
            self._goal_queue.put_nowait(goal)
        except queue.Full:
            self.get_logger().error(
                "Task 2 goal queue is full; the new cube position was rejected."
            )
            return

        if abs(float(message.z)) > 1.0e-9:
            self.get_logger().warn(
                "Task 2 uses the configured box-plate pick_z and ignores Point.z."
            )
        self.get_logger().info(
            f"Accepted cube {goal[0]}: centre=({x:.6f}, {y:.6f}) m, "
            f"radius={radius:.6f} m, angle={angle:.6f} rad."
        )

    def _worker_main(self) -> None:
        while not self._stop_event.is_set():
            try:
                queued_goal = self._goal_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                if queued_goal is None:
                    return
                job_id, start_x, start_y = queued_goal
                self._process_goal(job_id, start_x, start_y)
            except Exception as exc:  # noqa: BLE001 - hardware failures must be logged
                self._faulted = bool(self.get_parameter("execute").value)
                self.get_logger().error(f"TASK2_RESULT success=false error={exc}")
                if self._faulted:
                    self.get_logger().error(
                        "Further Task 2 goals are blocked to avoid continuing from "
                        "an unknown physical state."
                    )
            finally:
                self._goal_queue.task_done()

    def _process_goal(self, job_id: int, start_x: float, start_y: float) -> None:
        goal_x = float(self.get_parameter("goal_x").value)
        goal_y = float(self.get_parameter("goal_y").value)
        sequence = build_pick_and_place_sequence(
            start_x,
            start_y,
            goal_x=goal_x,
            goal_y=goal_y,
            travel_z=float(self.get_parameter("travel_z").value),
            pick_z=float(self.get_parameter("pick_z").value),
            place_z=float(self.get_parameter("place_z").value),
            motion_duration_s=float(self.get_parameter("motion_duration").value),
            gripper_duration_s=float(self.get_parameter("gripper_duration").value),
        )
        self._save_input(job_id, start_x, start_y, goal_x, goal_y)
        self.get_logger().info(
            f"Starting Task 2 job {job_id} with {len(sequence)} online-planned steps."
        )
        for step_index, step in enumerate(sequence, start=1):
            if self._stop_event.is_set():
                raise RuntimeError("Task 2 node is shutting down")
            if isinstance(step, CartesianStep):
                self._run_cartesian_step(job_id, step_index, step)
            elif isinstance(step, GripperStep):
                self._run_gripper_step(job_id, step_index, step)
            else:
                raise TypeError(f"unsupported Task 2 step: {step!r}")
        self.get_logger().info(
            f"TASK2_RESULT success=true job={job_id} start=({start_x:.6f}, "
            f"{start_y:.6f}) goal=({goal_x:.6f}, {goal_y:.6f})"
        )

    def _wait_for_joint_positions(self) -> JointVector:
        timeout_s = float(self.get_parameter("joint_state_timeout_s").value)
        stale_s = float(self.get_parameter("feedback_stale_timeout_s").value)
        deadline = time.monotonic() + timeout_s
        with self._feedback_condition:
            while not self._stop_event.is_set():
                now = time.monotonic()
                if (
                    self._latest_positions is not None
                    and self._latest_feedback_time is not None
                    and now - self._latest_feedback_time <= stale_s
                ):
                    return self._latest_positions
                remaining = deadline - now
                if remaining <= 0.0:
                    break
                self._feedback_condition.wait(timeout=min(remaining, 0.1))
        raise RuntimeError("fresh /joint_states feedback was not received in time")

    def _apply_board_compensation(self, requested: Sequence[float]) -> Tuple[float, float, float]:
        target = tuple(float(value) for value in requested)
        if not bool(self.get_parameter("apply_board_xy_compensation").value):
            return target  # type: ignore[return-value]

        matrix = self.get_parameter("board_xy_compensation_matrix").value
        offset = self.get_parameter("board_xy_compensation_offset_m").value
        corrected = compensate_board_xy(target, matrix, offset)
        if not board_xy_is_within_measured_bounds(target):
            self.get_logger().warn(
                "BOARD_XY_COMPENSATION_EXTRAPOLATION: requested Task 2 target "
                "lies outside the measured board-fit bounds."
            )
        self.get_logger().info(
            "BOARD_XY_COMPENSATION: "
            f"requested=({target[0]:.6f}, {target[1]:.6f}) m, "
            f"corrected=({corrected[0]:.6f}, {corrected[1]:.6f}) m."
        )
        return corrected

    def _run_cartesian_step(
        self, job_id: int, step_index: int, step: CartesianStep
    ) -> None:
        q_start = self._wait_for_joint_positions()
        ik_target = self._apply_board_compensation(step.target)
        tolerance = float(self.get_parameter("position_tolerance").value)
        ik_result = inverse_tcp(ik_target, seed=q_start[:3], tolerance=tolerance)
        if not ik_result.success:
            raise RuntimeError(
                f"{step.label} IK failed with {ik_result.error_norm:.6f} m error"
            )

        q_goal = (
            float(ik_result.q[0]),
            float(ik_result.q[1]),
            float(ik_result.q[2]),
            q_start[3],
        )
        trajectory = generate_joint_trajectory(
            q_start,
            q_goal,
            step.duration_s,
            float(self.get_parameter("rate_hz").value),
        )
        self._save_trajectory(job_id, step_index, step.label, trajectory)
        self.get_logger().info(
            f"Task 2 job {job_id} step {step_index}: {step.label}, "
            f"requested={step.target}, ik_target={ik_target}, duration={step.duration_s:.3f} s."
        )
        self._execute_trajectory(step.label, trajectory, expected_tcp=ik_target)

    def _run_gripper_step(
        self, job_id: int, step_index: int, step: GripperStep
    ) -> None:
        self._restore_spur_torque()
        q_start = self._wait_for_joint_positions()
        if step.action == "close":
            delta_counts = int(self.get_parameter("gripper_close_counts").value)
        elif step.action == "open":
            delta_counts = int(self.get_parameter("gripper_open_counts").value)
        else:
            raise ValueError(f"unknown gripper action: {step.action}")

        direction = float(self.get_parameter("spur_gear_direction").value)
        counts_per_revolution = float(
            self.get_parameter("spur_gear_counts_per_revolution").value
        )
        target_spur = (
            q_start[3]
            + direction * delta_counts * 2.0 * math.pi / counts_per_revolution
        )
        spur_min = float(self.get_parameter("spur_gear_min_position_rad").value)
        spur_max = float(self.get_parameter("spur_gear_max_position_rad").value)
        if not spur_min <= target_spur <= spur_max:
            raise RuntimeError(
                f"{step.action} requests Drive 3 position {target_spur:.6f} rad "
                f"outside [{spur_min:.6f}, {spur_max:.6f}] rad"
            )

        speed = float(self.get_parameter("spur_gear_speed_counts_per_s").value)
        minimum_duration = float(
            self.get_parameter("gripper_min_motion_duration_s").value
        )
        duration = max(step.duration_s, abs(delta_counts) / speed, minimum_duration)
        q_goal = (q_start[0], q_start[1], q_start[2], target_spur)
        trajectory = generate_joint_trajectory(
            q_start,
            q_goal,
            duration,
            float(self.get_parameter("rate_hz").value),
        )
        self._save_trajectory(job_id, step_index, step.label, trajectory)
        self.get_logger().info(
            f"Task 2 job {job_id} step {step_index}: {step.label}, "
            f"delta={delta_counts} counts, duration={duration:.3f} s."
        )

        count_tolerance = float(
            self.get_parameter("gripper_final_tolerance_counts").value
        )
        gripper_tolerance = count_tolerance * 2.0 * math.pi / counts_per_revolution
        joint_tolerance = min(
            float(self.get_parameter("final_joint_tolerance_rad").value),
            gripper_tolerance,
        )
        self._execute_trajectory(
            step.label,
            trajectory,
            expected_tcp=None,
            joint_tolerance_override=joint_tolerance,
        )

    def _restore_spur_torque(self) -> None:
        if not bool(self.get_parameter("execute").value):
            return
        timeout_s = float(self.get_parameter("torque_service_timeout_s").value)
        required = bool(self.get_parameter("require_torque_service").value)
        if not self._torque_client.wait_for_service(timeout_sec=timeout_s):
            if required:
                raise RuntimeError("Drive 3 torque-protection service is unavailable")
            self.get_logger().warn(
                "Drive 3 torque-protection service is unavailable; continuing "
                "because require_torque_service=false (expected with fake hardware)."
            )
            return

        future = self._torque_client.call_async(Trigger.Request())
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        if not completed.wait(timeout_s):
            raise RuntimeError("Drive 3 torque-protection service timed out")
        response = future.result()
        if response is None or not response.success:
            message = "no response" if response is None else response.message
            raise RuntimeError(
                f"Drive 3 torque protection was not restored: {message}"
            )

    def _publish_joint_command(self, positions: Sequence[float]) -> None:
        message = Float64MultiArray()
        message.data = [float(value) for value in positions]
        self._command_publisher.publish(message)

    def _execute_trajectory(
        self,
        label: str,
        trajectory: List[TrajectoryPoint],
        *,
        expected_tcp: Optional[Tuple[float, float, float]],
        joint_tolerance_override: Optional[float] = None,
    ) -> None:
        if not trajectory:
            raise RuntimeError(f"{label} generated an empty trajectory")
        if not bool(self.get_parameter("execute").value):
            self.get_logger().warn(
                f"execute=false: planned {label} but published no commands."
            )
            return
        if self._command_publisher.get_subscription_count() < 1:
            raise RuntimeError(
                "no controller subscribes to the configured position-command topic"
            )

        rate_hz = float(self.get_parameter("rate_hz").value)
        period_s = 1.0 / rate_hz
        stale_s = float(self.get_parameter("feedback_stale_timeout_s").value)
        next_tick = time.monotonic()
        for point in trajectory:
            if self._stop_event.is_set():
                raise RuntimeError("Task 2 motion interrupted by node shutdown")
            self._publish_joint_command(point.positions)
            with self._feedback_condition:
                feedback_time = self._latest_feedback_time
            if feedback_time is None or time.monotonic() - feedback_time > stale_s:
                raise RuntimeError(f"/joint_states stopped during {label}")
            next_tick += period_s
            time.sleep(max(0.0, next_tick - time.monotonic()))

        goal = tuple(float(value) for value in trajectory[-1].positions)
        hold_deadline = time.monotonic() + float(
            self.get_parameter("final_hold_s").value
        )
        while time.monotonic() < hold_deadline:
            self._publish_joint_command(goal)
            time.sleep(period_s)

        joint_tolerance = (
            float(joint_tolerance_override)
            if joint_tolerance_override is not None
            else float(self.get_parameter("final_joint_tolerance_rad").value)
        )
        tcp_tolerance = float(self.get_parameter("final_tcp_tolerance_m").value)
        feedback_timeout = float(
            self.get_parameter("final_feedback_timeout_s").value
        )
        fresh_after = time.monotonic()
        deadline = fresh_after + feedback_timeout
        last_joint_error: Optional[float] = None
        last_tcp_error: Optional[float] = None
        while time.monotonic() < deadline:
            self._publish_joint_command(goal)
            with self._feedback_condition:
                actual = self._latest_positions
                received_at = self._latest_feedback_time
            if actual is not None and received_at is not None and received_at >= fresh_after:
                joint_errors = [
                    goal_value - actual_value
                    for goal_value, actual_value in zip(goal, actual)
                ]
                last_joint_error = max(abs(error) for error in joint_errors)
                joint_ok = last_joint_error <= joint_tolerance
                tcp_ok = True
                if expected_tcp is not None:
                    actual_tcp = forward_tcp(actual[:3])
                    last_tcp_error = math.sqrt(
                        sum(
                            (target_value - actual_value) ** 2
                            for target_value, actual_value in zip(
                                expected_tcp, actual_tcp
                            )
                        )
                    )
                    tcp_ok = last_tcp_error <= tcp_tolerance
                if joint_ok and tcp_ok:
                    self.get_logger().info(
                        f"MOTION_RESULT reached=true label={label} "
                        f"max_joint_error={last_joint_error:.6f} rad "
                        f"tcp_error={last_tcp_error if last_tcp_error is not None else 'n/a'}"
                    )
                    return
            time.sleep(period_s)

        raise RuntimeError(
            f"MOTION_RESULT reached=false label={label} "
            f"max_joint_error={last_joint_error} tcp_error={last_tcp_error}"
        )

    def _output_directory(self) -> str:
        output_directory = str(self.get_parameter("output_directory").value)
        if not output_directory:
            raise ValueError("output_directory cannot be empty")
        os.makedirs(output_directory, exist_ok=True)
        return output_directory

    def _save_input(
        self,
        job_id: int,
        start_x: float,
        start_y: float,
        goal_x: float,
        goal_y: float,
    ) -> None:
        if not bool(self.get_parameter("save_csv").value):
            return
        path = os.path.join(self._output_directory(), f"job_{job_id:04d}_input.csv")
        with open(path, "w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["start_x", "start_y", "goal_x", "goal_y"])
            writer.writerow([start_x, start_y, goal_x, goal_y])

    def _save_trajectory(
        self,
        job_id: int,
        step_index: int,
        label: str,
        trajectory: List[TrajectoryPoint],
    ) -> None:
        if not bool(self.get_parameter("save_csv").value):
            return
        path = os.path.join(
            self._output_directory(),
            f"job_{job_id:04d}_step_{step_index:02d}_{label}.csv",
        )
        write_csv(path, trajectory)

    def stop_worker(self) -> None:
        """Stop the worker without starting another physical action."""

        self._stop_event.set()
        try:
            self._goal_queue.put_nowait(None)
        except queue.Full:
            pass
        with self._feedback_condition:
            self._feedback_condition.notify_all()
        self._worker.join(timeout=3.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[WP3Task2Online] = None
    try:
        node = WP3Task2Online()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop_worker()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
