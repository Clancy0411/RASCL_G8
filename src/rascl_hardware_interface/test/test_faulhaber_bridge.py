"""Software-only checks for the FAULHABER CSP/PDO bridge."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import time
import types
import unittest
from unittest import mock


def _load_bridge_module():
    pysoem = types.ModuleType("pysoem")
    pysoem.Master = object
    pysoem.SAFEOP_STATE = 0x04
    pysoem.OP_STATE = 0x08
    sys.modules.setdefault("pysoem", pysoem)

    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = object
    rclpy.node = rclpy_node
    sys.modules.setdefault("rclpy", rclpy)
    sys.modules.setdefault("rclpy.node", rclpy_node)

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Bool = type("Bool", (), {})
    sys.modules.setdefault("std_msgs", std_msgs)
    sys.modules.setdefault("std_msgs.msg", std_msgs_msg)

    std_srvs = types.ModuleType("std_srvs")
    std_srvs_srv = types.ModuleType("std_srvs.srv")
    std_srvs_srv.Trigger = type("Trigger", (), {})
    sys.modules.setdefault("std_srvs", std_srvs)
    sys.modules.setdefault("std_srvs.srv", std_srvs_srv)

    path = Path(__file__).parents[1] / "scripts" / "rascl_faulhaber_bridge.py"
    spec = importlib.util.spec_from_file_location("rascl_faulhaber_bridge_test", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


bridge = _load_bridge_module()


class FakeSlave:
    def __init__(self):
        self.values = {
            (bridge.POSITION_RXPDO, 0): bytes([2]),
            (bridge.POSITION_RXPDO, 1): bridge.POSITION_RXPDO_ENTRIES[0].to_bytes(4, "little"),
            (bridge.POSITION_RXPDO, 2): bridge.POSITION_RXPDO_ENTRIES[1].to_bytes(4, "little"),
            (bridge.POSITION_TXPDO, 0): bytes([2]),
            (bridge.POSITION_TXPDO, 1): bridge.POSITION_TXPDO_ENTRIES[0].to_bytes(4, "little"),
            (bridge.POSITION_TXPDO, 2): bridge.POSITION_TXPDO_ENTRIES[1].to_bytes(4, "little"),
        }
        self.writes = []
        self.name = "fake-drive"
        self.output = bytes(bridge.PDO_RX_SIZE_BYTES)
        self.input = (
            int(0x0040).to_bytes(2, "little")
            + int(0).to_bytes(4, "little", signed=True)
        )
        self.values[(bridge.STATUS_WORD, 0)] = int(0x0040).to_bytes(2, "little")
        self.values[(bridge.ACTUAL_POSITION, 0)] = int(0).to_bytes(
            4, "little", signed=True
        )
        self.values[(bridge.MODE_DISPLAY, 0)] = int(0).to_bytes(1, "little", signed=True)
        self.values[(bridge.SM2_PARAMETERS, bridge.SM_CYCLE_TIME_SUBINDEX)] = int(0).to_bytes(
            4, "little"
        )
        self.values[(bridge.POSITION_RANGE_LIMIT, 1)] = int(-(1 << 31)).to_bytes(
            4, "little", signed=True
        )
        self.values[(bridge.POSITION_RANGE_LIMIT, 2)] = int((1 << 31) - 1).to_bytes(
            4, "little", signed=True
        )
        self.values[(bridge.SOFTWARE_POSITION_LIMIT, 1)] = int(-(1 << 31)).to_bytes(
            4, "little", signed=True
        )
        self.values[(bridge.SOFTWARE_POSITION_LIMIT, 2)] = int((1 << 31) - 1).to_bytes(
            4, "little", signed=True
        )
        self.values[(bridge.FOLLOWING_ERROR_WINDOW, 0)] = int(32).to_bytes(4, "little")
        self.values[(bridge.FOLLOWING_ERROR_TIMEOUT, 0)] = int(48).to_bytes(2, "little")
        self.values[(bridge.ERROR_REGISTER, 0)] = int(0).to_bytes(1, "little")
        self.values[(bridge.PREDEFINED_ERROR_FIELD, 0)] = int(0).to_bytes(1, "little")
        self.values[
            (bridge.DIGITAL_INPUT_SETTINGS, bridge.LOWER_LIMIT_SWITCH_INPUTS)
        ] = int(0).to_bytes(1, "little")
        self.values[
            (bridge.DIGITAL_INPUT_SETTINGS, bridge.UPPER_LIMIT_SWITCH_INPUTS)
        ] = int(0).to_bytes(1, "little")
        self.values[
            (bridge.DIGITAL_INPUT_SETTINGS, bridge.LIMIT_SWITCH_OPTION_CODE)
        ] = int(2).to_bytes(1, "little", signed=True)
        self.values[
            (bridge.DIGITAL_INPUT_SETTINGS, bridge.REFERENCE_SWITCH_INPUT)
        ] = int(2).to_bytes(1, "little")
        self.values[
            (bridge.DIGITAL_INPUT_SETTINGS, bridge.INPUT_POLARITY)
        ] = int(0).to_bytes(1, "little")
        self.values[
            (bridge.DIGITAL_IO_STATUS, bridge.DIGITAL_INPUT_LOGICAL)
        ] = int(0).to_bytes(1, "little")
        self.values[
            (bridge.DIGITAL_IO_STATUS, bridge.DIGITAL_INPUT_PHYSICAL)
        ] = int(0).to_bytes(1, "little")
        self.values[(bridge.DEVICE_STATUS, 1)] = int(0).to_bytes(4, "little")
        self.values[(bridge.POSITION_DEMAND_VALUE, 0)] = int(0).to_bytes(
            4, "little", signed=True
        )
        self.values[(bridge.FOLLOWING_ERROR_ACTUAL_VALUE, 0)] = int(0).to_bytes(
            4, "little"
        )
        self.values[(bridge.VELOCITY_ACTUAL_VALUE, 0)] = int(0).to_bytes(
            4, "little", signed=True
        )
        self.values[(bridge.ACTUAL_TORQUE, 0)] = int(0).to_bytes(2, "little", signed=True)
        self.values[(bridge.TORQUE_DEMAND, 0)] = int(0).to_bytes(2, "little", signed=True)
        self.values[(bridge.ACTUAL_CURRENT, 0)] = int(0).to_bytes(2, "little", signed=True)
        self.values[(bridge.MAX_TORQUE, 0)] = int(6000).to_bytes(2, "little")
        self.values[(bridge.POSITIVE_TORQUE_LIMIT, 0)] = int(6000).to_bytes(2, "little")
        self.values[(bridge.NEGATIVE_TORQUE_LIMIT, 0)] = int(6000).to_bytes(2, "little")
        self.values[(bridge.MAX_MOTOR_SPEED, 0)] = int(32767).to_bytes(4, "little")
        self.values[(bridge.POSITION_CONTROL_PARAMETER_SET, 1)] = int(30).to_bytes(
            1, "little"
        )
        self.values[(bridge.MOTOR_APPLICATION_DATA, 1)] = int(1000).to_bytes(2, "little")
        self.values[(bridge.MOTOR_APPLICATION_DATA, 2)] = int(1000).to_bytes(2, "little")
        self.values[(bridge.MOTOR_APPLICATION_DATA, 3)] = int(2000).to_bytes(2, "little")
        for subindex, value in enumerate(
            [1200, 1200, 5200, 5200, 200, 2400, 2400], start=1
        ):
            self.values[(bridge.VOLTAGE_MONITOR, subindex)] = int(value).to_bytes(
                2, "little"
            )

    def sdo_read(self, index, subindex):
        return self.values[(index, subindex)]

    def sdo_write(self, index, subindex, payload):
        self.writes.append((index, subindex, bytes(payload)))
        self.values[(index, subindex)] = bytes(payload)
        if (index, subindex) == (bridge.MODE_OF_OPERATION, 0):
            self.values[(bridge.MODE_DISPLAY, 0)] = bytes(payload)
        if (index, subindex) == (bridge.MOTOR_APPLICATION_DATA, 3):
            rated_current = int.from_bytes(
                self.values[(bridge.MOTOR_APPLICATION_DATA, 1)], "little"
            )
            peak_current = int.from_bytes(payload, "little")
            effective_maximum = peak_current * 1000 // rated_current
            self.values[(bridge.MAX_TORQUE, 0)] = effective_maximum.to_bytes(
                2, "little"
            )
        if index == bridge.DIGITAL_INPUT_SETTINGS and subindex in (
            bridge.LOWER_LIMIT_SWITCH_INPUTS,
            bridge.UPPER_LIMIT_SWITCH_INPUTS,
        ):
            lower = int.from_bytes(
                self.values[
                    (bridge.DIGITAL_INPUT_SETTINGS, bridge.LOWER_LIMIT_SWITCH_INPUTS)
                ],
                "little",
            )
            upper = int.from_bytes(
                self.values[
                    (bridge.DIGITAL_INPUT_SETTINGS, bridge.UPPER_LIMIT_SWITCH_INPUTS)
                ],
                "little",
            )
            if lower == 0 and upper == 0:
                device_status = int.from_bytes(
                    self.values[(bridge.DEVICE_STATUS, 1)], "little"
                )
                device_status &= ~((1 << 6) | (1 << 7))
                self.values[(bridge.DEVICE_STATUS, 1)] = device_status.to_bytes(
                    4, "little"
                )


class FakeMaster:
    def __init__(self, slaves):
        self.slaves = slaves
        self.state = bridge.pysoem.SAFEOP_STATE
        self.expected_wkc = 3 * len(slaves)
        self.config_map_calls = 0
        self.controlwords = []
        self.write_state_calls = 0

    def open(self, _interface):
        return None

    def config_init(self):
        return len(self.slaves)

    def config_map(self):
        self.config_map_calls += 1
        return 6 * len(self.slaves) * 2

    def state_check(self, expected_state, _timeout):
        if expected_state == bridge.pysoem.SAFEOP_STATE:
            return bridge.pysoem.SAFEOP_STATE
        return self.state

    def write_state(self):
        self.write_state_calls += 1
        return 1

    def send_processdata(self):
        return 1

    def receive_processdata(self, _timeout):
        for slave in self.slaves:
            controlword, target = bridge.struct.unpack("<Hi", slave.output)
            self.controlwords.append(controlword)
            if controlword == bridge.CMD_SHUTDOWN:
                status = bridge.STATUS_READY_TO_SWITCH_ON
            elif controlword == bridge.CMD_SWITCH_ON:
                status = bridge.STATUS_SWITCHED_ON
            elif controlword == bridge.CMD_ENABLE_OPERATION:
                status = (
                    bridge.STATUS_OPERATION_ENABLED_STATE
                    | bridge.STATUS_CSP_TARGET_ACCEPTED
                )
            else:
                status = 0x0040
            slave.input = bridge.struct.pack("<Hi", status, target)
            slave.values[(bridge.STATUS_WORD, 0)] = status.to_bytes(2, "little")
            slave.values[(bridge.ACTUAL_POSITION, 0)] = target.to_bytes(
                4, "little", signed=True
            )
        return self.expected_wkc

    def close(self):
        return None


def make_bus(**overrides):
    arguments = dict(
        interface="dummy0",
        slave_indices=[0, 1, 2, 3],
        sdo_delay_s=0.0,
        verbose=False,
        control_mode="csp",
        pdo_cycle_ns=20_000_000,
        pdo_timeout_us=5_000,
        enable_dc_sync=False,
    )
    arguments.update(overrides)
    return bridge.FaulhaberBus(**arguments)


class BridgePDOTest(unittest.TestCase):
    def test_drive3_reference_move_is_relative_to_live_counts(self):
        drive = bridge.FaulhaberDrive(
            FakeSlave(), drive_id=3, sdo_delay_s=0.0, verbose=False
        )
        with (
            mock.patch.object(
                drive,
                "read_actual_position_counts",
                side_effect=[120_000, 169_950],
            ),
            mock.patch.object(
                drive,
                "read_status",
                return_value=(
                    bridge.STATUS_OPERATION_ENABLED_STATE
                    | bridge.STATUS_TARGET_REACHED
                ),
            ),
            mock.patch.object(drive, "move_absolute_counts") as move_absolute,
        ):
            source, target, actual = drive.move_relative_counts_and_wait(
                50_000, timeout_s=1.0, tolerance_counts=100
            )

        self.assertEqual((source, target, actual), (120_000, 170_000, 169_950))
        move_absolute.assert_called_once_with(170_000)

    def test_drive3_reference_allows_a_transient_following_error(self):
        drive = bridge.FaulhaberDrive(
            FakeSlave(), drive_id=3, sdo_delay_s=0.0, verbose=False
        )
        following_status = (
            bridge.STATUS_OPERATION_ENABLED_STATE
            | bridge.STATUS_FOLLOWING_OR_HOMING_ERROR
        )
        reached_status = (
            bridge.STATUS_OPERATION_ENABLED_STATE
            | bridge.STATUS_TARGET_REACHED
        )
        with (
            mock.patch.object(
                drive,
                "read_actual_position_counts",
                side_effect=[120_000, 140_000, 170_000],
            ),
            mock.patch.object(
                drive,
                "read_status",
                side_effect=[following_status, following_status, reached_status],
            ),
            mock.patch.object(drive, "move_absolute_counts") as move_absolute,
        ):
            source, target, actual = drive.move_relative_counts_and_wait(
                50_000,
                timeout_s=1.0,
                tolerance_counts=100,
                following_error_confirm_s=0.30,
            )

        self.assertEqual((source, target, actual), (120_000, 170_000, 170_000))
        move_absolute.assert_called_once_with(170_000)

    def test_drive3_reference_rejects_a_persistent_following_error(self):
        drive = bridge.FaulhaberDrive(
            FakeSlave(), drive_id=3, sdo_delay_s=0.0, verbose=False
        )
        following_status = (
            bridge.STATUS_OPERATION_ENABLED_STATE
            | bridge.STATUS_FOLLOWING_OR_HOMING_ERROR
        )
        with (
            mock.patch.object(
                drive,
                "read_actual_position_counts",
                side_effect=[120_000, 130_000, 130_100],
            ),
            mock.patch.object(drive, "read_status", return_value=following_status),
            mock.patch.object(drive, "move_absolute_counts"),
        ):
            with self.assertRaisesRegex(RuntimeError, "following error persisted"):
                drive.move_relative_counts_and_wait(
                    50_000,
                    timeout_s=1.0,
                    tolerance_counts=100,
                    following_error_confirm_s=0.01,
                )

    def test_homing_method_37_sets_current_drive_position_to_zero(self):
        slave = FakeSlave()
        drive = bridge.FaulhaberDrive(
            slave, drive_id=3, sdo_delay_s=0.0, verbose=False
        )
        completed_status = (
            bridge.STATUS_OPERATION_ENABLED_STATE
            | bridge.STATUS_TARGET_REACHED
            | bridge.STATUS_HOMING_ATTAINED
        )

        def write_controlword(command, delay=None):
            del delay
            if command == bridge.CMD_ENABLE_OPERATION:
                return bridge.STATUS_OPERATION_ENABLED_STATE
            return 0

        with (
            mock.patch.object(drive, "reset_fault_if_needed"),
            mock.patch.object(
                drive, "set_operation_mode", side_effect=lambda mode: mode
            ),
            mock.patch.object(
                drive, "write_controlword", side_effect=write_controlword
            ),
            mock.patch.object(drive, "read_status", return_value=completed_status),
            mock.patch.object(drive, "read_actual_position_counts", return_value=0),
        ):
            zero = drive.home_current_position(timeout_s=1.0)

        self.assertEqual(zero, 0)
        written_values = {
            (index, subindex): payload
            for index, subindex, payload in slave.writes
        }
        self.assertEqual(
            written_values[(bridge.HOMING_METHOD, 0)],
            int(bridge.HOMING_METHOD_CURRENT_POSITION).to_bytes(
                1, "little", signed=True
            ),
        )
        self.assertEqual(
            written_values[(bridge.HOMING_OFFSET, 0)],
            int(0).to_bytes(4, "little", signed=True),
        )

    def test_interval_homing_crosses_both_edges_and_zeros_positive_midpoint(self):
        slave = FakeSlave()
        drive = bridge.FaulhaberDrive(
            slave, drive_id=2, sdo_delay_s=0.0, verbose=False
        )
        operation_enabled = bridge.STATUS_OPERATION_ENABLED_STATE
        with (
            mock.patch.object(
                drive, "home_to_reference_switch", return_value=75
            ) as first_edge_home,
            mock.patch.object(
                drive,
                "read_actual_position_counts",
                side_effect=[80, 100, 160, 240],
            ),
            mock.patch.object(
                drive,
                "read_reference_input_active",
                # The native edge is initially sampled inactive. The scan must
                # observe the active interval before accepting the next low.
                side_effect=[False, True, True, False],
            ),
            mock.patch.object(drive, "read_status", return_value=operation_enabled),
            mock.patch.object(
                drive, "sdo_read_int", side_effect=[3_000, 1_000, 1_000, 0]
            ),
            mock.patch.object(drive, "configure_profile_motion") as configure_profile,
            mock.patch.object(drive, "move_absolute_counts") as start_traverse,
            mock.patch.object(
                drive, "halt_profile_position_motion", return_value=210
            ) as halt_traverse,
            mock.patch.object(
                drive, "move_absolute_counts_and_wait", return_value=120
            ) as move_midpoint,
            mock.patch.object(
                drive, "home_current_position", return_value=0
            ) as zero_midpoint,
        ):
            result = drive.home_to_reference_interval_midpoint(
                method=24,
                reference_input=2,
                offset_counts=0,
                search_speed=1_000,
                zero_speed=200,
                acceleration=1_000,
                timeout_s=1.0,
                interval_timeout_s=2.0,
                max_travel_counts=100_000,
                poll_s=0.001,
                midpoint_tolerance_counts=100,
            )

        first_edge_home.assert_called_once()
        configure_profile.assert_called_once_with(200, 1_000, 1_000)
        start_traverse.assert_called_once_with(100_000)
        halt_traverse.assert_called_once_with(1.0)
        move_midpoint.assert_called_once_with(120, 2.0)
        zero_midpoint.assert_called_once_with(1.0, 10)
        profile_type_writes = [
            int.from_bytes(payload, "little", signed=True)
            for index, subindex, payload in slave.writes
            if (index, subindex) == (bridge.MOTION_PROFILE_TYPE, 0)
        ]
        self.assertEqual(profile_type_writes, [1, 0])
        self.assertEqual(
            result,
            bridge.HomingIntervalResult(
                first_edge_counts=0,
                second_edge_counts=240,
                midpoint_counts=120,
                midpoint_actual_counts=120,
                zero_readback_counts=0,
            ),
        )

    def test_interval_homing_halt_keeps_operation_enabled(self):
        drive = bridge.FaulhaberDrive(
            FakeSlave(), drive_id=1, sdo_delay_s=0.0, verbose=False
        )
        with (
            mock.patch.object(
                drive, "write_controlword", return_value=bridge.STATUS_OPERATION_ENABLED_STATE
            ) as write_controlword,
            mock.patch.object(
                drive, "read_actual_position_counts", side_effect=[205, 210]
            ),
            mock.patch.object(
                drive, "sdo_read_int", side_effect=[15, 0]
            ),
            mock.patch.object(
                drive,
                "read_status",
                return_value=bridge.STATUS_OPERATION_ENABLED_STATE,
            ),
        ):
            stopped_at = drive.halt_profile_position_motion(timeout_s=1.0)

        self.assertEqual(stopped_at, 210)
        self.assertEqual(
            [call.args[0] for call in write_controlword.call_args_list],
            [bridge.CMD_HALT, bridge.CMD_ENABLE_OPERATION],
        )
        self.assertNotIn(
            bridge.CMD_DISABLE_VOLTAGE,
            [call.args[0] for call in write_controlword.call_args_list],
        )

    def test_interval_homing_rejects_nonzero_native_edge_offset(self):
        drive = bridge.FaulhaberDrive(
            FakeSlave(), drive_id=0, sdo_delay_s=0.0, verbose=False
        )
        with mock.patch.object(drive, "home_to_reference_switch") as native_home:
            with self.assertRaisesRegex(ValueError, "requires 0x607C=0"):
                drive.home_to_reference_interval_midpoint(
                    method=28,
                    reference_input=2,
                    offset_counts=1,
                    search_speed=1_000,
                    zero_speed=200,
                    acceleration=1_000,
                    timeout_s=1.0,
                    interval_timeout_s=2.0,
                    max_travel_counts=100_000,
                    poll_s=0.001,
                    midpoint_tolerance_counts=100,
                )

        native_home.assert_not_called()

    def test_interval_homing_method_28_traverses_negative_and_returns_midpoint(self):
        drive = bridge.FaulhaberDrive(
            FakeSlave(), drive_id=0, sdo_delay_s=0.0, verbose=False
        )
        with (
            mock.patch.object(drive, "home_to_reference_switch", return_value=-60),
            mock.patch.object(
                drive,
                "read_actual_position_counts",
                side_effect=[-70, -90, -220],
            ),
            mock.patch.object(
                drive,
                "read_reference_input_active",
                side_effect=[True, True, False],
            ),
            mock.patch.object(
                drive,
                "read_status",
                return_value=bridge.STATUS_OPERATION_ENABLED_STATE,
            ),
            mock.patch.object(
                drive, "sdo_read_int", side_effect=[3_000, 1_000, 1_000, 0]
            ),
            mock.patch.object(drive, "configure_profile_motion"),
            mock.patch.object(drive, "move_absolute_counts") as start_traverse,
            mock.patch.object(drive, "halt_profile_position_motion", return_value=-230),
            mock.patch.object(
                drive, "move_absolute_counts_and_wait", return_value=-110
            ) as move_midpoint,
            mock.patch.object(drive, "home_current_position", return_value=0),
        ):
            result = drive.home_to_reference_interval_midpoint(
                method=28,
                reference_input=2,
                offset_counts=0,
                search_speed=1_000,
                zero_speed=200,
                acceleration=1_000,
                timeout_s=1.0,
                interval_timeout_s=2.0,
                max_travel_counts=100_000,
                poll_s=0.001,
                midpoint_tolerance_counts=100,
            )

        start_traverse.assert_called_once_with(-100_000)
        move_midpoint.assert_called_once_with(-110, 2.0)
        self.assertEqual(result.second_edge_counts, -220)
        self.assertEqual(result.midpoint_counts, -110)
        self.assertEqual(result.zero_readback_counts, 0)

    def test_node_home_drive_records_interval_evidence_before_marking_homed(self):
        result = bridge.HomingIntervalResult(0, -240, -120, -118, 0)
        drive = mock.Mock()
        drive.home_to_reference_interval_midpoint.return_value = result
        node = object.__new__(bridge.RASCLFaulhaberBridge)
        node.spur_gear_reference_complete = True
        node.spur_gear_reference_source_counts = 1
        node.spur_gear_reference_target_counts = 2
        node.spur_gear_reference_pre_zero_counts = 3
        node.spur_gear_reference_zero_readback = 0
        node.homing_interval_results = {}
        node.homing_methods = [28]
        node.reference_inputs = [2]
        node.homing_offsets = [0]
        node.homing_search_speeds = [1_000]
        node.homing_zero_speeds = [200]
        node.homing_accelerations = [1_000]
        node.homing_interval_max_travel_counts = [100_000]
        node.homing_interval_timeout_s = 120.0
        node.homing_interval_poll_s = 0.01
        node.homing_midpoint_tolerance_counts = 100
        node.motion_timeout_s = 8.0
        node.bus = types.SimpleNamespace(
            drives=[drive],
            mark_drive_homing_started=mock.Mock(),
            mark_drive_homed=mock.Mock(),
        )
        logger = mock.Mock()
        node.get_logger = lambda: logger

        position = node._home_drive(0)

        self.assertEqual(position, 0)
        self.assertEqual(node.homing_interval_results[0], result)
        node.bus.mark_drive_homing_started.assert_called_once_with(0)
        node.bus.mark_drive_homed.assert_called_once_with(0)
        drive.home_to_reference_interval_midpoint.assert_called_once_with(
            method=28,
            reference_input=2,
            offset_counts=0,
            search_speed=1_000,
            zero_speed=200,
            acceleration=1_000,
            timeout_s=8.0,
            interval_timeout_s=120.0,
            max_travel_counts=100_000,
            poll_s=0.01,
            midpoint_tolerance_counts=100,
        )
        self.assertIn("entry=0", logger.warning.call_args.args[0])
        self.assertIn("midpoint=-120", logger.warning.call_args.args[0])

    def test_node_drive3_reference_records_method37_zero(self):
        drive = mock.Mock()
        drive.move_relative_counts_and_wait.return_value = (
            120_000,
            170_000,
            169_975,
        )
        drive.home_current_position.return_value = 0
        node = object.__new__(bridge.RASCLFaulhaberBridge)
        node.bus = types.SimpleNamespace(
            homing_complete=True,
            drives=[mock.Mock(), mock.Mock(), mock.Mock(), drive],
        )
        node.ignore_spur_gear_in_csp = False
        node.skip_spur_gear_homing = True
        node.spur_gear_reference_delta_counts = 50_000
        node.spur_gear_reference_timeout_s = 30.0
        node.spur_gear_reference_tolerance_counts = 100
        node.spur_gear_reference_profile_velocity = 3_000
        node.spur_gear_reference_profile_acceleration = 1_000
        node.spur_gear_reference_profile_deceleration = 1_000
        node.spur_gear_reference_following_error_confirm_s = 0.30
        node.spur_gear_reference_complete = False
        node.spur_gear_reference_source_counts = None
        node.spur_gear_reference_target_counts = None
        node.spur_gear_reference_pre_zero_counts = None
        node.spur_gear_reference_zero_readback = None
        logger = mock.Mock()
        node.get_logger = lambda: logger

        message = node._reference_spur_gear_after_arm_homing()

        drive.enable_operation.assert_called_once_with(bridge.MODE_PROFILE_POSITION)
        drive.configure_profile_motion.assert_called_once_with(3_000, 1_000, 1_000)
        drive.move_relative_counts_and_wait.assert_called_once_with(
            50_000, 30.0, 100, 0.30
        )
        drive.home_current_position.assert_called_once_with(30.0, 10)
        self.assertTrue(node.spur_gear_reference_complete)
        self.assertEqual(node.spur_gear_reference_pre_zero_counts, 169_975)
        self.assertEqual(node.spur_gear_reference_zero_readback, 0)
        self.assertIn("delta=50000", message)

    def test_node_drive3_reference_failure_disables_drive(self):
        drive = mock.Mock()
        drive.move_relative_counts_and_wait.side_effect = RuntimeError(
            "following error persisted"
        )
        node = object.__new__(bridge.RASCLFaulhaberBridge)
        node.bus = types.SimpleNamespace(
            homing_complete=True,
            drives=[mock.Mock(), mock.Mock(), mock.Mock(), drive],
        )
        node.ignore_spur_gear_in_csp = False
        node.skip_spur_gear_homing = True
        node.spur_gear_reference_delta_counts = 50_000
        node.spur_gear_reference_timeout_s = 30.0
        node.spur_gear_reference_tolerance_counts = 100
        node.spur_gear_reference_profile_velocity = 3_000
        node.spur_gear_reference_profile_acceleration = 1_000
        node.spur_gear_reference_profile_deceleration = 1_000
        node.spur_gear_reference_following_error_confirm_s = 0.30
        node.spur_gear_reference_complete = False
        logger = mock.Mock()
        node.get_logger = lambda: logger

        with self.assertRaisesRegex(RuntimeError, "following error persisted"):
            node._reference_spur_gear_after_arm_homing()

        drive.disable_operation.assert_called_once_with()
        drive.home_current_position.assert_not_called()
        self.assertFalse(node.spur_gear_reference_complete)

    def test_homing_csp_rejects_handoff_until_drive3_reference_is_complete(self):
        node = object.__new__(bridge.RASCLFaulhaberBridge)
        node.control_mode = "homing_csp"
        node.ignore_spur_gear_in_csp = False
        node.spur_gear_reference_complete = False
        node.spur_gear_reference_delta_counts = 50_000

        with self.assertRaisesRegex(RuntimeError, "Drive 3 reference is incomplete"):
            node._require_spur_gear_reference_for_csp()

        node.spur_gear_reference_complete = True
        node._require_spur_gear_reference_for_csp()

    def test_position_pdo_payload_is_six_bytes_and_little_endian(self):
        payload = bridge.FaulhaberBus._pack_rxpdo(0x000F, -123456)
        self.assertEqual(len(payload), bridge.PDO_RX_SIZE_BYTES)
        self.assertEqual(bridge.PDO_RX_SIZE_BYTES, 6)
        self.assertEqual(payload[:2], b"\x0f\x00")

        status, actual = bridge.FaulhaberBus._unpack_txpdo(
            b"\x27\x10" + int(-123456).to_bytes(4, "little", signed=True)
        )
        self.assertEqual(status, 0x1027)
        self.assertEqual(actual, -123456)

    def test_assignment_uses_factory_position_pdos_without_remapping_them(self):
        slave = FakeSlave()
        make_bus()._assign_factory_position_pdos(slave, slave_index=0)

        written_indices = {index for index, _subindex, _payload in slave.writes}
        self.assertEqual(
            written_indices, {bridge.PDO_RX_ASSIGNMENT, bridge.PDO_TX_ASSIGNMENT}
        )
        self.assertNotIn(
            (bridge.POSITION_RXPDO, 0),
            [(index, subindex) for index, subindex, _payload in slave.writes],
        )
        self.assertEqual(
            slave.values[(bridge.PDO_RX_ASSIGNMENT, 1)],
            bridge.POSITION_RXPDO.to_bytes(2, "little"),
        )
        self.assertEqual(
            slave.values[(bridge.PDO_TX_ASSIGNMENT, 1)],
            bridge.POSITION_TXPDO.to_bytes(2, "little"),
        )

    def test_invalid_sm_sync_cycle_is_rejected(self):
        for cycle_ns in (500_000, 1_500_000, 101_000_000):
            with self.subTest(cycle_ns=cycle_ns), self.assertRaises(ValueError):
                make_bus(pdo_cycle_ns=cycle_ns)

    def test_sm_sync_cycle_monitor_is_written_and_read_back(self):
        slave = FakeSlave()
        bus = make_bus(pdo_cycle_ns=20_000_000)
        bus._configure_sm_cycle_monitoring(slave, slave_index=0)

        self.assertEqual(
            slave.values[(bridge.SM2_PARAMETERS, bridge.SM_CYCLE_TIME_SUBINDEX)],
            int(20_000_000).to_bytes(4, "little"),
        )

    def test_csp_interpolation_rate_matches_twenty_ms_pdo_cycle(self):
        slave = FakeSlave()
        bus = make_bus(pdo_cycle_ns=20_000_000)
        bus._configure_cyclic_interpolation_rate(slave, slave_index=0)

        self.assertEqual(
            slave.values[(bridge.CYCLIC_MODE_INTERPOLATION_RATE, 0)],
            int(200).to_bytes(2, "little"),
        )

    def test_drive_following_error_monitor_is_relaxed_without_changing_limits(self):
        slave = FakeSlave()
        drive = bridge.FaulhaberDrive(slave, 2, sdo_delay_s=0.0, verbose=False)
        initial_limits = (
            slave.values[(bridge.POSITION_RANGE_LIMIT, 1)],
            slave.values[(bridge.POSITION_RANGE_LIMIT, 2)],
            slave.values[(bridge.SOFTWARE_POSITION_LIMIT, 1)],
            slave.values[(bridge.SOFTWARE_POSITION_LIMIT, 2)],
        )

        drive.configure_following_error_monitor(25_000, 250)
        diagnostics = drive.read_position_protection()

        self.assertEqual(diagnostics["following_error_window"], 25_000)
        self.assertEqual(diagnostics["following_error_timeout_ms"], 250)
        self.assertEqual(
            initial_limits,
            (
                slave.values[(bridge.POSITION_RANGE_LIMIT, 1)],
                slave.values[(bridge.POSITION_RANGE_LIMIT, 2)],
                slave.values[(bridge.SOFTWARE_POSITION_LIMIT, 1)],
                slave.values[(bridge.SOFTWARE_POSITION_LIMIT, 2)],
            ),
        )
        self.assertNotIn(
            bridge.POSITION_RANGE_LIMIT,
            {index for index, _subindex, _payload in slave.writes},
        )
        self.assertNotIn(
            bridge.SOFTWARE_POSITION_LIMIT,
            {index for index, _subindex, _payload in slave.writes},
        )

    def test_csp_limit_mapping_fix_preserves_homing_and_position_limits(self):
        slave = FakeSlave()
        slave.values[
            (bridge.DIGITAL_INPUT_SETTINGS, bridge.LOWER_LIMIT_SWITCH_INPUTS)
        ] = int(0x02).to_bytes(1, "little")
        slave.values[
            (bridge.DIGITAL_INPUT_SETTINGS, bridge.UPPER_LIMIT_SWITCH_INPUTS)
        ] = int(0x04).to_bytes(1, "little")
        slave.values[
            (bridge.DIGITAL_INPUT_SETTINGS, bridge.INPUT_POLARITY)
        ] = int(0x07).to_bytes(1, "little")
        slave.values[(bridge.DEVICE_STATUS, 1)] = int(
            (1 << 6) | (1 << 7)
        ).to_bytes(4, "little")
        drive = bridge.FaulhaberDrive(
            slave, drive_id=2, sdo_delay_s=0.0, verbose=False
        )
        protected_before = {
            key: bytes(value)
            for key, value in slave.values.items()
            if key[0] in (
                bridge.POSITION_RANGE_LIMIT,
                bridge.SOFTWARE_POSITION_LIMIT,
            )
        }

        before, after = drive.clear_limit_switch_mappings_for_csp()

        self.assertEqual(before["lower_limit_input_mask"], 0x02)
        self.assertEqual(before["upper_limit_input_mask"], 0x04)
        self.assertEqual(after["lower_limit_input_mask"], 0)
        self.assertEqual(after["upper_limit_input_mask"], 0)
        self.assertEqual(after["reference_input"], 2)
        self.assertEqual(after["input_polarity"], 0x07)
        self.assertEqual(after["limit_switch_option"], 2)
        self.assertEqual(after["device_status"] & ((1 << 6) | (1 << 7)), 0)
        self.assertEqual(
            protected_before,
            {
                key: value
                for key, value in slave.values.items()
                if key[0] in (
                    bridge.POSITION_RANGE_LIMIT,
                    bridge.SOFTWARE_POSITION_LIMIT,
                )
            },
        )
        self.assertEqual(
            {
                (index, subindex, payload)
                for index, subindex, payload in slave.writes
            },
            {
                (
                    bridge.DIGITAL_INPUT_SETTINGS,
                    bridge.LOWER_LIMIT_SWITCH_INPUTS,
                    b"\x00",
                ),
                (
                    bridge.DIGITAL_INPUT_SETTINGS,
                    bridge.UPPER_LIMIT_SWITCH_INPUTS,
                    b"\x00",
                ),
            },
        )

    def test_csp_limit_mapping_fix_can_be_disabled_for_rollback(self):
        slave = FakeSlave()
        slave.values[
            (bridge.DIGITAL_INPUT_SETTINGS, bridge.LOWER_LIMIT_SWITCH_INPUTS)
        ] = int(0x02).to_bytes(1, "little")
        bus = make_bus(clear_limit_switch_mappings_for_csp=False)
        bus.drives = [
            bridge.FaulhaberDrive(slave, drive_id=0, sdo_delay_s=0.0, verbose=False)
        ]
        bus.required_csp_drive_ids = {0}

        bus._configure_csp_limit_switch_mappings_locked()

        self.assertEqual(
            slave.values[
                (bridge.DIGITAL_INPUT_SETTINGS, bridge.LOWER_LIMIT_SWITCH_INPUTS)
            ],
            b"\x02",
        )
        self.assertEqual(slave.writes, [])

    def test_csp_torque_limit_is_written_and_read_back(self):
        slave = FakeSlave()
        for index in (
            bridge.MAX_TORQUE,
            bridge.POSITIVE_TORQUE_LIMIT,
            bridge.NEGATIVE_TORQUE_LIMIT,
        ):
            slave.values[(index, 0)] = int(200).to_bytes(2, "little")
        drive = bridge.FaulhaberDrive(slave, 2, sdo_delay_s=0.0, verbose=False)

        before, after = drive.configure_csp_torque_limit(1000)

        self.assertEqual(
            before,
            {
                "maximum_torque": 200,
                "positive_torque_limit": 200,
                "negative_torque_limit": 200,
            },
        )
        self.assertEqual(
            after,
            {
                "maximum_torque": 200,
                "positive_torque_limit": 1000,
                "negative_torque_limit": 1000,
            },
        )
        self.assertEqual(
            {
                (index, int.from_bytes(payload, "little"))
                for index, subindex, payload in slave.writes
                if subindex == 0
            },
            {
                (bridge.POSITIVE_TORQUE_LIMIT, 1000),
                (bridge.NEGATIVE_TORQUE_LIMIT, 1000),
            },
        )
        self.assertNotIn(
            bridge.MAX_TORQUE,
            {index for index, _subindex, _payload in slave.writes},
        )

    def test_csp_torque_limit_readback_mismatch_is_rejected(self):
        slave = FakeSlave()
        original_write = slave.sdo_write

        def discard_negative_limit(index, subindex, payload):
            if (index, subindex) == (bridge.NEGATIVE_TORQUE_LIMIT, 0):
                slave.writes.append((index, subindex, bytes(payload)))
                return
            original_write(index, subindex, payload)

        slave.sdo_write = discard_negative_limit
        drive = bridge.FaulhaberDrive(slave, 2, sdo_delay_s=0.0, verbose=False)

        with self.assertRaisesRegex(RuntimeError, "readback mismatch"):
            drive.configure_csp_torque_limit(1000)

    def test_drive2_peak_current_correction_raises_read_only_maximum(self):
        slave = FakeSlave()
        slave.values[(bridge.MOTOR_APPLICATION_DATA, 1)] = int(1100).to_bytes(
            2, "little"
        )
        slave.values[(bridge.MOTOR_APPLICATION_DATA, 2)] = int(1100).to_bytes(
            2, "little"
        )
        slave.values[(bridge.MOTOR_APPLICATION_DATA, 3)] = int(220).to_bytes(
            2, "little"
        )
        slave.values[(bridge.MAX_TORQUE, 0)] = int(200).to_bytes(2, "little")
        drive = bridge.FaulhaberDrive(slave, 2, sdo_delay_s=0.0, verbose=False)

        before, after, effective_maximum = (
            drive.ensure_peak_current_for_torque_limit(1000)
        )

        self.assertEqual(before["rated_current_ma"], 1100)
        self.assertEqual(before["continuous_current_ma"], 1100)
        self.assertEqual(before["peak_current_ma"], 220)
        self.assertEqual(after["rated_current_ma"], 1100)
        self.assertEqual(after["continuous_current_ma"], 1100)
        self.assertEqual(after["peak_current_ma"], 1100)
        self.assertEqual(effective_maximum, 1000)
        self.assertIn(
            (
                bridge.MOTOR_APPLICATION_DATA,
                3,
                int(1100).to_bytes(2, "little"),
            ),
            slave.writes,
        )

    def test_csp_handoff_corrects_drive2_and_drive3_peak_current(self):
        slaves = [FakeSlave() for _ in range(4)]
        for drive_id, rated_current, peak_current in (
            (2, 1100, 220),
            (3, 540, 81),
        ):
            slave = slaves[drive_id]
            slave.values[(bridge.MOTOR_APPLICATION_DATA, 1)] = int(
                rated_current
            ).to_bytes(2, "little")
            slave.values[(bridge.MOTOR_APPLICATION_DATA, 2)] = int(
                rated_current
            ).to_bytes(2, "little")
            slave.values[(bridge.MOTOR_APPLICATION_DATA, 3)] = int(
                peak_current
            ).to_bytes(2, "little")
            slave.values[(bridge.MAX_TORQUE, 0)] = int(
                peak_current * 1000 // rated_current
            ).to_bytes(2, "little")

        bus = make_bus()
        bus.drives = [
            bridge.FaulhaberDrive(slave, index, sdo_delay_s=0.0, verbose=False)
            for index, slave in enumerate(slaves)
        ]
        bus._configure_csp_torque_limits_locked()

        for drive_id, rated_current in ((2, 1100), (3, 540)):
            slave = slaves[drive_id]
            self.assertEqual(
                int.from_bytes(
                    slave.values[(bridge.MOTOR_APPLICATION_DATA, 3)], "little"
                ),
                rated_current,
            )
            self.assertEqual(
                int.from_bytes(slave.values[(bridge.MAX_TORQUE, 0)], "little"),
                1000,
            )
            self.assertIn(
                (
                    bridge.MOTOR_APPLICATION_DATA,
                    3,
                    int(rated_current).to_bytes(2, "little"),
                ),
                slave.writes,
            )

        for drive_id in (0, 1):
            self.assertNotIn(
                (bridge.MOTOR_APPLICATION_DATA, 3),
                {
                    (index, subindex)
                    for index, subindex, _payload in slaves[drive_id].writes
                },
            )

    def test_above_bridge_csp_torque_ceiling_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "1..6000"):
            make_bus(csp_torque_limit_per_mille=6001)

    def test_position_protection_read_retries_a_transient_mailbox_error(self):
        slave = FakeSlave()
        original_read = slave.sdo_read
        position_range_attempts = 0

        def flaky_read(index, subindex):
            nonlocal position_range_attempts
            if (index, subindex) == (bridge.POSITION_RANGE_LIMIT, 1):
                position_range_attempts += 1
                if position_range_attempts == 1:
                    raise RuntimeError("transient WKC")
            return original_read(index, subindex)

        slave.sdo_read = flaky_read
        drive = bridge.FaulhaberDrive(slave, 2, sdo_delay_s=0.0, verbose=False)

        diagnostics = drive.read_position_protection()

        self.assertEqual(position_range_attempts, 2)
        self.assertEqual(diagnostics["position_range_min"], -(1 << 31))

    def test_csp_connect_configures_interpolation_for_required_drives_only(self):
        slaves = [FakeSlave() for _ in range(4)]
        master = FakeMaster(slaves)
        original_master = bridge.pysoem.Master
        bridge.pysoem.Master = lambda: master
        try:
            make_bus(ignored_csp_drive_indices=[3]).connect()
        finally:
            bridge.pysoem.Master = original_master

        for slave in slaves[:3]:
            self.assertEqual(
                slave.values[(bridge.CYCLIC_MODE_INTERPOLATION_RATE, 0)],
                int(200).to_bytes(2, "little"),
            )
        self.assertNotIn((bridge.CYCLIC_MODE_INTERPOLATION_RATE, 0), slaves[3].values)

    def test_profile_homing_connection_stays_sdo_only_in_preop(self):
        master = FakeMaster([FakeSlave() for _ in range(4)])
        original_master = bridge.pysoem.Master
        bridge.pysoem.Master = lambda: master
        try:
            bus = make_bus(control_mode="profile")
            bus.connect()
        finally:
            bridge.pysoem.Master = original_master

        self.assertEqual(master.config_map_calls, 0)
        self.assertTrue(all(not slave.writes for slave in master.slaves))

    def test_homing_csp_connection_defers_pdo_mapping(self):
        master = FakeMaster([FakeSlave() for _ in range(4)])
        original_master = bridge.pysoem.Master
        bridge.pysoem.Master = lambda: master
        try:
            bus = make_bus(control_mode="homing_csp")
            bus.connect()
        finally:
            bridge.pysoem.Master = original_master

        self.assertEqual(master.config_map_calls, 0)
        self.assertTrue(all(not slave.writes for slave in master.slaves))

    def test_homing_csp_rejects_handoff_before_all_axes_are_homed(self):
        bus = make_bus(control_mode="homing_csp")
        slaves = [FakeSlave() for _ in range(4)]
        bus.master = FakeMaster(slaves)
        bus.drives = [
            bridge.FaulhaberDrive(slave, index, sdo_delay_s=0.0, verbose=False)
            for index, slave in enumerate(slaves)
        ]

        with self.assertRaisesRegex(RuntimeError, "not all required drives"):
            bus.enter_csp()
        self.assertEqual(bus.master.config_map_calls, 0)

        statuses = bus.exit_csp()
        self.assertEqual(len(statuses), 4)
        self.assertEqual(bus.master.write_state_calls, 0)

    def test_four_individual_homing_results_arm_handoff(self):
        bus = make_bus(control_mode="homing_csp")

        for drive_id in range(4):
            bus.mark_drive_homing_started(drive_id)
            self.assertFalse(bus.homing_complete)
            bus.mark_drive_homed(drive_id)

        self.assertTrue(bus.homing_complete)
        self.assertEqual(bus.homed_drive_ids, {0, 1, 2, 3})

        bus.mark_drive_homing_started(2)
        self.assertFalse(bus.homing_complete)

    def test_spur_gear_can_join_csp_without_being_homed(self):
        bus = make_bus(
            control_mode="homing_csp",
            required_homing_drive_indices=[0, 1, 2],
        )
        self.assertEqual(bus.required_homing_drive_ids, {0, 1, 2})
        self.assertEqual(bus.required_csp_drive_ids, {0, 1, 2, 3})
        self.assertEqual(bus.non_homing_csp_drive_ids, {3})

        for drive_id in range(3):
            bus.mark_drive_homing_started(drive_id)
            bus.mark_drive_homed(drive_id)
        self.assertTrue(bus.homing_complete)

    def test_deferred_csp_enables_non_homed_spur_gear(self):
        slaves = [FakeSlave() for _ in range(4)]
        for slave in slaves[:3]:
            slave.values[(bridge.STATUS_WORD, 0)] = int(
                bridge.STATUS_OPERATION_ENABLED_STATE
            ).to_bytes(2, "little")

        bus = make_bus(
            control_mode="homing_csp",
            required_homing_drive_indices=[0, 1, 2],
        )
        bus.master = FakeMaster(slaves)
        bus.drives = [
            bridge.FaulhaberDrive(slave, index, sdo_delay_s=0.0, verbose=False)
            for index, slave in enumerate(slaves)
        ]
        bus.mark_homing_complete(True)

        with mock.patch.object(bus.drives[3], "enable_operation") as enable_spur:
            bus._prepare_deferred_csp_locked()

        enable_spur.assert_called_once_with(bridge.MODE_PROFILE_POSITION)
        self.assertEqual(bus.master.config_map_calls, 1)

    def test_non_homed_spur_gear_enters_csp_with_homed_arm_axes(self):
        slaves = [FakeSlave() for _ in range(4)]
        for slave in slaves:
            slave.values[(bridge.STATUS_WORD, 0)] = int(
                bridge.STATUS_OPERATION_ENABLED_STATE
            ).to_bytes(2, "little")

        bus = make_bus(
            control_mode="homing_csp",
            pdo_cycle_ns=1_000_000,
            required_homing_drive_indices=[0, 1, 2],
        )
        bus.master = FakeMaster(slaves)
        bus.drives = [
            bridge.FaulhaberDrive(slave, index, sdo_delay_s=0.0, verbose=False)
            for index, slave in enumerate(slaves)
        ]
        bus.mark_homing_complete(True)

        with mock.patch.object(bus.drives[3], "enable_operation") as enable_spur:
            states = bus.enter_csp([10, 20, 30, 40])

        enable_spur.assert_called_once_with(bridge.MODE_PROFILE_POSITION)
        self.assertTrue(bus.csp_active)
        self.assertEqual([state[0] for state in states], [10, 20, 30, 40])
        self.assertEqual(
            int.from_bytes(
                slaves[2].values[(bridge.FOLLOWING_ERROR_WINDOW, 0)], "little"
            ),
            25_000,
        )
        self.assertEqual(
            int.from_bytes(
                slaves[2].values[(bridge.FOLLOWING_ERROR_TIMEOUT, 0)], "little"
            ),
            250,
        )
        for slave in slaves:
            self.assertEqual(
                int.from_bytes(slave.values[(bridge.MAX_TORQUE, 0)], "little"),
                6000,
            )
            self.assertEqual(
                int.from_bytes(
                    slave.values[(bridge.POSITIVE_TORQUE_LIMIT, 0)], "little"
                ),
                1000,
            )
            self.assertEqual(
                int.from_bytes(
                    slave.values[(bridge.NEGATIVE_TORQUE_LIMIT, 0)], "little"
                ),
                1000,
            )
        bus.exit_csp()

    def test_homing_csp_handoff_keeps_enable_operation_controlword(self):
        positions = [10, 20, 30, 40]
        slaves = [FakeSlave() for _ in range(4)]
        for slave, position in zip(slaves, positions):
            slave.values[(bridge.STATUS_WORD, 0)] = int(
                bridge.STATUS_OPERATION_ENABLED_STATE
            ).to_bytes(2, "little")
            slave.values[(bridge.ACTUAL_POSITION, 0)] = int(position).to_bytes(
                4, "little", signed=True
            )

        bus = make_bus(control_mode="homing_csp", pdo_cycle_ns=1_000_000)
        bus.master = FakeMaster(slaves)
        bus.drives = [
            bridge.FaulhaberDrive(slave, index, sdo_delay_s=0.0, verbose=False)
            for index, slave in enumerate(slaves)
        ]
        bus.mark_homing_complete(True)

        states = bus.enter_csp()
        handoff_controlwords = list(bus.master.controlwords)

        self.assertEqual(bus.master.config_map_calls, 1)
        self.assertEqual([state[0] for state in states], positions)
        self.assertTrue(handoff_controlwords)
        self.assertEqual(set(handoff_controlwords), {bridge.CMD_ENABLE_OPERATION})
        self.assertNotIn(bridge.CMD_SHUTDOWN, handoff_controlwords)
        self.assertNotIn(bridge.CMD_DISABLE_OPERATION, handoff_controlwords)
        self.assertNotIn(bridge.CMD_DISABLE_VOLTAGE, handoff_controlwords)

        bus.exit_csp()

    def test_homing_csp_handoff_requires_every_drive_enabled(self):
        slaves = [FakeSlave() for _ in range(4)]
        for slave in slaves:
            slave.values[(bridge.STATUS_WORD, 0)] = int(
                bridge.STATUS_OPERATION_ENABLED_STATE
            ).to_bytes(2, "little")
        slaves[2].values[(bridge.STATUS_WORD, 0)] = int(
            bridge.STATUS_SWITCHED_ON
        ).to_bytes(2, "little")

        bus = make_bus(control_mode="homing_csp")
        bus.master = FakeMaster(slaves)
        bus.drives = [
            bridge.FaulhaberDrive(slave, index, sdo_delay_s=0.0, verbose=False)
            for index, slave in enumerate(slaves)
        ]
        bus.mark_homing_complete(True)

        with self.assertRaisesRegex(RuntimeError, "Operation Enabled"):
            bus.enter_csp()
        self.assertEqual(bus.master.config_map_calls, 0)
        self.assertEqual(bus.master.controlwords, [])

    def test_ignored_spur_gear_does_not_block_homing_and_stays_disabled(self):
        positions = [10, 20, 30, 40]
        slaves = [FakeSlave() for _ in range(4)]
        for slave, position in zip(slaves, positions):
            slave.values[(bridge.STATUS_WORD, 0)] = int(
                bridge.STATUS_OPERATION_ENABLED_STATE
            ).to_bytes(2, "little")
            slave.values[(bridge.ACTUAL_POSITION, 0)] = int(position).to_bytes(
                4, "little", signed=True
            )

        bus = make_bus(
            control_mode="homing_csp",
            pdo_cycle_ns=1_000_000,
            ignored_csp_drive_indices=[3],
        )
        bus.master = FakeMaster(slaves)
        bus.drives = [
            bridge.FaulhaberDrive(slave, index, sdo_delay_s=0.0, verbose=False)
            for index, slave in enumerate(slaves)
        ]

        for drive_id in range(3):
            bus.mark_drive_homing_started(drive_id)
            bus.mark_drive_homed(drive_id)
        self.assertTrue(bus.homing_complete)
        self.assertEqual(bus.required_homing_drive_ids, {0, 1, 2})

        states = bus.enter_csp()
        time.sleep(0.005)
        self.assertEqual(len(states), 4)
        for slave in slaves[:3]:
            self.assertEqual(
                int.from_bytes(slave.values[(bridge.MAX_TORQUE, 0)], "little"),
                6000,
            )
            self.assertEqual(
                int.from_bytes(
                    slave.values[(bridge.POSITIVE_TORQUE_LIMIT, 0)], "little"
                ),
                1000,
            )
            self.assertEqual(
                int.from_bytes(
                    slave.values[(bridge.NEGATIVE_TORQUE_LIMIT, 0)], "little"
                ),
                1000,
            )
        self.assertEqual(
            int.from_bytes(slaves[3].values[(bridge.MAX_TORQUE, 0)], "little"),
            6000,
        )
        self.assertEqual(
            bridge.struct.unpack("<Hi", slaves[3].output)[0],
            bridge.CMD_DISABLE_VOLTAGE,
        )

        bus.set_csp_targets([11, 22, 33, 999])
        self.assertEqual(bus.target_counts[3], bus.latest_states[3][0])
        bus.exit_csp()

    def test_csp_activation_runs_a_repeating_pdo_loop(self):
        bus = make_bus(pdo_cycle_ns=1_000_000)
        slaves = [FakeSlave() for _ in range(4)]
        bus.master = FakeMaster(slaves)
        bus.drives = [
            bridge.FaulhaberDrive(slave, index, sdo_delay_s=0.0, verbose=False)
            for index, slave in enumerate(slaves)
        ]

        states = bus.enter_csp([10, 20, 30, 40])
        self.assertTrue(bus.csp_active)
        self.assertTrue(all(state[2] == bridge.MODE_CYCLIC_SYNC_POSITION for state in states))

        bus.set_csp_targets([11, 22, 33, 44])
        # Leave enough margin for a scheduling quantum on Windows CI.
        time.sleep(0.05)
        self.assertEqual([state[0] for state in bus.get_csp_states()], [11, 22, 33, 44])
        self.assertEqual(bus.last_working_counter, bus.master.expected_wkc)

        statuses = bus.exit_csp()
        self.assertEqual(len(statuses), 4)
        self.assertFalse(bus.csp_active)
        self.assertEqual(bus.target_counts, [])
        self.assertEqual(bus.latest_states, [])

    def test_following_error_includes_target_and_actual_pdo_snapshot(self):
        bus = make_bus()
        slaves = [FakeSlave() for _ in range(4)]
        slaves[2].values[(bridge.DEVICE_STATUS, 1)] = int(
            (1 << 5) | (1 << 14)
        ).to_bytes(4, "little")
        slaves[2].values[(bridge.ERROR_REGISTER, 0)] = int(0x20).to_bytes(1, "little")
        slaves[2].values[(bridge.PREDEFINED_ERROR_FIELD, 0)] = int(1).to_bytes(1, "little")
        slaves[2].values[(bridge.PREDEFINED_ERROR_FIELD, 1)] = int(0x8611).to_bytes(
            4, "little"
        )
        bus.drives = [
            bridge.FaulhaberDrive(slave, index, sdo_delay_s=0.0, verbose=False)
            for index, slave in enumerate(slaves)
        ]
        bus.target_counts = [100, 200, 300, 400]
        states = [
            (90, bridge.STATUS_OPERATION_ENABLED_STATE, bridge.MODE_CYCLIC_SYNC_POSITION),
            (210, bridge.STATUS_OPERATION_ENABLED_STATE, bridge.MODE_CYCLIC_SYNC_POSITION),
            (
                250,
                bridge.STATUS_OPERATION_ENABLED_STATE
                | bridge.STATUS_FOLLOWING_OR_HOMING_ERROR,
                bridge.MODE_CYCLIC_SYNC_POSITION,
            ),
            (390, bridge.STATUS_OPERATION_ENABLED_STATE, bridge.MODE_CYCLIC_SYNC_POSITION),
        ]

        with self.assertRaisesRegex(
            RuntimeError,
            r"Drive 2 CSP following error; statusword=0x2027; "
            r"CSP_SNAPSHOT .*D2\(target=300,actual=250,error=50,status=0x2027,mode=8\).*; "
            r"DRIVE_DIAG D2; 0x2324\.01=0x00004020\[following_error,torque_limited\]; "
            r"0x1001=0x20\[device_profile\]; 0x1003=\[0x00008611\].*; "
            r"TORQUE_SNAPSHOT D0\(demand/actual=0/0,max/pos/neg=6000/6000/6000\).*"
            r"D2\(demand/actual=0/0,max/pos/neg=6000/6000/6000\)",
        ):
            bus._validate_running_states(states)

    def test_non_fault_stall_triggers_staged_live_diagnostics(self):
        messages = []
        bus = make_bus(
            csp_stall_error_counts=25_000,
            csp_stall_progress_counts=100,
            csp_stall_timeout_ms=500,
            diagnostic_logger=messages.append,
        )
        slaves = [FakeSlave() for _ in range(4)]
        slaves[1].values[(bridge.DEVICE_STATUS, 1)] = int(1 << 14).to_bytes(
            4, "little"
        )
        bus.drives = [
            bridge.FaulhaberDrive(slave, index, sdo_delay_s=0.0, verbose=False)
            for index, slave in enumerate(slaves)
        ]
        bus.target_counts = [0, 100_000, 0, 0]
        states = [
            (0, bridge.STATUS_OPERATION_ENABLED_STATE, bridge.MODE_CYCLIC_SYNC_POSITION),
            (
                0,
                bridge.STATUS_OPERATION_ENABLED_STATE
                | bridge.STATUS_INTERNAL_LIMIT_ACTIVE,
                bridge.MODE_CYCLIC_SYNC_POSITION,
            ),
            (0, bridge.STATUS_OPERATION_ENABLED_STATE, bridge.MODE_CYCLIC_SYNC_POSITION),
            (0, bridge.STATUS_OPERATION_ENABLED_STATE, bridge.MODE_CYCLIC_SYNC_POSITION),
        ]
        bus._reset_stall_monitor_locked(states)
        detected = bus._detect_csp_stalls_locked(
            states, now_ns=bus._stall_anchor_ns[1] + 500_000_000
        )
        self.assertEqual(detected, [1])

        bus._start_live_stall_diagnostics_locked(states, detected)
        while bus.live_diagnostic_pending:
            bus._advance_live_stall_diagnostics_locked()

        self.assertIn("CSP_STALL_DETECTED drives=D1", messages[0])
        self.assertIn("CSP_STALL_SNAPSHOT causes=D1=TORQUE_LIMIT_REPORTED", messages[-1])
        self.assertIn("0x2324.01=0x00004000[torque_limited]", messages[-1])
        self.assertIn(
            "input_config(0x2310.01/.02/.03/.04/.10", messages[-1]
        )
        self.assertIn("input_state(0x2311.01/.02 logical/physical)", messages[-1])
        self.assertIn(
            "voltage_10mV(0x2325.01-.07", messages[-1]
        )
        self.assertIn("/2400/2400", messages[-1])
        self.assertIn("internal_limit_active", messages[-1])
        self.assertEqual(bus.last_stall_snapshot, messages[-1])

    def test_encoder_progress_resets_stall_timer(self):
        bus = make_bus(
            csp_stall_error_counts=25_000,
            csp_stall_progress_counts=100,
            csp_stall_timeout_ms=500,
        )
        bus.target_counts = [0, 100_000, 0, 0]
        initial_states = [
            (0, bridge.STATUS_OPERATION_ENABLED_STATE, bridge.MODE_CYCLIC_SYNC_POSITION)
            for _ in range(4)
        ]
        bus._reset_stall_monitor_locked(initial_states)
        start_ns = bus._stall_anchor_ns[1]
        progressed_states = list(initial_states)
        progressed_states[1] = (
            200,
            bridge.STATUS_OPERATION_ENABLED_STATE,
            bridge.MODE_CYCLIC_SYNC_POSITION,
        )

        self.assertEqual(
            bus._detect_csp_stalls_locked(
                progressed_states, now_ns=start_ns + 500_000_000
            ),
            [],
        )
        self.assertEqual(
            bus._detect_csp_stalls_locked(
                progressed_states, now_ns=start_ns + 999_000_000
            ),
            [],
        )
        self.assertEqual(
            bus._detect_csp_stalls_locked(
                progressed_states, now_ns=start_ns + 1_000_000_000
            ),
            [1],
        )


if __name__ == "__main__":
    unittest.main()
