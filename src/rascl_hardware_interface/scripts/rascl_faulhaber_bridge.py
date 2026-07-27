#!/usr/bin/env python3
"""EtherCAT bridge for the RASCL ros2_control hardware interface.

The C++ SystemInterface uses a small line-based TCP protocol.  This node owns
the pysoem master, keeps the proven SDO path for homing/profile fallback, and
runs the WP3 CSP position stream through cyclic PDO process data.
"""

from __future__ import annotations

import socket
import struct
import sys
import threading
import time
from typing import Callable, Dict, List, NamedTuple, Optional, Sequence, Tuple

import pysoem
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

# CiA 402 object dictionary indices.
ERROR_REGISTER = 0x1001
PREDEFINED_ERROR_FIELD = 0x1003
CONTROL_WORD = 0x6040
STATUS_WORD = 0x6041
MODE_OF_OPERATION = 0x6060
MODE_DISPLAY = 0x6061
POSITION_DEMAND_VALUE = 0x6062
TARGET_POSITION = 0x607A
ACTUAL_POSITION = 0x6064
VELOCITY_ACTUAL_VALUE = 0x606C
MAX_TORQUE = 0x6072
TORQUE_DEMAND = 0x6074
ACTUAL_TORQUE = 0x6077
ACTUAL_CURRENT = 0x6078
HOMING_OFFSET = 0x607C
HOMING_METHOD = 0x6098
PROFILE_VELOCITY = 0x6081
PROFILE_ACCELERATION = 0x6083
PROFILE_DECELERATION = 0x6084
MOTION_PROFILE_TYPE = 0x6086
HOMING_SPEED = 0x6099
HOMING_SEARCH_SPEED = 0x01
HOMING_ZERO_SPEED = 0x02
HOMING_ACCELERATION = 0x609A
FOLLOWING_ERROR_WINDOW = 0x6065
FOLLOWING_ERROR_TIMEOUT = 0x6066
POSITION_RANGE_LIMIT = 0x607B
SOFTWARE_POSITION_LIMIT = 0x607D
MAX_MOTOR_SPEED = 0x6080
POSITIVE_TORQUE_LIMIT = 0x60E0
NEGATIVE_TORQUE_LIMIT = 0x60E1
FOLLOWING_ERROR_ACTUAL_VALUE = 0x60F4

# FAULHABER manufacturer objects used only for failure diagnostics.
DEVICE_STATUS = 0x2324
VOLTAGE_MONITOR = 0x2325
DEVICE_STATUS_FLAGS = {
    2: "velocity_deviation",
    5: "following_error",
    6: "positive_limit_switch",
    7: "negative_limit_switch",
    8: "software_limit_positive",
    9: "software_limit_negative",
    13: "voltage_limited",
    14: "torque_limited",
    15: "speed_limited",
    16: "temperature_warning",
    17: "temperature_shutdown",
    18: "supply_overvoltage",
    19: "controller_undervoltage",
    20: "motor_undervoltage",
    21: "motor_overspeed",
    22: "safety_monitoring",
}
ERROR_REGISTER_FLAGS = {
    0: "generic",
    1: "current",
    2: "voltage",
    3: "temperature",
    4: "communication",
    5: "device_profile",
    7: "manufacturer",
}
POSITION_CONTROL_PARAMETER_SET = 0x2348
MOTOR_APPLICATION_DATA = 0x2329

# CiA-402 torque values are expressed in per-mille of the motor rated torque.
# The manual lists 6000 as the factory value for these objects; the bridge uses
# that as its explicit override ceiling.  The 1000 default gives every CSP axis
# full rated torque without requesting an over-rated peak.
MAX_TORQUE_PER_MILLE = 6000

# FAULHABER CSP target-position interpolation.  The value is the desired
# target refresh interval expressed as multiples of the drive's fixed 100 us
# internal position-control update.
CYCLIC_MODE_INTERPOLATION_RATE = 0x2332
INTERPOLATION_QUANTUM_NS = 100_000

# FAULHABER digital I/O objects used by the tested automatic-homing workflow.
DIGITAL_INPUT_SETTINGS = 0x2310
DIGITAL_IO_STATUS = 0x2311
DIGITAL_INPUT_LOGICAL = 0x01
DIGITAL_INPUT_PHYSICAL = 0x02
LOWER_LIMIT_SWITCH_INPUTS = 0x01
UPPER_LIMIT_SWITCH_INPUTS = 0x02
LIMIT_SWITCH_OPTION_CODE = 0x03
REFERENCE_SWITCH_INPUT = 0x04
INPUT_POLARITY = 0x10
DIGOUT_WRITE = 0x04
DIGOUT1_ON = 0x00FD
DIGOUT1_OFF = 0x00FC
DIGOUT1_TOGGLE = 0x00FE

# CiA 402 modes.
MODE_PROFILE_POSITION = 1
MODE_HOMING = 6
MODE_CYCLIC_SYNC_POSITION = 8
HOMING_METHOD_CURRENT_POSITION = 37

# CiA 402 control words.
CMD_SHUTDOWN = 0x0006
CMD_SWITCH_ON = 0x0007
CMD_DISABLE_OPERATION = 0x0007
CMD_ENABLE_OPERATION = 0x000F
CMD_DISABLE_VOLTAGE = 0x0000
CMD_FAULT_RESET = 0x0080
CMD_START_MOTION = 0x003F
CMD_START_HOMING = CMD_ENABLE_OPERATION | 0x0010
CMD_HALT = CMD_ENABLE_OPERATION | 0x0100

# Statusword state/operation bits.
STATUS_STATE_MASK = 0x006F
STATUS_READY_TO_SWITCH_ON = 0x0021
STATUS_SWITCHED_ON = 0x0023
STATUS_OPERATION_ENABLED_STATE = 0x0027
STATUS_FAULT = 1 << 3
STATUS_TARGET_REACHED = 1 << 10
STATUS_INTERNAL_LIMIT_ACTIVE = 1 << 11
STATUS_CSP_TARGET_ACCEPTED = 1 << 12
STATUS_HOMING_ATTAINED = 1 << 12
STATUS_FOLLOWING_OR_HOMING_ERROR = 1 << 13

STATUSWORD_MONITOR_FLAGS = {
    10: "target_reached",
    11: "internal_limit_active",
    12: "csp_target_accepted",
    13: "following_error",
}

# Read-only objects sampled one at a time after a non-fault CSP stall is
# detected.  One mailbox request per PDO cycle keeps process data flowing while
# preserving the drive-side evidence that is absent from /joint_states.
LIVE_DIAGNOSTIC_READS = (
    ("device_status", DEVICE_STATUS, 1, False),
    ("device_supply_actual_10mv", VOLTAGE_MONITOR, 6, False),
    ("motor_supply_actual_10mv", VOLTAGE_MONITOR, 7, False),
    ("error_register", ERROR_REGISTER, 0, False),
    ("position_demand", POSITION_DEMAND_VALUE, 0, True),
    ("position_actual", ACTUAL_POSITION, 0, True),
    ("following_error_actual", FOLLOWING_ERROR_ACTUAL_VALUE, 0, True),
    ("velocity_actual", VELOCITY_ACTUAL_VALUE, 0, True),
    ("torque_demand", TORQUE_DEMAND, 0, True),
    ("torque_actual", ACTUAL_TORQUE, 0, True),
    ("current_actual", ACTUAL_CURRENT, 0, True),
    ("maximum_torque", MAX_TORQUE, 0, False),
    ("positive_torque_limit", POSITIVE_TORQUE_LIMIT, 0, False),
    ("negative_torque_limit", NEGATIVE_TORQUE_LIMIT, 0, False),
    ("maximum_motor_speed", MAX_MOTOR_SPEED, 0, False),
    ("position_range_min", POSITION_RANGE_LIMIT, 1, True),
    ("position_range_max", POSITION_RANGE_LIMIT, 2, True),
    ("software_limit_min", SOFTWARE_POSITION_LIMIT, 1, True),
    ("software_limit_max", SOFTWARE_POSITION_LIMIT, 2, True),
    (
        "lower_limit_input_mask",
        DIGITAL_INPUT_SETTINGS,
        LOWER_LIMIT_SWITCH_INPUTS,
        False,
    ),
    (
        "upper_limit_input_mask",
        DIGITAL_INPUT_SETTINGS,
        UPPER_LIMIT_SWITCH_INPUTS,
        False,
    ),
    ("limit_switch_option", DIGITAL_INPUT_SETTINGS, LIMIT_SWITCH_OPTION_CODE, True),
    ("reference_input", DIGITAL_INPUT_SETTINGS, REFERENCE_SWITCH_INPUT, False),
    ("input_polarity", DIGITAL_INPUT_SETTINGS, INPUT_POLARITY, False),
    ("digital_input_logical", DIGITAL_IO_STATUS, DIGITAL_INPUT_LOGICAL, False),
    ("digital_input_physical", DIGITAL_IO_STATUS, DIGITAL_INPUT_PHYSICAL, False),
    ("following_window", FOLLOWING_ERROR_WINDOW, 0, False),
    ("following_timeout_ms", FOLLOWING_ERROR_TIMEOUT, 0, False),
    ("position_gain", POSITION_CONTROL_PARAMETER_SET, 1, False),
    ("rated_current_ma", MOTOR_APPLICATION_DATA, 1, False),
    ("continuous_current_ma", MOTOR_APPLICATION_DATA, 2, False),
    ("peak_current_ma", MOTOR_APPLICATION_DATA, 3, False),
    ("device_supply_lower_10mv", VOLTAGE_MONITOR, 1, False),
    ("motor_supply_lower_10mv", VOLTAGE_MONITOR, 2, False),
    ("motor_supply_max_10mv", VOLTAGE_MONITOR, 3, False),
    ("motor_supply_upper_10mv", VOLTAGE_MONITOR, 4, False),
    ("voltage_error_delay_ms", VOLTAGE_MONITOR, 5, False),
)

# The FAULHABER EtherCAT manual defines RxPDO2/TxPDO2 as the standard position
# process image for PP and CSP.  Their mapping-count subindices are read-only,
# so we keep the factory mapping and only assign these PDOs to SM2/SM3.
POSITION_RXPDO = 0x1601
POSITION_TXPDO = 0x1A01
PDO_RX_ASSIGNMENT = 0x1C12
PDO_TX_ASSIGNMENT = 0x1C13
SM2_PARAMETERS = 0x1C32
SM_CYCLE_TIME_SUBINDEX = 0x02
POSITION_RXPDO_ENTRIES = (0x60400010, 0x607A0020)
POSITION_TXPDO_ENTRIES = (0x60410010, 0x60640020)
PDO_RX_SIZE_BYTES = 6
PDO_TX_SIZE_BYTES = 6

PDOState = Tuple[int, int, int]  # actual_position, statusword, mode_display


class HomingIntervalResult(NamedTuple):
    """Encoder evidence and final zero readback for one interval-centre Home."""

    first_edge_counts: int
    second_edge_counts: int
    midpoint_counts: int
    midpoint_actual_counts: int
    zero_readback_counts: int


class FaulhaberDrive:
    """Convenience wrapper around one FAULHABER EtherCAT slave."""

    def __init__(self, slave, drive_id: int, sdo_delay_s: float, verbose: bool) -> None:
        self.slave = slave
        self.drive_id = drive_id
        self.sdo_delay_s = sdo_delay_s
        self.verbose = verbose

    def sdo_write_int(
        self,
        index: int,
        subindex: int,
        value: int,
        size: int,
        signed: bool = False,
    ) -> None:
        self.slave.sdo_write(index, subindex, int(value).to_bytes(size, "little", signed=signed))

    def sdo_read_int(self, index: int, subindex: int, signed: bool = False) -> int:
        data = self.slave.sdo_read(index, subindex)
        return int.from_bytes(data, "little", signed=signed)

    def sdo_write_int_retry(
        self,
        index: int,
        subindex: int,
        value: int,
        size: int,
        signed: bool = False,
        attempts: int = 3,
    ) -> None:
        """Retry an idempotent PRE-OP configuration write after mailbox WKC loss."""

        last_error: Optional[Exception] = None
        for attempt in range(1, attempts + 1):
            try:
                self.sdo_write_int(index, subindex, value, size, signed=signed)
                return
            except Exception as exc:
                last_error = exc
                if attempt < attempts:
                    time.sleep(max(self.sdo_delay_s, 0.02))
        assert last_error is not None
        raise last_error

    def sdo_read_int_retry(
        self,
        index: int,
        subindex: int,
        signed: bool = False,
        attempts: int = 3,
    ) -> int:
        """Retry a PRE-OP diagnostic/configuration read after mailbox WKC loss."""

        last_error: Optional[Exception] = None
        for attempt in range(1, attempts + 1):
            try:
                return self.sdo_read_int(index, subindex, signed=signed)
            except Exception as exc:
                last_error = exc
                if attempt < attempts:
                    time.sleep(max(self.sdo_delay_s, 0.02))
        assert last_error is not None
        raise last_error

    def read_status(self) -> int:
        return self.sdo_read_int(STATUS_WORD, 0, signed=False)

    def read_actual_position_counts(self) -> int:
        return self.sdo_read_int(ACTUAL_POSITION, 0, signed=True)

    def read_mode_display(self) -> int:
        return self.sdo_read_int(MODE_DISPLAY, 0, signed=True)

    def write_controlword(self, value: int, delay: Optional[float] = None) -> int:
        if self.verbose:
            print(f"[Drive {self.drive_id}] Controlword <- 0x{value:04X}")
        self.sdo_write_int(CONTROL_WORD, 0, value, size=2, signed=False)
        time.sleep(self.sdo_delay_s if delay is None else delay)
        return self.read_status()

    def reset_fault_if_needed(self) -> None:
        status = self.read_status()
        if status & STATUS_FAULT:
            print(f"[Drive {self.drive_id}] Fault detected. Sending fault reset.")
            self.write_controlword(CMD_FAULT_RESET, delay=0.5)

    def set_operation_mode(self, mode: int) -> int:
        self.sdo_write_int(MODE_OF_OPERATION, 0, mode, size=1, signed=True)
        time.sleep(self.sdo_delay_s)
        display = self.read_mode_display()
        if self.verbose:
            print(f"[Drive {self.drive_id}] mode requested={mode}, display={display}")
        return display

    def configure_profile_motion(self, velocity: int, acceleration: int, deceleration: int) -> None:
        if velocity > 0:
            self.sdo_write_int(PROFILE_VELOCITY, 0, velocity, size=4, signed=False)
        if acceleration > 0:
            self.sdo_write_int(PROFILE_ACCELERATION, 0, acceleration, size=4, signed=False)
        if deceleration > 0:
            self.sdo_write_int(PROFILE_DECELERATION, 0, deceleration, size=4, signed=False)

    def enable_operation(self, mode: int = MODE_PROFILE_POSITION) -> int:
        self.reset_fault_if_needed()
        if self.set_operation_mode(mode) != mode:
            raise RuntimeError(f"Drive {self.drive_id} rejected operation mode {mode}")
        self.write_controlword(CMD_SHUTDOWN)
        self.write_controlword(CMD_SWITCH_ON)
        status = self.write_controlword(CMD_ENABLE_OPERATION)
        if (status & STATUS_STATE_MASK) != STATUS_OPERATION_ENABLED_STATE:
            raise RuntimeError(
                f"Drive {self.drive_id} did not enter Operation Enabled; statusword=0x{status:04X}"
            )
        return status

    def disable_operation(self) -> int:
        status = self.write_controlword(CMD_DISABLE_OPERATION)
        self.write_controlword(CMD_DISABLE_VOLTAGE)
        return status

    def move_absolute_counts(self, target_counts: int) -> None:
        self.set_operation_mode(MODE_PROFILE_POSITION)
        self.sdo_write_int(TARGET_POSITION, 0, int(target_counts), size=4, signed=True)
        time.sleep(self.sdo_delay_s)
        self.write_controlword(CMD_ENABLE_OPERATION)
        self.write_controlword(CMD_START_MOTION)

    def move_absolute_counts_and_wait(self, target_counts: int, timeout_s: float) -> int:
        self.move_absolute_counts(target_counts)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            status = self.read_status()
            if status & STATUS_FAULT:
                raise RuntimeError(f"Fault during motion; statusword=0x{status:04X}")
            if status & STATUS_TARGET_REACHED:
                return self.read_actual_position_counts()
            time.sleep(0.05)
        raise TimeoutError(f"Drive {self.drive_id}: motion timed out after {timeout_s:.1f} seconds")

    def move_relative_counts_and_wait(
        self,
        delta_counts: int,
        timeout_s: float,
        tolerance_counts: int = 100,
        following_error_confirm_s: float = 0.0,
    ) -> Tuple[int, int, int]:
        """Execute and verify one Profile Position move relative to live feedback."""

        if delta_counts == 0:
            raise ValueError("Relative position increment must be non-zero")
        if timeout_s <= 0.0:
            raise ValueError("Motion timeout must be positive")
        if tolerance_counts < 0:
            raise ValueError("Position tolerance must be non-negative")
        if following_error_confirm_s < 0.0:
            raise ValueError("Following-error confirmation time must be non-negative")

        source_counts = self.read_actual_position_counts()
        target_counts = source_counts + int(delta_counts)
        self.move_absolute_counts(target_counts)
        deadline = time.monotonic() + timeout_s
        actual_counts = source_counts
        status = self.read_status()
        following_error_since: Optional[float] = None
        while time.monotonic() < deadline:
            now = time.monotonic()
            status = self.read_status()
            actual_counts = self.read_actual_position_counts()
            if status & STATUS_FAULT:
                raise RuntimeError(
                    f"Drive {self.drive_id}: fault during relative motion; "
                    f"statusword=0x{status:04X}"
                )
            if status & STATUS_FOLLOWING_OR_HOMING_ERROR:
                if following_error_since is None:
                    following_error_since = now
                    if self.verbose and following_error_confirm_s > 0.0:
                        print(
                            f"[Drive {self.drive_id}] transient following error; "
                            f"waiting up to {following_error_confirm_s:.2f} s for recovery "
                            f"(target={target_counts}, actual={actual_counts})"
                        )
                if (
                    following_error_confirm_s <= 0.0
                    or now - following_error_since >= following_error_confirm_s
                ):
                    raise RuntimeError(
                        f"Drive {self.drive_id}: following error persisted for "
                        f"{now - following_error_since:.3f} s during relative motion; "
                        f"source={source_counts} target={target_counts} "
                        f"actual={actual_counts} error={target_counts - actual_counts} "
                        f"statusword=0x{status:04X}"
                    )
            else:
                if following_error_since is not None and self.verbose:
                    print(
                        f"[Drive {self.drive_id}] transient following error cleared; "
                        f"target={target_counts}, actual={actual_counts}"
                    )
                following_error_since = None

            if following_error_since is None and (
                status & STATUS_TARGET_REACHED
                and abs(actual_counts - target_counts) <= tolerance_counts
            ):
                return source_counts, target_counts, actual_counts
            time.sleep(0.05)

        raise TimeoutError(
            f"Drive {self.drive_id}: relative motion timed out after {timeout_s:.1f} seconds; "
            f"source={source_counts} target={target_counts} actual={actual_counts} "
            f"statusword=0x{status:04X}"
        )

    def home_current_position(
        self,
        timeout_s: float,
        tolerance_counts: int = 10,
    ) -> int:
        """Use FAULHABER Homing Method 37 to make the current position zero."""

        if timeout_s <= 0.0:
            raise ValueError("Homing timeout must be positive")
        if tolerance_counts < 0:
            raise ValueError("Zero-position tolerance must be non-negative")

        self.reset_fault_if_needed()
        self.sdo_write_int(
            HOMING_METHOD,
            0,
            HOMING_METHOD_CURRENT_POSITION,
            size=1,
            signed=True,
        )
        self.sdo_write_int(HOMING_OFFSET, 0, 0, size=4, signed=True)
        self.write_controlword(CMD_SHUTDOWN)
        self.write_controlword(CMD_SWITCH_ON)
        if self.set_operation_mode(MODE_HOMING) != MODE_HOMING:
            raise RuntimeError(f"Drive {self.drive_id} did not enter Homing mode")
        status = self.write_controlword(CMD_ENABLE_OPERATION)
        if (status & STATUS_STATE_MASK) != STATUS_OPERATION_ENABLED_STATE:
            raise RuntimeError(
                f"Drive {self.drive_id} is not Operation Enabled; statusword=0x{status:04X}"
            )

        self.write_controlword(CMD_ENABLE_OPERATION)
        self.write_controlword(CMD_START_HOMING)
        deadline = time.monotonic() + timeout_s
        actual_counts = self.read_actual_position_counts()
        while time.monotonic() < deadline:
            status = self.read_status()
            if status & STATUS_FAULT:
                self.write_controlword(CMD_DISABLE_VOLTAGE)
                raise RuntimeError(
                    f"Drive {self.drive_id}: fault while setting current position as zero; "
                    f"statusword=0x{status:04X}"
                )
            if status & STATUS_FOLLOWING_OR_HOMING_ERROR:
                self.write_controlword(CMD_DISABLE_VOLTAGE)
                raise RuntimeError(
                    f"Drive {self.drive_id}: Homing Method 37 failed; "
                    f"statusword=0x{status:04X}"
                )
            if (status & STATUS_HOMING_ATTAINED) and (status & STATUS_TARGET_REACHED):
                actual_counts = self.read_actual_position_counts()
                if abs(actual_counts) > tolerance_counts:
                    raise RuntimeError(
                        f"Drive {self.drive_id}: Homing Method 37 completed but "
                        f"actual position is {actual_counts}, expected 0 "
                        f"(tolerance {tolerance_counts})"
                    )
                self.write_controlword(CMD_ENABLE_OPERATION)
                self.set_operation_mode(MODE_PROFILE_POSITION)
                return actual_counts
            time.sleep(0.05)

        self.write_controlword(CMD_DISABLE_OPERATION)
        self.write_controlword(CMD_DISABLE_VOLTAGE)
        raise TimeoutError(
            f"Drive {self.drive_id}: Homing Method 37 timed out after {timeout_s:.1f} seconds; "
            f"actual={actual_counts}"
        )

    def home_to_reference_switch(
        self,
        method: int,
        reference_input: int,
        offset_counts: int,
        search_speed: int,
        zero_speed: int,
        acceleration: int,
        timeout_s: float,
    ) -> int:
        """Run the reference-switch homing procedure validated on auto_homing."""

        if method not in (24, 28):
            raise ValueError(f"Drive {self.drive_id}: unsupported homing method {method}")
        if not 1 <= reference_input <= 8:
            raise ValueError(f"Drive {self.drive_id}: invalid reference input {reference_input}")
        if search_speed <= 0 or zero_speed <= 0 or acceleration <= 0:
            raise ValueError("Homing speeds and acceleration must be positive")

        self.reset_fault_if_needed()
        self.sdo_write_int(
            DIGITAL_INPUT_SETTINGS,
            REFERENCE_SWITCH_INPUT,
            reference_input,
            size=1,
            signed=False,
        )
        self.sdo_write_int(HOMING_METHOD, 0, method, size=1, signed=True)
        self.sdo_write_int(HOMING_OFFSET, 0, offset_counts, size=4, signed=True)
        self.sdo_write_int(HOMING_SPEED, HOMING_SEARCH_SPEED, search_speed, size=4)
        self.sdo_write_int(HOMING_SPEED, HOMING_ZERO_SPEED, zero_speed, size=4)
        self.sdo_write_int(HOMING_ACCELERATION, 0, acceleration, size=4)

        self.write_controlword(CMD_SHUTDOWN)
        self.write_controlword(CMD_SWITCH_ON)
        if self.set_operation_mode(MODE_HOMING) != MODE_HOMING:
            raise RuntimeError(f"Drive {self.drive_id} did not enter Homing mode")
        status = self.write_controlword(CMD_ENABLE_OPERATION)
        if (status & STATUS_STATE_MASK) != STATUS_OPERATION_ENABLED_STATE:
            raise RuntimeError(
                f"Drive {self.drive_id} is not Operation Enabled; statusword=0x{status:04X}"
            )

        # Generate the required bit-4 rising edge.
        self.write_controlword(CMD_ENABLE_OPERATION)
        self.write_controlword(CMD_START_HOMING)

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            status = self.read_status()
            if status & STATUS_FAULT:
                self.write_controlword(CMD_DISABLE_VOLTAGE)
                raise RuntimeError(
                    f"Drive {self.drive_id}: fault during homing; statusword=0x{status:04X}"
                )
            if status & STATUS_FOLLOWING_OR_HOMING_ERROR:
                self.write_controlword(CMD_DISABLE_VOLTAGE)
                raise RuntimeError(
                    f"Drive {self.drive_id}: homing error; statusword=0x{status:04X}"
                )
            if (status & STATUS_HOMING_ATTAINED) and (status & STATUS_TARGET_REACHED):
                self.write_controlword(CMD_ENABLE_OPERATION)
                self.set_operation_mode(MODE_PROFILE_POSITION)
                return self.read_actual_position_counts()
            time.sleep(0.05)

        self.write_controlword(CMD_DISABLE_OPERATION)
        self.write_controlword(CMD_DISABLE_VOLTAGE)
        raise TimeoutError(
            f"Drive {self.drive_id}: homing timed out after {timeout_s:.1f} seconds"
        )

    @staticmethod
    def homing_search_direction(method: int) -> int:
        """Return the encoder direction used by the configured switch method."""

        if method == 24:
            return 1
        if method == 28:
            return -1
        raise ValueError(f"Unsupported interval-centre homing method {method}")

    def read_reference_input_active(self, reference_input: int) -> bool:
        """Read one polarity-corrected reference input from 0x2311:01."""

        if not 1 <= reference_input <= 8:
            raise ValueError(f"Drive {self.drive_id}: invalid reference input {reference_input}")
        logical_inputs = self.sdo_read_int(
            DIGITAL_IO_STATUS, DIGITAL_INPUT_LOGICAL, signed=False
        )
        return bool(logical_inputs & (1 << (reference_input - 1)))

    def halt_profile_position_motion(self, timeout_s: float) -> int:
        """Decelerate a Profile Position move while retaining motor torque."""

        if timeout_s <= 0.0:
            raise ValueError("Halt timeout must be positive")

        self.write_controlword(CMD_HALT)
        deadline = time.monotonic() + timeout_s
        actual_counts = self.read_actual_position_counts()
        velocity = self.sdo_read_int(VELOCITY_ACTUAL_VALUE, 0, signed=True)
        while time.monotonic() < deadline:
            status = self.read_status()
            actual_counts = self.read_actual_position_counts()
            velocity = self.sdo_read_int(VELOCITY_ACTUAL_VALUE, 0, signed=True)
            if status & STATUS_FAULT:
                self.write_controlword(CMD_DISABLE_VOLTAGE)
                raise RuntimeError(
                    f"Drive {self.drive_id}: fault while halting interval traversal; "
                    f"statusword=0x{status:04X}"
                )
            if abs(velocity) <= 1:
                self.write_controlword(CMD_ENABLE_OPERATION)
                return actual_counts
            time.sleep(0.01)

        self.write_controlword(CMD_DISABLE_VOLTAGE)
        raise TimeoutError(
            f"Drive {self.drive_id}: interval traversal did not halt within "
            f"{timeout_s:.1f} seconds; actual={actual_counts}, velocity={velocity}"
        )

    def home_to_reference_interval_midpoint(
        self,
        method: int,
        reference_input: int,
        offset_counts: int,
        search_speed: int,
        zero_speed: int,
        acceleration: int,
        timeout_s: float,
        interval_timeout_s: float,
        max_travel_counts: int,
        poll_s: float,
        midpoint_tolerance_counts: int,
    ) -> HomingIntervalResult:
        """Find both switch edges, return to their midpoint, and zero there.

        The drive's native Homing method first finds the proven entry edge. The
        bridge then traverses the active reference interval in the same encoder
        direction at the lower ``zero_speed``. The first inactive sample is the
        second edge. A sinusoidal Profile Position curve is used for this
        traversal and the midpoint return. Method 37 establishes the returned
        midpoint as zero.
        """

        if max_travel_counts <= 0:
            raise ValueError("Homing interval maximum travel must be positive")
        if timeout_s <= 0.0:
            raise ValueError("Homing interval timeout must be positive")
        if interval_timeout_s <= 0.0:
            raise ValueError("Homing interval traversal timeout must be positive")
        if poll_s <= 0.0:
            raise ValueError("Homing interval poll period must be positive")
        if midpoint_tolerance_counts < 0:
            raise ValueError("Homing midpoint tolerance must be non-negative")
        if offset_counts != 0:
            raise ValueError(
                f"Drive {self.drive_id}: interval-centre Homing requires "
                f"0x607C=0, got offset_counts={offset_counts}"
            )

        direction = self.homing_search_direction(method)
        first_edge_stop_counts = self.home_to_reference_switch(
            method=method,
            reference_input=reference_input,
            offset_counts=offset_counts,
            search_speed=search_speed,
            zero_speed=zero_speed,
            acceleration=acceleration,
            timeout_s=timeout_s,
        )
        # Native Homing latches the electrical edge and, with 0x607C=0,
        # defines that captured position as exactly zero. 0x6064 read after
        # completion is the decelerated stop position and must not replace the
        # captured edge coordinate in the midpoint calculation.
        first_edge_counts = 0
        if self.verbose:
            print(
                f"[Drive {self.drive_id}] first Homing edge=0 counts; "
                f"post-edge stop={first_edge_stop_counts} counts"
            )
        search_start_counts = self.read_actual_position_counts()
        search_target_counts = first_edge_counts + direction * max_travel_counts
        if not -(1 << 31) <= search_target_counts <= (1 << 31) - 1:
            raise ValueError(
                f"Drive {self.drive_id}: interval search target "
                f"{search_target_counts} exceeds signed 32-bit position range"
            )

        original_profile = (
            self.sdo_read_int(PROFILE_VELOCITY, 0, signed=False),
            self.sdo_read_int(PROFILE_ACCELERATION, 0, signed=False),
            self.sdo_read_int(PROFILE_DECELERATION, 0, signed=False),
            self.sdo_read_int(MOTION_PROFILE_TYPE, 0, signed=True),
        )
        motion_active = False
        second_edge_counts: Optional[int] = None
        midpoint_counts: Optional[int] = None
        midpoint_actual_counts: Optional[int] = None
        zero_readback_counts: Optional[int] = None
        try:
            # The first native edge may be sampled on either side of the
            # electrical transition. Do not accept an inactive state as the
            # second edge until the active interval has actually been observed.
            active_interval_seen = self.read_reference_input_active(reference_input)
            # The native switch-seek speed is intentionally not reused here:
            # the opposite edge and midpoint need the slower Homing speed.
            # A sinusoidal acceleration profile reduces excitation of the
            # geared arm while stopping at the edge and reversing to midpoint.
            self.sdo_write_int(
                MOTION_PROFILE_TYPE, 0, 1, size=2, signed=True
            )
            self.configure_profile_motion(zero_speed, acceleration, acceleration)
            self.move_absolute_counts(search_target_counts)
            motion_active = True
            deadline = time.monotonic() + interval_timeout_s
            actual_counts = search_start_counts
            status = self.read_status()
            internal_limit_seen = bool(status & STATUS_INTERNAL_LIMIT_ACTIVE)

            while time.monotonic() < deadline:
                status = self.read_status()
                actual_counts = self.read_actual_position_counts()
                reference_active = self.read_reference_input_active(reference_input)
                internal_limit_seen = internal_limit_seen or bool(
                    status & STATUS_INTERNAL_LIMIT_ACTIVE
                )
                if status & STATUS_FAULT:
                    raise RuntimeError(
                        f"Drive {self.drive_id}: fault while traversing Homing interval; "
                        f"statusword=0x{status:04X}"
                    )
                if status & STATUS_FOLLOWING_OR_HOMING_ERROR:
                    raise RuntimeError(
                        f"Drive {self.drive_id}: following error while traversing "
                        f"Homing interval; actual={actual_counts}, "
                        f"statusword=0x{status:04X}"
                    )
                if reference_active:
                    active_interval_seen = True
                elif (
                    active_interval_seen
                    and abs(actual_counts - first_edge_counts) > 0
                ):
                    second_edge_counts = actual_counts
                    break

                if status & STATUS_TARGET_REACHED:
                    raise RuntimeError(
                        f"Drive {self.drive_id}: reference input did not become inactive "
                        f"within {max_travel_counts} counts after the first edge; "
                        f"last_actual={actual_counts}, "
                        f"reference_active={str(reference_active).lower()}, "
                        f"active_interval_seen={str(active_interval_seen).lower()}, "
                        f"internal_limit_seen={str(internal_limit_seen).lower()}"
                    )
                time.sleep(poll_s)

            if second_edge_counts is None:
                raise TimeoutError(
                    f"Drive {self.drive_id}: second Homing edge was not found within "
                    f"{interval_timeout_s:.1f} seconds; first_edge={first_edge_counts}, "
                    f"last_actual={actual_counts}, reference_active="
                    f"{str(reference_active).lower()}, internal_limit_seen="
                    f"{str(internal_limit_seen).lower()}"
                )

            self.halt_profile_position_motion(timeout_s)
            motion_active = False
            directed_width = (second_edge_counts - first_edge_counts) * direction
            if directed_width <= 0:
                raise RuntimeError(
                    f"Drive {self.drive_id}: invalid Homing interval order; "
                    f"direction={direction:+d}, first_edge={first_edge_counts}, "
                    f"second_edge={second_edge_counts}"
                )

            midpoint_counts = first_edge_counts + (
                second_edge_counts - first_edge_counts
            ) // 2
            motion_active = True
            midpoint_actual_counts = self.move_absolute_counts_and_wait(
                midpoint_counts, interval_timeout_s
            )
            motion_active = False
            if (
                abs(midpoint_actual_counts - midpoint_counts)
                > midpoint_tolerance_counts
            ):
                raise RuntimeError(
                    f"Drive {self.drive_id}: midpoint move ended at "
                    f"{midpoint_actual_counts}, expected {midpoint_counts} "
                    f"(tolerance {midpoint_tolerance_counts})"
                )
            zero_readback_counts = self.home_current_position(
                timeout_s, midpoint_tolerance_counts
            )
        except Exception:
            if motion_active:
                try:
                    self.halt_profile_position_motion(timeout_s)
                except Exception as stop_exc:
                    try:
                        self.write_controlword(CMD_DISABLE_VOLTAGE)
                    except Exception:
                        pass
                    print(
                        f"[Drive {self.drive_id}] Homing interval failure and halt failed: "
                        f"{stop_exc}"
                    )
            raise
        finally:
            try:
                self.sdo_write_int(
                    PROFILE_VELOCITY, 0, original_profile[0], size=4, signed=False
                )
                self.sdo_write_int(
                    PROFILE_ACCELERATION, 0, original_profile[1], size=4, signed=False
                )
                self.sdo_write_int(
                    PROFILE_DECELERATION, 0, original_profile[2], size=4, signed=False
                )
                self.sdo_write_int(
                    MOTION_PROFILE_TYPE, 0, original_profile[3], size=2, signed=True
                )
            except Exception as restore_exc:
                print(
                    f"[Drive {self.drive_id}] Could not restore Profile Position "
                    f"parameters after Homing interval search: {restore_exc}"
                )

        assert second_edge_counts is not None
        assert midpoint_counts is not None
        assert midpoint_actual_counts is not None
        assert zero_readback_counts is not None
        return HomingIntervalResult(
            first_edge_counts=first_edge_counts,
            second_edge_counts=second_edge_counts,
            midpoint_counts=midpoint_counts,
            midpoint_actual_counts=midpoint_actual_counts,
            zero_readback_counts=zero_readback_counts,
        )

    def set_digout1(self, on: bool) -> None:
        value = DIGOUT1_ON if on else DIGOUT1_OFF
        self.sdo_write_int(DIGITAL_IO_STATUS, DIGOUT_WRITE, value, size=2, signed=False)

    def toggle_digout1(self) -> None:
        self.sdo_write_int(
            DIGITAL_IO_STATUS,
            DIGOUT_WRITE,
            DIGOUT1_TOGGLE,
            size=2,
            signed=False,
        )

    def read_digital_inputs(self) -> Tuple[int, int, int]:
        logical = self.sdo_read_int(DIGITAL_IO_STATUS, DIGITAL_INPUT_LOGICAL)
        physical = self.sdo_read_int(DIGITAL_IO_STATUS, DIGITAL_INPUT_PHYSICAL)
        polarity = self.sdo_read_int(DIGITAL_INPUT_SETTINGS, INPUT_POLARITY)
        return logical, physical, polarity

    def read_digital_input_configuration(self) -> dict[str, int]:
        """Read the input states and every mapping relevant to Homing/CSP limits."""

        return {
            "lower_limit_input_mask": self.sdo_read_int_retry(
                DIGITAL_INPUT_SETTINGS, LOWER_LIMIT_SWITCH_INPUTS
            ),
            "upper_limit_input_mask": self.sdo_read_int_retry(
                DIGITAL_INPUT_SETTINGS, UPPER_LIMIT_SWITCH_INPUTS
            ),
            "limit_switch_option": self.sdo_read_int_retry(
                DIGITAL_INPUT_SETTINGS, LIMIT_SWITCH_OPTION_CODE, signed=True
            ),
            "reference_input": self.sdo_read_int_retry(
                DIGITAL_INPUT_SETTINGS, REFERENCE_SWITCH_INPUT
            ),
            "input_polarity": self.sdo_read_int_retry(
                DIGITAL_INPUT_SETTINGS, INPUT_POLARITY
            ),
            "logical_inputs": self.sdo_read_int_retry(
                DIGITAL_IO_STATUS, DIGITAL_INPUT_LOGICAL
            ),
            "physical_inputs": self.sdo_read_int_retry(
                DIGITAL_IO_STATUS, DIGITAL_INPUT_PHYSICAL
            ),
            "device_status": self.sdo_read_int_retry(DEVICE_STATUS, 1),
        }

    def clear_limit_switch_mappings_for_csp(
        self,
    ) -> Tuple[dict[str, int], dict[str, int]]:
        """Remove stale lower/upper limit-input mappings and verify the live result.

        Automatic Homing uses the dedicated reference input at ``0x2310:04``.
        The robot does not use the same sensor inputs as persistent lower/upper
        travel switches.  Old mappings at ``0x2310:01/:02`` can therefore stop a
        valid CSP move while still allowing Homing to finish.  Only those two
        volatile mappings are cleared; reference input, polarity, limit-stop
        behavior, and the position limits at ``0x607B/0x607D`` are untouched.
        No parameter-store command is issued.
        """

        before = self.read_digital_input_configuration()
        for subindex in (LOWER_LIMIT_SWITCH_INPUTS, UPPER_LIMIT_SWITCH_INPUTS):
            self.sdo_write_int_retry(
                DIGITAL_INPUT_SETTINGS,
                subindex,
                0,
                size=1,
                signed=False,
            )

        # Give the drive's cyclic diagnosis one update before checking 0x2324.
        time.sleep(max(self.sdo_delay_s, 0.02))
        after = self.read_digital_input_configuration()
        if (
            after["lower_limit_input_mask"] != 0
            or after["upper_limit_input_mask"] != 0
        ):
            raise RuntimeError(
                f"Drive {self.drive_id} limit-input readback mismatch: "
                f"lower=0x{after['lower_limit_input_mask']:02X}, "
                f"upper=0x{after['upper_limit_input_mask']:02X}"
            )
        if after["device_status"] & ((1 << 6) | (1 << 7)):
            raise RuntimeError(
                f"Drive {self.drive_id} still reports an active physical limit after "
                "0x2310:01/:02 were cleared; "
                f"0x2324.01=0x{after['device_status']:08X}"
            )
        return before, after

    def read_position_protection(self) -> dict[str, int]:
        """Read Drive-side limits and following-error settings without changing them.

        Both ``0x607B`` and ``0x607D`` are in the target-position path for CSP.
        They are intentionally only observed here: the physical travel range of
        the arm is not known accurately enough to replace a persisted drive
        safety limit from software.
        """

        return {
            "position_range_min": self.sdo_read_int_retry(
                POSITION_RANGE_LIMIT, 1, signed=True
            ),
            "position_range_max": self.sdo_read_int_retry(
                POSITION_RANGE_LIMIT, 2, signed=True
            ),
            "software_limit_min": self.sdo_read_int_retry(
                SOFTWARE_POSITION_LIMIT, 1, signed=True
            ),
            "software_limit_max": self.sdo_read_int_retry(
                SOFTWARE_POSITION_LIMIT, 2, signed=True
            ),
            "following_error_window": self.sdo_read_int_retry(
                FOLLOWING_ERROR_WINDOW, 0, signed=False
            ),
            "following_error_timeout_ms": self.sdo_read_int_retry(
                FOLLOWING_ERROR_TIMEOUT, 0, signed=False
            ),
        }

    def configure_following_error_monitor(
        self, window_counts: int, timeout_ms: int
    ) -> None:
        """Set and read back the CiA-402 following-error monitor for this session."""

        if not 1 <= int(window_counts) <= 0xFFFFFFFF:
            raise ValueError("following-error window must be an unsigned 32-bit positive count")
        if not 1 <= int(timeout_ms) <= 0xFFFF:
            raise ValueError("following-error timeout must be 1..65535 ms")

        self.sdo_write_int_retry(
            FOLLOWING_ERROR_WINDOW, 0, int(window_counts), size=4, signed=False
        )
        self.sdo_write_int_retry(
            FOLLOWING_ERROR_TIMEOUT, 0, int(timeout_ms), size=2, signed=False
        )
        configured_window = self.sdo_read_int_retry(FOLLOWING_ERROR_WINDOW, 0)
        configured_timeout = self.sdo_read_int_retry(FOLLOWING_ERROR_TIMEOUT, 0)
        if configured_window != int(window_counts) or configured_timeout != int(timeout_ms):
            raise RuntimeError(
                "following-error readback mismatch: "
                f"window={configured_window} counts, timeout={configured_timeout} ms"
            )

    def read_torque_limits(self, retry: bool = True) -> dict[str, int]:
        """Read the three CiA-402 torque limit objects in rated-torque per-mille."""

        read = self.sdo_read_int_retry if retry else self.sdo_read_int
        return {
            "maximum_torque": read(MAX_TORQUE, 0, signed=False),
            "positive_torque_limit": read(
                POSITIVE_TORQUE_LIMIT, 0, signed=False
            ),
            "negative_torque_limit": read(
                NEGATIVE_TORQUE_LIMIT, 0, signed=False
            ),
        }

    def configure_csp_torque_limit(
        self, limit_per_mille: int
    ) -> Tuple[dict[str, int], dict[str, int]]:
        """Apply and verify the writable directional CSP torque limits.

        ``1000`` is 100% of rated motor torque.  No CiA-301 parameter-store
        request is issued, so power-cycling restores the controller's stored
        configuration.  The MC5004 EtherCAT firmware used by this robot exposes
        ``0x6072`` as read-only, so it is observed before/after but never
        written; ``0x60E0`` and ``0x60E1`` are the writable application limits.
        """

        limit = int(limit_per_mille)
        if not 1 <= limit <= MAX_TORQUE_PER_MILLE:
            raise ValueError(
                f"CSP torque limit must be 1..{MAX_TORQUE_PER_MILLE} per-mille"
            )

        before = self.read_torque_limits(retry=True)
        for index in (POSITIVE_TORQUE_LIMIT, NEGATIVE_TORQUE_LIMIT):
            self.sdo_write_int_retry(index, 0, limit, size=2, signed=False)
        after = self.read_torque_limits(retry=True)
        if (
            after["positive_torque_limit"] != limit
            or after["negative_torque_limit"] != limit
        ):
            raise RuntimeError(
                f"Drive {self.drive_id} CSP directional torque-limit readback "
                f"mismatch: expected positive/negative={limit}/{limit}, "
                f"actual={after['positive_torque_limit']}/"
                f"{after['negative_torque_limit']}"
            )
        return before, after

    def read_motor_current_parameters(self) -> dict[str, int]:
        """Read rated, continuous and peak motor currents from 0x2329 in mA."""

        return {
            "rated_current_ma": self.sdo_read_int_retry(
                MOTOR_APPLICATION_DATA, 1, signed=False
            ),
            "continuous_current_ma": self.sdo_read_int_retry(
                MOTOR_APPLICATION_DATA, 2, signed=False
            ),
            "peak_current_ma": self.sdo_read_int_retry(
                MOTOR_APPLICATION_DATA, 3, signed=False
            ),
        }

    def ensure_peak_current_for_torque_limit(
        self, limit_per_mille: int
    ) -> Tuple[dict[str, int], dict[str, int], int]:
        """Raise an undersized peak current for the requested torque limit.

        On this MC5004 firmware, read-only ``0x6072`` is derived from
        ``peak_current / rated_current * 1000``.  An undersized peak-current
        parameter therefore caps the effective torque even when writable
        ``0x60E0/0x60E1`` are higher.  At the default 1000-per-mille limit this
        raises peak current to rated current.  The session-only correction
        leaves the rated and continuous-current motor parameters unchanged.
        """

        before = self.read_motor_current_parameters()
        rated_current = before["rated_current_ma"]
        if not 1 <= rated_current <= 0xFFFF:
            raise RuntimeError(
                f"Drive {self.drive_id} invalid rated current: {rated_current} mA"
            )

        required_peak_current = (
            rated_current * int(limit_per_mille) + 999
        ) // 1000
        if not 1 <= required_peak_current <= 0xFFFF:
            raise RuntimeError(
                f"Drive {self.drive_id} required peak current is outside U16: "
                f"{required_peak_current} mA"
            )

        if before["peak_current_ma"] < required_peak_current:
            self.sdo_write_int_retry(
                MOTOR_APPLICATION_DATA,
                3,
                required_peak_current,
                size=2,
                signed=False,
            )
            time.sleep(max(self.sdo_delay_s, 0.02))

        after = self.read_motor_current_parameters()
        if after["peak_current_ma"] < required_peak_current:
            raise RuntimeError(
                f"Drive {self.drive_id} peak-current readback remains below required: "
                f"required={required_peak_current} mA, "
                f"peak={after['peak_current_ma']} mA"
            )

        effective_maximum = self.sdo_read_int_retry(MAX_TORQUE, 0, signed=False)
        if effective_maximum < int(limit_per_mille):
            raise RuntimeError(
                f"Drive {self.drive_id} read-only 0x6072 remains {effective_maximum} "
                "after peak-current correction; expected at least "
                f"{int(limit_per_mille)}"
            )
        return before, after, effective_maximum

    @staticmethod
    def _decode_flags(value: int, definitions: dict[int, str]) -> str:
        flags = [name for bit, name in definitions.items() if value & (1 << bit)]
        return ",".join(flags) if flags else "none"

    def capture_fault_diagnostics(self) -> str:
        """Collect a best-effort, read-only SDO snapshot immediately after a fault.

        This is called from the PDO thread before it asks EtherCAT for SAFE-OP.
        SDO failures are recorded in the returned text and never replace the
        original PDO fault, which is the safety-critical event.
        """

        unavailable: list[str] = []

        def read(
            label: str, index: int, subindex: int = 0, signed: bool = False
        ) -> Optional[int]:
            try:
                return self.sdo_read_int(index, subindex, signed=signed)
            except Exception as exc:
                unavailable.append(f"{label}:{type(exc).__name__}")
                return None

        device_status = read("device_status", DEVICE_STATUS, 1)
        error_register = read("error_register", ERROR_REGISTER)
        error_history_count = read("error_history_count", PREDEFINED_ERROR_FIELD)
        error_history: List[int] = []
        if error_history_count is not None:
            for subindex in range(1, min(int(error_history_count), 8) + 1):
                entry = read(f"error_history_{subindex}", PREDEFINED_ERROR_FIELD, subindex)
                if entry is not None:
                    error_history.append(entry)

        position_demand = read("position_demand", POSITION_DEMAND_VALUE, signed=True)
        position_actual = read("position_actual", ACTUAL_POSITION, signed=True)
        following_actual = read(
            "following_error_actual", FOLLOWING_ERROR_ACTUAL_VALUE, signed=True
        )
        velocity_actual = read("velocity_actual", VELOCITY_ACTUAL_VALUE, signed=True)
        torque_demand = read("torque_demand", TORQUE_DEMAND, signed=True)
        torque_actual = read("torque_actual", ACTUAL_TORQUE, signed=True)
        current_actual = read("current_actual", ACTUAL_CURRENT, signed=True)
        device_supply_actual = read(
            "device_supply_actual", VOLTAGE_MONITOR, 6
        )
        motor_supply_actual = read(
            "motor_supply_actual", VOLTAGE_MONITOR, 7
        )
        maximum_torque = read("maximum_torque", MAX_TORQUE)
        positive_torque_limit = read("positive_torque_limit", POSITIVE_TORQUE_LIMIT)
        negative_torque_limit = read("negative_torque_limit", NEGATIVE_TORQUE_LIMIT)
        maximum_motor_speed = read("maximum_motor_speed", MAX_MOTOR_SPEED)
        position_gain = read("position_gain", POSITION_CONTROL_PARAMETER_SET, 1)
        rated_current = read("rated_current", MOTOR_APPLICATION_DATA, 1)
        continuous_current = read("continuous_current", MOTOR_APPLICATION_DATA, 2)
        peak_current = read("peak_current", MOTOR_APPLICATION_DATA, 3)
        lower_limit_input_mask = read(
            "lower_limit_input_mask",
            DIGITAL_INPUT_SETTINGS,
            LOWER_LIMIT_SWITCH_INPUTS,
        )
        upper_limit_input_mask = read(
            "upper_limit_input_mask",
            DIGITAL_INPUT_SETTINGS,
            UPPER_LIMIT_SWITCH_INPUTS,
        )
        limit_switch_option = read(
            "limit_switch_option",
            DIGITAL_INPUT_SETTINGS,
            LIMIT_SWITCH_OPTION_CODE,
            signed=True,
        )
        reference_input = read(
            "reference_input",
            DIGITAL_INPUT_SETTINGS,
            REFERENCE_SWITCH_INPUT,
        )
        input_polarity = read(
            "input_polarity",
            DIGITAL_INPUT_SETTINGS,
            INPUT_POLARITY,
        )
        logical_inputs = read(
            "logical_inputs",
            DIGITAL_IO_STATUS,
            DIGITAL_INPUT_LOGICAL,
        )
        physical_inputs = read(
            "physical_inputs",
            DIGITAL_IO_STATUS,
            DIGITAL_INPUT_PHYSICAL,
        )

        parts = [f"DRIVE_DIAG D{self.drive_id}"]
        if device_status is not None:
            parts.append(
                f"0x2324.01=0x{device_status:08X}"
                f"[{self._decode_flags(device_status, DEVICE_STATUS_FLAGS)}]"
            )
        if error_register is not None:
            parts.append(
                f"0x1001=0x{error_register:02X}"
                f"[{self._decode_flags(error_register, ERROR_REGISTER_FLAGS)}]"
            )
        parts.append(
            "0x1003=[" + ",".join(f"0x{entry:08X}" for entry in error_history) + "]"
        )
        parts.append(f"0x6062={position_demand}")
        parts.append(f"0x6064={position_actual}")
        parts.append(f"0x60F4={following_actual}")
        parts.append(f"0x606C={velocity_actual}")
        parts.append(f"0x6074/0x6077(demand/actual)={torque_demand}/{torque_actual}")
        parts.append(f"0x6078(current_actual)={current_actual}")
        parts.append(
            "voltage_10mV(0x2325.06/.07 device/motor_actual)="
            f"{device_supply_actual}/{motor_supply_actual}"
        )
        parts.append(
            "limits(0x6072/0x60E0/0x60E1/0x6080)="
            f"{maximum_torque}/{positive_torque_limit}/{negative_torque_limit}/"
            f"{maximum_motor_speed}"
        )
        parts.append(
            "motor_mA(0x2329.01/.02/.03 rated/continuous/peak)="
            f"{rated_current}/{continuous_current}/{peak_current}"
        )
        parts.append(
            "input_config(0x2310.01/.02/.03/.04/.10 "
            "lower/upper/option/reference/polarity)="
            f"{lower_limit_input_mask}/{upper_limit_input_mask}/"
            f"{limit_switch_option}/{reference_input}/{input_polarity}"
        )
        parts.append(
            "input_state(0x2311.01/.02 logical/physical)="
            f"{logical_inputs}/{physical_inputs}"
        )
        parts.append(f"0x2348.01(Kv)={position_gain}")
        if unavailable:
            parts.append("unavailable=" + ",".join(unavailable))
        return "; ".join(parts)

    def format_live_diagnostics(
        self, values: Dict[str, int], unavailable: Sequence[str]
    ) -> str:
        """Format a staged, read-only diagnostic sample for a live CSP stall."""

        device_status = values.get("device_status")
        error_register = values.get("error_register")
        parts = [f"LIVE_DIAG D{self.drive_id}"]
        if device_status is not None:
            parts.append(
                f"0x2324.01=0x{device_status:08X}"
                f"[{self._decode_flags(device_status, DEVICE_STATUS_FLAGS)}]"
            )
        if error_register is not None:
            parts.append(
                f"0x1001=0x{error_register:02X}"
                f"[{self._decode_flags(error_register, ERROR_REGISTER_FLAGS)}]"
            )
        parts.append(
            "position(0x6062/0x6064/0x60F4 demand/actual/following)="
            f"{values.get('position_demand')}/{values.get('position_actual')}/"
            f"{values.get('following_error_actual')}"
        )
        parts.append(f"0x606C(velocity)={values.get('velocity_actual')}")
        parts.append(
            "torque(0x6074/0x6077 demand/actual)="
            f"{values.get('torque_demand')}/{values.get('torque_actual')}"
        )
        parts.append(f"0x6078(current)={values.get('current_actual')}")
        parts.append(
            "torque_speed_limits(0x6072/0x60E0/0x60E1/0x6080)="
            f"{values.get('maximum_torque')}/{values.get('positive_torque_limit')}/"
            f"{values.get('negative_torque_limit')}/{values.get('maximum_motor_speed')}"
        )
        parts.append(
            "position_range(0x607B)="
            f"[{values.get('position_range_min')},{values.get('position_range_max')}]"
        )
        parts.append(
            "software_limit(0x607D)="
            f"[{values.get('software_limit_min')},{values.get('software_limit_max')}]"
        )
        parts.append(
            "input_config(0x2310.01/.02/.03/.04/.10 "
            "lower/upper/option/reference/polarity)="
            f"{values.get('lower_limit_input_mask')}/"
            f"{values.get('upper_limit_input_mask')}/"
            f"{values.get('limit_switch_option')}/"
            f"{values.get('reference_input')}/"
            f"{values.get('input_polarity')}"
        )
        parts.append(
            "input_state(0x2311.01/.02 logical/physical)="
            f"{values.get('digital_input_logical')}/"
            f"{values.get('digital_input_physical')}"
        )
        parts.append(
            "following_monitor(0x6065/0x6066)="
            f"{values.get('following_window')}/{values.get('following_timeout_ms')}"
        )
        parts.append(f"0x2348.01(Kv)={values.get('position_gain')}")
        parts.append(
            "motor_mA(0x2329.01/.02/.03 rated/continuous/peak)="
            f"{values.get('rated_current_ma')}/{values.get('continuous_current_ma')}/"
            f"{values.get('peak_current_ma')}"
        )
        parts.append(
            "voltage_10mV(0x2325.01-.07 device_low/motor_low/motor_max/"
            "motor_high/delay_ms/device_actual/motor_actual)="
            f"{values.get('device_supply_lower_10mv')}/"
            f"{values.get('motor_supply_lower_10mv')}/"
            f"{values.get('motor_supply_max_10mv')}/"
            f"{values.get('motor_supply_upper_10mv')}/"
            f"{values.get('voltage_error_delay_ms')}/"
            f"{values.get('device_supply_actual_10mv')}/"
            f"{values.get('motor_supply_actual_10mv')}"
        )
        if unavailable:
            parts.append("unavailable=" + ",".join(unavailable))
        return "; ".join(parts)


class FaulhaberBus:
    """Own the pysoem master and the deterministic CSP/PDO exchange thread."""

    def __init__(
        self,
        interface: str,
        slave_indices: List[int],
        sdo_delay_s: float,
        verbose: bool,
        control_mode: str,
        pdo_cycle_ns: int,
        pdo_timeout_us: int,
        enable_dc_sync: bool,
        csp_torque_limit_per_mille: int = 1000,
        ignored_csp_drive_indices: Optional[Sequence[int]] = None,
        required_homing_drive_indices: Optional[Sequence[int]] = None,
        drive2_following_error_window_counts: int = 25_000,
        drive2_following_error_timeout_ms: int = 250,
        csp_stall_error_counts: int = 25_000,
        csp_stall_progress_counts: int = 100,
        csp_stall_timeout_ms: int = 500,
        diagnostic_logger: Optional[Callable[[str], None]] = None,
        clear_limit_switch_mappings_for_csp: bool = True,
    ) -> None:
        self.interface = interface
        self.slave_indices = slave_indices
        self.sdo_delay_s = sdo_delay_s
        self.verbose = verbose
        self.control_mode = control_mode
        self.pdo_cycle_ns = pdo_cycle_ns
        self.pdo_timeout_us = pdo_timeout_us
        self.enable_dc_sync = enable_dc_sync
        self.csp_torque_limit_per_mille = int(csp_torque_limit_per_mille)
        if not 1 <= self.csp_torque_limit_per_mille <= MAX_TORQUE_PER_MILLE:
            raise ValueError(
                "csp_torque_limit_per_mille must be "
                f"1..{MAX_TORQUE_PER_MILLE}"
            )
        self.csp_stall_error_counts = int(csp_stall_error_counts)
        self.csp_stall_progress_counts = int(csp_stall_progress_counts)
        self.csp_stall_timeout_ms = int(csp_stall_timeout_ms)
        self.drive2_following_error_window_counts = int(
            drive2_following_error_window_counts
        )
        self.drive2_following_error_timeout_ms = int(
            drive2_following_error_timeout_ms
        )
        if not 1 <= self.drive2_following_error_window_counts <= 0xFFFFFFFF:
            raise ValueError(
                "drive2_following_error_window_counts must be an unsigned "
                "32-bit positive count"
            )
        if not 1 <= self.drive2_following_error_timeout_ms <= 0xFFFF:
            raise ValueError(
                "drive2_following_error_timeout_ms must be 1..65535"
            )
        if self.csp_stall_error_counts <= 0:
            raise ValueError("csp_stall_error_counts must be positive")
        if self.csp_stall_progress_counts <= 0:
            raise ValueError("csp_stall_progress_counts must be positive")
        if self.csp_stall_timeout_ms <= 0:
            raise ValueError("csp_stall_timeout_ms must be positive")
        self.diagnostic_logger = diagnostic_logger or (
            lambda message: print(f"[EtherCAT] {message}")
        )
        self.clear_limit_switch_mappings_for_csp = bool(
            clear_limit_switch_mappings_for_csp
        )
        self.ignored_csp_drive_ids = {
            int(index) for index in (ignored_csp_drive_indices or [])
        }
        invalid_ignored = self.ignored_csp_drive_ids.difference(
            range(len(self.slave_indices))
        )
        if invalid_ignored:
            raise ValueError(f"Invalid ignored CSP drive indices: {sorted(invalid_ignored)}")
        self.required_csp_drive_ids = set(range(len(self.slave_indices))).difference(
            self.ignored_csp_drive_ids
        )
        self.required_homing_drive_ids = (
            set(self.required_csp_drive_ids)
            if required_homing_drive_indices is None
            else {int(index) for index in required_homing_drive_indices}
        )
        invalid_homing = self.required_homing_drive_ids.difference(
            self.required_csp_drive_ids
        )
        if invalid_homing:
            raise ValueError(
                "Homing-required drives cannot be CSP-ignored: "
                f"{sorted(invalid_homing)}"
            )
        if not self.required_homing_drive_ids:
            raise ValueError("At least one drive must remain enabled for CSP")
        self.non_homing_csp_drive_ids = self.required_csp_drive_ids.difference(
            self.required_homing_drive_ids
        )

        self.master: Optional[pysoem.Master] = None
        self.drives: List[FaulhaberDrive] = []
        self.pdo_lock = threading.RLock()
        self.pdo_stop_event = threading.Event()
        self.pdo_thread: Optional[threading.Thread] = None
        self.csp_active = False
        self.target_counts: List[int] = []
        self.latest_states: List[PDOState] = []
        self.pdo_error: Optional[str] = None
        self.last_working_counter = 0
        self.homing_complete = False
        self.homed_drive_ids = set()
        self.deferred_csp_prepared = False

        self._stall_anchor_ns: List[int] = []
        self._stall_anchor_actual: List[int] = []
        self._stall_anchor_error: List[int] = []
        self._stall_reported_drive_ids: set[int] = set()
        self._live_diagnostic_queue: List[Tuple[int, str, int, int, bool]] = []
        self._live_diagnostic_values: Dict[int, Dict[str, int]] = {}
        self._live_diagnostic_unavailable: Dict[int, List[str]] = {}
        self._live_diagnostic_base = ""
        self._live_diagnostic_targets: List[int] = []
        self.live_diagnostic_pending = False
        self.last_stall_snapshot = "No CSP stall has been detected in this session."

        # Drive 3's gripper-close guard changes 0x60E0/0x60E1 while CSP is
        # running.  Mailbox operations are deliberately staged at one SDO per
        # PDO cycle so the 50 Hz process-data watchdog is never starved.
        self.current_spur_torque_limit_per_mille: Optional[int] = None
        self._spur_torque_queue: List[
            Tuple[str, str, int, int, int, bool, int]
        ] = []
        self._spur_torque_values: Dict[str, int] = {}
        self._spur_torque_target: Optional[int] = None
        self._spur_torque_error: Optional[str] = None
        self._spur_torque_message = "No live Drive 3 torque-limit request."
        self._spur_torque_pending = False
        self._spur_torque_event = threading.Event()

        # The close-contact snapshot uses the same staged mailbox discipline.
        # It records Drive 3 PDO state immediately, then fills in the detailed
        # drive-side torque/current/status values over following cycles.
        self._spur_contact_queue: List[Tuple[str, int, int, bool, int]] = []
        self._spur_contact_values: Dict[str, int] = {}
        self._spur_contact_unavailable: List[str] = []
        self._spur_contact_state: Optional[PDOState] = None
        self._spur_contact_target: Optional[int] = None
        self._spur_contact_pending = False
        self._spur_contact_event = threading.Event()
        self.last_spur_contact_snapshot = (
            "No Drive 3 close-contact snapshot has been requested in this session."
        )

        self._validate_cycle_configuration()

    def mark_homing_complete(self, complete: bool) -> None:
        """Set the aggregate Homing state; primarily used by bus-level tests."""

        if complete:
            self.homed_drive_ids = set(self.required_homing_drive_ids)
        else:
            self.homed_drive_ids.clear()
        self.homing_complete = bool(complete)

    def mark_drive_homing_started(self, drive_id: int) -> None:
        """Invalidate one axis until its current Homing attempt succeeds."""

        self.homed_drive_ids.discard(int(drive_id))
        self.homing_complete = self.required_homing_drive_ids.issubset(
            self.homed_drive_ids
        )

    def mark_drive_homed(self, drive_id: int) -> None:
        """Record one successfully homed axis in this EtherCAT session."""

        self.homed_drive_ids.add(int(drive_id))
        self.homing_complete = self.required_homing_drive_ids.issubset(
            self.homed_drive_ids
        )

    def _validate_cycle_configuration(self) -> None:
        if self.pdo_timeout_us <= 0:
            raise ValueError("pdo_timeout_us must be positive")
        if self.enable_dc_sync:
            valid_dc = self.pdo_cycle_ns == 500_000 or (
                1_000_000 <= self.pdo_cycle_ns <= 50_000_000
                and self.pdo_cycle_ns % 1_000_000 == 0
            )
            if not valid_dc:
                raise ValueError(
                    "DC cycle must be 500000 ns or a 1 ms multiple between 1 ms and 50 ms"
                )
        elif not (
            1_000_000 <= self.pdo_cycle_ns <= 100_000_000
            and self.pdo_cycle_ns % 1_000_000 == 0
        ):
            raise ValueError("SM-Sync cycle must be a 1 ms multiple between 1 ms and 100 ms")
        self._cyclic_interpolation_rate()

    def _cyclic_interpolation_rate(self) -> int:
        """Return the FAULHABER 0x2332.00 value for the configured PDO period."""

        if self.pdo_cycle_ns % INTERPOLATION_QUANTUM_NS != 0:
            raise ValueError(
                "PDO cycle must be an exact multiple of 100000 ns for "
                "FAULHABER CSP interpolation"
            )
        interpolation_rate = self.pdo_cycle_ns // INTERPOLATION_QUANTUM_NS
        if not 1 <= interpolation_rate <= 0xFFFF:
            raise ValueError(
                "FAULHABER CSP interpolation rate must fit the U16 range: "
                f"{interpolation_rate}"
            )
        return int(interpolation_rate)

    @staticmethod
    def _sdo_write_int_raw(
        slave,
        index: int,
        subindex: int,
        value: int,
        size: int,
        signed: bool = False,
    ) -> None:
        slave.sdo_write(index, subindex, int(value).to_bytes(size, "little", signed=signed))

    @staticmethod
    def _sdo_read_int_raw(slave, index: int, subindex: int, signed: bool = False) -> int:
        return int.from_bytes(slave.sdo_read(index, subindex), "little", signed=signed)

    def _configure_cyclic_interpolation_rate(self, slave, slave_index: int) -> None:
        """Match target interpolation in the drive to the CSP PDO refresh period."""

        interpolation_rate = self._cyclic_interpolation_rate()
        self._sdo_write_int_raw(
            slave,
            CYCLIC_MODE_INTERPOLATION_RATE,
            0,
            interpolation_rate,
            size=2,
        )
        configured_rate = self._sdo_read_int_raw(
            slave, CYCLIC_MODE_INTERPOLATION_RATE, 0
        )
        if configured_rate != interpolation_rate:
            raise RuntimeError(
                f"Slave {slave_index}: 0x2332.00 readback is {configured_rate}, "
                f"expected {interpolation_rate}"
            )
        print(
            f"[EtherCAT] Slave {slave_index}: CSP interpolation 0x2332.00 "
            f"configured to {configured_rate} x 100 us "
            f"({self.pdo_cycle_ns} ns PDO cycle)"
        )

    def _verify_factory_position_pdos(self, slave, slave_index: int) -> None:
        rx_count = self._sdo_read_int_raw(slave, POSITION_RXPDO, 0)
        tx_count = self._sdo_read_int_raw(slave, POSITION_TXPDO, 0)
        rx_entries = tuple(
            self._sdo_read_int_raw(slave, POSITION_RXPDO, subindex)
            for subindex in range(1, rx_count + 1)
        )
        tx_entries = tuple(
            self._sdo_read_int_raw(slave, POSITION_TXPDO, subindex)
            for subindex in range(1, tx_count + 1)
        )
        if rx_entries != POSITION_RXPDO_ENTRIES or tx_entries != POSITION_TXPDO_ENTRIES:
            raise RuntimeError(
                f"Slave {slave_index} position PDO differs from the FAULHABER factory mapping: "
                f"Rx={rx_entries}, Tx={tx_entries}"
            )

    def _assign_factory_position_pdos(self, slave, slave_index: int) -> None:
        """Assign RxPDO2/TxPDO2 without writing their read-only mapping counts."""

        self._verify_factory_position_pdos(slave, slave_index)
        print(
            f"[EtherCAT] Slave {slave_index}: assigning factory Position PDOs "
            f"Rx=0x{POSITION_RXPDO:04X}, Tx=0x{POSITION_TXPDO:04X}"
        )
        try:
            self._sdo_write_int_raw(slave, PDO_RX_ASSIGNMENT, 0, 0, size=1)
            self._sdo_write_int_raw(slave, PDO_RX_ASSIGNMENT, 1, POSITION_RXPDO, size=2)
            self._sdo_write_int_raw(slave, PDO_RX_ASSIGNMENT, 0, 1, size=1)
            self._sdo_write_int_raw(slave, PDO_TX_ASSIGNMENT, 0, 0, size=1)
            self._sdo_write_int_raw(slave, PDO_TX_ASSIGNMENT, 1, POSITION_TXPDO, size=2)
            self._sdo_write_int_raw(slave, PDO_TX_ASSIGNMENT, 0, 1, size=1)
        except Exception as exc:
            raise RuntimeError(
                f"Slave {slave_index}: assigning factory Position PDOs failed: {exc}"
            ) from exc

    def _configure_sm_cycle_monitoring(self, slave, slave_index: int) -> None:
        """Set the SM-Sync arrival-time monitor while the slave is in PRE-OP."""

        self._sdo_write_int_raw(
            slave,
            SM2_PARAMETERS,
            SM_CYCLE_TIME_SUBINDEX,
            self.pdo_cycle_ns,
            size=4,
        )
        configured_cycle = self._sdo_read_int_raw(
            slave, SM2_PARAMETERS, SM_CYCLE_TIME_SUBINDEX
        )
        if configured_cycle != self.pdo_cycle_ns:
            raise RuntimeError(
                f"Slave {slave_index}: SM2 cycle readback is {configured_cycle} ns, "
                f"expected {self.pdo_cycle_ns} ns"
            )
        print(
            f"[EtherCAT] Slave {slave_index}: SM2 cycle monitoring "
            f"configured for {configured_cycle} ns"
        )

    def connect(self) -> None:
        self.master = pysoem.Master()
        print(f"[EtherCAT] Opening interface: {self.interface}")
        self.master.open(self.interface)
        if self.master.config_init() <= 0:
            raise RuntimeError("No EtherCAT slaves found")
        print(f"[EtherCAT] Found {len(self.master.slaves)} slave(s)")

        for drive_id, slave_index in enumerate(self.slave_indices):
            if slave_index < 0 or slave_index >= len(self.master.slaves):
                raise RuntimeError(
                    f"slave_indices[{drive_id}]={slave_index}, but only "
                    f"{len(self.master.slaves)} slave(s) were found"
                )
            if self.control_mode == "csp":
                if drive_id not in self.ignored_csp_drive_ids:
                    self._configure_cyclic_interpolation_rate(
                        self.master.slaves[slave_index], slave_index
                    )
                self._assign_factory_position_pdos(self.master.slaves[slave_index], slave_index)
                if not self.enable_dc_sync:
                    self._configure_sm_cycle_monitoring(
                        self.master.slaves[slave_index], slave_index
                    )

        if self.control_mode == "csp":
            mapped_bytes = self.master.config_map()
            print(f"[EtherCAT] Process image mapped ({mapped_bytes} bytes)")

            if self.enable_dc_sync:
                self.master.config_dc()
                for slave_index in self.slave_indices:
                    self.master.slaves[slave_index].dc_sync(True, self.pdo_cycle_ns, 0)
                print(f"[EtherCAT] DC-Sync configured with cycle {self.pdo_cycle_ns} ns")
            else:
                print(f"[EtherCAT] SM-Sync selected with cycle {self.pdo_cycle_ns} ns")
        else:
            # Preserve the auto_homing branch's proven mailbox-only PRE-OP
            # workflow. Profile Position and Homing use SDOs and do not need a
            # process image or an EtherCAT OP transition.
            if self.control_mode == "homing_csp":
                print(
                    "[EtherCAT] Homing-to-CSP session starts SDO-only in PRE-OP; "
                    "PDO mapping is deferred until home_all succeeds"
                )
            else:
                print("[EtherCAT] Profile/Homing uses SDO-only PRE-OP; PDO mapping skipped")

        self.drives = []
        for drive_id, slave_index in enumerate(self.slave_indices):
            slave = self.master.slaves[slave_index]
            print(f"[EtherCAT] Drive {drive_id} uses slave {slave_index}: {slave.name}")
            self.drives.append(FaulhaberDrive(slave, drive_id, self.sdo_delay_s, self.verbose))

    @staticmethod
    def _pack_rxpdo(controlword: int, target_position: int) -> bytes:
        if not -(1 << 31) <= int(target_position) < (1 << 31):
            raise ValueError(f"CSP target {target_position} is outside signed 32-bit range")
        return struct.pack("<Hi", int(controlword) & 0xFFFF, int(target_position))

    @staticmethod
    def _unpack_txpdo(data: bytes) -> Tuple[int, int]:
        if len(data) != PDO_TX_SIZE_BYTES:
            raise RuntimeError(
                f"TxPDO size mismatch: expected {PDO_TX_SIZE_BYTES} bytes, got {len(data)}"
            )
        statusword, actual_position = struct.unpack("<Hi", bytes(data))
        return int(statusword), int(actual_position)

    def _exchange_pdo_locked(
        self, controlword: int, require_expected_wkc: bool = False
    ) -> List[PDOState]:
        if self.master is None:
            raise RuntimeError("EtherCAT master is not connected")
        if len(self.target_counts) != len(self.drives):
            raise RuntimeError("CSP target cache is not initialized")

        for drive, target in zip(self.drives, self.target_counts):
            drive_controlword = (
                CMD_DISABLE_VOLTAGE
                if drive.drive_id in self.ignored_csp_drive_ids
                else controlword
            )
            payload = self._pack_rxpdo(drive_controlword, target)
            if len(payload) != PDO_RX_SIZE_BYTES:
                raise RuntimeError("Internal RxPDO packing error")
            drive.slave.output = payload

        self.master.send_processdata()
        working_counter = self.master.receive_processdata(self.pdo_timeout_us)
        self.last_working_counter = working_counter
        if working_counter <= 0:
            raise RuntimeError(f"EtherCAT PDO working counter is {working_counter}")
        expected_wkc = self.master.expected_wkc
        if require_expected_wkc and working_counter != expected_wkc:
            raise RuntimeError(
                f"EtherCAT PDO working counter is {working_counter}, expected {expected_wkc}"
            )

        states: List[PDOState] = []
        for index, drive in enumerate(self.drives):
            statusword, actual = self._unpack_txpdo(drive.slave.input)
            mode = (
                self.latest_states[index][2]
                if index < len(self.latest_states)
                else MODE_CYCLIC_SYNC_POSITION
            )
            states.append((actual, statusword, mode))
        self.latest_states = states
        return states

    def _prepare_deferred_csp_locked(self) -> None:
        """Configure PDOs in the existing master after SDO-only Homing."""

        if self.control_mode == "csp":
            return
        if self.control_mode != "homing_csp":
            raise RuntimeError("Deferred CSP preparation requires control_mode:=homing_csp")
        if not self.homing_complete:
            raise RuntimeError(
                "CSP handoff rejected: not all required drives were homed in this bridge session"
            )
        if self.master is None:
            raise RuntimeError("EtherCAT master is not connected")

        statuses = {
            drive.drive_id: drive.read_status()
            for drive in self.drives
            if drive.drive_id in self.required_homing_drive_ids
        }
        if any(
            (status & STATUS_STATE_MASK) != STATUS_OPERATION_ENABLED_STATE
            for status in statuses.values()
        ):
            raise RuntimeError(
                "CSP handoff requires every required drive to remain Operation Enabled: "
                + " ".join(
                    f"drive{i}=0x{status:04X}" for i, status in statuses.items()
                )
            )

        # A non-homed CSP drive (currently Drive 3/gripper) must still enter
        # Operation Enabled before PDO activation.  This is deliberately done
        # through the normal CiA-402 Profile Position sequence while the master
        # remains in PRE-OP; it does not run a reference search or alter 0x607C.
        for drive in self.drives:
            if drive.drive_id in self.non_homing_csp_drive_ids:
                drive.enable_operation(MODE_PROFILE_POSITION)

        if self.deferred_csp_prepared:
            return

        for drive_id, slave_index in enumerate(self.slave_indices):
            slave = self.master.slaves[slave_index]
            if drive_id not in self.ignored_csp_drive_ids:
                self._configure_cyclic_interpolation_rate(slave, slave_index)
            self._assign_factory_position_pdos(slave, slave_index)
            if not self.enable_dc_sync:
                self._configure_sm_cycle_monitoring(slave, slave_index)

        mapped_bytes = self.master.config_map()
        print(f"[EtherCAT] Deferred process image mapped ({mapped_bytes} bytes)")
        if self.enable_dc_sync:
            self.master.config_dc()
            for slave_index in self.slave_indices:
                self.master.slaves[slave_index].dc_sync(True, self.pdo_cycle_ns, 0)
            print(f"[EtherCAT] DC-Sync configured with cycle {self.pdo_cycle_ns} ns")
        else:
            print(f"[EtherCAT] SM-Sync selected with cycle {self.pdo_cycle_ns} ns")

        self.deferred_csp_prepared = True

    @staticmethod
    def _format_torque_limits(values: dict[str, int]) -> str:
        return (
            f"{values['maximum_torque']}/"
            f"{values['positive_torque_limit']}/"
            f"{values['negative_torque_limit']}"
        )

    def _configure_csp_torque_limits_locked(self) -> None:
        """Set and verify writable CSP torque limits on participating drives."""

        changes: List[str] = []
        for drive in self.drives:
            if drive.drive_id not in self.required_csp_drive_ids:
                continue
            before, after = drive.configure_csp_torque_limit(
                self.csp_torque_limit_per_mille
            )
            # Drives 2 and 3 were both found with undersized stored peak-current
            # parameters (220/1100 mA and 81/540 mA respectively).  Since
            # read-only 0x6072 is derived from that ratio, merely raising the
            # writable directional limits leaves either axis torque-capped.
            if drive.drive_id in (2, 3):
                current_before, current_after, effective_maximum = (
                    drive.ensure_peak_current_for_torque_limit(
                        self.csp_torque_limit_per_mille
                    )
                )
                after = drive.read_torque_limits(retry=True)
                if effective_maximum != after["maximum_torque"]:
                    raise RuntimeError(
                        f"Drive {drive.drive_id} inconsistent 0x6072 readback "
                        "after peak-current "
                        f"correction: {effective_maximum} != "
                        f"{after['maximum_torque']}"
                    )
                current_text = (
                    "; motor_mA(rated/continuous/peak) "
                    f"{current_before['rated_current_ma']}/"
                    f"{current_before['continuous_current_ma']}/"
                    f"{current_before['peak_current_ma']} -> "
                    f"{current_after['rated_current_ma']}/"
                    f"{current_after['continuous_current_ma']}/"
                    f"{current_after['peak_current_ma']}"
                )
            else:
                try:
                    current_parameters = drive.read_motor_current_parameters()
                    current_text = (
                        f"; motor_mA="
                        f"{current_parameters['rated_current_ma']}/"
                        f"{current_parameters['continuous_current_ma']}/"
                        f"{current_parameters['peak_current_ma']}"
                    )
                except Exception as exc:
                    current_text = f"; motor_mA=unavailable({type(exc).__name__})"
            changes.append(
                f"D{drive.drive_id} max/pos/neg "
                f"{self._format_torque_limits(before)} -> "
                f"{self._format_torque_limits(after)}"
                f"{current_text}"
            )
            if drive.drive_id == len(self.drives) - 1:
                self.current_spur_torque_limit_per_mille = int(
                    after["positive_torque_limit"]
                )
        message = (
            "CSP directional torque limits verified for this session only "
            "(0x6072 read-only; 0x60E0/0x60E1 writable; 1000=rated torque): "
            + "; ".join(changes)
        )
        print("[EtherCAT] " + message)
        self.diagnostic_logger("CSP_TORQUE_CONFIGURATION " + message)

    @staticmethod
    def _format_digital_input_configuration(values: dict[str, int]) -> str:
        device_flags = FaulhaberDrive._decode_flags(
            values["device_status"], DEVICE_STATUS_FLAGS
        )
        return (
            "lower/upper="
            f"0x{values['lower_limit_input_mask']:02X}/"
            f"0x{values['upper_limit_input_mask']:02X},"
            f"option={values['limit_switch_option']},"
            f"reference={values['reference_input']},"
            f"polarity=0x{values['input_polarity']:02X},"
            "logical/physical="
            f"0x{values['logical_inputs']:02X}/0x{values['physical_inputs']:02X},"
            f"device=0x{values['device_status']:08X}[{device_flags}]"
        )

    def _configure_csp_limit_switch_mappings_locked(self) -> None:
        """Remove stale hardware-limit mappings before selecting CSP.

        Drives 0-2 use a dedicated reference switch for Homing, while Drive 3
        uses the fixed relative reference.  None of those reference signals is
        a bidirectional travel limit.  Persisted mappings at 0x2310:01/:02 can
        nevertheless assert statusword bit 11 and stop a valid CSP command.
        """

        changes: List[str] = []
        for drive in self.drives:
            if drive.drive_id not in self.required_csp_drive_ids:
                continue
            if self.clear_limit_switch_mappings_for_csp:
                before, after = drive.clear_limit_switch_mappings_for_csp()
                changes.append(
                    f"D{drive.drive_id} "
                    f"{self._format_digital_input_configuration(before)} -> "
                    f"{self._format_digital_input_configuration(after)}"
                )
            else:
                current = drive.read_digital_input_configuration()
                changes.append(
                    f"D{drive.drive_id} preserved "
                    f"{self._format_digital_input_configuration(current)}"
                )

        action = (
            "cleared and verified"
            if self.clear_limit_switch_mappings_for_csp
            else "preserved by parameter"
        )
        message = (
            "CSP lower/upper limit-input mappings "
            f"{action}; Homing reference, polarity and 0x607B/0x607D unchanged; "
            "no parameter-store command: "
            + "; ".join(changes)
        )
        print("[EtherCAT] " + message)
        self.diagnostic_logger("CSP_LIMIT_SWITCH_CONFIGURATION " + message)

    def _configure_csp_following_error_monitor_locked(self) -> None:
        """Reapply Drive 2's finite monitor after all Homing mode transitions."""

        drive_id = 2
        if drive_id not in self.required_csp_drive_ids:
            return
        if len(self.drives) <= drive_id:
            raise RuntimeError("CSP configuration requires Drive 2")

        drive = self.drives[drive_id]
        before_window = drive.sdo_read_int_retry(FOLLOWING_ERROR_WINDOW, 0)
        before_timeout = drive.sdo_read_int_retry(FOLLOWING_ERROR_TIMEOUT, 0)
        drive.configure_following_error_monitor(
            self.drive2_following_error_window_counts,
            self.drive2_following_error_timeout_ms,
        )
        after_window = drive.sdo_read_int_retry(FOLLOWING_ERROR_WINDOW, 0)
        after_timeout = drive.sdo_read_int_retry(FOLLOWING_ERROR_TIMEOUT, 0)
        message = (
            "Drive 2 CSP following-error monitor reapplied after Homing and "
            "verified for this session only: "
            f"0x6065 {before_window} -> {after_window} counts; "
            f"0x6066 {before_timeout} -> {after_timeout} ms; "
            "no parameter-store command"
        )
        print("[EtherCAT] " + message)
        self.diagnostic_logger("CSP_FOLLOWING_ERROR_CONFIGURATION " + message)

    def _capture_torque_snapshot(self) -> str:
        """Capture actual torque and limits for all drives after a CSP fault."""

        entries: List[str] = []
        for drive in self.drives:
            unavailable: List[str] = []

            def read(label: str, index: int, signed: bool = False) -> Optional[int]:
                try:
                    return drive.sdo_read_int(index, 0, signed=signed)
                except Exception as exc:
                    unavailable.append(f"{label}:{type(exc).__name__}")
                    return None

            demand = read("demand", TORQUE_DEMAND, signed=True)
            actual = read("actual", ACTUAL_TORQUE, signed=True)
            maximum = read("max", MAX_TORQUE)
            positive = read("pos", POSITIVE_TORQUE_LIMIT)
            negative = read("neg", NEGATIVE_TORQUE_LIMIT)
            entry = (
                f"D{drive.drive_id}(demand/actual={demand}/{actual},max/pos/neg="
                f"{maximum}/{positive}/{negative}"
            )
            if unavailable:
                entry += ",unavailable=" + ",".join(unavailable)
            entries.append(entry + ")")
        return "TORQUE_SNAPSHOT " + "; ".join(entries)

    def _request_operational_locked(self, controlword: int = CMD_SHUTDOWN) -> None:
        if self.master is None:
            raise RuntimeError("EtherCAT master is not connected")
        safe_state = self.master.state_check(pysoem.SAFEOP_STATE, 100_000)
        if safe_state != pysoem.SAFEOP_STATE:
            raise RuntimeError(f"EtherCAT master did not reach SAFE-OP; state=0x{safe_state:02X}")

        # A valid output image must be present before requesting OP.
        self._exchange_pdo_locked(controlword)
        self.master.state = pysoem.OP_STATE
        self.master.write_state()

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            self._exchange_pdo_locked(controlword)
            if self.master.state_check(pysoem.OP_STATE, 5_000) == pysoem.OP_STATE:
                print("[EtherCAT] Master reached OP state")
                return
            time.sleep(self.pdo_cycle_ns / 1_000_000_000.0)
        raise RuntimeError("EtherCAT master did not reach OP state")

    def _wait_cia402_state_locked(
        self,
        controlword: int,
        expected_state: int,
        require_csp_accept: bool = False,
    ) -> List[PDOState]:
        states: List[PDOState] = []
        for _ in range(25):
            states = self._exchange_pdo_locked(controlword, require_expected_wkc=True)
            required_states = [
                (drive_id, state)
                for drive_id, state in enumerate(states)
                if drive_id in self.required_csp_drive_ids
            ]
            if any(status & STATUS_FAULT for _, (_, status, _) in required_states):
                raise RuntimeError(
                    "Drive fault during CSP activation: "
                    + " ".join(
                        f"drive{i}=0x{s:04X}" for i, (_, s, _) in required_states
                    )
                )
            state_ok = all(
                (status & STATUS_STATE_MASK) == expected_state
                for _, (_, status, _) in required_states
            )
            accept_ok = not require_csp_accept or all(
                status & STATUS_CSP_TARGET_ACCEPTED
                for _, (_, status, _) in required_states
            )
            if state_ok and accept_ok:
                return states
            time.sleep(self.pdo_cycle_ns / 1_000_000_000.0)
        raise RuntimeError(
            f"CiA 402 transition to 0x{expected_state:04X} timed out: "
            + " ".join(
                f"drive{i}=0x{s:04X}"
                for i, (_, s, _) in enumerate(states)
                if i in self.required_csp_drive_ids
            )
        )

    def enter_csp(self, target_counts: Optional[Sequence[int]] = None) -> List[PDOState]:
        if self.control_mode not in ("csp", "homing_csp"):
            raise RuntimeError("Bridge was not started with control_mode:=csp")
        if self.csp_active:
            return self.get_csp_states()

        with self.pdo_lock:
            preserve_homing_hold = self.control_mode == "homing_csp"
            if preserve_homing_hold and not self.homing_complete:
                raise RuntimeError(
                    "CSP handoff rejected: not all required drives were homed in this bridge session"
                )

            initial_targets = (
                [drive.read_actual_position_counts() for drive in self.drives]
                if target_counts is None
                else [int(value) for value in target_counts]
            )
            if len(initial_targets) != len(self.drives):
                raise RuntimeError(f"Expected {len(self.drives)} CSP targets, got {len(initial_targets)}")

            # Keep the proven Homing search parameters untouched.  Torque is
            # raised only at the CSP handoff.  Stale lower/upper limit-input
            # mappings are removed after Homing and every write is read back
            # before PDO motion is allowed to start.
            self._configure_csp_limit_switch_mappings_locked()
            self._configure_csp_following_error_monitor_locked()
            self._configure_csp_torque_limits_locked()

            if preserve_homing_hold:
                self._prepare_deferred_csp_locked()

            for drive in self.drives:
                if drive.drive_id in self.ignored_csp_drive_ids:
                    print(
                        f"[EtherCAT] Drive {drive.drive_id} is ignored for CSP "
                        "and held at Disable Voltage"
                    )
                    continue
                drive.reset_fault_if_needed()
                if drive.set_operation_mode(MODE_CYCLIC_SYNC_POSITION) != MODE_CYCLIC_SYNC_POSITION:
                    raise RuntimeError(f"Drive {drive.drive_id} rejected CSP mode")

            if preserve_homing_hold:
                handoff_statuses = {
                    drive.drive_id: drive.read_status()
                    for drive in self.drives
                    if drive.drive_id in self.required_csp_drive_ids
                }
                if any(
                    (status & STATUS_STATE_MASK) != STATUS_OPERATION_ENABLED_STATE
                    for status in handoff_statuses.values()
                ):
                    raise RuntimeError(
                        "A drive lost Operation Enabled while selecting CSP; "
                        "continuous-hold handoff is not supported by this drive state: "
                        + " ".join(
                            f"drive{i}=0x{status:04X}"
                            for i, status in handoff_statuses.items()
                        )
                    )

            self.target_counts = initial_targets
            modes = [drive.read_mode_display() for drive in self.drives]
            self.latest_states = [(target, 0, mode) for target, mode in zip(initial_targets, modes)]
            self.pdo_error = None

            try:
                if preserve_homing_hold:
                    # Homing leaves every required drive Operation Enabled. Keep
                    # 0x000F on their first valid PDO and use actual positions as
                    # targets. Ignored drives receive Disable Voltage instead.
                    self._request_operational_locked(CMD_ENABLE_OPERATION)
                    states = self._wait_cia402_state_locked(
                        CMD_ENABLE_OPERATION,
                        STATUS_OPERATION_ENABLED_STATE,
                        require_csp_accept=True,
                    )
                    if self.ignored_csp_drive_ids:
                        print(
                            "[EtherCAT] Homing-to-CSP handoff completed for required "
                            "drives; ignored drives remain Disable Voltage"
                        )
                    else:
                        print(
                            "[EtherCAT] Homing-to-CSP handoff completed without "
                            "Shutdown/Disable controlwords"
                        )
                else:
                    self._request_operational_locked()
                    self._wait_cia402_state_locked(CMD_SHUTDOWN, STATUS_READY_TO_SWITCH_ON)
                    self._wait_cia402_state_locked(CMD_SWITCH_ON, STATUS_SWITCHED_ON)
                    states = self._wait_cia402_state_locked(
                        CMD_ENABLE_OPERATION,
                        STATUS_OPERATION_ENABLED_STATE,
                        require_csp_accept=True,
                    )
            except Exception:
                self._safeop_locked()
                raise

            if any(
                modes[drive_id] != MODE_CYCLIC_SYNC_POSITION
                for drive_id in self.required_csp_drive_ids
            ):
                self._safeop_locked()
                raise RuntimeError(f"Unexpected CSP mode displays: {modes}")

            self._reset_stall_monitor_locked(states)
            self.csp_active = True
            self.pdo_stop_event.clear()
            self.pdo_thread = threading.Thread(
                target=self._pdo_loop,
                name="rascl_csp_pdo",
                daemon=True,
            )
            self.pdo_thread.start()
            return states

    def _format_csp_snapshot(self, states: Sequence[PDOState]) -> str:
        """Format the last cyclic target/feedback values for a fault report.

        Values are deliberately kept in the native drive-count domain: this is
        the only representation shared by the EtherCAT PDO and the drive's
        following-error monitor.  The helper is called only on an invalid CSP
        state, so normal 50 Hz operation does not produce diagnostic output.
        """

        entries: List[str] = []
        for drive_id, (actual, statusword, mode) in enumerate(states):
            target: Optional[int] = None
            if drive_id < len(self.target_counts):
                target = int(self.target_counts[drive_id])

            if target is None:
                target_text = "?"
                error_text = "?"
            else:
                target_text = str(target)
                error_text = str(target - actual)

            entries.append(
                f"D{drive_id}(target={target_text},actual={actual},"
                f"error={error_text},status=0x{statusword:04X},mode={mode})"
                f"[flags={FaulhaberDrive._decode_flags(statusword, STATUSWORD_MONITOR_FLAGS)}]"
            )
        return "CSP_SNAPSHOT " + "; ".join(entries)

    def _reset_stall_monitor_locked(self, states: Sequence[PDOState]) -> None:
        now_ns = time.monotonic_ns()
        self._stall_anchor_ns = [now_ns for _ in states]
        self._stall_anchor_actual = [int(state[0]) for state in states]
        self._stall_anchor_error = [
            abs(int(self.target_counts[index]) - int(state[0]))
            for index, state in enumerate(states)
        ]
        self._stall_reported_drive_ids.clear()
        self._live_diagnostic_queue.clear()
        self._live_diagnostic_values.clear()
        self._live_diagnostic_unavailable.clear()
        self._live_diagnostic_base = ""
        self._live_diagnostic_states: List[PDOState] = []
        self._live_diagnostic_targets = []
        self.live_diagnostic_pending = False
        self.last_stall_snapshot = "No CSP stall has been detected in this session."

    def _cancel_spur_staged_requests_locked(self, reason: str) -> None:
        """Wake service callers if CSP stops while a staged request is active."""

        if self._spur_torque_pending:
            self._spur_torque_error = reason
            self._spur_torque_message = f"Drive 3 torque-limit request aborted: {reason}"
            self._spur_torque_pending = False
            self._spur_torque_queue.clear()
            self._spur_torque_event.set()
        if self._spur_contact_pending:
            self.last_spur_contact_snapshot = (
                f"Drive 3 contact snapshot aborted: {reason}"
            )
            self._spur_contact_pending = False
            self._spur_contact_queue.clear()
            self._spur_contact_event.set()

    def request_spur_torque_limit(
        self, limit_per_mille: int, timeout_s: float = 3.0
    ) -> str:
        """Stage and verify a live Drive 3 directional torque-limit change."""

        limit = int(limit_per_mille)
        if not 1 <= limit <= MAX_TORQUE_PER_MILLE:
            raise ValueError(
                f"Drive 3 torque limit must be 1..{MAX_TORQUE_PER_MILLE} per-mille"
            )
        with self.pdo_lock:
            if self.pdo_error:
                raise RuntimeError(f"CSP/PDO loop failed: {self.pdo_error}")
            if not self.csp_active:
                raise RuntimeError("Drive 3 torque limit can only change while CSP is active")
            drive_id = len(self.drives) - 1
            if drive_id not in self.required_csp_drive_ids:
                raise RuntimeError("Drive 3 is disabled in this CSP session")
            if self._spur_torque_pending:
                raise RuntimeError("another Drive 3 torque-limit request is still pending")

            self._spur_torque_target = limit
            self._spur_torque_values = {}
            self._spur_torque_error = None
            self._spur_torque_message = (
                f"Drive 3 torque-limit request to {limit} per-mille is pending"
            )
            # Each operation gets up to three PDO cycles.  This tolerates a
            # transient mailbox WKC loss without ever retrying twice in one
            # process-data period.
            self._spur_torque_queue = [
                ("write", "positive_write", POSITIVE_TORQUE_LIMIT, 0, limit, False, 3),
                ("write", "negative_write", NEGATIVE_TORQUE_LIMIT, 0, limit, False, 3),
                ("read", "positive", POSITIVE_TORQUE_LIMIT, 0, 0, False, 3),
                ("read", "negative", NEGATIVE_TORQUE_LIMIT, 0, 0, False, 3),
            ]
            self._spur_torque_pending = True
            self._spur_torque_event.clear()

        if not self._spur_torque_event.wait(float(timeout_s)):
            raise TimeoutError("Drive 3 torque-limit readback did not finish within 3 seconds")
        with self.pdo_lock:
            if self._spur_torque_error:
                raise RuntimeError(self._spur_torque_error)
            return self._spur_torque_message

    def _advance_spur_torque_request_locked(self) -> bool:
        """Perform at most one mailbox operation for a live torque request."""

        if not self._spur_torque_pending:
            return False
        if not self._spur_torque_queue:
            return False

        operation = self._spur_torque_queue.pop(0)
        action, label, index, subindex, value, signed, attempts_left = operation
        drive = self.drives[-1]
        try:
            if action == "write":
                drive.sdo_write_int(
                    index, subindex, value, size=2, signed=signed
                )
            else:
                self._spur_torque_values[label] = drive.sdo_read_int(
                    index, subindex, signed=signed
                )
        except Exception as exc:
            if attempts_left > 1:
                self._spur_torque_queue.insert(
                    0,
                    (
                        action,
                        label,
                        index,
                        subindex,
                        value,
                        signed,
                        attempts_left - 1,
                    ),
                )
                return True
            self._spur_torque_error = (
                f"Drive 3 live torque-limit {action} 0x{index:04X}:{subindex:02X} "
                f"failed after 3 PDO cycles ({type(exc).__name__}: {exc})"
            )
            self._spur_torque_message = self._spur_torque_error
            self._spur_torque_pending = False
            self._spur_torque_queue.clear()
            self._spur_torque_event.set()
            self.diagnostic_logger("SPUR_TORQUE_GUARD_FAILED " + self._spur_torque_error)
            return True

        if self._spur_torque_queue:
            return True

        assert self._spur_torque_target is not None
        positive = self._spur_torque_values.get("positive")
        negative = self._spur_torque_values.get("negative")
        if positive != self._spur_torque_target or negative != self._spur_torque_target:
            self._spur_torque_error = (
                "Drive 3 live torque-limit readback mismatch: "
                f"expected={self._spur_torque_target}, "
                f"positive/negative={positive}/{negative}"
            )
            self._spur_torque_message = self._spur_torque_error
        else:
            self.current_spur_torque_limit_per_mille = self._spur_torque_target
            self._spur_torque_message = (
                "Drive 3 CSP torque limit verified for this session: "
                f"0x60E0/0x60E1={positive}/{negative} per-mille "
                "(1000=rated torque)"
            )
            self.diagnostic_logger(
                "SPUR_TORQUE_GUARD " + self._spur_torque_message
            )
        self._spur_torque_pending = False
        self._spur_torque_event.set()
        return True

    def request_spur_contact_snapshot(self, timeout_s: float = 3.0) -> str:
        """Capture a detailed Drive 3 hold/contact snapshot without pausing PDO."""

        with self.pdo_lock:
            if self.pdo_error:
                raise RuntimeError(f"CSP/PDO loop failed: {self.pdo_error}")
            if not self.csp_active:
                raise RuntimeError("Drive 3 contact snapshot requires active CSP")
            if self._spur_contact_pending:
                raise RuntimeError("another Drive 3 contact snapshot is still pending")
            drive_id = len(self.drives) - 1
            if drive_id not in self.required_csp_drive_ids:
                raise RuntimeError("Drive 3 is disabled in this CSP session")

            self._spur_contact_state = self.latest_states[drive_id]
            self._spur_contact_target = int(self.target_counts[drive_id])
            self._spur_contact_values = {}
            self._spur_contact_unavailable = []
            self._spur_contact_queue = [
                (label, index, subindex, signed, 3)
                for label, index, subindex, signed in LIVE_DIAGNOSTIC_READS
            ]
            self._spur_contact_pending = True
            self._spur_contact_event.clear()
            self.last_spur_contact_snapshot = (
                "Drive 3 close-contact snapshot is pending"
            )

        if not self._spur_contact_event.wait(float(timeout_s)):
            raise TimeoutError("Drive 3 contact snapshot did not finish within 3 seconds")
        with self.pdo_lock:
            return self.last_spur_contact_snapshot

    def _advance_spur_contact_snapshot_locked(self) -> bool:
        """Read at most one Drive 3 diagnostic object in the current PDO cycle."""

        if not self._spur_contact_pending:
            return False
        if self._spur_contact_queue:
            operation = self._spur_contact_queue.pop(0)
            label, index, subindex, signed, attempts_left = operation
            try:
                self._spur_contact_values[label] = self.drives[-1].sdo_read_int(
                    index, subindex, signed=signed
                )
            except Exception as exc:
                if attempts_left > 1:
                    self._spur_contact_queue.insert(
                        0, (label, index, subindex, signed, attempts_left - 1)
                    )
                    return True
                self._spur_contact_unavailable.append(
                    f"{label}:{type(exc).__name__}"
                )
            if self._spur_contact_queue:
                return True

        assert self._spur_contact_state is not None
        assert self._spur_contact_target is not None
        actual, statusword, mode = self._spur_contact_state
        details = self.drives[-1].format_live_diagnostics(
            self._spur_contact_values, self._spur_contact_unavailable
        )
        self.last_spur_contact_snapshot = (
            "SPUR_CONTACT_SNAPSHOT "
            f"target={self._spur_contact_target},actual={actual},"
            f"error={self._spur_contact_target - actual},"
            f"status=0x{statusword:04X},mode={mode}; {details}"
        )
        self._spur_contact_pending = False
        self._spur_contact_event.set()
        self.diagnostic_logger(self.last_spur_contact_snapshot)
        return True

    def _detect_csp_stalls_locked(
        self, states: Sequence[PDOState], now_ns: Optional[int] = None
    ) -> List[int]:
        """Return axes that have a large command error and no encoder progress."""

        current_ns = time.monotonic_ns() if now_ns is None else int(now_ns)
        if len(self._stall_anchor_ns) != len(states):
            self._reset_stall_monitor_locked(states)
            return []

        stalled: List[int] = []
        timeout_ns = self.csp_stall_timeout_ms * 1_000_000
        for drive_id, (actual, _statusword, _mode) in enumerate(states):
            if drive_id in self.ignored_csp_drive_ids:
                continue
            target = int(self.target_counts[drive_id])
            actual = int(actual)
            error = abs(target - actual)
            if error < self.csp_stall_error_counts:
                self._stall_anchor_ns[drive_id] = current_ns
                self._stall_anchor_actual[drive_id] = actual
                self._stall_anchor_error[drive_id] = error
                self._stall_reported_drive_ids.discard(drive_id)
                continue

            encoder_progress = abs(actual - self._stall_anchor_actual[drive_id])
            error_progress = self._stall_anchor_error[drive_id] - error
            if (
                encoder_progress >= self.csp_stall_progress_counts
                or error_progress >= self.csp_stall_progress_counts
            ):
                self._stall_anchor_ns[drive_id] = current_ns
                self._stall_anchor_actual[drive_id] = actual
                self._stall_anchor_error[drive_id] = error
                continue

            if (
                drive_id not in self._stall_reported_drive_ids
                and current_ns - self._stall_anchor_ns[drive_id] >= timeout_ns
            ):
                self._stall_reported_drive_ids.add(drive_id)
                stalled.append(drive_id)
        return stalled

    @staticmethod
    def _classify_live_stall(
        statusword: int,
        target: int,
        actual: int,
        values: Dict[str, int],
    ) -> str:
        """Classify only evidence exposed by the drive; never invent a cause."""

        device_status = int(values.get("device_status", 0))
        reported: List[str] = []
        if device_status & (1 << 14):
            reported.append("TORQUE_LIMIT_REPORTED")
        if device_status & ((1 << 13) | (1 << 18) | (1 << 19) | (1 << 20)):
            reported.append("VOLTAGE_OR_SUPPLY_LIMIT_REPORTED")
        if device_status & ((1 << 6) | (1 << 7) | (1 << 8) | (1 << 9)):
            reported.append("POSITION_OR_LIMIT_SWITCH_REPORTED")
        if device_status & ((1 << 2) | (1 << 15) | (1 << 21)):
            reported.append("VELOCITY_OR_SPEED_LIMIT_REPORTED")
        if device_status & ((1 << 16) | (1 << 17)):
            reported.append("TEMPERATURE_LIMIT_REPORTED")
        if device_status & (1 << 22):
            reported.append("SAFETY_MONITORING_REPORTED")
        if device_status & (1 << 5):
            reported.append("FOLLOWING_ERROR_REPORTED")
        if reported:
            return "+".join(reported)
        if statusword & STATUS_INTERNAL_LIMIT_ACTIVE:
            return "INTERNAL_LIMIT_ACTIVE_UNSPECIFIED"

        demand = values.get("position_demand")
        velocity = values.get("velocity_actual")
        if demand is not None and abs(target - int(demand)) >= 25_000:
            return "DRIVE_DEMAND_NOT_FOLLOWING_PDO_TARGET"
        if velocity is not None and abs(int(velocity)) <= 1 and abs(target - actual) >= 25_000:
            return "POSITION_LOOP_STALLED_WITHOUT_LIMIT_FLAG"
        return "UNCLASSIFIED_STALL"

    def _start_live_stall_diagnostics_locked(
        self, states: Sequence[PDOState], stalled_drive_ids: Sequence[int]
    ) -> None:
        drive_ids = sorted(set(int(value) for value in stalled_drive_ids))
        if not drive_ids:
            return
        detected = (
            "CSP_STALL_DETECTED drives="
            + ",".join(f"D{drive_id}" for drive_id in drive_ids)
            + f"; no_progress_ms={self.csp_stall_timeout_ms}; "
            + self._format_csp_snapshot(states)
        )
        self.diagnostic_logger(detected)
        if self.live_diagnostic_pending:
            new_drive_ids = [
                drive_id
                for drive_id in drive_ids
                if drive_id not in self._live_diagnostic_values
            ]
            for drive_id in new_drive_ids:
                self._live_diagnostic_values[drive_id] = {}
                self._live_diagnostic_unavailable[drive_id] = []
                self._live_diagnostic_states[drive_id] = states[drive_id]
                self._live_diagnostic_targets[drive_id] = self.target_counts[drive_id]
                self._live_diagnostic_queue.extend(
                    (drive_id, label, index, subindex, signed)
                    for label, index, subindex, signed in LIVE_DIAGNOSTIC_READS
                )
            if new_drive_ids:
                self._live_diagnostic_base += "; ADDITIONAL_" + detected
                self.last_stall_snapshot = (
                    self._live_diagnostic_base + "; LIVE_DIAG=pending"
                )
            return

        self._live_diagnostic_base = detected
        self._live_diagnostic_states = list(states)
        self._live_diagnostic_targets = list(self.target_counts)
        self._live_diagnostic_values = {drive_id: {} for drive_id in drive_ids}
        self._live_diagnostic_unavailable = {drive_id: [] for drive_id in drive_ids}
        self._live_diagnostic_queue = [
            (drive_id, label, index, subindex, signed)
            for label, index, subindex, signed in LIVE_DIAGNOSTIC_READS
            for drive_id in drive_ids
        ]
        self.live_diagnostic_pending = True
        self.last_stall_snapshot = detected + "; LIVE_DIAG=pending"

    def _advance_live_stall_diagnostics_locked(self) -> None:
        """Read at most one SDO per PDO cycle and publish the completed snapshot."""

        if not self.live_diagnostic_pending:
            return
        if self._live_diagnostic_queue:
            drive_id, label, index, subindex, signed = self._live_diagnostic_queue.pop(0)
            try:
                value = self.drives[drive_id].sdo_read_int(
                    index, subindex, signed=signed
                )
                self._live_diagnostic_values[drive_id][label] = value
            except Exception as exc:
                self._live_diagnostic_unavailable[drive_id].append(
                    f"{label}:{type(exc).__name__}"
                )
            if self._live_diagnostic_queue:
                return

        details: List[str] = []
        causes: List[str] = []
        for drive_id in sorted(self._live_diagnostic_values):
            values = self._live_diagnostic_values[drive_id]
            actual, statusword, _mode = self._live_diagnostic_states[drive_id]
            target = int(self._live_diagnostic_targets[drive_id])
            cause = self._classify_live_stall(
                statusword, target, int(actual), values
            )
            causes.append(f"D{drive_id}={cause}")
            details.append(
                self.drives[drive_id].format_live_diagnostics(
                    values, self._live_diagnostic_unavailable[drive_id]
                )
            )
        snapshot = (
            "CSP_STALL_SNAPSHOT causes="
            + ",".join(causes)
            + "; "
            + self._live_diagnostic_base
            + "; "
            + "; ".join(details)
        )
        self.last_stall_snapshot = snapshot
        self.live_diagnostic_pending = False
        self.diagnostic_logger(snapshot)

    def _validate_running_states(self, states: Sequence[PDOState]) -> None:
        for drive_id, (_, statusword, mode) in enumerate(states):
            if drive_id in self.ignored_csp_drive_ids:
                continue
            if statusword & STATUS_FAULT:
                # Capture the four compact torque states first; some drives
                # stop answering less critical SDOs shortly after a fault.
                torque_snapshot = self._capture_torque_snapshot()
                diagnostics = self.drives[drive_id].capture_fault_diagnostics()
                raise RuntimeError(
                    f"Drive {drive_id} fault; statusword=0x{statusword:04X}; "
                    f"{self._format_csp_snapshot(states)}; {diagnostics}; "
                    f"{torque_snapshot}"
                )
            if statusword & STATUS_FOLLOWING_OR_HOMING_ERROR:
                torque_snapshot = self._capture_torque_snapshot()
                diagnostics = self.drives[drive_id].capture_fault_diagnostics()
                raise RuntimeError(
                    f"Drive {drive_id} CSP following error; statusword=0x{statusword:04X}; "
                    f"{self._format_csp_snapshot(states)}; {diagnostics}; "
                    f"{torque_snapshot}"
                )
            if (statusword & STATUS_STATE_MASK) != STATUS_OPERATION_ENABLED_STATE:
                raise RuntimeError(
                    f"Drive {drive_id} left Operation Enabled; statusword=0x{statusword:04X}; "
                    f"{self._format_csp_snapshot(states)}"
                )
            if mode != MODE_CYCLIC_SYNC_POSITION:
                raise RuntimeError(
                    f"Drive {drive_id} mode display changed to {mode}; "
                    f"{self._format_csp_snapshot(states)}"
                )

    def _pdo_loop(self) -> None:
        next_cycle_ns = time.monotonic_ns()
        try:
            while not self.pdo_stop_event.is_set():
                with self.pdo_lock:
                    states = self._exchange_pdo_locked(
                        CMD_ENABLE_OPERATION, require_expected_wkc=True
                    )
                    self._validate_running_states(states)
                    stalled_drive_ids = self._detect_csp_stalls_locked(states)
                    self._start_live_stall_diagnostics_locked(
                        states, stalled_drive_ids
                    )
                    # Across all live mailbox work, perform at most one SDO per
                    # process-data cycle. Torque switching has highest priority
                    # because group 15 waits for its verified readback before
                    # publishing a close/open trajectory.
                    if not self._advance_spur_torque_request_locked():
                        if not self._advance_spur_contact_snapshot_locked():
                            self._advance_live_stall_diagnostics_locked()

                next_cycle_ns += self.pdo_cycle_ns
                remaining_s = (next_cycle_ns - time.monotonic_ns()) / 1_000_000_000.0
                if remaining_s > 0:
                    self.pdo_stop_event.wait(remaining_s)
                else:
                    # Do not accumulate drift after an overrun.
                    next_cycle_ns = time.monotonic_ns()
        except Exception as exc:
            with self.pdo_lock:
                self.pdo_error = str(exc)
                # A lost/invalid cyclic exchange must immediately stop applying
                # process-data outputs.  Keep the original error for the TCP
                # client, then make the best-effort EtherCAT state transition.
                self._cancel_spur_staged_requests_locked(
                    f"CSP/PDO loop stopped: {exc}"
                )
                self._safeop_locked()
            self.diagnostic_logger(f"CSP_PDO_STOPPED {exc}")
            print(f"[EtherCAT] CSP/PDO loop stopped: {exc}")
        finally:
            with self.pdo_lock:
                self.csp_active = False

    def set_csp_targets(self, target_counts: Sequence[int]) -> List[PDOState]:
        with self.pdo_lock:
            if self.pdo_error:
                raise RuntimeError(f"CSP/PDO loop failed: {self.pdo_error}")
            if not self.csp_active:
                raise RuntimeError("CSP mode is not active. Call ENTER_CSP_ALL first.")
            targets = [int(value) for value in target_counts]
            if len(targets) != len(self.drives):
                raise RuntimeError(f"Expected {len(self.drives)} CSP targets, got {len(targets)}")
            for target in targets:
                self._pack_rxpdo(CMD_ENABLE_OPERATION, target)
            for drive_id in self.ignored_csp_drive_ids:
                if drive_id < len(self.latest_states):
                    targets[drive_id] = self.latest_states[drive_id][0]
            self.target_counts = targets
            return list(self.latest_states)

    def get_csp_states(self) -> List[PDOState]:
        with self.pdo_lock:
            if self.pdo_error:
                raise RuntimeError(f"CSP/PDO loop failed: {self.pdo_error}")
            if not self.latest_states:
                raise RuntimeError("No CSP/PDO state has been received")
            return list(self.latest_states)

    def _safeop_locked(self) -> None:
        if self.master is None:
            return
        try:
            self.master.state = pysoem.SAFEOP_STATE
            self.master.write_state()
            self.master.state_check(pysoem.SAFEOP_STATE, 100_000)
        except Exception as exc:
            print(f"[EtherCAT] WARNING: could not request SAFE-OP: {exc}")

    def exit_csp(self) -> List[int]:
        if (
            not self.csp_active
            and (self.pdo_thread is None or not self.pdo_thread.is_alive())
            and not self.target_counts
        ):
            # An activation rejected before PDO preparation must not move the
            # Homing master out of PRE-OP or remove its Profile hold.
            return [drive.read_status() for drive in self.drives]

        self.pdo_stop_event.set()
        thread = self.pdo_thread
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=max(1.0, 5.0 * self.pdo_cycle_ns / 1_000_000_000.0))

        with self.pdo_lock:
            self._cancel_spur_staged_requests_locked("CSP session is exiting")
            statuses = [state[1] for state in self.latest_states]
            try:
                if self.target_counts and self.master is not None:
                    states = self._exchange_pdo_locked(
                        CMD_DISABLE_OPERATION, require_expected_wkc=True
                    )
                    statuses = [state[1] for state in states]
                    self._exchange_pdo_locked(CMD_DISABLE_VOLTAGE)
            except Exception as exc:
                print(f"[EtherCAT] WARNING: final CSP disable cycle failed: {exc}")
            self._safeop_locked()
            self.csp_active = False
            self.pdo_thread = None
            self.target_counts = []
            self.latest_states = []
            self.pdo_error = None
            return statuses

    def close(self) -> None:
        if self.csp_active or (self.pdo_thread is not None and self.pdo_thread.is_alive()):
            self.exit_csp()
        elif self.drives:
            # The dedicated homing/Profile launch has no PDO loop to perform
            # the shutdown sequence, so explicitly remove drive voltage before
            # closing the EtherCAT master.
            for drive in self.drives:
                try:
                    drive.disable_operation()
                except Exception as exc:
                    print(
                        f"[EtherCAT] WARNING: drive {drive.drive_id} could not be "
                        f"disabled during shutdown: {exc}"
                    )
        if self.master is not None:
            if self.enable_dc_sync:
                for slave_index in self.slave_indices:
                    try:
                        self.master.slaves[slave_index].dc_sync(False, 0, 0)
                    except Exception:
                        pass
            try:
                self.master.close()
            except Exception:
                pass
        self.drives.clear()


class RASCLFaulhaberBridge(Node):
    """ROS 2 services and TCP protocol around :class:`FaulhaberBus`."""

    def __init__(self) -> None:
        super().__init__("rascl_faulhaber_bridge")

        self.declare_parameter("interface", "robot_interface")
        self.declare_parameter("slave_indices", [0, 1, 2, 3])
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 15001)
        self.declare_parameter("control_mode", "csp")
        self.declare_parameter("sdo_delay_s", 0.05)
        self.declare_parameter("motion_timeout_s", 8.0)
        self.declare_parameter("verbose", True)
        self.declare_parameter("profile_velocity", 0)
        self.declare_parameter("profile_acceleration", 0)
        self.declare_parameter("profile_deceleration", 0)
        self.declare_parameter("pdo_cycle_ns", 20_000_000)
        self.declare_parameter("pdo_timeout_us", 5_000)
        self.declare_parameter("enable_dc_sync", False)
        self.declare_parameter("csp_torque_limit_per_mille", 1000)
        self.declare_parameter("spur_close_torque_limit_per_mille", 300)
        self.declare_parameter("spur_hold_torque_limit_per_mille", 100)
        self.declare_parameter("clear_limit_switch_mappings_for_csp", True)
        self.declare_parameter("ignore_spur_gear_in_csp", False)
        self.declare_parameter("skip_spur_gear_homing", True)
        self.declare_parameter("spur_gear_reference_delta_counts", 50_000)
        self.declare_parameter("spur_gear_reference_timeout_s", 30.0)
        self.declare_parameter("spur_gear_reference_tolerance_counts", 100)
        self.declare_parameter("spur_gear_reference_profile_velocity", 3_000)
        self.declare_parameter("spur_gear_reference_profile_acceleration", 1_000)
        self.declare_parameter("spur_gear_reference_profile_deceleration", 1_000)
        self.declare_parameter("spur_gear_reference_following_error_confirm_s", 0.30)
        # Drive 2's factory 32-count / 48-ms monitor is far below the normal
        # compliant motion lag of this 196:1 arm axis. Keep a finite, axis-
        # local monitor for CSP instead of disabling following-error detection.
        self.declare_parameter("drive2_following_error_window_counts", 25_000)
        self.declare_parameter("drive2_following_error_timeout_ms", 250)
        self.declare_parameter("csp_stall_error_counts", 25_000)
        self.declare_parameter("csp_stall_progress_counts", 100)
        self.declare_parameter("csp_stall_timeout_ms", 500)

        # Values validated on the auto_homing branch.
        self.declare_parameter("homing_methods", [28, 28, 24, 24])
        self.declare_parameter("reference_inputs", [2, 2, 2, 1])
        self.declare_parameter("homing_offsets", [0, 0, 0, 0])
        self.declare_parameter("homing_search_speeds", [1000, 1000, 1000, 1000])
        self.declare_parameter("homing_zero_speeds", [200, 200, 200, 200])
        self.declare_parameter("homing_accelerations", [1000, 1000, 1000, 1000])
        self.declare_parameter("homing_interval_max_travel_drive0_counts", 100_000)
        self.declare_parameter("homing_interval_max_travel_drive1_counts", 300_000)
        self.declare_parameter("homing_interval_max_travel_drive2_counts", 300_000)
        self.declare_parameter("homing_interval_max_travel_drive3_counts", 100_000)
        self.declare_parameter("homing_interval_timeout_s", 120.0)
        self.declare_parameter("homing_interval_poll_s", 0.01)
        self.declare_parameter("homing_midpoint_tolerance_counts", 500)
        self.declare_parameter("home_adjust_profile_velocity", 1000)
        self.declare_parameter("home_adjust_timeout_s", 120.0)
        self.declare_parameter("home_adjust_tolerance_counts", 100)
        self.declare_parameter("home_adjust_following_error_confirm_s", 0.30)
        self.declare_parameter("test_drive_index", 0)
        self.declare_parameter("test_relative_counts", 0)

        self.interface = str(self.get_parameter("interface").value)
        self.slave_indices = [int(v) for v in self.get_parameter("slave_indices").value]
        self.host = str(self.get_parameter("host").value)
        self.port = int(self.get_parameter("port").value)
        self.control_mode = str(self.get_parameter("control_mode").value).lower()
        if self.control_mode not in ("profile", "csp", "homing_csp"):
            raise ValueError("control_mode must be 'profile', 'csp', or 'homing_csp'")
        self.sdo_delay_s = float(self.get_parameter("sdo_delay_s").value)
        self.motion_timeout_s = float(self.get_parameter("motion_timeout_s").value)
        self.verbose = bool(self.get_parameter("verbose").value)
        self.profile_velocity = int(self.get_parameter("profile_velocity").value)
        self.profile_acceleration = int(self.get_parameter("profile_acceleration").value)
        self.profile_deceleration = int(self.get_parameter("profile_deceleration").value)
        self.pdo_cycle_ns = int(self.get_parameter("pdo_cycle_ns").value)
        self.pdo_timeout_us = int(self.get_parameter("pdo_timeout_us").value)
        self.enable_dc_sync = bool(self.get_parameter("enable_dc_sync").value)
        self.csp_torque_limit_per_mille = int(
            self.get_parameter("csp_torque_limit_per_mille").value
        )
        self.spur_close_torque_limit_per_mille = int(
            self.get_parameter("spur_close_torque_limit_per_mille").value
        )
        self.spur_hold_torque_limit_per_mille = int(
            self.get_parameter("spur_hold_torque_limit_per_mille").value
        )
        if not 1 <= self.spur_close_torque_limit_per_mille <= MAX_TORQUE_PER_MILLE:
            raise ValueError(
                "spur_close_torque_limit_per_mille must be "
                f"1..{MAX_TORQUE_PER_MILLE}"
            )
        if self.spur_close_torque_limit_per_mille > self.csp_torque_limit_per_mille:
            raise ValueError(
                "spur_close_torque_limit_per_mille cannot exceed "
                "csp_torque_limit_per_mille"
            )
        if not 1 <= self.spur_hold_torque_limit_per_mille <= (
            self.spur_close_torque_limit_per_mille
        ):
            raise ValueError(
                "spur_hold_torque_limit_per_mille must be 1.."
                "spur_close_torque_limit_per_mille"
            )
        self.clear_limit_switch_mappings_for_csp = bool(
            self.get_parameter("clear_limit_switch_mappings_for_csp").value
        )
        self.ignore_spur_gear_in_csp = bool(
            self.get_parameter("ignore_spur_gear_in_csp").value
        )
        self.skip_spur_gear_homing = bool(
            self.get_parameter("skip_spur_gear_homing").value
        )
        self.spur_gear_reference_delta_counts = int(
            self.get_parameter("spur_gear_reference_delta_counts").value
        )
        self.spur_gear_reference_timeout_s = float(
            self.get_parameter("spur_gear_reference_timeout_s").value
        )
        self.spur_gear_reference_tolerance_counts = int(
            self.get_parameter("spur_gear_reference_tolerance_counts").value
        )
        self.spur_gear_reference_profile_velocity = int(
            self.get_parameter("spur_gear_reference_profile_velocity").value
        )
        self.spur_gear_reference_profile_acceleration = int(
            self.get_parameter("spur_gear_reference_profile_acceleration").value
        )
        self.spur_gear_reference_profile_deceleration = int(
            self.get_parameter("spur_gear_reference_profile_deceleration").value
        )
        self.spur_gear_reference_following_error_confirm_s = float(
            self.get_parameter("spur_gear_reference_following_error_confirm_s").value
        )
        if self.spur_gear_reference_delta_counts == 0:
            raise ValueError("spur_gear_reference_delta_counts must be non-zero")
        if self.spur_gear_reference_timeout_s <= 0.0:
            raise ValueError("spur_gear_reference_timeout_s must be positive")
        if self.spur_gear_reference_tolerance_counts < 0:
            raise ValueError("spur_gear_reference_tolerance_counts must be non-negative")
        if min(
            self.spur_gear_reference_profile_velocity,
            self.spur_gear_reference_profile_acceleration,
            self.spur_gear_reference_profile_deceleration,
        ) <= 0:
            raise ValueError("Drive 3 reference profile parameters must be positive")
        if self.spur_gear_reference_following_error_confirm_s <= 0.0:
            raise ValueError(
                "spur_gear_reference_following_error_confirm_s must be positive"
            )
        self.drive2_following_error_window_counts = int(
            self.get_parameter("drive2_following_error_window_counts").value
        )
        self.drive2_following_error_timeout_ms = int(
            self.get_parameter("drive2_following_error_timeout_ms").value
        )
        self.csp_stall_error_counts = int(
            self.get_parameter("csp_stall_error_counts").value
        )
        self.csp_stall_progress_counts = int(
            self.get_parameter("csp_stall_progress_counts").value
        )
        self.csp_stall_timeout_ms = int(
            self.get_parameter("csp_stall_timeout_ms").value
        )

        self.homing_methods = self._int_parameter_list("homing_methods")
        self.reference_inputs = self._int_parameter_list("reference_inputs")
        self.homing_offsets = self._int_parameter_list("homing_offsets")
        self.homing_search_speeds = self._int_parameter_list("homing_search_speeds")
        self.homing_zero_speeds = self._int_parameter_list("homing_zero_speeds")
        self.homing_accelerations = self._int_parameter_list("homing_accelerations")
        self.homing_interval_max_travel_counts = [
            int(
                self.get_parameter(
                    f"homing_interval_max_travel_drive{drive_index}_counts"
                ).value
            )
            for drive_index in range(4)
        ]
        self.homing_interval_timeout_s = float(
            self.get_parameter("homing_interval_timeout_s").value
        )
        self.homing_interval_poll_s = float(
            self.get_parameter("homing_interval_poll_s").value
        )
        self.homing_midpoint_tolerance_counts = int(
            self.get_parameter("homing_midpoint_tolerance_counts").value
        )
        self.home_adjust_profile_velocity = int(
            self.get_parameter("home_adjust_profile_velocity").value
        )
        self.home_adjust_timeout_s = float(
            self.get_parameter("home_adjust_timeout_s").value
        )
        self.home_adjust_tolerance_counts = int(
            self.get_parameter("home_adjust_tolerance_counts").value
        )
        self.home_adjust_following_error_confirm_s = float(
            self.get_parameter("home_adjust_following_error_confirm_s").value
        )
        self._validate_homing_parameters()

        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.spur_gear_reference_complete = False
        self.spur_gear_reference_source_counts: Optional[int] = None
        self.spur_gear_reference_target_counts: Optional[int] = None
        self.spur_gear_reference_pre_zero_counts: Optional[int] = None
        self.spur_gear_reference_zero_readback: Optional[int] = None
        self.homing_interval_results: Dict[int, HomingIntervalResult] = {}
        self.bus = FaulhaberBus(
            self.interface,
            self.slave_indices,
            self.sdo_delay_s,
            self.verbose,
            self.control_mode,
            self.pdo_cycle_ns,
            self.pdo_timeout_us,
            self.enable_dc_sync,
            self.csp_torque_limit_per_mille,
            [len(self.slave_indices) - 1] if self.ignore_spur_gear_in_csp else [],
            (
                list(range(len(self.slave_indices) - 1))
                if self.skip_spur_gear_homing
                else list(range(len(self.slave_indices)))
            ),
            csp_stall_error_counts=self.csp_stall_error_counts,
            csp_stall_progress_counts=self.csp_stall_progress_counts,
            csp_stall_timeout_ms=self.csp_stall_timeout_ms,
            drive2_following_error_window_counts=(
                self.drive2_following_error_window_counts
            ),
            drive2_following_error_timeout_ms=(
                self.drive2_following_error_timeout_ms
            ),
            diagnostic_logger=self.get_logger().warning,
            clear_limit_switch_mappings_for_csp=(
                self.clear_limit_switch_mappings_for_csp
            ),
        )
        self.get_logger().info(
            f"Connecting EtherCAT on {self.interface}; control_mode={self.control_mode}"
        )
        self.get_logger().info(
            "CSP stall diagnostics: "
            f"error>={self.csp_stall_error_counts} counts, "
            f"progress<{self.csp_stall_progress_counts} counts for "
            f"{self.csp_stall_timeout_ms} ms"
        )
        self.get_logger().info(
            "Drive 3 two-stage close guard: approach/hold "
            "0x60E0/0x60E1="
            f"{self.spur_close_torque_limit_per_mille}/"
            f"{self.spur_hold_torque_limit_per_mille} per-mille; open and "
            f"custom motion restore {self.csp_torque_limit_per_mille} per-mille"
        )
        if self.clear_limit_switch_mappings_for_csp:
            self.get_logger().info(
                "CSP handoff will clear and verify volatile lower/upper "
                "limit-input mappings 0x2310:01/:02; Homing reference, input "
                "polarity and 0x607B/0x607D remain unchanged"
            )
        else:
            self.get_logger().warning(
                "CSP handoff will preserve 0x2310:01/:02 by parameter; "
                "a mapped active input may stop otherwise valid motion"
            )
        self.get_logger().info(
            "Drive 0-2 Homing uses the centre of the reference-input interval: "
            "find the first edge with the configured native method, traverse the "
            "active interval and return to (entry+exit)/2 at the lower Homing "
            "zero speed with a sinusoidal profile, then set that midpoint to "
            "zero with Method 37; second-edge travel guards D0/D1/D2="
            f"{self.homing_interval_max_travel_counts[:3]} counts"
        )
        if self.ignore_spur_gear_in_csp:
            self.get_logger().warning(
                "Emergency three-axis fallback: spur_gear_joint (Drive 3) will not Home, "
                "will remain Disable Voltage in CSP, and its CSP targets will be ignored"
            )
        elif self.skip_spur_gear_homing:
            self.get_logger().info(
                "Drive 3 skips sensor Homing; after Drives 0-2 Home it will move "
                f"{self.spur_gear_reference_delta_counts:+d} counts and use "
                "Homing Method 37 to set that position to 0"
            )
        self.bus.connect()
        self._configure_drive2_csp_protection()
        for drive in self.bus.drives:
            drive.configure_profile_motion(
                self.profile_velocity,
                self.profile_acceleration,
                self.profile_deceleration,
            )

        self.enable_all_srv = self.create_service(Trigger, "~/enable_all", self.on_enable_all)
        self.disable_all_srv = self.create_service(Trigger, "~/disable_all", self.on_disable_all)
        self.home_one_srv = self.create_service(Trigger, "~/home_one", self.on_home_one)
        self.home_all_srv = self.create_service(Trigger, "~/home_all", self.on_home_all)
        self.adjust_home_counts_srv = self.create_service(
            Trigger, "~/adjust_home_counts", self.on_adjust_home_counts
        )
        self.set_current_arm_home_srv = self.create_service(
            Trigger, "~/set_current_arm_home", self.on_set_current_arm_home
        )
        self.goto_home_all_srv = self.create_service(
            Trigger, "~/goto_home_all", self.on_goto_home_all
        )
        self.read_digital_inputs_srv = self.create_service(
            Trigger, "~/read_digital_inputs", self.on_read_digital_inputs
        )
        self.read_drive2_diagnostics_srv = self.create_service(
            Trigger, "~/read_drive2_diagnostics", self.on_read_drive2_diagnostics
        )
        self.read_csp_stall_snapshot_srv = self.create_service(
            Trigger, "~/read_csp_stall_snapshot", self.on_read_csp_stall_snapshot
        )
        self.read_spur_gear_counts_srv = self.create_service(
            Trigger, "~/read_spur_gear_counts", self.on_read_spur_gear_counts
        )
        self.enable_spur_close_guard_srv = self.create_service(
            Trigger, "~/enable_spur_close_guard", self.on_enable_spur_close_guard
        )
        self.enable_spur_hold_guard_srv = self.create_service(
            Trigger, "~/enable_spur_hold_guard", self.on_enable_spur_hold_guard
        )
        self.restore_spur_torque_srv = self.create_service(
            Trigger, "~/restore_spur_torque", self.on_restore_spur_torque
        )
        self.capture_spur_contact_snapshot_srv = self.create_service(
            Trigger,
            "~/capture_spur_contact_snapshot",
            self.on_capture_spur_contact_snapshot,
        )
        self.blink_digout1_srv = self.create_service(
            Trigger, "~/blink_digout1", self.on_blink_digout1
        )
        self.digout1_sub = self.create_subscription(Bool, "~/digout1", self.on_digout1, 10)
        self.home_done_pub = self.create_publisher(Bool, "~/home_done", 10)

        self.server_thread = threading.Thread(target=self.tcp_server_loop, daemon=True)
        self.server_thread.start()
        self.get_logger().info(f"TCP bridge listening on {self.host}:{self.port}")

    def _int_parameter_list(self, name: str) -> List[int]:
        return [int(value) for value in self.get_parameter(name).value]

    def _validate_homing_parameters(self) -> None:
        expected = len(self.slave_indices)
        parameters = {
            "homing_methods": self.homing_methods,
            "reference_inputs": self.reference_inputs,
            "homing_offsets": self.homing_offsets,
            "homing_search_speeds": self.homing_search_speeds,
            "homing_zero_speeds": self.homing_zero_speeds,
            "homing_accelerations": self.homing_accelerations,
            "homing_interval_max_travel_counts": (
                self.homing_interval_max_travel_counts
            ),
        }
        for name, values in parameters.items():
            if len(values) != expected:
                raise ValueError(f"{name} needs {expected} entries, got {len(values)}")
        if any(value <= 0 for value in self.homing_interval_max_travel_counts):
            raise ValueError(
                "homing_interval_max_travel_counts entries must be positive"
            )
        if self.homing_interval_timeout_s <= 0.0:
            raise ValueError("homing_interval_timeout_s must be positive")
        if self.homing_interval_poll_s <= 0.0:
            raise ValueError("homing_interval_poll_s must be positive")
        if self.homing_midpoint_tolerance_counts < 0:
            raise ValueError("homing_midpoint_tolerance_counts must be non-negative")
        if self.home_adjust_profile_velocity <= 0:
            raise ValueError("home_adjust_profile_velocity must be positive")
        if self.home_adjust_timeout_s <= 0.0:
            raise ValueError("home_adjust_timeout_s must be positive")
        if self.home_adjust_tolerance_counts < 0:
            raise ValueError("home_adjust_tolerance_counts must be non-negative")
        if self.home_adjust_following_error_confirm_s < 0.0:
            raise ValueError(
                "home_adjust_following_error_confirm_s must be non-negative"
            )

    def _drive2_position_protection_message(self) -> str:
        if len(self.bus.drives) <= 2:
            raise RuntimeError("Drive 2 is not configured")
        values = self.bus.drives[2].read_position_protection()
        return (
            "Drive 2 protection: "
            f"0x607B position_range=[{values['position_range_min']}, "
            f"{values['position_range_max']}], "
            f"0x607D software_limit=[{values['software_limit_min']}, "
            f"{values['software_limit_max']}], "
            f"0x6065 following_window={values['following_error_window']} counts, "
            f"0x6066 following_timeout={values['following_error_timeout_ms']} ms"
        )

    def _configure_drive2_csp_protection(self) -> None:
        """Report Drive 2 limits and apply the finite CSP following-error monitor.

        No write to 0x607B or 0x607D occurs here. The bridge also deliberately
        does not issue a CiA-301 parameter-store command, so it does not request
        a persistent change to the controller's non-volatile configuration.
        """

        if self.control_mode not in ("csp", "homing_csp"):
            return
        if len(self.bus.drives) <= 2:
            raise RuntimeError("CSP configuration requires Drive 2")

        drive = self.bus.drives[2]
        # The following-error settings are required for this CSP session, so
        # read/write/readback them independently from optional limit reporting.
        # Some MC5004 units return a transient mailbox WKC error on the first
        # 0x607B read immediately after PRE-OP discovery.
        before_window = drive.sdo_read_int_retry(FOLLOWING_ERROR_WINDOW, 0)
        before_timeout = drive.sdo_read_int_retry(FOLLOWING_ERROR_TIMEOUT, 0)
        drive.configure_following_error_monitor(
            self.drive2_following_error_window_counts,
            self.drive2_following_error_timeout_ms,
        )
        try:
            position_protection = self._drive2_position_protection_message()
        except Exception as exc:
            position_protection = (
                "Drive 2 0x607B/0x607D diagnostic unavailable after retries "
                f"({type(exc).__name__}); bridge startup continues and group 6 will retry"
            )
        self.get_logger().warning(
            "Drive 2 CSP following-error monitor changed for this session only: "
            f"0x6065 {before_window} -> "
            f"{self.drive2_following_error_window_counts} counts; "
            f"0x6066 {before_timeout} -> "
            f"{self.drive2_following_error_timeout_ms} ms. "
            "0x607B/0x607D were read only, not modified. "
            + position_protection
        )

    def _ensure_non_csp_operation(self, operation: str) -> None:
        if self.bus.csp_active:
            raise RuntimeError(
                f"Cannot {operation} while CSP/PDO is active. Stop ros2_control and use "
                "the homing launch first."
            )
        if self.control_mode == "csp":
            raise RuntimeError(
                f"Cannot {operation} from a standalone CSP bridge. Use homing.launch.py."
            )
        if self.bus.deferred_csp_prepared:
            raise RuntimeError(
                f"Cannot {operation} after PDO preparation. Support the arm, stop both "
                "launches, and start a new Homing session."
            )

    def _home_drive(self, drive_index: int) -> int:
        self.spur_gear_reference_complete = False
        self.spur_gear_reference_source_counts = None
        self.spur_gear_reference_target_counts = None
        self.spur_gear_reference_pre_zero_counts = None
        self.spur_gear_reference_zero_readback = None
        self.homing_interval_results.pop(drive_index, None)
        self.bus.mark_drive_homing_started(drive_index)
        drive = self.bus.drives[drive_index]
        result = drive.home_to_reference_interval_midpoint(
            method=self.homing_methods[drive_index],
            reference_input=self.reference_inputs[drive_index],
            offset_counts=self.homing_offsets[drive_index],
            search_speed=self.homing_search_speeds[drive_index],
            zero_speed=self.homing_zero_speeds[drive_index],
            acceleration=self.homing_accelerations[drive_index],
            timeout_s=self.motion_timeout_s,
            interval_timeout_s=self.homing_interval_timeout_s,
            max_travel_counts=self.homing_interval_max_travel_counts[drive_index],
            poll_s=self.homing_interval_poll_s,
            midpoint_tolerance_counts=self.homing_midpoint_tolerance_counts,
        )
        self.homing_interval_results[drive_index] = result
        self.get_logger().warning(
            f"HOMING_INTERVAL drive={drive_index} "
            f"entry={result.first_edge_counts} "
            f"exit={result.second_edge_counts} "
            f"width={abs(result.second_edge_counts - result.first_edge_counts)} "
            f"midpoint={result.midpoint_counts} "
            f"reached={result.midpoint_actual_counts} "
            f"zero={result.zero_readback_counts} "
            f"zero_tolerance={self.homing_midpoint_tolerance_counts}"
        )
        self.bus.mark_drive_homed(drive_index)
        return result.zero_readback_counts

    def _homing_interval_message(self, drive_index: int) -> str:
        result = self.homing_interval_results[drive_index]
        return (
            f"drive{drive_index}_interval("
            f"entry={result.first_edge_counts},"
            f"exit={result.second_edge_counts},"
            f"width={abs(result.second_edge_counts - result.first_edge_counts)},"
            f"midpoint={result.midpoint_counts},"
            f"reached={result.midpoint_actual_counts},"
            f"zero={result.zero_readback_counts},"
            f"zero_tolerance={self.homing_midpoint_tolerance_counts})"
        )

    def _reference_spur_gear_after_arm_homing(self) -> str:
        """Move Drive 3 by the configured increment, then make that position zero."""

        self.spur_gear_reference_complete = False
        if not self.bus.homing_complete:
            raise RuntimeError(
                "Drive 3 reference requires all Drive 0-2 Homing operations to succeed first"
            )
        if self.ignore_spur_gear_in_csp:
            self.spur_gear_reference_complete = True
            return "drive3=ignored"
        if not self.skip_spur_gear_homing:
            # The four-axis sensor-Homing fallback already gives Drive 3 a zero.
            self.spur_gear_reference_complete = True
            zero = self.bus.drives[-1].read_actual_position_counts()
            self.spur_gear_reference_zero_readback = zero
            return f"drive3_sensor_home={zero}"

        drive = self.bus.drives[-1]
        drive.enable_operation(MODE_PROFILE_POSITION)
        drive.configure_profile_motion(
            self.spur_gear_reference_profile_velocity,
            self.spur_gear_reference_profile_acceleration,
            self.spur_gear_reference_profile_deceleration,
        )
        try:
            source, target, actual = drive.move_relative_counts_and_wait(
                self.spur_gear_reference_delta_counts,
                self.spur_gear_reference_timeout_s,
                self.spur_gear_reference_tolerance_counts,
                self.spur_gear_reference_following_error_confirm_s,
            )
            zero = drive.home_current_position(
                self.spur_gear_reference_timeout_s,
                self.homing_midpoint_tolerance_counts,
            )
        except Exception:
            # A failed Profile Position or Method 37 command may otherwise keep
            # Drive 3 enabled after the service returns. Stop it before rejecting
            # the Home sequence.
            try:
                drive.disable_operation()
            except Exception as stop_exc:
                self.get_logger().error(
                    f"Drive 3 reference failed and Disable Voltage also failed: {stop_exc}"
                )
            raise
        self.spur_gear_reference_source_counts = source
        self.spur_gear_reference_target_counts = target
        self.spur_gear_reference_pre_zero_counts = actual
        self.spur_gear_reference_zero_readback = zero
        self.spur_gear_reference_complete = True
        message = (
            "drive3_reference("
            f"source={source},delta={self.spur_gear_reference_delta_counts},"
            f"target={target},reached={actual},zero={zero},method=37)"
        )
        self.get_logger().warning("SPUR_REFERENCE " + message)
        return message

    def _require_spur_gear_reference_for_csp(self) -> None:
        if (
            self.control_mode == "homing_csp"
            and not self.ignore_spur_gear_in_csp
            and not self.spur_gear_reference_complete
        ):
            raise RuntimeError(
                "CSP handoff rejected: Drive 3 reference is incomplete; run home_all "
                "or complete Drive 0-2 home_one so Drive 3 can move "
                f"{self.spur_gear_reference_delta_counts:+d} counts and set zero"
            )

    def _spur_gear_counts_message(self) -> str:
        drive_index = len(self.bus.drives) - 1
        if self.bus.csp_active:
            actual, status, mode = self.bus.get_csp_states()[drive_index]
            source = "PDO"
        else:
            drive = self.bus.drives[drive_index]
            actual = drive.read_actual_position_counts()
            status = drive.read_status()
            mode = drive.read_mode_display()
            source = "SDO"
        return (
            f"Drive {drive_index}: absolute_counts={actual}, "
            f"statusword=0x{status:04X}, mode={mode}, source={source}, "
            f"reference_complete={str(self.spur_gear_reference_complete).lower()}, "
            f"reference_delta={self.spur_gear_reference_delta_counts}, "
            f"pre_zero_raw={self.spur_gear_reference_pre_zero_counts}"
        )

    def publish_home_done(self) -> None:
        msg = Bool()
        msg.data = True
        self.home_done_pub.publish(msg)

    def on_enable_all(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        try:
            with self.lock:
                if self.bus.csp_active or self.control_mode == "csp":
                    self._require_spur_gear_reference_for_csp()
                    states = self.bus.enter_csp()
                    statuses = [state[1] for state in states]
                elif self.bus.deferred_csp_prepared:
                    raise RuntimeError("Restart the Homing session after CSP has exited")
                else:
                    statuses = [
                        drive.enable_operation()
                        for drive in self.bus.drives
                        if drive.drive_id in self.bus.required_homing_drive_ids
                    ]
            response.success = True
            response.message = "Enabled required drives: " + " ".join(
                f"0x{status:04X}" for status in statuses
            )
        except Exception as exc:
            response.success = False
            response.message = f"Enable failed: {exc}"
        return response

    def on_disable_all(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        try:
            with self.lock:
                statuses = (
                    self.bus.exit_csp()
                    if self.bus.csp_active
                    else [drive.disable_operation() for drive in self.bus.drives]
                )
            response.success = True
            response.message = "Disabled all drives: " + " ".join(
                f"0x{status:04X}" for status in statuses
            )
        except Exception as exc:
            response.success = False
            response.message = f"Disable failed: {exc}"
        return response

    def on_home_one(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        try:
            drive_index = int(self.get_parameter("test_drive_index").value)
            if not 0 <= drive_index < len(self.bus.drives):
                raise ValueError(f"test_drive_index={drive_index} is out of range")
            if drive_index not in self.bus.required_homing_drive_ids:
                response.success = True
                response.message = (
                    f"Drive {drive_index} does not use sensor Homing in this session; "
                    "complete Drives 0-2 so its fixed relative reference and Method 37 "
                    "zero can run automatically"
                )
                return response
            with self.lock:
                self._ensure_non_csp_operation("home")
                position = self._home_drive(drive_index)
                spur_reference = (
                    self._reference_spur_gear_after_arm_homing()
                    if self.bus.homing_complete
                    else ""
                )
            response.success = True
            suffix = (
                f"; {spur_reference}; CSP handoff armed"
                if self.bus.homing_complete
                else ""
            )
            response.message = (
                f"Drive {drive_index} interval-centre Homing completed; "
                f"actual_position={position}; "
                f"{self._homing_interval_message(drive_index)}{suffix}"
            )
        except Exception as exc:
            response.success = False
            response.message = f"Home one failed: {exc}"
        return response

    def on_home_all(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        try:
            with self.lock:
                self._ensure_non_csp_operation("home")
                positions = {
                    index: self._home_drive(index)
                    for index in sorted(self.bus.required_homing_drive_ids)
                }
                spur_reference = self._reference_spur_gear_after_arm_homing()
            self.publish_home_done()
            response.success = True
            response.message = (
                "Homing completed for required drives; CSP handoff armed: "
                + " ".join(
                    f"drive{index}={position}" for index, position in positions.items()
                )
                + "; "
                + " ".join(
                    self._homing_interval_message(index)
                    for index in sorted(positions)
                )
                + f" {spur_reference}"
            )
        except Exception as exc:
            response.success = False
            response.message = f"Home failed: {exc}"
        return response

    def _adjust_homed_drive_counts(self, drive_index: int, delta_counts: int) -> str:
        """Move one homed arm drive by a relative Profile Position increment."""

        if drive_index not in (0, 1, 2):
            raise ValueError(
                f"Home fine adjustment only supports Drive 0-2, got Drive {drive_index}"
            )
        if drive_index >= len(self.bus.drives):
            raise ValueError(f"Drive {drive_index} is not configured")
        if delta_counts == 0:
            raise ValueError("Home fine-adjustment delta must be non-zero")
        if not self.bus.homing_complete:
            raise RuntimeError(
                "Home fine adjustment requires successful Homing of all Drive 0-2 first"
            )
        if not self.spur_gear_reference_complete:
            raise RuntimeError(
                "Home fine adjustment requires the Drive 3 reference sequence to finish"
            )

        drive = self.bus.drives[drive_index]
        limit_mapping_state = "preserved"
        if self.clear_limit_switch_mappings_for_csp:
            # The reference sensor is not a persistent travel limit.  Clear
            # only 0x2310:01/:02 before Profile Position so an asserted stale
            # mapping cannot block one fine-adjustment direction.  The Homing
            # reference at 0x2310:04 and all position-limit objects stay intact.
            drive.clear_limit_switch_mappings_for_csp()
            limit_mapping_state = "cleared"
        drive.enable_operation(MODE_PROFILE_POSITION)
        drive.configure_profile_motion(
            self.home_adjust_profile_velocity,
            self.homing_accelerations[drive_index],
            self.homing_accelerations[drive_index],
        )
        source, target, actual = drive.move_relative_counts_and_wait(
            delta_counts,
            self.home_adjust_timeout_s,
            self.home_adjust_tolerance_counts,
            self.home_adjust_following_error_confirm_s,
        )
        error = actual - target
        message = (
            f"Drive {drive_index} Home fine adjustment completed: "
            f"source={source}, delta={delta_counts:+d}, target={target}, "
            f"actual={actual}, target_error={error:+d}, "
            f"correction_from_homed_zero={actual} counts, "
            f"limit_input_mappings={limit_mapping_state}"
        )
        self.get_logger().warning("HOME_FINE_ADJUST " + message)
        return message

    def on_adjust_home_counts(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        try:
            drive_index = int(self.get_parameter("test_drive_index").value)
            delta_counts = int(self.get_parameter("test_relative_counts").value)
            with self.lock:
                self._ensure_non_csp_operation("fine-adjust Home")
                response.message = self._adjust_homed_drive_counts(
                    drive_index, delta_counts
                )
            response.success = True
        except Exception as exc:
            response.success = False
            response.message = f"Home fine adjustment failed: {exc}"
        return response

    def _set_current_arm_position_as_home(self) -> str:
        """Define the current Drive 0-2 positions as this session's Home zero."""

        if not self.bus.homing_complete:
            raise RuntimeError(
                "Setting current pose as Home requires successful Homing of all "
                "Drive 0-2 first"
            )
        if not self.spur_gear_reference_complete:
            raise RuntimeError(
                "Setting current pose as Home requires the Drive 3 reference "
                "sequence to finish"
            )

        drive_indices = sorted(self.bus.required_homing_drive_ids)
        if drive_indices != [0, 1, 2]:
            raise RuntimeError(
                "Manual arm Home expects required Homing drives [0, 1, 2], got "
                f"{drive_indices}"
            )

        before = {
            drive_index: self.bus.drives[drive_index].read_actual_position_counts()
            for drive_index in drive_indices
        }
        before_message = " ".join(
            f"drive{drive_index}_before={before[drive_index]}"
            for drive_index in drive_indices
        )
        # Log the captured calibration values before the first Method 37 write,
        # so they remain available even if a later drive fails.
        self.get_logger().warning("MANUAL_ARM_HOME_CAPTURE " + before_message)

        after: Dict[int, int] = {}
        for drive_index in drive_indices:
            try:
                after[drive_index] = self.bus.drives[
                    drive_index
                ].home_current_position(
                    self.home_adjust_timeout_s,
                    self.home_adjust_tolerance_counts,
                )
            except Exception as exc:
                completed = ",".join(str(index) for index in sorted(after)) or "none"
                raise RuntimeError(
                    f"Drive {drive_index} Method 37 failed after completed drives "
                    f"[{completed}]; captured values: {before_message}; {exc}"
                ) from exc

        after_message = " ".join(
            f"drive{drive_index}_after={after[drive_index]}"
            for drive_index in drive_indices
        )
        message = (
            "Current arm pose set as session Home with Method 37; "
            f"{before_message}; {after_message}; Drive 3 unchanged"
        )
        self.get_logger().warning("MANUAL_ARM_HOME_SET " + message)
        return message

    def on_set_current_arm_home(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        try:
            with self.lock:
                self._ensure_non_csp_operation("set current arm pose as Home")
                response.message = self._set_current_arm_position_as_home()
            self.publish_home_done()
            response.success = True
        except Exception as exc:
            response.success = False
            response.message = f"Set current arm Home failed: {exc}"
        return response

    def on_goto_home_all(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        try:
            with self.lock:
                self._ensure_non_csp_operation("move to home")
                positions = {
                    drive.drive_id: drive.move_absolute_counts_and_wait(
                        0, self.motion_timeout_s
                    )
                    for drive in self.bus.drives
                    if drive.drive_id in self.bus.required_homing_drive_ids
                }
            self.publish_home_done()
            response.success = True
            response.message = "Moved required drives to zero: " + " ".join(
                f"drive{index}={position}" for index, position in positions.items()
            )
        except Exception as exc:
            response.success = False
            response.message = f"Goto home failed: {exc}"
        return response

    def on_read_digital_inputs(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        try:
            with self.lock:
                self._ensure_non_csp_operation("read digital inputs through SDO")
                results = []
                for drive in self.bus.drives:
                    values = drive.read_digital_input_configuration()
                    flags = FaulhaberDrive._decode_flags(
                        values["device_status"], DEVICE_STATUS_FLAGS
                    )
                    results.append(
                        f"Drive {drive.drive_id}: "
                        f"physical=0x{values['physical_inputs']:02X}/"
                        f"{values['physical_inputs']:08b}, "
                        f"logical=0x{values['logical_inputs']:02X}/"
                        f"{values['logical_inputs']:08b}, "
                        f"polarity=0x{values['input_polarity']:02X}; "
                        "0x2310 lower/upper/option/reference="
                        f"0x{values['lower_limit_input_mask']:02X}/"
                        f"0x{values['upper_limit_input_mask']:02X}/"
                        f"{values['limit_switch_option']}/"
                        f"{values['reference_input']}; "
                        f"0x2324.01=0x{values['device_status']:08X} [{flags}]"
                    )
            response.success = True
            response.message = " | ".join(results)
        except Exception as exc:
            response.success = False
            response.message = f"Read digital inputs failed: {exc}"
        return response

    def on_read_drive2_diagnostics(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        try:
            with self.lock:
                self._ensure_non_csp_operation("read Drive 2 protection through SDO")
                response.success = True
                response.message = self._drive2_position_protection_message()
        except Exception as exc:
            response.success = False
            response.message = f"Read Drive 2 diagnostics failed: {exc}"
        return response

    def on_read_csp_stall_snapshot(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Return the latest auto-captured stall without issuing live SDOs here."""

        deadline = time.monotonic() + 3.0
        while self.bus.live_diagnostic_pending and time.monotonic() < deadline:
            time.sleep(0.05)
        response.success = not self.bus.live_diagnostic_pending
        response.message = self.bus.last_stall_snapshot
        if self.bus.live_diagnostic_pending:
            response.message += "; staged LIVE_DIAG still pending; wait and retry"
        return response

    def on_read_spur_gear_counts(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Return the exact Drive 3 position in the current method-37 coordinate."""

        try:
            with self.lock:
                response.message = self._spur_gear_counts_message()
            response.success = self.spur_gear_reference_complete
            if not response.success:
                response.message += "; zero reference is not complete"
        except Exception as exc:
            response.success = False
            response.message = f"Read Drive 3 counts failed: {exc}"
        return response

    def on_enable_spur_close_guard(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Lower Drive 3 directional torque before the close trajectory starts."""

        try:
            response.message = self.bus.request_spur_torque_limit(
                self.spur_close_torque_limit_per_mille
            )
            response.success = True
        except Exception as exc:
            response.success = False
            response.message = f"Enable Drive 3 close guard failed: {exc}"
        return response

    def on_restore_spur_torque(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Restore Drive 3's normal CSP torque before open/custom motion."""

        try:
            response.message = self.bus.request_spur_torque_limit(
                self.csp_torque_limit_per_mille
            )
            response.success = True
        except Exception as exc:
            response.success = False
            response.message = f"Restore Drive 3 CSP torque failed: {exc}"
        return response

    def on_enable_spur_hold_guard(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Lower Drive 3 from approach torque to its gentle contact hold."""

        try:
            response.message = self.bus.request_spur_torque_limit(
                self.spur_hold_torque_limit_per_mille
            )
            response.success = True
        except Exception as exc:
            response.success = False
            response.message = f"Enable Drive 3 hold guard failed: {exc}"
        return response

    def on_capture_spur_contact_snapshot(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Log Drive 3 torque/current/status through staged live SDO reads."""

        try:
            response.message = self.bus.request_spur_contact_snapshot()
            response.success = True
        except Exception as exc:
            response.success = False
            response.message = f"Capture Drive 3 contact snapshot failed: {exc}"
        return response

    def on_digout1(self, msg: Bool) -> None:
        try:
            with self.lock:
                self._ensure_non_csp_operation("write DigOut1 through SDO")
                self.bus.drives[0].set_digout1(bool(msg.data))
        except Exception as exc:
            self.get_logger().error(f"DigOut1 command failed: {exc}")

    def on_blink_digout1(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        try:
            with self.lock:
                self._ensure_non_csp_operation("blink DigOut1")
                self.bus.drives[0].set_digout1(True)
                time.sleep(0.5)
                self.bus.drives[0].set_digout1(False)
            response.success = True
            response.message = "DigOut1 blink finished."
        except Exception as exc:
            response.success = False
            response.message = f"DigOut1 blink failed: {exc}"
        return response

    def tcp_server_loop(self) -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((self.host, self.port))
            server.listen(1)
            server.settimeout(0.5)
            while not self.stop_event.is_set():
                try:
                    connection, address = server.accept()
                except socket.timeout:
                    continue
                except OSError:
                    break
                self.get_logger().info(f"Hardware client connected from {address}")
                with connection:
                    connection_file = connection.makefile("rwb")
                    while not self.stop_event.is_set():
                        line = connection_file.readline()
                        if not line:
                            break
                        command = line.decode("utf-8", errors="replace").strip()
                        response = self.handle_tcp_command(command)
                        connection_file.write((response + "\n").encode("utf-8"))
                        connection_file.flush()
                self.get_logger().info("Hardware client disconnected")

    @staticmethod
    def _format_pdo_response(states: Sequence[PDOState]) -> str:
        response = ["OK"]
        for actual, status, mode in states:
            response.extend((str(actual), f"0x{status:04X}", str(mode)))
        return " ".join(response)

    def handle_tcp_command(self, command: str) -> str:
        try:
            parts = command.split()
            if not parts:
                return "ERR empty command"
            operation = parts[0].upper()

            with self.lock:
                if operation == "PING":
                    return "OK"

                if operation == "ENABLE_ALL":
                    if self.bus.csp_active or self.control_mode == "csp":
                        return "ERR use ENTER_CSP_ALL when control_mode=csp"
                    if self.bus.deferred_csp_prepared:
                        return "ERR restart the Homing session after CSP has exited"
                    statuses = [drive.enable_operation() for drive in self.bus.drives]
                    return "OK " + " ".join(f"0x{status:04X}" for status in statuses)

                if operation == "DISABLE_ALL":
                    statuses = (
                        self.bus.exit_csp()
                        if self.bus.csp_active
                        else [drive.disable_operation() for drive in self.bus.drives]
                    )
                    return "OK " + " ".join(f"0x{status:04X}" for status in statuses)

                if operation == "GET_ALL":
                    if self.bus.csp_active or self.bus.latest_states:
                        states = self.bus.get_csp_states()
                        response = ["OK"]
                        for actual, status, _mode in states:
                            response.extend((str(actual), f"0x{status:04X}"))
                        return " ".join(response)
                    response = ["OK"]
                    for drive in self.bus.drives:
                        response.extend(
                            (str(drive.read_actual_position_counts()), f"0x{drive.read_status():04X}")
                        )
                    return " ".join(response)

                if operation == "GET_MODE_ALL":
                    modes = (
                        [state[2] for state in self.bus.get_csp_states()]
                        if self.bus.latest_states
                        else [drive.read_mode_display() for drive in self.bus.drives]
                    )
                    return "OK " + " ".join(str(mode) for mode in modes)

                if operation == "ENTER_CSP_ALL":
                    if len(parts) not in (1, len(self.bus.drives) + 1):
                        return f"ERR usage ENTER_CSP_ALL [<{len(self.bus.drives)} counts>]"
                    self._require_spur_gear_reference_for_csp()
                    targets = [int(value) for value in parts[1:]] if len(parts) > 1 else None
                    return self._format_pdo_response(self.bus.enter_csp(targets))

                if operation == "EXIT_CSP_ALL":
                    statuses = self.bus.exit_csp()
                    return "OK " + " ".join(f"0x{status:04X}" for status in statuses)

                if operation == "CSP_SETPOINT_ALL":
                    if len(parts) != len(self.bus.drives) + 1:
                        return f"ERR usage CSP_SETPOINT_ALL <{len(self.bus.drives)} counts>"
                    targets = [int(value) for value in parts[1:]]
                    return self._format_pdo_response(self.bus.set_csp_targets(targets))

                if operation == "MOVE_ALL":
                    self._ensure_non_csp_operation("use Profile Position commands")
                    if len(parts) != len(self.bus.drives) + 1:
                        return f"ERR usage MOVE_ALL <{len(self.bus.drives)} counts>"
                    for drive, target in zip(self.bus.drives, map(int, parts[1:])):
                        drive.move_absolute_counts(target)
                    return "OK"

                if operation == "MOVE_ABS":
                    self._ensure_non_csp_operation("use Profile Position commands")
                    if len(parts) != 3:
                        return "ERR usage MOVE_ABS <drive_index> <counts>"
                    self.bus.drives[int(parts[1])].move_absolute_counts(int(parts[2]))
                    return "OK"

                if operation == "HOME":
                    self._ensure_non_csp_operation("home")
                    if len(parts) != 2:
                        return "ERR usage HOME <drive_index>"
                    drive_index = int(parts[1])
                    position = self._home_drive(drive_index)
                    self.publish_home_done()
                    return f"OK {position}"

                if operation == "GOTO_HOME":
                    self._ensure_non_csp_operation("move to home")
                    if len(parts) == 1:
                        positions = [
                            drive.move_absolute_counts_and_wait(0, self.motion_timeout_s)
                            for drive in self.bus.drives
                        ]
                        self.publish_home_done()
                        return "OK " + " ".join(str(position) for position in positions)
                    if len(parts) == 2:
                        position = self.bus.drives[int(parts[1])].move_absolute_counts_and_wait(
                            0, self.motion_timeout_s
                        )
                        self.publish_home_done()
                        return f"OK {position}"
                    return "ERR usage GOTO_HOME [drive_index]"

                if operation == "DIGOUT1":
                    self._ensure_non_csp_operation("write DigOut1")
                    if len(parts) not in (2, 3):
                        return "ERR usage DIGOUT1 [drive_index] <ON|OFF|TOGGLE>"
                    drive_index = 0 if len(parts) == 2 else int(parts[1])
                    state = parts[-1].upper()
                    if state == "ON":
                        self.bus.drives[drive_index].set_digout1(True)
                    elif state == "OFF":
                        self.bus.drives[drive_index].set_digout1(False)
                    elif state == "TOGGLE":
                        self.bus.drives[drive_index].toggle_digout1()
                    else:
                        return "ERR DIGOUT1 state must be ON, OFF or TOGGLE"
                    return "OK"

                return f"ERR unknown command {operation}"
        except Exception as exc:
            return f"ERR {type(exc).__name__}: {exc}"

    def destroy_node(self) -> bool:
        self.stop_event.set()
        try:
            self.bus.close()
        except Exception as exc:
            self.get_logger().error(f"EtherCAT shutdown failed: {exc}")
        return super().destroy_node()


def main(args=None) -> None:
    # ros2 launch captures child stdout through a pipe, where Python otherwise
    # uses block buffering.  EtherCAT handoff/configuration messages must be
    # visible in T1 before the operator sends motion commands.
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(line_buffering=True, write_through=True)
        except (AttributeError, ValueError):
            pass
    rclpy.init(args=args)
    node = None
    try:
        node = RASCLFaulhaberBridge()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
