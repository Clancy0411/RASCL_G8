#!/usr/bin/env python3
"""EtherCAT bridge for the RASCL ros2_control hardware interface.

The C++ SystemInterface uses a small line-based TCP protocol.  This node owns
the pysoem master, keeps the proven SDO path for homing/profile fallback, and
runs the WP3 CSP position stream through cyclic PDO process data.
"""

from __future__ import annotations

import socket
import struct
import threading
import time
from typing import List, Optional, Sequence, Tuple

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
ACTUAL_TORQUE = 0x6077
HOMING_OFFSET = 0x607C
HOMING_METHOD = 0x6098
PROFILE_VELOCITY = 0x6081
PROFILE_ACCELERATION = 0x6083
PROFILE_DECELERATION = 0x6084
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

# CiA 402 control words.
CMD_SHUTDOWN = 0x0006
CMD_SWITCH_ON = 0x0007
CMD_DISABLE_OPERATION = 0x0007
CMD_ENABLE_OPERATION = 0x000F
CMD_DISABLE_VOLTAGE = 0x0000
CMD_FAULT_RESET = 0x0080
CMD_START_MOTION = 0x003F
CMD_START_HOMING = CMD_ENABLE_OPERATION | 0x0010

# Statusword state/operation bits.
STATUS_STATE_MASK = 0x006F
STATUS_READY_TO_SWITCH_ON = 0x0021
STATUS_SWITCHED_ON = 0x0023
STATUS_OPERATION_ENABLED_STATE = 0x0027
STATUS_FAULT = 1 << 3
STATUS_TARGET_REACHED = 1 << 10
STATUS_CSP_TARGET_ACCEPTED = 1 << 12
STATUS_HOMING_ATTAINED = 1 << 12
STATUS_FOLLOWING_OR_HOMING_ERROR = 1 << 13

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
        following_actual = read("following_error_actual", FOLLOWING_ERROR_ACTUAL_VALUE)
        velocity_actual = read("velocity_actual", VELOCITY_ACTUAL_VALUE, signed=True)
        torque_actual = read("torque_actual", ACTUAL_TORQUE, signed=True)
        maximum_torque = read("maximum_torque", MAX_TORQUE)
        positive_torque_limit = read("positive_torque_limit", POSITIVE_TORQUE_LIMIT)
        negative_torque_limit = read("negative_torque_limit", NEGATIVE_TORQUE_LIMIT)
        maximum_motor_speed = read("maximum_motor_speed", MAX_MOTOR_SPEED)
        position_gain = read("position_gain", POSITION_CONTROL_PARAMETER_SET, 1)

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
        parts.append(f"0x6077={torque_actual}")
        parts.append(
            "limits(0x6072/0x60E0/0x60E1/0x6080)="
            f"{maximum_torque}/{positive_torque_limit}/{negative_torque_limit}/"
            f"{maximum_motor_speed}"
        )
        parts.append(f"0x2348.01(Kv)={position_gain}")
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
        ignored_csp_drive_indices: Optional[Sequence[int]] = None,
        required_homing_drive_indices: Optional[Sequence[int]] = None,
    ) -> None:
        self.interface = interface
        self.slave_indices = slave_indices
        self.sdo_delay_s = sdo_delay_s
        self.verbose = verbose
        self.control_mode = control_mode
        self.pdo_cycle_ns = pdo_cycle_ns
        self.pdo_timeout_us = pdo_timeout_us
        self.enable_dc_sync = enable_dc_sync
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
            )
        return "CSP_SNAPSHOT " + "; ".join(entries)

    def _validate_running_states(self, states: Sequence[PDOState]) -> None:
        for drive_id, (_, statusword, mode) in enumerate(states):
            if drive_id in self.ignored_csp_drive_ids:
                continue
            if statusword & STATUS_FAULT:
                diagnostics = self.drives[drive_id].capture_fault_diagnostics()
                raise RuntimeError(
                    f"Drive {drive_id} fault; statusword=0x{statusword:04X}; "
                    f"{self._format_csp_snapshot(states)}; {diagnostics}"
                )
            if statusword & STATUS_FOLLOWING_OR_HOMING_ERROR:
                diagnostics = self.drives[drive_id].capture_fault_diagnostics()
                raise RuntimeError(
                    f"Drive {drive_id} CSP following error; statusword=0x{statusword:04X}; "
                    f"{self._format_csp_snapshot(states)}; {diagnostics}"
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
                self._safeop_locked()
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
        self.declare_parameter("ignore_spur_gear_in_csp", False)
        self.declare_parameter("skip_spur_gear_homing", True)
        # Drive 2's factory 32-count / 48-ms monitor is far below the normal
        # compliant motion lag of this 196:1 arm axis. Keep a finite, axis-
        # local monitor for CSP instead of disabling following-error detection.
        self.declare_parameter("drive2_following_error_window_counts", 25_000)
        self.declare_parameter("drive2_following_error_timeout_ms", 250)

        # Values validated on the auto_homing branch.
        self.declare_parameter("homing_methods", [28, 28, 24, 24])
        self.declare_parameter("reference_inputs", [2, 2, 2, 1])
        self.declare_parameter("homing_offsets", [0, 0, 0, 0])
        self.declare_parameter("homing_search_speeds", [1000, 1000, 1000, 1000])
        self.declare_parameter("homing_zero_speeds", [200, 200, 200, 200])
        self.declare_parameter("homing_accelerations", [1000, 1000, 1000, 1000])
        self.declare_parameter("test_drive_index", 0)

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
        self.ignore_spur_gear_in_csp = bool(
            self.get_parameter("ignore_spur_gear_in_csp").value
        )
        self.skip_spur_gear_homing = bool(
            self.get_parameter("skip_spur_gear_homing").value
        )
        self.drive2_following_error_window_counts = int(
            self.get_parameter("drive2_following_error_window_counts").value
        )
        self.drive2_following_error_timeout_ms = int(
            self.get_parameter("drive2_following_error_timeout_ms").value
        )

        self.homing_methods = self._int_parameter_list("homing_methods")
        self.reference_inputs = self._int_parameter_list("reference_inputs")
        self.homing_offsets = self._int_parameter_list("homing_offsets")
        self.homing_search_speeds = self._int_parameter_list("homing_search_speeds")
        self.homing_zero_speeds = self._int_parameter_list("homing_zero_speeds")
        self.homing_accelerations = self._int_parameter_list("homing_accelerations")
        self._validate_homing_parameters()

        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.bus = FaulhaberBus(
            self.interface,
            self.slave_indices,
            self.sdo_delay_s,
            self.verbose,
            self.control_mode,
            self.pdo_cycle_ns,
            self.pdo_timeout_us,
            self.enable_dc_sync,
            [len(self.slave_indices) - 1] if self.ignore_spur_gear_in_csp else [],
            (
                list(range(len(self.slave_indices) - 1))
                if self.skip_spur_gear_homing
                else list(range(len(self.slave_indices)))
            ),
        )
        self.get_logger().info(
            f"Connecting EtherCAT on {self.interface}; control_mode={self.control_mode}"
        )
        if self.ignore_spur_gear_in_csp:
            self.get_logger().warning(
                "Emergency three-axis fallback: spur_gear_joint (Drive 3) will not Home, "
                "will remain Disable Voltage in CSP, and its CSP targets will be ignored"
            )
        elif self.skip_spur_gear_homing:
            self.get_logger().info(
                "Drive 3 spur_gear_joint skips Homing but will be enabled and validated in CSP"
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
        self.goto_home_all_srv = self.create_service(
            Trigger, "~/goto_home_all", self.on_goto_home_all
        )
        self.read_digital_inputs_srv = self.create_service(
            Trigger, "~/read_digital_inputs", self.on_read_digital_inputs
        )
        self.read_drive2_diagnostics_srv = self.create_service(
            Trigger, "~/read_drive2_diagnostics", self.on_read_drive2_diagnostics
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
        }
        for name, values in parameters.items():
            if len(values) != expected:
                raise ValueError(f"{name} needs {expected} entries, got {len(values)}")

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
        self.bus.mark_drive_homing_started(drive_index)
        drive = self.bus.drives[drive_index]
        position = drive.home_to_reference_switch(
            method=self.homing_methods[drive_index],
            reference_input=self.reference_inputs[drive_index],
            offset_counts=self.homing_offsets[drive_index],
            search_speed=self.homing_search_speeds[drive_index],
            zero_speed=self.homing_zero_speeds[drive_index],
            acceleration=self.homing_accelerations[drive_index],
            timeout_s=self.motion_timeout_s,
        )
        self.bus.mark_drive_homed(drive_index)
        return position

    def publish_home_done(self) -> None:
        msg = Bool()
        msg.data = True
        self.home_done_pub.publish(msg)

    def on_enable_all(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        try:
            with self.lock:
                if self.bus.csp_active or self.control_mode == "csp":
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
                    f"Drive {drive_index} does not require Homing in this session; "
                    "it will be enabled for CSP"
                )
                return response
            with self.lock:
                self._ensure_non_csp_operation("home")
                position = self._home_drive(drive_index)
            response.success = True
            suffix = "; CSP handoff armed" if self.bus.homing_complete else ""
            response.message = (
                f"Drive {drive_index} homing completed; actual_position={position}{suffix}"
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
            self.publish_home_done()
            response.success = True
            response.message = (
                "Homing completed for required drives; CSP handoff armed: "
                + " ".join(
                    f"drive{index}={position}" for index, position in positions.items()
                )
            )
        except Exception as exc:
            response.success = False
            response.message = f"Home failed: {exc}"
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
                    logical, physical, polarity = drive.read_digital_inputs()
                    results.append(
                        f"Drive {drive.drive_id}: physical=0x{physical:02X}/{physical:08b}, "
                        f"logical=0x{logical:02X}/{logical:08b}, polarity=0x{polarity:02X}"
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
