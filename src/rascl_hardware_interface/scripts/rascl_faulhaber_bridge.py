#!/usr/bin/env python3
"""Local pysoem bridge for the RASCL ros2_control hardware interface.

The C++ SystemInterface speaks a small line-based TCP protocol to this node.
The node owns the pysoem EtherCAT master and performs CiA 402 SDO access for
all configured Faulhaber MC 5004 P ET drives.
"""

import socket
import threading
import time
from typing import List, Optional

import pysoem
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

# CiA 402 object dictionary indices.
CONTROL_WORD = 0x6040
STATUS_WORD = 0x6041
MODE_OF_OPERATION = 0x6060
MODE_DISPLAY = 0x6061
TARGET_POSITION = 0x607A
ACTUAL_POSITION = 0x6064
HOMING_OFFSET = 0x607C
HOMING_METHOD = 0x6098
PROFILE_VELOCITY = 0x6081
PROFILE_ACCELERATION = 0x6083
PROFILE_DECELERATION = 0x6084

# FAULHABER digital output object used in WP2.1 for the DigOut1 LED.
DIGITAL_IO_STATUS = 0x2311
DIGOUT_WRITE = 0x04
DIGOUT1_ON = 0x00FD
DIGOUT1_OFF = 0x00FC
DIGOUT1_TOGGLE = 0x00FE
#add on 7/10
DIGITAL_INPUT_SETTIN = 0x2310

DIGITAL_INPUT_LOGICAL = 0x01
DIGITAL_INPUT_PHYSICAL = 0x02
REFERENCE_SWITCH_INPUT = 0x04
INPUT_POLARITY = 0x10

HOMING_SPEED = 0x6099

HOMING_ACCELERATION = 0x609A

# CiA 402 modes.
MODE_PROFILE_POSITION = 1
MODE_HOMING = 6

# CiA 402 control words.
CMD_SHUTDOWN = 0x0006
CMD_SWITCH_ON = 0x0007
CMD_DISABLE_OPERATION = 0x0007
CMD_ENABLE_OPERATION = 0x000F
CMD_DISABLE_VOLTAGE = 0x0000
CMD_FAULT_RESET = 0x0080
CMD_START_MOTION = 0x003F
CMD_START_HOMING = CMD_ENABLE_OPERATION | 0x0010

# Statusword bits.
STATUS_OPERATION_ENABLED = 1 << 2
STATUS_FAULT = 1 << 3
STATUS_TARGET_REACHED = 1 << 10
STATUS_HOMING_ATTAINED = 1 << 12
STATUS_HOMING_ERROR = 1 << 13


class FaulhaberDrive:
    """Convenience wrapper around one Faulhaber EtherCAT slave."""

    # The wrapper intentionally uses blocking SDO calls. The ros2_control update
    # rate is low, and keeping the command sequence explicit makes the CiA 402
    # state transitions easier to inspect during lab debugging.

    def __init__(self, slave, drive_id: int, sdo_delay_s: float, verbose: bool) -> None:
        self.slave = slave
        self.drive_id = drive_id
        self.sdo_delay_s = sdo_delay_s
        self.verbose = verbose

    def sdo_write_int(self, index: int, subindex: int, value: int, size: int, signed: bool = False) -> None:
        # pysoem expects raw little-endian bytes for SDO writes.
        self.slave.sdo_write(index, subindex, int(value).to_bytes(size, "little", signed=signed))

    def sdo_read_int(self, index: int, subindex: int, signed: bool = False) -> int:
        # Convert the raw little-endian SDO payload back to a Python integer.
        data = self.slave.sdo_read(index, subindex)
        return int.from_bytes(data, "little", signed=signed)

    def read_status(self) -> int:
        return self.sdo_read_int(STATUS_WORD, 0, signed=False)

    def write_controlword(self, value: int, delay: Optional[float] = None) -> int:
        if self.verbose:
            print(f"[Drive {self.drive_id}] Controlword <- 0x{value:04X}")
        self.sdo_write_int(CONTROL_WORD, 0, value, size=2, signed=False)
        time.sleep(self.sdo_delay_s if delay is None else delay)
        return self.read_status()

    def reset_fault_if_needed(self) -> None:
        # A drive in Fault cannot be enabled until the CiA 402 fault-reset bit is pulsed.
        status = self.read_status()
        if status & STATUS_FAULT:
            print(f"[Drive {self.drive_id}] Fault detected. Sending fault reset.")
            self.write_controlword(CMD_FAULT_RESET, delay=0.5)

    def set_operation_mode(self, mode: int) -> int:
        self.sdo_write_int(MODE_OF_OPERATION, 0, mode, size=1, signed=True)
        time.sleep(self.sdo_delay_s)
        display = self.sdo_read_int(MODE_DISPLAY, 0, signed=True)
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

    def enable_operation(self) -> int:
        # Standard CiA 402 transition: Shutdown -> Switch On -> Enable Operation.
        self.reset_fault_if_needed()
        self.write_controlword(CMD_SHUTDOWN)
        self.write_controlword(CMD_SWITCH_ON)
        status = self.write_controlword(CMD_ENABLE_OPERATION)
        if not (status & STATUS_OPERATION_ENABLED):
            raise RuntimeError(
                f"Drive {self.drive_id} did not enter Operation Enabled. Statusword=0x{status:04X}"
            )
        self.set_operation_mode(MODE_PROFILE_POSITION)
        return status

    def disable_operation(self) -> int:
        status = self.write_controlword(CMD_DISABLE_OPERATION)
        self.write_controlword(CMD_DISABLE_VOLTAGE)
        return status

    def read_actual_position_counts(self) -> int:
        return self.sdo_read_int(ACTUAL_POSITION, 0, signed=True)

    def move_absolute_counts(self, target_counts: int) -> None:
        # Commands are absolute raw counts; ROS radians are converted in the C++ layer.
        self.set_operation_mode(MODE_PROFILE_POSITION)
        self.sdo_write_int(TARGET_POSITION, 0, int(target_counts), size=4, signed=True)
        time.sleep(self.sdo_delay_s)
        # The new set-point is triggered by a bit-4 rising edge in Profile Position mode.
        self.write_controlword(CMD_ENABLE_OPERATION)
        self.write_controlword(CMD_START_MOTION)

    def move_absolute_counts_and_wait(self, target_counts: int, timeout_s: float) -> int:
        # Used by explicit homing helpers, not by the regular ros2_control write loop.
        self.move_absolute_counts(target_counts)
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            status = self.read_status()
            if status & STATUS_FAULT:
                raise RuntimeError(f"Fault during motion. Statusword=0x{status:04X}")
            if status & STATUS_TARGET_REACHED:
                break
            time.sleep(0.05)
        return self.read_actual_position_counts()

    def set_current_position_as_home(self, timeout_s: float) -> int:
        # Homing method 37 tells the drive to treat the current position as home.
        # This does not search for a physical switch; it only resets the reference.
        self.set_operation_mode(MODE_HOMING)
        self.sdo_write_int(HOMING_METHOD, 0, 37, size=1, signed=True)
        self.sdo_write_int(HOMING_OFFSET, 0, 0, size=4, signed=True)
        self.write_controlword(CMD_ENABLE_OPERATION)
        self.write_controlword(CMD_START_HOMING)

        deadline = time.time() + timeout_s
        while time.time() < deadline:
            status = self.read_status()
            if status & STATUS_HOMING_ERROR:
                raise RuntimeError(f"Homing error. Statusword=0x{status:04X}")
            if (status & STATUS_TARGET_REACHED) and (status & STATUS_HOMING_ATTAINED):
                break
            time.sleep(0.05)

        self.write_controlword(CMD_ENABLE_OPERATION)
        self.set_operation_mode(MODE_PROFILE_POSITION)
        return self.read_actual_position_counts()

    def set_digout1(self, on: bool) -> None:
        value = DIGOUT1_ON if on else DIGOUT1_OFF
        if self.verbose:
            print(f"[Drive {self.drive_id}] DigOut1 {'ON' if on else 'OFF'}")
        self.sdo_write_int(DIGITAL_IO_STATUS, DIGOUT_WRITE, value, size=2, signed=False)

    def toggle_digout1(self) -> None:
        if self.verbose:
            print(f"[Drive {self.drive_id}] DigOut1 TOGGLE")
        self.sdo_write_int(DIGITAL_IO_STATUS, DIGOUT_WRITE, DIGOUT1_TOGGLE, size=2, signed=False)

    def read_digital_inputs(self) -> tuple[int, int, int]:

        logical = self.sdo_read_int(DIGITAL_IO_STATUS,DIGITAL_INPUT_LOGICAL,
        signed=False,)
        physical = self.sdo_read_int(
        DIGITAL_IO_STATUS,
        DIGITAL_INPUT_PHYSICAL,
        signed=False,
       )
        polarity = self.sdo_read_int(
        DIGITAL_INPUT_SETTINGS,
        INPUT_POLARITY,
        signed=False,
       )
        return logical, physical, polarity

class FaulhaberBus:
    """Owns the pysoem Master and exposes the selected drive objects."""

    # The bridge may see more EtherCAT slaves than the four joints. slave_indices
    # selects which slaves are presented to the hardware interface, in joint order.

    def __init__(self, interface: str, slave_indices: List[int], sdo_delay_s: float, verbose: bool) -> None:
        self.interface = interface
        self.slave_indices = slave_indices
        self.sdo_delay_s = sdo_delay_s
        self.verbose = verbose
        self.master: Optional[pysoem.Master] = None
        self.drives: List[FaulhaberDrive] = []

    def connect(self) -> None:
        # Create and configure the EtherCAT master before constructing drive wrappers.
        self.master = pysoem.Master()
        print(f"[EtherCAT] Opening interface: {self.interface}")
        self.master.open(self.interface)

        if self.master.config_init() <= 0:
            raise RuntimeError("No EtherCAT slaves found")

        print(f"[EtherCAT] Found {len(self.master.slaves)} slave(s)")
        self.master.config_map()
        print("[EtherCAT] PDO mapping configured")

        self.drives.clear()
        # drive_id is the logical joint index used by ROS; slave_index is EtherCAT order.
        for drive_id, slave_index in enumerate(self.slave_indices):
            if slave_index >= len(self.master.slaves):
                raise RuntimeError(
                    f"slave_indices[{drive_id}]={slave_index}, but only {len(self.master.slaves)} slave(s) found"
                )
            slave = self.master.slaves[slave_index]
            print(f"[EtherCAT] Drive {drive_id} uses slave {slave_index}: {slave.name}")
            self.drives.append(FaulhaberDrive(slave, drive_id, self.sdo_delay_s, self.verbose))

    def close(self) -> None:
        if self.master is not None:
            try:
                self.master.close()
            except Exception:
                pass
        self.drives.clear()


class RASCLFaulhaberBridge(Node):
    """ROS 2 node that serves hardware-interface TCP commands."""

    # The node exposes both ROS services for manual operation and a TCP protocol for
    # the C++ SystemInterface. All hardware access is protected by one re-entrant
    # lock because the Faulhaber drives are commanded through blocking SDO calls.

    def __init__(self) -> None:
        super().__init__("rascl_faulhaber_bridge")

        # ROS parameters mirror the launch-file arguments used in the lab setup.
        self.declare_parameter("interface", "robot_interface")
        self.declare_parameter("slave_indices", [0, 1, 2, 3])
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 15001)
        self.declare_parameter("sdo_delay_s", 0.05)
        self.declare_parameter("motion_timeout_s", 8.0)
        self.declare_parameter("verbose", True)
        self.declare_parameter("profile_velocity", 0)
        self.declare_parameter("profile_acceleration", 0)
        self.declare_parameter("profile_deceleration", 0)
        self.declare_parameter("homing_methods", [29, 29, 29, 29])
        self.declare_parameter("homing_offsets", [0, 0, 0, 0])
        self.declare_parameter("reference_inputs", [1, 1, 1, 1])
        self.declare_parameter("homing_search_speeds", [1000, 1000, 1000, 1000])
        self.declare_parameter("homing_zero_speeds", [200, 200, 200, 200])
        self.declare_parameter("homing_accelerations", [1000, 1000, 1000, 1000])

        self.interface = str(self.get_parameter("interface").value)
        self.slave_indices = [int(v) for v in self.get_parameter("slave_indices").value]
        self.host = str(self.get_parameter("host").value)
        self.port = int(self.get_parameter("port").value)
        self.sdo_delay_s = float(self.get_parameter("sdo_delay_s").value)
        self.motion_timeout_s = float(self.get_parameter("motion_timeout_s").value)
        self.verbose = bool(self.get_parameter("verbose").value)
        self.profile_velocity = int(self.get_parameter("profile_velocity").value)
        self.profile_acceleration = int(self.get_parameter("profile_acceleration").value)
        self.profile_deceleration = int(self.get_parameter("profile_deceleration").value)

        # The lock serializes service callbacks and TCP commands on the same bus.
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.bus = FaulhaberBus(self.interface, self.slave_indices, self.sdo_delay_s, self.verbose)

        self.get_logger().info(f"Connecting EtherCAT on interface: {self.interface}")
        self.bus.connect()
        for drive in self.bus.drives:
            # Zero values leave the existing drive profile-motion settings unchanged.
            drive.configure_profile_motion(
                self.profile_velocity,
                self.profile_acceleration,
                self.profile_deceleration,
            )

        # Services are useful for manual testing outside the ros2_control lifecycle.
        self.enable_all_srv = self.create_service(Trigger, "~/enable_all", self.on_enable_all)
        self.disable_all_srv = self.create_service(Trigger, "~/disable_all", self.on_disable_all)
        self.home_all_srv = self.create_service(Trigger, "~/home_all", self.on_home_all)
        self.read_digital_inputs_srv = self.create_service(Trigger, "~/read_digital_inputs", self.on_read_digital_inputs, )
        self.goto_home_all_srv = self.create_service(Trigger, "~/goto_home_all", self.on_goto_home_all)
        self.blink_digout1_srv = self.create_service(Trigger, "~/blink_digout1", self.on_blink_digout1)
        self.digout1_sub = self.create_subscription(Bool, "~/digout1", self.on_digout1, 10)
        self.home_done_pub = self.create_publisher(Bool, "~/home_done", 10)

        # The TCP server runs in the background while rclpy handles ROS services.
        self.server_thread = threading.Thread(target=self.tcp_server_loop, daemon=True)
        self.server_thread.start()
        self.get_logger().info(f"TCP bridge listening on {self.host}:{self.port}")

    def publish_home_done(self) -> None:
        # A simple event topic helps external scripts observe manual home operations.
        msg = Bool()
        msg.data = True
        self.home_done_pub.publish(msg)

    def on_enable_all(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        try:
            with self.lock:
                statuses = [drive.enable_operation() for drive in self.bus.drives]
            response.success = True
            response.message = "Enabled all drives: " + " ".join(f"0x{s:04X}" for s in statuses)
        except Exception as exc:
            response.success = False
            response.message = f"Enable failed: {exc}"
        return response

    def on_disable_all(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        try:
            with self.lock:
                statuses = [drive.disable_operation() for drive in self.bus.drives]
            response.success = True
            response.message = "Disabled all drives: " + " ".join(f"0x{s:04X}" for s in statuses)
        except Exception as exc:
            response.success = False
            response.message = f"Disable failed: {exc}"
        return response

    def on_home_all(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        # Set every drive's current raw count as its zero position.
        try:
            with self.lock:
                positions = []
                for index, drive in enumerate(self.bus.drives):
                    position = drive.home_to_reference_switch(
                    method=self.homing_methods[index],
                    reference_input=self.reference_inputs[index],
                    offset_counts=self.homing_offsets[index],
                    search_speed=self.homing_search_speeds[index],
                    zero_speed=self.homing_zero_speeds[index],
                    acceleration=self.homing_accelerations[index],
                    timeout_s=self.motion_timeout_s,
                    )
                    positions.append(position)
            self.publish_home_done()
            response.success = True
            response.message = "Home set for all drives: " + " ".join(str(p) for p in positions)
        except Exception as exc:
            response.success = False
            response.message = f"Home failed: {exc}"
        return response

    def on_goto_home_all(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        # Move all drives back to raw count zero after a previous home operation.
        try:
            with self.lock:
                positions = [drive.move_absolute_counts_and_wait(0, self.motion_timeout_s) for drive in self.bus.drives]
            self.publish_home_done()
            response.success = True
            response.message = "Moved all drives to zero: " + " ".join(str(p) for p in positions)
        except Exception as exc:
            response.success = False
            response.message = f"Goto home failed: {exc}"
        return response

    def on_digout1(self, msg: Bool) -> None:
        try:
            with self.lock:
                self.bus.drives[0].set_digout1(bool(msg.data))
        except Exception as exc:
            self.get_logger().error(f"DigOut1 command failed: {exc}")

    def on_blink_digout1(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        try:
            with self.lock:
                self.bus.drives[0].set_digout1(True)
                time.sleep(0.5)
                self.bus.drives[0].set_digout1(False)
            response.success = True
            response.message = "DigOut1 blink finished."
        except Exception as exc:
            response.success = False
            response.message = f"DigOut1 blink failed: {exc}"
        return response
    
    # add on 7/10     
    def on_read_digital_inputs(self, request: Trigger.Request, response: Trigger.Response, ) -> Trigger.Response:
        try:
            results = []

            with self.lock:
                for drive in self.bus.drives:
                    logical, physical, polarity = drive.read_digital_inputs()
                    number, threshold, reference = (
                        drive.read_digital_input_configuration()
                    )
                    results.append(
                        f"Drive {drive.drive_id}: "
                        f"physical=0x{physical:02X}/{physical:08b}, "
                        f"logical=0x{logical:02X}/{logical:08b}, "
                        f"polarity=0x{polarity:02X}, "
                        f"inputs={number}, "
                        f"threshold={threshold}, "
                        f"reference_input={reference}"
                        )

            response.success = True
            response.message = " | ".join(results)

        except Exception as exc:
           response.success = False
           response.message = f"Read digital inputs failed: {exc}"

        return response
    
    def tcp_server_loop(self) -> None:
        # The line-based protocol is intentionally small and deterministic:
        # one command line in, one response line out.
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind((self.host, self.port))
            srv.listen(1)
            srv.settimeout(0.5)

            while not self.stop_event.is_set():
                try:
                    conn, addr = srv.accept()
                except socket.timeout:
                    continue
                except OSError:
                    break

                self.get_logger().info(f"Hardware client connected from {addr}")
                with conn:
                    conn_file = conn.makefile("rwb")
                    while not self.stop_event.is_set():
                        line = conn_file.readline()
                        if not line:
                            break
                        command = line.decode("utf-8", errors="replace").strip()
                        response = self.handle_tcp_command(command)
                        conn_file.write((response + "\n").encode("utf-8"))
                        conn_file.flush()

                self.get_logger().info("Hardware client disconnected")

    def handle_tcp_command(self, command: str) -> str:
        try:
            # Commands are ASCII tokens so they can be tested with simple socket scripts.
            parts = command.split()
            if not parts:
                return "ERR empty command"

            op = parts[0].upper()
            with self.lock:
                if op == "PING":
                    return "OK"

                if op == "ENABLE_ALL":
                    statuses = [drive.enable_operation() for drive in self.bus.drives]
                    return "OK " + " ".join(f"0x{s:04X}" for s in statuses)

                if op == "DISABLE_ALL":
                    statuses = [drive.disable_operation() for drive in self.bus.drives]
                    return "OK " + " ".join(f"0x{s:04X}" for s in statuses)

                if op == "GET_ALL":
                    # Return count/status pairs for each drive in logical joint order.
                    response = ["OK"]
                    for drive in self.bus.drives:
                        response.append(str(drive.read_actual_position_counts()))
                        response.append(f"0x{drive.read_status():04X}")
                    return " ".join(response)

                if op == "MOVE_ALL":
                    # MOVE_ALL is used by the hardware interface write() method.
                    if len(parts) != len(self.bus.drives) + 1:
                        return f"ERR usage MOVE_ALL <{len(self.bus.drives)} counts>"
                    targets = [int(value) for value in parts[1:]]
                    for drive, target in zip(self.bus.drives, targets):
                        drive.move_absolute_counts(target)
                    return "OK"

                if op == "MOVE_ABS":
                    if len(parts) != 3:
                        return "ERR usage MOVE_ABS <drive_index> <counts>"
                    drive_index = int(parts[1])
                    target = int(parts[2])
                    self.bus.drives[drive_index].move_absolute_counts(target)
                    return "OK"

                if op == "GOTO_HOME":
                    if len(parts) == 1:
                        positions = [drive.move_absolute_counts_and_wait(0, self.motion_timeout_s) for drive in self.bus.drives]
                        self.publish_home_done()
                        return "OK " + " ".join(str(pos) for pos in positions)
                    if len(parts) == 2:
                        drive_index = int(parts[1])
                        pos = self.bus.drives[drive_index].move_absolute_counts_and_wait(0, self.motion_timeout_s)
                        self.publish_home_done()
                        return f"OK {pos}"
                    return "ERR usage GOTO_HOME [drive_index]"

                if op == "DIGOUT1":
                    if len(parts) not in (2, 3):
                        return "ERR usage DIGOUT1 [drive_index] <ON|OFF|TOGGLE>"
                    if len(parts) == 2:
                        drive_index = 0
                        state = parts[1].upper()
                    else:
                        drive_index = int(parts[1])
                        state = parts[2].upper()
                    if state == "ON":
                        self.bus.drives[drive_index].set_digout1(True)
                    elif state == "OFF":
                        self.bus.drives[drive_index].set_digout1(False)
                    elif state == "TOGGLE":
                        self.bus.drives[drive_index].toggle_digout1()
                    else:
                        return "ERR DIGOUT1 state must be ON, OFF or TOGGLE"
                    return "OK"

                return f"ERR unknown command {op}"

        except Exception as exc:
            return f"ERR {type(exc).__name__}: {exc}"

    def destroy_node(self) -> bool:
        # Stop the TCP loop and close the EtherCAT master before the ROS node exits.
        self.stop_event.set()
        try:
            self.bus.close()
        except Exception:
            pass
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
