"""Software-only checks for the FAULHABER CSP/PDO bridge."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import time
import types
import unittest


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

    def sdo_read(self, index, subindex):
        return self.values[(index, subindex)]

    def sdo_write(self, index, subindex, payload):
        self.writes.append((index, subindex, bytes(payload)))
        self.values[(index, subindex)] = bytes(payload)
        if (index, subindex) == (bridge.MODE_OF_OPERATION, 0):
            self.values[(bridge.MODE_DISPLAY, 0)] = bytes(payload)


class FakeMaster:
    def __init__(self, slaves):
        self.slaves = slaves
        self.state = bridge.pysoem.SAFEOP_STATE
        self.expected_wkc = 3 * len(slaves)
        self.config_map_calls = 0

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
        return 1

    def send_processdata(self):
        return 1

    def receive_processdata(self, _timeout):
        for slave in self.slaves:
            controlword, target = bridge.struct.unpack("<Hi", slave.output)
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
        time.sleep(0.01)
        self.assertEqual([state[0] for state in bus.get_csp_states()], [11, 22, 33, 44])
        self.assertEqual(bus.last_working_counter, bus.master.expected_wkc)

        statuses = bus.exit_csp()
        self.assertEqual(len(statuses), 4)
        self.assertFalse(bus.csp_active)
        self.assertEqual(bus.target_counts, [])
        self.assertEqual(bus.latest_states, [])


if __name__ == "__main__":
    unittest.main()
