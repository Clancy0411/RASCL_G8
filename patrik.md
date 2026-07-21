rascl-container:~/ws$ bash ./rascl_debug.sh 6
waiting for service to become available...
requester: making request: std_srvs.srv.Trigger_Request()

response:
std_srvs.srv.Trigger_Response(success=True, message='Drive 0: physical=0x07/00000111, logical=0x00/00000000, polarity=0x07 | Drive 1: physical=0x03/00000011, logical=0x04/00000100, polarity=0x07 | Drive 2: physical=0x07/00000111, logical=0x00/00000000, polarity=0x07 | Drive 3: physical=0x00/00000000, logical=0x01/00000001, polarity=0x01')

home_all 只会运动 Drive 0-2；预装的 Drive 3 不执行 Home，随后仍会进入 CSP。
waiting for service to become available...
requester: making request: std_srvs.srv.Trigger_Request()

response:
std_srvs.srv.Trigger_Response(success=True, message='Homing completed for required drives; CSP handoff armed: drive0=-875 drive1=-769 drive2=935')

waiting for service to become available...
requester: making request: std_srvs.srv.Trigger_Request()

response:
std_srvs.srv.Trigger_Response(success=True, message='Drive 2 protection: 0x607B position_range=[-2147483648, 2147483647], 0x607D software_limit=[-802816, 802816], 0x6065 following_window=25000 counts, 0x6066 following_timeout=250 ms')


rascl-container:~/ws$ bash ./rascl_debug.sh 4
Homing bridge 将在 T1 持续运行，直到整个 CSP 会话结束。
Drive 0-2 自动 Homing；预装的 Drive 3 不 Homing，但会参与后续 CSP。
Drive 2 CSP following-error：窗口 25000 counts，超时 250 ms；内部限位只读取、不改写。
[INFO] [launch]: All log files can be found below /root/.ros/log/2026-07-21-21-34-02-497689-irs-rascl06-267
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [rascl_faulhaber_bridge.py-1]: process started with pid [270]
[rascl_faulhaber_bridge.py-1] [INFO] [1784669642.631714812] [rascl_faulhaber_bridge]: Connecting EtherCAT on enx3c18a0256deb; control_mode=homing_csp
[rascl_faulhaber_bridge.py-1] [INFO] [1784669642.632035274] [rascl_faulhaber_bridge]: Drive 3 spur_gear_joint skips Homing but will be enabled and validated in CSP
[rascl_faulhaber_bridge.py-1] [EtherCAT] Opening interface: enx3c18a0256deb
[rascl_faulhaber_bridge.py-1] [EtherCAT] Found 4 slave(s)
[rascl_faulhaber_bridge.py-1] [EtherCAT] Homing-to-CSP session starts SDO-only in PRE-OP; PDO mapping is deferred until home_all succeeds
[rascl_faulhaber_bridge.py-1] [EtherCAT] Drive 0 uses slave 0: MC5004
[rascl_faulhaber_bridge.py-1] [EtherCAT] Drive 1 uses slave 1: MC5004
[rascl_faulhaber_bridge.py-1] [EtherCAT] Drive 2 uses slave 2: MC5004
[rascl_faulhaber_bridge.py-1] [EtherCAT] Drive 3 uses slave 3: MC5004
[rascl_faulhaber_bridge.py-1] Traceback (most recent call last):
[rascl_faulhaber_bridge.py-1]   File "/root/ws/install/rascl_hardware_interface/lib/rascl_hardware_interface/rascl_faulhaber_bridge.py", line 1794, in <module>
[rascl_faulhaber_bridge.py-1]     main()
[rascl_faulhaber_bridge.py-1]   File "/root/ws/install/rascl_hardware_interface/lib/rascl_hardware_interface/rascl_faulhaber_bridge.py", line 1785, in main
[rascl_faulhaber_bridge.py-1]     node = RASCLFaulhaberBridge()
[rascl_faulhaber_bridge.py-1]            ^^^^^^^^^^^^^^^^^^^^^^
[rascl_faulhaber_bridge.py-1]   File "/root/ws/install/rascl_hardware_interface/lib/rascl_hardware_interface/rascl_faulhaber_bridge.py", line 1320, in __init__
[rascl_faulhaber_bridge.py-1]     self._configure_drive2_csp_protection()
[rascl_faulhaber_bridge.py-1]   File "/root/ws/install/rascl_hardware_interface/lib/rascl_hardware_interface/rascl_faulhaber_bridge.py", line 1396, in _configure_drive2_csp_protection
[rascl_faulhaber_bridge.py-1]     before = drive.read_position_protection()
[rascl_faulhaber_bridge.py-1]              ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
[rascl_faulhaber_bridge.py-1]   File "/root/ws/install/rascl_hardware_interface/lib/rascl_hardware_interface/rascl_faulhaber_bridge.py", line 345, in read_position_protection
[rascl_faulhaber_bridge.py-1]     "position_range_min": self.sdo_read_int(POSITION_RANGE_LIMIT, 1, signed=True),
[rascl_faulhaber_bridge.py-1]                           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
[rascl_faulhaber_bridge.py-1]   File "/root/ws/install/rascl_hardware_interface/lib/rascl_hardware_interface/rascl_faulhaber_bridge.py", line 166, in sdo_read_int
[rascl_faulhaber_bridge.py-1]     data = self.slave.sdo_read(index, subindex)
[rascl_faulhaber_bridge.py-1]            ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
[rascl_faulhaber_bridge.py-1]   File "src/pysoem/pysoem.pyx", line 909, in pysoem.pysoem.CdefSlave.sdo_read
[rascl_faulhaber_bridge.py-1] pysoem.pysoem.WkcError
[ERROR] [rascl_faulhaber_bridge.py-1]: process has died [pid 270, exit code 1, cmd '/root/ws/install/rascl_hardware_interface/lib/rascl_hardware_interface/rascl_faulhaber_bridge.py --ros-args -r __node:=rascl_faulhaber_bridge --params-file /tmp/launch_params_18gi27y7'].
