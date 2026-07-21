rascl-container:~/ws$ bash ./rascl_debug.sh 4
Homing bridge 将在 T1 持续运行，直到整个 CSP 会话结束。
Drive 0-2 自动 Homing；预装的 Drive 3 不 Homing，但会参与后续 CSP。
Drive 2 CSP following-error：窗口 25000 counts，超时 250 ms；内部限位只读取、不改写。
Drive 0-3 进入 CSP 前会把可写的 0x60E0/0x60E1 设为 1000（1000=额定转矩）并回读；只读 0x6072 仅记录，不写入永久存储。
[INFO] [launch]: All log files can be found below /root/.ros/log/2026-07-21-22-15-46-343048-irs-rascl06-217
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [rascl_faulhaber_bridge.py-1]: process started with pid [220]
[rascl_faulhaber_bridge.py-1] [INFO] [1784672146.638676067] [rascl_faulhaber_bridge]: Connecting EtherCAT on enx3c18a0256deb; control_mode=homing_csp
[rascl_faulhaber_bridge.py-1] [INFO] [1784672146.638994109] [rascl_faulhaber_bridge]: Drive 3 spur_gear_joint skips Homing but will be enabled and validated in CSP
[rascl_faulhaber_bridge.py-1] [EtherCAT] Opening interface: enx3c18a0256deb
[rascl_faulhaber_bridge.py-1] [EtherCAT] Found 4 slave(s)
[rascl_faulhaber_bridge.py-1] [EtherCAT] Homing-to-CSP session starts SDO-only in PRE-OP; PDO mapping is deferred until home_all succeeds
[rascl_faulhaber_bridge.py-1] [EtherCAT] Drive 0 uses slave 0: MC5004
[rascl_faulhaber_bridge.py-1] [EtherCAT] Drive 1 uses slave 1: MC5004
[rascl_faulhaber_bridge.py-1] [EtherCAT] Drive 2 uses slave 2: MC5004
[rascl_faulhaber_bridge.py-1] [EtherCAT] Drive 3 uses slave 3: MC5004
[rascl_faulhaber_bridge.py-1] [WARN] [1784672146.690293716] [rascl_faulhaber_bridge]: Drive 2 CSP following-error monitor changed for this session only: 0x6065 25000 -> 25000 counts; 0x6066 250 -> 250 ms. 0x607B/0x607D were read only, not modified. Drive 2 protection: 0x607B position_range=[-2147483648, 2147483647], 0x607D software_limit=[-802816, 802816], 0x6065 following_window=25000 counts, 0x6066 following_timeout=250 ms
[rascl_faulhaber_bridge.py-1] [INFO] [1784672146.692841752] [rascl_faulhaber_bridge]: TCP bridge listening on 127.0.0.1:15001
[rascl_faulhaber_bridge.py-1] [Drive 0] Controlword <- 0x0006
[rascl_faulhaber_bridge.py-1] [Drive 0] Controlword <- 0x0007
[rascl_faulhaber_bridge.py-1] [Drive 0] mode requested=6, display=6
[rascl_faulhaber_bridge.py-1] [Drive 0] Controlword <- 0x000F
[rascl_faulhaber_bridge.py-1] [Drive 0] Controlword <- 0x000F
[rascl_faulhaber_bridge.py-1] [Drive 0] Controlword <- 0x001F
[rascl_faulhaber_bridge.py-1] [Drive 0] Controlword <- 0x000F
[rascl_faulhaber_bridge.py-1] [Drive 0] mode requested=1, display=1
[rascl_faulhaber_bridge.py-1] [Drive 1] Controlword <- 0x0006
[rascl_faulhaber_bridge.py-1] [Drive 1] Controlword <- 0x0007
[rascl_faulhaber_bridge.py-1] [Drive 1] mode requested=6, display=6
[rascl_faulhaber_bridge.py-1] [Drive 1] Controlword <- 0x000F
[rascl_faulhaber_bridge.py-1] [Drive 1] Controlword <- 0x000F
[rascl_faulhaber_bridge.py-1] [Drive 1] Controlword <- 0x001F
[rascl_faulhaber_bridge.py-1] [Drive 1] Controlword <- 0x000F
[rascl_faulhaber_bridge.py-1] [Drive 1] mode requested=1, display=1
[rascl_faulhaber_bridge.py-1] [Drive 2] Controlword <- 0x0006
[rascl_faulhaber_bridge.py-1] [Drive 2] Controlword <- 0x0007
[rascl_faulhaber_bridge.py-1] [Drive 2] mode requested=6, display=6
[rascl_faulhaber_bridge.py-1] [Drive 2] Controlword <- 0x000F
[rascl_faulhaber_bridge.py-1] [Drive 2] Controlword <- 0x000F
[rascl_faulhaber_bridge.py-1] [Drive 2] Controlword <- 0x001F
[rascl_faulhaber_bridge.py-1] [Drive 2] Controlword <- 0x000F
[rascl_faulhaber_bridge.py-1] [Drive 2] mode requested=1, display=1
[rascl_faulhaber_bridge.py-1] [INFO] [1784672201.479714942] [rascl_faulhaber_bridge]: Hardware client connected from ('127.0.0.1', 39184)
[rascl_faulhaber_bridge.py-1] [EtherCAT] CSP directional torque limits verified for this session only (0x6072 read-only; 0x60E0/0x60E1 writable; 1000=rated torque): D0 max/pos/neg 700/700/700 -> 700/1000/1000; motor_mA=1100/1100/770; D1 max/pos/neg 700/700/700 -> 700/1000/1000; motor_mA=1100/1100/770; D2 max/pos/neg 200/200/200 -> 200/1000/1000; motor_mA=1100/1100/220; D3 max/pos/neg 150/150/150 -> 150/1000/1000; motor_mA=540/540/81
[rascl_faulhaber_bridge.py-1] [EtherCAT] WARNING: writable directional limits were raised, but the read-only effective maximum remains below the requested limit: D0 0x6072=700; D1 0x6072=700; D2 0x6072=200; D3 0x6072=150. Inspect motor_mA (0x2329 rated/continuous/peak) before changing motor parameters.
[rascl_faulhaber_bridge.py-1] [Drive 3] mode requested=1, display=1
[rascl_faulhaber_bridge.py-1] [Drive 3] Controlword <- 0x0006
[rascl_faulhaber_bridge.py-1] [Drive 3] Controlword <- 0x0007
[rascl_faulhaber_bridge.py-1] [Drive 3] Controlword <- 0x000F
[rascl_faulhaber_bridge.py-1] [EtherCAT] Slave 0: CSP interpolation 0x2332.00 configured to 200 x 100 us (20000000 ns PDO cycle)
[rascl_faulhaber_bridge.py-1] [EtherCAT] Slave 0: assigning factory Position PDOs Rx=0x1601, Tx=0x1A01
[rascl_faulhaber_bridge.py-1] [EtherCAT] Slave 0: SM2 cycle monitoring configured for 20000000 ns
[rascl_faulhaber_bridge.py-1] [EtherCAT] Slave 1: CSP interpolation 0x2332.00 configured to 200 x 100 us (20000000 ns PDO cycle)
[rascl_faulhaber_bridge.py-1] [EtherCAT] Slave 1: assigning factory Position PDOs Rx=0x1601, Tx=0x1A01
[rascl_faulhaber_bridge.py-1] [EtherCAT] Slave 1: SM2 cycle monitoring configured for 20000000 ns
[rascl_faulhaber_bridge.py-1] [EtherCAT] Slave 2: CSP interpolation 0x2332.00 configured to 200 x 100 us (20000000 ns PDO cycle)
[rascl_faulhaber_bridge.py-1] [EtherCAT] Slave 2: assigning factory Position PDOs Rx=0x1601, Tx=0x1A01
[rascl_faulhaber_bridge.py-1] [EtherCAT] Slave 2: SM2 cycle monitoring configured for 20000000 ns
[rascl_faulhaber_bridge.py-1] [EtherCAT] Slave 3: CSP interpolation 0x2332.00 configured to 200 x 100 us (20000000 ns PDO cycle)
[rascl_faulhaber_bridge.py-1] [EtherCAT] Slave 3: assigning factory Position PDOs Rx=0x1601, Tx=0x1A01
[rascl_faulhaber_bridge.py-1] [EtherCAT] Slave 3: SM2 cycle monitoring configured for 20000000 ns
[rascl_faulhaber_bridge.py-1] [EtherCAT] Deferred process image mapped (48 bytes)
[rascl_faulhaber_bridge.py-1] [EtherCAT] SM-Sync selected with cycle 20000000 ns
[rascl_faulhaber_bridge.py-1] [Drive 0] mode requested=8, display=8
[rascl_faulhaber_bridge.py-1] [Drive 1] mode requested=8, display=8
[rascl_faulhaber_bridge.py-1] [Drive 2] mode requested=8, display=8
[rascl_faulhaber_bridge.py-1] [Drive 3] mode requested=8, display=8
[rascl_faulhaber_bridge.py-1] [EtherCAT] Master reached OP state
[rascl_faulhaber_bridge.py-1] [EtherCAT] Homing-to-CSP handoff completed without Shutdown/Disable controlwords
