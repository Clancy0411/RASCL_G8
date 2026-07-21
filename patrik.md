rascl-container:~/ws$ bash ./rascl_debug.sh 4
Homing bridge 将在 T1 持续运行，直到整个 CSP 会话结束。
Drive 0-2 自动 Homing；预装的 Drive 3 不 Homing，但会参与后续 CSP。
Drive 2 CSP following-error：窗口 25000 counts，超时 250 ms；内部限位只读取、不改写。
Drive 0-3 进入 CSP 前会把可写的 0x60E0/0x60E1 设为 1000（1000=额定转矩）并回读；只读 0x6072 仅记录，不写入永久存储。
[INFO] [launch]: All log files can be found below /root/.ros/log/2026-07-21-22-11-07-023694-irs-rascl06-217
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [rascl_faulhaber_bridge.py-1]: process started with pid [220]
[rascl_faulhaber_bridge.py-1] [INFO] [1784671867.321110586] [rascl_faulhaber_bridge]: Connecting EtherCAT on enx3c18a0256deb; control_mode=homing_csp
[rascl_faulhaber_bridge.py-1] [INFO] [1784671867.321421741] [rascl_faulhaber_bridge]: Drive 3 spur_gear_joint skips Homing but will be enabled and validated in CSP
[rascl_faulhaber_bridge.py-1] [WARN] [1784671867.373056899] [rascl_faulhaber_bridge]: Drive 2 CSP following-error monitor changed for this session only: 0x6065 16384 -> 25000 counts; 0x6066 48 -> 250 ms. 0x607B/0x607D were read only, not modified. Drive 2 protection: 0x607B position_range=[-2147483648, 2147483647], 0x607D software_limit=[-802816, 802816], 0x6065 following_window=25000 counts, 0x6066 following_timeout=250 ms
[rascl_faulhaber_bridge.py-1] [INFO] [1784671867.375774799] [rascl_faulhaber_bridge]: TCP bridge listening on 127.0.0.1:15001
[rascl_faulhaber_bridge.py-1] [INFO] [1784671890.224800322] [rascl_faulhaber_bridge]: Hardware client connected from ('127.0.0.1', 37944)
