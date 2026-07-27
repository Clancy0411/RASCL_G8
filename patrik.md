
rascl-container:~/ws$ cd /root/ws
bash ./rascl_debug.sh 6
waiting for service to become available...
requester: making request: std_srvs.srv.Trigger_Request()

response:
std_srvs.srv.Trigger_Response(success=True, message='Drive 0: physical=0x07/00000111, logical=0x00/00000000, polarity=0x07; 0x2310 lower/upper/option/reference=0x01/0x04/1/2; 0x2324.01=0x0000000B [none] | Drive 1: physical=0x03/00000011, logical=0x04/00000100, polarity=0x07; 0x2310 lower/upper/option/reference=0x01/0x04/1/2; 0x2324.01=0x0400004B [positive_limit_switch] | Drive 2: physical=0x07/00000111, logical=0x00/00000000, polarity=0x07; 0x2310 lower/upper/option/reference=0x01/0x04/1/2; 0x2324.01=0x0000000B [none] | Drive 3: physical=0x01/00000001, logical=0x00/00000000, polarity=0x01; 0x2310 lower/upper/option/reference=0x00/0x00/1/1; 0x2324.01=0x00000003 [none]')

home_all 先让 Drive 0-2 穿过各自参考输入区间并回到中点置零；成功后 Drive 3 自动相对运动 50000 counts，再把到达位置设为 0 counts。
waiting for service to become available...
requester: making request: std_srvs.srv.Trigger_Request()

response:
std_srvs.srv.Trigger_Response(success=False, message='Home failed: Drive 1: Homing Method 37 completed but actual position is 26, expected 0 (tolerance 10)')
ERROR: home_all 或 Drive 3 参考运动/置零失败；禁止进入 CSP




rascl-container:~/ws$ bash ./rascl_debug.sh 4
Homing bridge 将在 T1 持续运行，直到整个 CSP 会话结束。
Drive 0-2 自动寻找参考输入区间两端，以 200 的低速正弦曲线回到 (entry+exit)/2 并置零；D0/D1/D2 第二边沿最大搜索距离分别为 100000/300000/300000 counts，穿越/回中点超时 120.0 s。
三轴到位后 Drive 3 相对运动 50000 counts，并以 Method 37 把到达位置设为 0 counts。
Drive 3 参考运动：速度 3000 counts/s，加/减速度 1000/1000，following-error 持续 0.30 s 才中断。
Drive 2 CSP following-error：窗口 25000 counts，超时 250 ms；0x607B/0x607D 软件位置限位只读取、不改写。
CSP 交接会清零并回读验证 Drive 0-3 的 0x2310:01/:02 正/负限位输入映射；Homing 参考输入、极性与软件位置限位保持不变。
CSP 停滞诊断：误差 >= 25000 counts 且 500 ms 内进展 < 100 counts 时自动抓取驱动快照。
Drive 0-3 进入 CSP 前会把可写的 0x60E0/0x60E1 设为 1000（1000=额定转矩）并回读；只读 0x6072 仅记录，不写入永久存储。
Drive 2/3 在 CSP 交接时会把过低的 0x2329:03 峰值电流提高到满足目标转矩所需值（实机曾分别为 220→1100 mA、81→540 mA），并要求只读 0x6072 回读不低于 1000；Drive 0/1 电流参数不改。
[INFO] [launch]: All log files can be found below /root/.ros/log/2026-07-27-16-52-37-272091-irs-rascl06-222
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [rascl_faulhaber_bridge.py-1]: process started with pid [225]
[rascl_faulhaber_bridge.py-1] [INFO] [1785171157.583541364] [rascl_faulhaber_bridge]: Connecting EtherCAT on enx3c18a0256deb; control_mode=homing_csp
[rascl_faulhaber_bridge.py-1] [INFO] [1785171157.583980319] [rascl_faulhaber_bridge]: CSP stall diagnostics: error>=25000 counts, progress<100 counts for 500 ms
[rascl_faulhaber_bridge.py-1] [INFO] [1785171157.584375822] [rascl_faulhaber_bridge]: CSP handoff will clear and verify volatile lower/upper limit-input mappings 0x2310:01/:02; Homing reference, input polarity and 0x607B/0x607D remain unchanged
[rascl_faulhaber_bridge.py-1] [INFO] [1785171157.584785543] [rascl_faulhaber_bridge]: Drive 0-2 Homing uses the centre of the reference-input interval: find the first edge with the configured native method, traverse the active interval and return to (entry+exit)/2 at the lower Homing zero speed with a sinusoidal profile, then set that midpoint to zero with Method 37; second-edge travel guards D0/D1/D2=[100000, 300000, 300000] counts
[rascl_faulhaber_bridge.py-1] [INFO] [1785171157.585189767] [rascl_faulhaber_bridge]: Drive 3 skips sensor Homing; after Drives 0-2 Home it will move +50000 counts and use Homing Method 37 to set that position to 0
[rascl_faulhaber_bridge.py-1] [EtherCAT] Opening interface: enx3c18a0256deb
[rascl_faulhaber_bridge.py-1] [EtherCAT] Found 4 slave(s)
[rascl_faulhaber_bridge.py-1] [EtherCAT] Homing-to-CSP session starts SDO-only in PRE-OP; PDO mapping is deferred until home_all succeeds
[rascl_faulhaber_bridge.py-1] [EtherCAT] Drive 0 uses slave 0: MC5004
[rascl_faulhaber_bridge.py-1] [EtherCAT] Drive 1 uses slave 1: MC5004
[rascl_faulhaber_bridge.py-1] [EtherCAT] Drive 2 uses slave 2: MC5004
[rascl_faulhaber_bridge.py-1] [EtherCAT] Drive 3 uses slave 3: MC5004
[rascl_faulhaber_bridge.py-1] [WARN] [1785171157.635133719] [rascl_faulhaber_bridge]: Drive 2 CSP following-error monitor changed for this session only: 0x6065 16384 -> 25000 counts; 0x6066 48 -> 250 ms. 0x607B/0x607D were read only, not modified. Drive 2 protection: 0x607B position_range=[-2147483648, 2147483647], 0x607D software_limit=[-802816, 802816], 0x6065 following_window=25000 counts, 0x6066 following_timeout=250 ms
[rascl_faulhaber_bridge.py-1] [INFO] [1785171157.638125684] [rascl_faulhaber_bridge]: TCP bridge listening on 127.0.0.1:15001
[rascl_faulhaber_bridge.py-1] [Drive 0] Controlword <- 0x0006
[rascl_faulhaber_bridge.py-1] [Drive 0] Controlword <- 0x0007
[rascl_faulhaber_bridge.py-1] [Drive 0] mode requested=6, display=6
[rascl_faulhaber_bridge.py-1] [Drive 0] Controlword <- 0x000F
[rascl_faulhaber_bridge.py-1] [Drive 0] Controlword <- 0x000F
[rascl_faulhaber_bridge.py-1] [Drive 0] Controlword <- 0x001F
[rascl_faulhaber_bridge.py-1] [Drive 0] Controlword <- 0x000F
[rascl_faulhaber_bridge.py-1] [Drive 0] mode requested=1, display=1
[rascl_faulhaber_bridge.py-1] [Drive 0] first Homing edge=0 counts; post-edge stop=-817 counts
[rascl_faulhaber_bridge.py-1] [Drive 0] mode requested=1, display=1
[rascl_faulhaber_bridge.py-1] [Drive 0] Controlword <- 0x000F
[rascl_faulhaber_bridge.py-1] [Drive 0] Controlword <- 0x003F
[rascl_faulhaber_bridge.py-1] [Drive 0] Controlword <- 0x010F
[rascl_faulhaber_bridge.py-1] [Drive 0] Controlword <- 0x000F
[rascl_faulhaber_bridge.py-1] [Drive 0] mode requested=1, display=1
[rascl_faulhaber_bridge.py-1] [Drive 0] Controlword <- 0x000F
[rascl_faulhaber_bridge.py-1] [Drive 0] Controlword <- 0x003F
[rascl_faulhaber_bridge.py-1] [Drive 0] Controlword <- 0x0006
[rascl_faulhaber_bridge.py-1] [Drive 0] Controlword <- 0x0007
[rascl_faulhaber_bridge.py-1] [Drive 0] mode requested=6, display=6
[rascl_faulhaber_bridge.py-1] [Drive 0] Controlword <- 0x000F
[rascl_faulhaber_bridge.py-1] [Drive 0] Controlword <- 0x000F
[rascl_faulhaber_bridge.py-1] [Drive 0] Controlword <- 0x001F
[rascl_faulhaber_bridge.py-1] [Drive 0] Controlword <- 0x000F
[rascl_faulhaber_bridge.py-1] [Drive 0] mode requested=1, display=1
[rascl_faulhaber_bridge.py-1] [WARN] [1785171171.029770611] [rascl_faulhaber_bridge]: HOMING_INTERVAL drive=0 entry=0 exit=-57190 width=57190 midpoint=-28595 reached=-28592 zero=0
[rascl_faulhaber_bridge.py-1] [Drive 1] Controlword <- 0x0006
[rascl_faulhaber_bridge.py-1] [Drive 1] Controlword <- 0x0007
[rascl_faulhaber_bridge.py-1] [Drive 1] mode requested=6, display=6
[rascl_faulhaber_bridge.py-1] [Drive 1] Controlword <- 0x000F
[rascl_faulhaber_bridge.py-1] [Drive 1] Controlword <- 0x000F
[rascl_faulhaber_bridge.py-1] [Drive 1] Controlword <- 0x001F
[rascl_faulhaber_bridge.py-1] [Drive 1] Controlword <- 0x000F
[rascl_faulhaber_bridge.py-1] [Drive 1] mode requested=1, display=1
[rascl_faulhaber_bridge.py-1] [Drive 1] first Homing edge=0 counts; post-edge stop=-769 counts
[rascl_faulhaber_bridge.py-1] [Drive 1] mode requested=1, display=1
[rascl_faulhaber_bridge.py-1] [Drive 1] Controlword <- 0x000F
[rascl_faulhaber_bridge.py-1] [Drive 1] Controlword <- 0x003F
[rascl_faulhaber_bridge.py-1] [Drive 1] Controlword <- 0x010F
[rascl_faulhaber_bridge.py-1] [Drive 1] Controlword <- 0x000F
[rascl_faulhaber_bridge.py-1] [Drive 1] mode requested=1, display=1
[rascl_faulhaber_bridge.py-1] [Drive 1] Controlword <- 0x000F
[rascl_faulhaber_bridge.py-1] [Drive 1] Controlword <- 0x003F
[rascl_faulhaber_bridge.py-1] [Drive 1] Controlword <- 0x0006
[rascl_faulhaber_bridge.py-1] [Drive 1] Controlword <- 0x0007
[rascl_faulhaber_bridge.py-1] [Drive 1] mode requested=6, display=6
[rascl_faulhaber_bridge.py-1] [Drive 1] Controlword <- 0x000F
[rascl_faulhaber_bridge.py-1] [Drive 1] Controlword <- 0x000F
[rascl_faulhaber_bridge.py-1] [Drive 1] Controlword <- 0x001F
