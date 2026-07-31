T1
rascl-container:~/ws$ bash ./rascl_debug.sh 4
The Homing bridge remains active in T1 until the entire CSP session ends.
Drives 0-2 automatically find both edges of their reference-input intervals, return to (entry+exit)/2 with a low-speed sinusoidal profile at 200, and set zero. The D0/D1/D2 second-edge travel limits are 100000/300000/300000 counts, and the traverse/return timeout is 120.0 s.
Homing midpoint arrival and Method 37 zero readback share a 500-count tolerance; the overly strict 10-count check that could reject valid motion is no longer used.
After all three axes arrive, Drive 3 moves by 50000 counts and uses Method 37 to set the reached position to 0 counts.
Drive 3 reference motion: velocity 3000 counts/s, acceleration/deceleration 1000/1000, and abort only after following error persists for 0.30 s.
Drive 2 CSP following error: window 25000 counts, timeout 250 ms; 0x607B/0x607D software position limits are read only and are not modified.
The CSP handoff clears and verifies the Drive 0-3 positive/negative limit-input mappings at 0x2310:01/:02; the Homing reference input, polarity, and software position limits remain unchanged.
CSP stall diagnostics automatically capture a drive snapshot when error >= 25000 counts and progress < 100 counts over 500 ms.
Before Drives 0-3 enter CSP, writable 0x60E0/0x60E1 are set to 1000 (1000=rated torque) and read back; read-only 0x6072 is logged only and is not written to persistent storage.
Group 15 close uses 300 per mille Drive 3 torque to overcome slide friction, then drops immediately to 100 per mille holding torque after contact detection; open/custom-count moves restore 1000 per mille.
At CSP handoff, Drives 2/3 raise an insufficient 0x2329:03 peak-current setting to the value required by the target torque (observed hardware values: 220->1100 mA and 81->540 mA); read-only 0x6072 must read back at least 1000. Drive 0/1 current parameters are unchanged.
[INFO] [launch]: All log files can be found below /root/.ros/log/2026-07-27-19-55-47-730900-irs-rascl06-2827
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [rascl_faulhaber_bridge.py-1]: process started with pid [2830]
[rascl_faulhaber_bridge.py-1] [INFO] [1785182147.894249034] [rascl_faulhaber_bridge]: Connecting EtherCAT on enx3c18a0256deb; control_mode=homing_csp
[rascl_faulhaber_bridge.py-1] [INFO] [1785182147.894728078] [rascl_faulhaber_bridge]: CSP stall diagnostics: error>=25000 counts, progress<100 counts for 500 ms
[rascl_faulhaber_bridge.py-1] [INFO] [1785182147.895179352] [rascl_faulhaber_bridge]: Drive 3 two-stage close guard: approach/hold 0x60E0/0x60E1=300/100 per-mille; open and custom motion restore 1000 per-mille
[rascl_faulhaber_bridge.py-1] [INFO] [1785182147.895621406] [rascl_faulhaber_bridge]: CSP handoff will clear and verify volatile lower/upper limit-input mappings 0x2310:01/:02; Homing reference, input polarity and 0x607B/0x607D remain unchanged
[rascl_faulhaber_bridge.py-1] [INFO] [1785182147.896069673] [rascl_faulhaber_bridge]: Drive 0-2 Homing uses the centre of the reference-input interval: find the first edge with the configured native method, traverse the active interval and return to (entry+exit)/2 at the lower Homing zero speed with a sinusoidal profile, then set that midpoint to zero with Method 37; second-edge travel guards D0/D1/D2=[100000, 300000, 300000] counts
[rascl_faulhaber_bridge.py-1] [INFO] [1785182147.896555525] [rascl_faulhaber_bridge]: Drive 3 skips sensor Homing; after Drives 0-2 Home it will move +50000 counts and use Homing Method 37 to set that position to 0
[rascl_faulhaber_bridge.py-1] [EtherCAT] Opening interface: enx3c18a0256deb
[rascl_faulhaber_bridge.py-1] [EtherCAT] Found 4 slave(s)
[rascl_faulhaber_bridge.py-1] [EtherCAT] Homing-to-CSP session starts SDO-only in PRE-OP; PDO mapping is deferred until home_all succeeds
[rascl_faulhaber_bridge.py-1] [EtherCAT] Drive 0 uses slave 0: MC5004
[rascl_faulhaber_bridge.py-1] [EtherCAT] Drive 1 uses slave 1: MC5004
[rascl_faulhaber_bridge.py-1] [EtherCAT] Drive 2 uses slave 2: MC5004
[rascl_faulhaber_bridge.py-1] [EtherCAT] Drive 3 uses slave 3: MC5004
[rascl_faulhaber_bridge.py-1] [WARN] [1785182147.950997209] [rascl_faulhaber_bridge]: Drive 2 CSP following-error monitor changed for this session only: 0x6065 25000 -> 25000 counts; 0x6066 250 -> 250 ms. 0x607B/0x607D were read only, not modified. Drive 2 protection: 0x607B position_range=[-2147483648, 2147483647], 0x607D software_limit=[-802816, 802816], 0x6065 following_window=25000 counts, 0x6066 following_timeout=250 ms
[rascl_faulhaber_bridge.py-1] [INFO] [1785182147.954877326] [rascl_faulhaber_bridge]: TCP bridge listening on 127.0.0.1:15001
[rascl_faulhaber_bridge.py-1] [Drive 0] Controlword <- 0x0006
[rascl_faulhaber_bridge.py-1] [Drive 0] Controlword <- 0x0007
[rascl_faulhaber_bridge.py-1] [Drive 0] mode requested=6, display=6
[rascl_faulhaber_bridge.py-1] [Drive 0] Controlword <- 0x000F
[rascl_faulhaber_bridge.py-1] [Drive 0] Controlword <- 0x000F
[rascl_faulhaber_bridge.py-1] [Drive 0] Controlword <- 0x001F
[rascl_faulhaber_bridge.py-1] [Drive 0] Controlword <- 0x0000
[rascl_faulhaber_bridge.py-1] [Drive 0] Controlword <- 0x0006
[rascl_faulhaber_bridge.py-1] [Drive 0] Controlword <- 0x0007
[rascl_faulhaber_bridge.py-1] [Drive 0] mode requested=6, display=6
[rascl_faulhaber_bridge.py-1] [Drive 0] Controlword <- 0x000F
[rascl_faulhaber_bridge.py-1] [Drive 0] Controlword <- 0x000F
[rascl_faulhaber_bridge.py-1] [Drive 0] Controlword <- 0x001F
[rascl_faulhaber_bridge.py-1] [Drive 0] Controlword <- 0x0000


T2
rascl-container:~/ws$ bash ./rascl_debug.sh 6
waiting for service to become available...
requester: making request: std_srvs.srv.Trigger_Request()

response:
std_srvs.srv.Trigger_Response(success=True, message='Drive 0: physical=0x07/00000111, logical=0x00/00000000, polarity=0x07; 0x2310 lower/upper/option/reference=0x00/0x00/1/2; 0x2324.01=0x00001013 [none] | Drive 1: physical=0x03/00000011, logical=0x04/00000100, polarity=0x07; 0x2310 lower/upper/option/reference=0x00/0x00/1/2; 0x2324.01=0x04001113 [software_limit_positive] | Drive 2: physical=0x07/00000111, logical=0x00/00000000, polarity=0x07; 0x2310 lower/upper/option/reference=0x00/0x00/1/2; 0x2324.01=0x00001013 [none] | Drive 3: physical=0x01/00000001, logical=0x00/00000000, polarity=0x01; 0x2310 lower/upper/option/reference=0x00/0x00/1/1; 0x2324.01=0x00001003 [none]')

home_all first moves Drives 0-2 through their reference-input intervals, returns each to its midpoint, and sets zero; after success, Drive 3 automatically moves by 50000 counts and sets the reached position to 0 counts.
waiting for service to become available...
requester: making request: std_srvs.srv.Trigger_Request()

response:
std_srvs.srv.Trigger_Response(success=False, message='Home failed: Drive 0: homing error; statusword=0x2427')
ERROR: home_all or the Drive 3 reference move/zeroing failed; CSP entry is blocked
rascl-container:~/ws$ bash ./rascl_debug.sh 6
waiting for service to become available...
requester: making request: std_srvs.srv.Trigger_Request()

response:
std_srvs.srv.Trigger_Response(success=True, message='Drive 0: physical=0x07/00000111, logical=0x00/00000000, polarity=0x07; 0x2310 lower/upper/option/reference=0x00/0x00/1/2; 0x2324.01=0x00000003 [none] | Drive 1: physical=0x03/00000011, logical=0x04/00000100, polarity=0x07; 0x2310 lower/upper/option/reference=0x00/0x00/1/2; 0x2324.01=0x04001113 [software_limit_positive] | Drive 2: physical=0x07/00000111, logical=0x00/00000000, polarity=0x07; 0x2310 lower/upper/option/reference=0x00/0x00/1/2; 0x2324.01=0x00001013 [none] | Drive 3: physical=0x01/00000001, logical=0x00/00000000, polarity=0x01; 0x2310 lower/upper/option/reference=0x00/0x00/1/1; 0x2324.01=0x00001003 [none]')

home_all first moves Drives 0-2 through their reference-input intervals, returns each to its midpoint, and sets zero; after success, Drive 3 automatically moves by 50000 counts and sets the reached position to 0 counts.
waiting for service to become available...
requester: making request: std_srvs.srv.Trigger_Request()

response:
std_srvs.srv.Trigger_Response(success=False, message='Home failed: Drive 0: homing error; statusword=0x2427')
ERROR: home_all or the Drive 3 reference move/zeroing failed; CSP entry is blocked
