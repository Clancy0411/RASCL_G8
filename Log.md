## 2026-07-14 Automatic Homing and URDF Zero Calibration

The nominal ROS hardware-layer setting `home_offset_counts=[0,-802816,-802816,0]` was implemented. The raw switch positions at automatic Home remain approximately `[0,0,0,0] counts`, while ROS/URDF should report approximately `[0,+pi/2,+pi/2,0] rad`. The `3588dc98` URDF pose at `q=0` and TCP `[0.29756,-0.00177,0.043001] m` remain unchanged. These nominal values assume exactly 90 degrees at both joints; final hardware calibration still requires reading `0x6064` at the original physical URDF-zero pose.

The former section 7.7 item, "initial-position adjustment not implemented," is now handled by the software count offset. CSP/PDO operation and the final offset values still require step-by-step hardware verification using the new Debug Guide.

The Homing-to-CSP workflow was also corrected: the bridge must not be stopped after Home, because cleanup would issue Disable Operation/Voltage. `homing.launch.py` now retains the same EtherCAT master; after `home_all` succeeds, `ros2_control.launch.py start_bridge:=false` performs delayed PDO configuration and enters CSP. The handoff sends only Enable Operation, and the initial target uses the current actual counts.

## 2026-07-22 Initial Single-Pose TCP Correction

Based on measurements at automatic Home, the old model TCP was corrected by `[-0.023,0,+0.043] m` in `base_link`. A fixed `tcp_link` was added at local `lowerarm` coordinates `[0.11616,0.043,0.0179] m`; TF, FK, IK, and debug group 13 now use this frame. The physical position of `spur_gear_joint` and gripper motion were unchanged. The nominal automatic-Home TCP is `[0.18456,-0.00177,0.336001] m`. This result came from only one pose and still requires external measurements at additional poses.

## 2026-07-22 Second TCP Correction at the Current Test Point (Later Used as the Gear-Surface Reference)

At program target `XYZ=[0.16,-0.16,0.05] m`, the measured physical values in `Y/X/Z` order were `[0.14,-0.16,0.05] m`. Following the project's physical-axis convention, the corresponding `base_link` numerical error `[-0.020,0,0] m` was transformed into `lowerarm`, producing the fixed TCP `[0.11478978,0.02881369,0.03193108] m` used at that time. The nominal zero-pose TCP was `[0.27318978,-0.01580108,0.07181469] m`, and the automatic-Home TCP was `[0.18318978,-0.01580108,0.32181469] m`. This correction was retained later as the gear-surface reference point.

## 2026-07-22 TCP Changed to the Gripper Center

The original motion coordinates were confirmed to represent the gear surface. The grasp center lies approximately `20 mm` from that surface along the gripper extension direction, or half the gripper length. Retaining the existing surface calibration and adding `0.020 m` along `lowerarm +X` gives the current fixed TCP `[0.13478978,0.02881369,0.03193108] m`. The new nominal zero-pose TCP is `[0.29318978,-0.01580108,0.07181469] m`, and the automatic-Home TCP is `[0.20318978,-0.01580108,0.32181469] m`. At the same old joint pose, the grasp center is displaced `20 mm` from the gear surface along the tool direction; this offset must not be added to a fixed horizontal `base_link` coordinate.

The new debugging instructions are in `README_RASCL_Group8.md`.





## 6.2 The Test Successfully Generated a Trajectory File

### Output
head /tmp/rascl_wp3_tsk1_last_trajectory.csv
time_from_start,shoulder_joint,upperarm_joint,lowerarm_joint,spur_gear_joint
0.000000,0.000000000,0.000000000,0.000000000,0.000000000
0.100000,-0.000001057,-0.000037122,-0.000123951,0.000000000
0.200000,-0.000008138,-0.000285761,-0.000954150,0.000000000
0.300000,-0.000026411,-0.000927358,-0.003096434,0.000000000
0.400000,-0.000060153,-0.002112131,-0.007052369,0.000000000
0.500000,-0.000112803,-0.003960800,-0.013225041,0.000000000
0.600000,-0.000187008,-0.006566328,-0.021924854,0.000000000
0.700000,-0.000284675,-0.009995654,-0.033375314,0.000000000
0.800000,-0.000407019,-0.014291428,-0.047718830,0.000000000


The output shows that the trajectory starts at 0 seconds with all initial joint angles at 0.
The values of shoulder_joint, upperarm_joint, and lowerarm_joint then change gradually, confirming that the program generated a continuous, smooth trajectory.

Because `execute:=false`, this step did not move the physical robot; it only verified planning, IK, and CSV generation.
The x-axis follows the arm extension direction, positive outward from the robot.
The y-axis is horizontal, positive to the left.
The z-axis is vertical, positive upward.


## 6.2 RViz Should Show Smooth Arm Motion

Testing of the wp3_tsk1 minimum-jerk trajectory program continued on the physical RASCL robot.

It was confirmed that both controllers become active after physical-hardware startup and that `/joint_states` can be read. Small joint commands sent through `/rascl_position_controller/commands` produce small robot motions, confirming the complete chain from the ROS 2 command topic through the controller, hardware interface, EtherCAT/Faulhaber bridge, and physical motors.

The WP3 program itself was then tested. It can read the current joint state, calculate the current TCP position, solve IK, generate a minimum-jerk trajectory, and save the CSV file. For a very small target, the IK result was approximately:

q_arm = [-0.00022, -0.00144, -0.01722] 

The largest joint change is only about 1 degree, and the physical test was relatively stable.

For a somewhat larger target, for example:

target = (0.29, 0.00, 0.05) 

The IK result became:

q_arm = [-0.00608, -0.06696, -0.19718] 

Here, lowerarm_joint must move approximately -0.197 rad, or about -11.3 degrees. Executing this larger trajectory caused the physical ros2_control interface to report an error:

Afterward, `/joint_states` was also unavailable, requiring the launch process to be stopped, processes cleaned up, and the hardware restarted.

Therefore, larger targets should first be verified with fake hardware/RViz. If they run correctly there, the WP3 program itself has no obvious problem. Physical-robot testing should then proceed incrementally with smaller displacements, longer durations, and a lower `rate_hz`.



x=0.5 failed
The requested Cartesian target is probably outside the reachable workspace, too close to a singularity, or blocked by the current joint limits. Best error was 0.1992 m.


x=0.29 y=0 z=0.1 work successlly
but after that give command again, always meets,
[ERROR] [1782836702.566606570] [wp3_tsk1]: No joint state received within 5.0 s
solution:change controllers.yaml ---update_rate to 50 or 100

>> TO DO
change original state in RVIZ 
automatical homing with green light sensor






2026-07-07


## Error ：pysoem WkcError during SDO write

### Error Message

```text
pysoem.pysoem.WkcError
```

Full context:

```text
[rascl_faulhaber_bridge]: Connecting EtherCAT on interface: enx3c18a0264863
[EtherCAT] Opening interface: enx3c18a0264863
[EtherCAT] Found 4 slave(s)
[EtherCAT] Configuring CSP PDO mapping for slave 0
[EtherCAT] Configuring CSP PDO mapping for slave 1
Traceback (most recent call last):
  File "/root/ws/install/rascl_hardware_interface/lib/rascl_hardware_interface/rascl_faulhaber_bridge.py", line 465, in __init__
    self.bus.connect()
  File "/root/ws/install/rascl_hardware_interface/lib/rascl_hardware_interface/rascl_faulhaber_bridge.py", line 256, in connect
    self.configure_csp_pdo_mapping(self.master.slaves[slave_index], slave_index)
  File "/root/ws/install/rascl_hardware_interface/lib/rascl_hardware_interface/rascl_faulhaber_bridge.py", line 293, in configure_csp_pdo_mapping
    self._sdo_write_int_raw(slave, PDO_RX_MAPPING, 0, 0, size=1)
  File "/root/ws/install/rascl_hardware_interface/lib/rascl_hardware_interface/rascl_faulhaber_bridge.py", line 284, in _sdo_write_int_raw
    slave.sdo_write(index, subindex, int(value).to_bytes(size, "little", signed=signed))
  File "src/pysoem/pysoem.pyx", line 972, in pysoem.pysoem.CdefSlave.sdo_write
pysoem.pysoem.WkcError
```

### Cause Assessment

This was not a missing-slave problem, because the log already contained:

```text
Found 4 slave(s)
```

The actual problem was an SDO write failure on a slave while configuring the CSP PDO mapping.

Failure location:

```python
self._sdo_write_int_raw(slave, PDO_RX_MAPPING, 0, 0, size=1)
```

This operation attempts to clear subindex 0 of the RxPDO mapping in preparation for remapping.

Possible causes:

1. The current drive state does not permit PDO mapping changes.
2. The drive must be in PRE-OP before PDO remapping.
3. Some Faulhaber drives may not support the current object-dictionary access pattern.
4. Slave 1 may be in an abnormal state because the log suggests slave 0 passed before slave 1 failed.
5. CSP PDO mapping should not run during the Profile regression test.

---

## 8. Code Summary Related to Error 3

### 8.1 Key Code in connect()

File:

```text
src/rascl_hardware_interface/scripts/rascl_faulhaber_bridge.py
```

Code inspected at the time:

```python
def connect(self) -> None:
    # Create and configure the EtherCAT master before constructing drive wrappers.
    self.master = pysoem.Master()
    print(f"[EtherCAT] Opening interface: {self.interface}")
    self.master.open(self.interface)

    if self.master.config_init() <= 0:
        raise RuntimeError("No EtherCAT slaves found")

    print(f"[EtherCAT] Found {len(self.master.slaves)} slave(s)")

    if self.configure_pdo_mapping:
        for slave_index in self.slave_indices:
            if slave_index >= len(self.master.slaves):
                raise RuntimeError(
                    f"slave index {slave_index} requested, but only {len(self.master.slaves)} slave(s) found"
                )
            self.configure_csp_pdo_mapping(self.master.slaves[slave_index], slave_index)

    self.master.config_map()
    print("[EtherCAT] PDO mapping configured")
```

### 8.2 Problem

The key condition is:

```python
if self.configure_pdo_mapping:
```

Whenever `self.configure_pdo_mapping` is `True`, the following runs:

```python
self.configure_csp_pdo_mapping(...)
```

The observed error occurred inside `configure_csp_pdo_mapping()`.

### 8.3 Failure Location in configure_csp_pdo_mapping()

Code inspected at the time:

```python
def configure_csp_pdo_mapping(self, slave, slave_index: int) -> None:
    # Mapping is done in PRE-OP before config_map().  If a lab drive rejects
    # remapping, launch with configure_pdo_mapping:=false and inspect the default
    # PDO layout before retrying.
    print(f"[EtherCAT] Configuring CSP PDO mapping for slave {slave_index}")

    # RxPDO 0x1600
    self._sdo_write_int_raw(slave, PDO_RX_MAPPING, 0, 0, size=1)
    self._sdo_write_int_raw(slave, PDO_RX_MAPPING, 1, 0x60400010, size=4)
    self._sdo_write_int_raw(slave, PDO_RX_MAPPING, 2, 0x607A0020, size=4)
    self._sdo_write_int_raw(slave, PDO_RX_MAPPING, 3, 0x60600008, size=4)
    self._sdo_write_int_raw(slave, PDO_RX_MAPPING, 0, 3, size=1)

    # TxPDO 0x1A00
    self._sdo_write_int_raw(slave, PDO_TX_MAPPING, 0, 0, size=1)
    self._sdo_write_int_raw(slave, PDO_TX_MAPPING, 1, 0x60410010, size=4)
    self._sdo_write_int_raw(slave, PDO_TX_MAPPING, 2, 0x60640020, size=4)
    self._sdo_write_int_raw(slave, PDO_TX_MAPPING, 3, 0x60610008, size=4)
    self._sdo_write_int_raw(slave, PDO_TX_MAPPING, 0, 3, size=1)

    # Assign the single RxPDO and TxPDO.
    self._sdo_write_int_raw(slave, PDO_RX_ASSIGNMENT, 0, 0, size=1)
    self._sdo_write_int_raw(slave, PDO_RX_ASSIGNMENT, 1, PDO_RX_MAPPING, size=2)
    self._sdo_write_int_raw(slave, PDO_RX_ASSIGNMENT, 0, 1, size=1)

    self._sdo_write_int_raw(slave, PDO_TX_ASSIGNMENT, 0, 0, size=1)
    self._sdo_write_int_raw(slave, PDO_TX_ASSIGNMENT, 1, PDO_TX_MAPPING, size=2)
    self._sdo_write_int_raw(slave, PDO_TX_ASSIGNMENT, 0, 1, size=1)
```

The actual failure occurred at the first SDO write:

```python
self._sdo_write_int_raw(slave, PDO_RX_MAPPING, 0, 0, size=1)
```

---

## 9. Most Important Conclusions at the Time

### Conclusion 1

The fake hardware and the WP3 Task 1 upper-layer minimum-jerk node worked successfully.

### Conclusion 2

The physical EtherCAT interface should be:

```text
enx3c18a0264863
```

because that interface found:

```text
Found 4 slave(s)
```

### Conclusion 3

The physical-hardware Profile Position regression test failed not because slaves were missing, but because the bridge still attempted to configure CSP PDO mapping when starting in profile mode.

### Conclusion 4

`configure_pdo_mapping:=false` was not taking effect because the launch output still showed:

```text
Configuring CSP PDO mapping for slave 0
Configuring CSP PDO mapping for slave 1
```

This indicated one of the following:

1. The launch argument may not have been passed correctly to the bridge.
2. The bridge default for `configure_pdo_mapping` may still have been True.
3. The install directory may still have contained old code.
4. The launch file may have accepted the argument without adding it to the bridge node parameters.

---

## 10. Suggested Order for Continuing Debugging

## Step 1: Confirm Where configure_pdo_mapping Is Defined and Passed

```bash
cd /root/ws
grep -R -n "configure_pdo_mapping" src/rascl_description src/rascl_hardware_interface
```

Inspect especially:

```text
src/rascl_description/launch/ros2_control.launch.py
src/rascl_hardware_interface/scripts/rascl_faulhaber_bridge.py
```

---

## Step 2: Check Whether the Launch File Declares and Passes the Argument

```bash
nl -ba src/rascl_description/launch/ros2_control.launch.py | sed -n '1,220p'
```

Confirm that the launch file contains something similar to:

```python
DeclareLaunchArgument(
    "configure_pdo_mapping",
    default_value="true",
)
```

and that the bridge node parameters contain:

```python
"configure_pdo_mapping": LaunchConfiguration("configure_pdo_mapping"),
```

If only `DeclareLaunchArgument` exists and the value is not passed to the Node, the command-line argument will not reach the bridge.

---

## Step 3: Temporary Direct Workaround

To complete the Profile Position regression test first, the bridge default could temporarily be changed to False.

Search for:

```bash
grep -n "configure_pdo_mapping" src/rascl_hardware_interface/scripts/rascl_faulhaber_bridge.py
```

Find something similar to:

```python
self.declare_parameter("configure_pdo_mapping", True)
```

Temporarily change it to:

```python
self.declare_parameter("configure_pdo_mapping", False)
```

Or, if it is:

```python
self.configure_pdo_mapping = True
```

Temporarily change it to:

```python
self.configure_pdo_mapping = False
```

Then rebuild.
## 2026-07-22 Simplified Group 15 Gripper Open/Close

Group `15` no longer asks for arbitrary relative counts and motion duration. ASCII input `close`/`c` moves Drive 3 by `-110000 counts` from the current position to tighten the grip; `open`/`o` moves it by `+110000 counts` to release. Both actions continue to use 50 Hz minimum-jerk CSP at the default `10000 counts/s` (about 11 seconds) while holding Drives 0-2 at their current positions. CSP, feedback, concurrency, and URDF-limit checks remain unchanged.

## 2026-07-22 Group 15 Joint-State Startup Timeout Fix

Group `15` originally waited 3 seconds for `/joint_states` during preflight, while the actual motion node waited only 1 second. Two consecutive physical logs contained only `SPUR_TRACE start`, with no `progress/complete`, and Drive 3 remained near `-1536 counts`, showing that the node exited on a DDS first-feedback timeout before publishing the trajectory. Both waits now default to 5 seconds and can be overridden with `RASCL_SPUR_GEAR_FEEDBACK_TIMEOUT_S`; motion-node exceptions also record `SPUR_TRACE failed`. EtherCAT, CSP, counts, direction, and torque parameters are unchanged.

## 2026-07-23 Restored Custom Relative Counts for Drive 3

Group `15` retains the `close/c=-110000` and `open/o=+110000` shortcuts while restoring direct input of any nonzero signed integer count value. A custom value remains relative to the current Drive 3 position and uses the same URDF limits, controller/concurrency checks, 50 Hz minimum-jerk trajectory, automatic duration, and `SPUR_TRACE` feedback. It is not an absolute encoder target.

## 2026-07-23 Drive 3 Torque-Limit and Contact-Termination Fix

Physical logs confirmed that Drive 3 `0x2329:03=81 mA` produced only `0x6072=150` (15% rated torque). Although `0x60E0/0x60E1` were set to `1000`, negative-direction gripping still became `torque_limited` and eventually stopped the entire CSP session with following error `statusword=0x3027`. Drive 3 now receives the same session-level peak-current correction as Drive 2 at CSP handoff, typically `81->540 mA`, with mandatory readback `0x6072>=1000`; no parameters are stored persistently.

Group `15` close/open became maximum-travel shortcuts. If command/feedback error reaches the default `2000 counts / 0.04 s`, the script records `SPUR_CONTACT`, retracts the target to the measured position before a drive following error, and returns `SPUR_RESULT outcome=contact_or_endpoint`. Directly entered signed counts still require exact relative motion and do not enable contact-based early termination.

## 2026-07-23 Drive 3 Gripper Shortcut Direction Correction

Based on the physical direction, the group `15` shortcuts were reversed: `close/c` became a maximum relative `+110000 counts` with continued `2000 counts / 0.04 s` tracking-error detection, early stop after gripping, and hold at the measured position. `open/o` became a fixed relative `-200000 counts`, requiring full arrival without contact-based early termination. The semantics of directly entered custom signed counts were unchanged.

## 2026-07-23 Increased Drive 3 Maximum Closing Travel

After mounting the gripper, the `close/c` maximum travel of `+110000 counts` was insufficient to contact the block. Only the maximum closing travel was increased to `+500000 counts`; the `2000 counts / 0.04 s` contact stop, `open/o=-200000 counts`, and custom-count logic were unchanged. At the default `10000 counts/s`, a move without early contact can take about 50 seconds.

## 2026-07-23 Expanded Project-Side Drive 3 Position Limits

Log `ros_logs_20260723_131436.tar.gz` showed Drive 3 at approximately `374357 counts / 1.777884 rad` after the previous `open=-200000 counts`. The following `close=+500000 counts` target was approximately `874357 counts / 4.1525 rad`, above the old `+3.1415 rad` limit, so script preflight rejected it before sending any drive command. Project-side `spur_gear_joint` limits in the physical URDF, ros2_control, Python kinematics, and debug script were therefore expanded consistently from about `[-pi,+pi]` to `[-2*pi,+2*pi] = [-6.283185307,+6.283185307] rad`. Drive 0-2 limits, Drive 3 close/open direction and travel, contact stop, Homing, CSP, and internal drive objects `0x607B/0x607D` were not modified.

## 2026-07-23 TCP X +20 mm from current command point

Per user request, the fixed TCP was moved another `0.020 m` along `lowerarm +X` from the current commanded point. The TCP origin changed from `[0.13478978,0.02881369,0.03193108] m` to `[0.15478978,0.02881369,0.03193108] m`, for a total `40 mm` offset from the calibrated gear-surface reference. Nominal zero TCP is now `[0.31318978,-0.01580108,0.07181469] m`; nominal automatic Home TCP is now `[0.22318978,-0.01580108,0.32181469] m`.

## 2026-07-23 Revert additional local TCP X +20 mm

Real-hardware testing showed that the additional `lowerarm +X` shift projected mainly into `base_link` Z at the tested poses, increasing the physical Z discrepancy without removing the horizontal error. The last additional `0.020 m` was therefore reverted. The fixed TCP origin is again `[0.13478978,0.02881369,0.03193108] m`, which is `20 mm` from the calibrated gear-surface reference. Nominal zero TCP is `[0.29318978,-0.01580108,0.07181469] m`; nominal automatic Home TCP is `[0.20318978,-0.01580108,0.32181469] m`.

## 2026-07-23 Global physical-X +20 mm calibration

The remaining target-to-real-position discrepancy is handled as a project-level global calibration rather than a manual target edit or another rotating TCP offset. External measurements use Y/X/Z, so physical +X maps to numeric `base_link` +Y. The `base_link -> shoulder_joint` model origin now uses Y=`-0.020 m`; with unchanged requested targets, IK consequently moves the real gripper `+0.020 m` in physical X. The fixed TCP remains `[0.13478978,0.02881369,0.03193108] m`. Nominal zero TCP becomes `[0.29318978,-0.03580108,0.07181469] m`, and nominal automatic Home TCP becomes `[0.20318978,-0.03580108,0.32181469] m`.

## 2026-07-23 Group 15 false-contact rejection

Logs `ros_logs_20260723_141113.tar.gz` showed repeated `close` actions stopping after about `25k counts` while Drive 3 was still progressing normally. The old detector interpreted ordinary minimum-jerk command lag (`2019/2150 counts`) as contact because it checked only `error>=2000 counts` for `0.04 s`. Contact confirmation now additionally requires encoder progress to remain at or below `100 counts` for `0.10 s`, evaluates each fresh `/joint_states` sample only once, and accepts lag only in the commanded closing direction. The `2000-count` pre-following-error threshold, `+500000-count` maximum close travel, exact `open=-200000 counts`, torque configuration, and drive protections remain unchanged.

## 2026-07-24 Drive 3 fixed reference and absolute-count readback

After Drives 0–2 complete sensor Homing, Drive 3 now starts from live `0x6064`,
moves `-50000 counts` in Profile Position, verifies the endpoint within
`100 counts`, and runs FAULHABER Homing Method 37 to define the reached position
as `0 counts`. The bridge refuses CSP handoff if either the relative move or the
zero readback fails. Group `15` remains a relative-count CSP control. New debug
group `17` reads Drive 3 `absolute_counts` from SDO before CSP or the bridge PDO
cache during CSP, allowing measured open and closed positions to be recorded.
Group `6` now stops immediately on a failed Home/reference service response
instead of continuing because the ROS CLI process itself exited successfully.

## 2026-07-24 Drive 3 direction reversed again

The Drive 3 encoder-to-URDF mapping is now `direction=-1`. The fixed Homing
reference move is reversed from `-50000` to `+50000` raw counts before Method
37 defines the reached position as zero. The group `15` shortcuts are reversed
with it: `close/c=-500000 counts` maximum travel and
`open/o=+200000 counts` exact travel. Direct signed integer group `15` input
keeps raw encoder semantics, so a positive custom value still increases the
group `17` `absolute_counts` readback.

## 2026-07-27 Drive 0–2 reference-interval midpoint Homing

Drive 0–2 no longer keep the first detected reference-switch edge as their raw
zero. The existing FAULHABER methods `[28,28,24]` still find the proven first
edge. Each drive then continues in the same encoder direction at the configured
`1000` search speed until the polarity-corrected reference input becomes
inactive, records that exit count, halts without removing torque, returns to
`(entry+exit)/2`, and uses Method 37 to define the midpoint as `0 counts`.
Because `0x607C=0`, the native latched edge is exactly `entry=0`; the
post-edge deceleration stop readback is diagnostic only and is not averaged.

The finite second-edge and midpoint moves are limited to `100000 counts` and
`120 s` per phase; the initial native search keeps its original `8 s` timeout.
Fault, following error, a missing exit edge, or midpoint error prevents the axis
from being marked Homed and blocks CSP handoff. `home_one/home_all` and debug
groups `5/6` now report and verify `entry`, `exit`, `width`, `midpoint`,
`reached`, and `zero` for every required arm drive. Drive directions, URDF/count
offsets, Drive 3 reference logic, CSP/PDO, and Cartesian planning are unchanged.

## 2026-07-27 midpoint Homing real-hardware correction

The first real interval-centre run completed Drive 0 with
`entry=0, exit=-59657, midpoint=-29829, reached=-29827, zero=0`. Drive 1 then
remained inside its active reference interval after the old `100000-count`
guard, so `home_all` stopped before Drive 2 was attempted. FAULHABER Method 28
still searches and traverses in the negative encoder direction; that direction
was not reversed.

The finite second-edge traversal and midpoint return now use the lower
`homing_zero_speeds=200` instead of the `1000` native switch-seek speed. They
temporarily select the FAULHABER sinusoidal Profile Position curve
(`0x6086:00=1`) and restore the previous profile afterward. The Drive 0/1/2
second-edge travel guards are now independent defaults of
`100000/300000/300000 counts`; the timeout remains `120 s`. A guard failure now
also reports `last_actual`, `reference_active`, `active_interval_seen`, and
`internal_limit_seen`. Native first-edge methods, directions, inputs, offsets,
Drive 3, CSP/PDO, URDF, and Cartesian planning are unchanged.

The next real run validated the smooth interval search on Drives 0 and 1:
Drive 0 returned `entry=0, exit=-57190, midpoint=-28595, reached=-28592,
zero=0`. Drive 1 also found its interval and returned to midpoint, but its
successful Method 37 completion was rejected before Drive 2 started because
the immediate zero readback was `26 counts` while the call had silently reduced
the configured `100-count` midpoint tolerance to `10 counts`. The implementation
now uses one easy-to-understand `500-count` tolerance for midpoint arrival and
the immediate Method 37 readback, including Drive 3, and interval evidence
reports `zero_tolerance=500`. A readback such as `26 counts` is about
`0.000051 rad` (`0.0029 deg`) and no longer blocks the next axis; values outside
the configured tolerance still fail Homing.

## 2026-07-27 measured lowerarm-to-TCP length

Real-hardware measurement established that the ideal TCP is `170 mm` along
lowerarm +X from the remote reference screw hole. The drawing's `17.9 mm`
perpendicular offset remains independent and unchanged. The fixed planning
frame therefore changed from `[0.13916,0,0.0179] m` to
`[0.170,0,0.0179] m` in `lowerarm`.

The physical `spur_gear_joint` remains at its CAD origin
`[0.13916,0,0.0179] m`; its mesh, Drive 3 control, Homing, CSP and count
mapping were not changed. URDF, Python FK/IK, the motion-node description,
regression tests and calibration documents now use the same ideal TCP.
Nominal model values are `[0.36840,0.01823,0.043001] m` at `q=[0,0,0]` and
`[0.27840,0.01823,0.293001] m` at
`q=[0,+pi/2,+pi/2]`.

## 2026-07-27 remove base-to-shoulder linear displacement

The single-point XY correction had moved `base_link -> shoulder_joint` to
`[0.040,0.020,0.057441] m`, visibly displacing the shoulder rotation axis from
the base. It has been removed and the uncompensated CAD alignment
`[0,0,0.057441] m` restored in both URDF and Python kinematics.

The measured ideal TCP remains `[0.170,0,0.0179] m` in `lowerarm`; Homing, CSP,
Drive 3 and encoder mappings are unchanged. Removing the base XY translation
changes the nominal TCP to `[0.32840,-0.00177,0.043001] m` at `q=[0,0,0]` and
`[0.23840,-0.00177,0.293001] m` at
`q=[0,+pi/2,+pi/2]`.

## 2026-07-27 double Drive 3 CSP motion speed

Debug group `15` now defaults to `20000 counts/s` instead of
`10000 counts/s` for `close`, `open`, and direct signed relative-count
commands. The automatically calculated duration is therefore halved:
`500000 counts` takes about `25 s` and `200000 counts` about `10 s`.
The 50 Hz minimum-jerk CSP path, relative-count semantics, travel limits,
contact detector, arm-joint hold and Drive 3 Homing reference profile
(`3000 counts/s`) are unchanged.

## 2026-07-27 gentle Drive 3 contact close

The former group `15` close could apply the normal `1000`-per-mille Drive 3
torque until a 2000-count lag was detected, which was too late for the gripper.
Close now stages and verifies session-only approach
`0x60E0/0x60E1=300` at one SDO per PDO cycle, approaches at
`20000 counts/s`, detects contact at
`300 counts / 50-count maximum progress / 0.06 s`, and holds with only
`100 counts` of closing preload. At contact it immediately stages
`0x60E0/0x60E1=100` for holding.
Open and direct relative-count commands restore verified
`0x60E0/0x60E1=1000` and retain `20000 counts/s`.

After close, group `15` requests a staged `SPUR_CONTACT_SNAPSHOT` containing
Drive 3 position error, status, torque demand/actual value, actual current and
the existing limit/input diagnostics. Torque switching and snapshot collection
perform at most one mailbox operation per 50 Hz PDO cycle; Homing, the Drive 3
Method-37 reference profile and arm Cartesian trajectories are unchanged.

## 2026-07-27 Drive 0–2 counts-level Home fine adjustment

Debug groups `19`, `20`, and `21` now provide signed relative Profile Position
moves for Drives 0, 1, and 2 after successful `home_all` and before CSP
preparation. Each call starts from live encoder feedback, moves at
`1000 counts/s`, waits for verified arrival, and returns source, delta, target,
actual, target error, and `correction_from_homed_zero`. Repeated calls are
allowed; the last field is already the current cumulative displacement from
the Method-37 Home zero.

The service rejects Drive 3, zero increments, incomplete Homing/Drive 3
reference, and any call after PDO preparation or CSP activation. It does not
rerun Homing, write Method 37, change Drive 3, or persist calibration.
Before moving it clears only the selected drive's volatile
`0x2310:01/:02` mappings so a stale limit flag in the reference region cannot
block one Profile Position direction; `0x2310:04`, polarity and
`0x607B/0x607D` are preserved.

Debug group `22` now captures the three current correction counts and runs
Method 37 on Drives 0–2 without issuing a position target. It verifies all
three zero readbacks and explicitly leaves Drive 3 unchanged. This lets the
fine-adjusted physical pose enter CSP as the existing nominal Home for the
current session. The captured values are logged before the first Method 37
write so a later-axis failure does not erase the calibration evidence.

This also corrects the earlier permanent-calibration wording:
`home_offset_counts` changes the encoder-to-URDF mapping but cannot move the
next automatic Homing endpoint. Permanent physical correction must move each
axis by the measured post-midpoint count and then run Method 37 during
`home_all`.
