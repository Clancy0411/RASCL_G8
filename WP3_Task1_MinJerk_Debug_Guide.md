# WP3 Task 1 Physical-Hardware Homing + CSP/PDO Debug Guide

## Shortest Physical-Hardware Workflow

Use this sequence for every complete restart. It includes only the steps required
to start the physical robot, Home it, enter CSP, and send TCP targets. Do not add a
build, test, or fake-hardware run when the code, Docker image, and `install/` tree
have not changed. Run group `1` once, with every physical process stopped, only
after changing code, pulling a new revision, or receiving a missing-package error
from group `4`.

### A. Open Three Terminals

In three Ubuntu terminals, run:

```bash
cd ~/RASCL_G8
bash ./rosws.sh
```

After the prompt changes to `rascl-container:~/ws$`, name the container terminals
T1, T2, and T3. In each terminal, run:

```bash
cd /root/ws
bash ./rascl_debug.sh
```

The script does not switch terminals. Groups `4` and `7` are foreground processes
and must occupy T1 and T2 respectively.

### B. Direct Physical Startup

Run these group numbers in exactly this terminal order:

```text
T1: 4
    | keep the bridge running; do not press Ctrl-C
T2: 6
    | D0-D2 return to the midpoint of their sensor intervals;
    | D3 then moves +50000 counts and is zeroed; success=True is required
T2: optional 19 / 20 / 21
    | calibration only: trim D0 / D1 / D2 with relative counts;
    | record correction_from_homed_zero
T2: 22 during calibration
    | no motion; Method 37 sets the current D0-D2 pose as session Home;
    | D3 remains unchanged
T2: 7
    | keep ros2_control running; do not press Ctrl-C
T3: 8 -> 13 -> 28
```

For one TCP target, use T3 group `23`. For the complete Task 1 sequence, use group
`28`. Groups `24 -> 25 -> 26 -> 27` remain available for individual stages.

### Required Checkpoints

1. **T1 group 4: start the only EtherCAT/Homing bridge.**

   - The current workstation interface is `enx3c18a0256deb`. Group `4` uses it by
     default. On another workstation, pass
     `RASCL_INTERFACE=<interface-name>`.
   - Required software is checked before physical motion. Missing packages stop the
     process and point to group `1`.
   - Drives 0-2 use sensor-interval midpoint Homing. The native method finds the
     first edge at speed `1000`. The bridge then uses speed `200` and a sinusoidal
     profile to cross the active interval, records the second edge, Halts, returns
     to `(entry+exit)/2`, and applies Method 37 to set the midpoint to `0 counts`.
     D0/D1 traverse in the negative direction; D2 traverses in the positive
     direction. Joint direction and URDF offset are not changed.
   - After all three axes arrive, Drive 3 moves `+50000 counts` from its live
     position and Method 37 sets the reached position to `0 counts`.
   - Startup must report `Drive 2 CSP following-error monitor changed`. The current
     session uses `0x6065/0x6066 = 25000 counts / 250 ms`. Objects `0x607B/0x607D`
     are read but not written. A transient PRE-OP `WkcError` is retried; a persistent
     read failure is warned and group `6` reads again.
   - CSP torque settings do not alter Homing. During group `7` handoff, writable
     Drive 0-3 `0x60E0/0x60E1` become `1000` and are verified. Insufficient Drive
     2/3 peak currents at `0x2329:03` are raised for the current session from the
     observed `220/81 mA` to `1100/540 mA`. Rated and continuous current at
     `0x2329:01/:02` are unchanged. Nothing is stored persistently.
   - Wait for `TCP bridge listening on 127.0.0.1:15001`.
   - Never stop T1 or start a second bridge until shutdown or a complete restart.

2. **T2 group 6: run `home_all`.**

   - There is no second confirmation; selecting group `6` starts motion.
   - Initial output records each drive input state and
     `0x2310 lower/upper/option/reference` before the CSP correction.
   - Drives 0-2 move first. Every axis must report
     `driveN_interval(entry=...,exit=...,width=...,midpoint=...,reached=...,zero=...,zero_tolerance=500)`.
     `abs(zero)` must not exceed `500 counts`. A missing interval record stops the
     script. Drive 3 then performs its `+50000 counts` reference and zeroing.
   - Required output includes `success=True`, `CSP handoff armed`, and
     `drive3_reference(...delta=50000,...zero=0,method=37)`.
   - The automatic count query must report `absolute_counts` near `0` and
     `reference_complete=true`.
   - The automatic Drive 2 diagnostic read follows. Preserve an unusual `0x607B`
     or `0x607D` value; do not clear it without diagnosis.
   - Do not enter group `7` unless all checks appear.

3. **T2 groups 19/20/21: calibration-only Home trim.**

   - Groups `19/20/21` trim Drives `0/1/2` respectively using signed relative
     counts and may be repeated.
   - `correction_from_homed_zero` is the live cumulative count offset from the
     current Method 37 Homing zero; do not add values manually.
   - These groups do not re-Home, reset zero, move Drive 3, or store parameters.
   - At the verified physical Home, record all three cumulative values and run
     group `22`.

4. **T2 group 22: set the current pose as session Home during calibration.**

   - Method 37 defines the current Drive 0-2 positions as `0 counts` without active
     motion. The Drive 3 gripper zero is unchanged.
   - Save `drive0_before/drive1_before/drive2_before`; these are the corrections
     needed to make future automatic Homing reach this pose. Each `driveN_after`
     should be near `0`.
   - This zero lasts only for the current power/EtherCAT session. Group `6` overwrites it.

5. **T2 group 7: start CSP ros2_control.**

   - Group `7` must reuse the bridge still running in T1.
   - T1 must report `CSP_LIMIT_SWITCH_CONFIGURATION`. Final D0-D3 `lower/upper`
     values must be `0x00/0x00`, and `device` must not contain
     `positive_limit_switch` or `negative_limit_switch`. Only volatile mappings
     `0x2310:01/:02` are cleared. Homing reference, input polarity, and
     `0x607B/0x607D` remain unchanged. Failed readback blocks CSP.
   - T1 must report `CSP_FOLLOWING_ERROR_CONFIGURATION` with final
     `0x6065=25000 counts` and `0x6066=250 ms`. Some drives restore old values after
     leaving Homing; handoff rewrites and verifies them.
   - T1 must report `CSP directional torque limits verified for this session only`.
     D0-D3 final positive/negative values must be `1000/1000`. Expected first-time
     corrections include Drive 2 `max/pos/neg 200/200/200 -> 1000/1000/1000` with
     motor current `1100/1100/220 -> 1100/1100/1100`, and Drive 3
     `150/150/150 -> 1000/1000/1000` with current
     `540/540/81 -> 540/540/540`. A previously corrected session may already show
     final values. Drive 0/1 read-only `max` values may remain unchanged, and their
     current parameters are not modified.
   - `0x6072` is read only. Any failed peak-current write/readback or
     `0x6072 >= 1000` check blocks CSP.
   - T1 must report `Master reached OP state` and successful Homing-to-CSP handoff.
   - T2 must report `Activated real RASCL hardware in csp mode`, then remain occupied.

6. **T3 group 8: check controllers and joint state.**

   - `joint_state_broadcaster` and `rascl_position_controller` must both be `active`.
   - `/joint_states` must remain continuous, with four axes near
     `[0,+1.5708,+1.5708,0]`.
   - Do not run group `9` or `10` if either check fails.

7. **T3 group 13: inspect the current model TCP.**

   - This reads `base_link -> tcp_link`. The current TCP is the ideal point measured
     170 mm along lowerarm +X. Nominal Home is near
     `[0.23840,-0.00177,0.293001] m`.
   - Record the result and externally measure the same physical reference point.

8. **T3 group 14: enter the next target.**

   - Enter TCP `x/y/z` in metres and motion duration in seconds.
   - Integers and decimals are accepted; `5` is sent to ROS as `5.0` seconds.
   - This group sets a target but does not move.

9. **T3 group 9: plan only.**

   - IK must succeed, the process must exit normally, and the CSV must contain no
     `nan/inf`.
   - Success records an authorization bound to the current CSP session and exact
     target. Group `10` may run from the menu or in another script invocation.

10. **T3 group 10: execute.**

    - The script rechecks both active controllers and `/joint_states`.
    - There is no second confirmation; the authorized trajectory starts immediately.
    - Success requires `MOTION_RESULT reached=true`. Final four-axis feedback and
      TCP error are checked; publishing all commands while the robot stalls is a
      failure.
    - On failure, the latest `CSP_STALL_SNAPSHOT` is printed. Run group `12`
      immediately and stop sending targets. Group `16` reads the same snapshot.
    - Authorization is cleared after every execution. A new target requires
      `14 -> 9 -> 10` again.

11. **T3 group 23: enter, plan, and execute one target.**

    - This is the recommended group for repeated Cartesian commands. It reads
      x/y/z/duration, reuses the group `9` IK, CSV, and fixed-board XY-compensation
      checks, then reuses group `10` controller, feedback, and endpoint checks.
    - Planning failure or missing authorization produces no motion. A successful
      plan executes the same target immediately.
    - Groups `14`, `9`, and `10` remain available separately for inspection.

12. **T3 groups 24-27 and 28: fixed Task 1 stages.**

    - Groups `24/25/26/27` execute Task 1 stages `1/2/3/4`. Coordinates are fixed
      in the script, and selecting a group starts immediately.
    - Every Cartesian point is planned before execution. All Cartesian and gripper
      CSP actions take `5 s`, except descents explicitly marked `10 s`. Each action
      starts immediately after its predecessor.
    - Close/open reuse exact group `15` relative moves
      `-150000/+150000 counts`.
    - Any planning or execution failure stops the stage and suppresses later actions.
    - Group `28` executes all stages in order with no inserted delay.

### Drive 3 Gripper and Custom Counts

After group `6` and group `7`, run group `15` in T3. Enter an ASCII shortcut or any
nonzero signed integer count value:

```text
close or c = exact relative -150000 counts from the current position
open  or o = exact relative +150000 counts from the current position
+2000       = add 2000 counts from the current position
-150000     = subtract 150000 counts from the current position
```

Drive 3 absolute counts use the current Method 37 zero. Numeric group `15` input is
still a relative increment. Close, open, and custom counts use normal
`0x60E0/0x60E1=1000` and a `20000 counts/s`, 50 Hz minimum-jerk CSP trajectory.
There is no contact detection, early stop, or automatic close/hold torque guard.
Drives 0-2 hold their current `/joint_states`. An unreachable exact target fails.

Use group `17` before and after motion to read `absolute_counts`. Group `15` clears
old Cartesian authorization. The next Cartesian motion may use group `23` or
`14 -> 9 -> 10`. Alternating `15 -> 23 -> 15` in one CSP session is supported;
concurrent Drive 3 and Cartesian publishers are forbidden.

### Task 2 Fixed Pick-and-Place

After entering CSP, run group `29` in T3 and enter only start x/y in metres. The
script calculates `r=sqrt(x^2+y^2)`. Every action takes `5 s`:

```text
common start:
  (x,y,0.10) -> (x,y,0.045) -> close -> (x,y,0.10)

0.17 <= r <= 0.20:
  -> (0.1812,-0.0336,0.10) -> (0.1812,-0.0336,0.045)

r < 0.17:
  -> (0.1517,-0.0282,0.10) -> (0.1517,-0.0282,0.045)
  -> (0.1812,-0.0336,0.045)

r > 0.20:
  -> (0.2107,-0.0391,0.10) -> (0.2107,-0.0391,0.045)
  -> (0.1812,-0.0336,0.045)

common finish:
  open -> (0.1812,-0.0336,0.10)
```

Actions run continuously with no inserted wait or confirmation.

### C. Repeated Targets During a Healthy CSP Session

Keep T1 group `4` and T2 group `7` running. Repeat only in T3:

```text
23  (or 14 -> 9 -> 10)
```

Authorization is valid only for the current group `7` session and the exact target.
It expires when group `7` exits, the target changes, or one motion completes.

If IK/planning fails while T1/T2 show no PDO, WKC, or following error and both
controllers remain active, CSP need not restart. Choose another target and run
group `23` again, or repeat `14 -> 9`. Never run group `10` for a failed plan.

### D. Complete Restart After CSP or Controller Failure

For PDO/WKC/following error, `MOTION_RESULT reached=false`, SAFE-OP, inactive
controller, group `7` exit, or T1/T2 failure:

1. Stop targets and support the arm immediately; use emergency stop if needed.
2. If T2 still runs, press `Ctrl-C` and wait for ros2_control to exit.
3. Press `Ctrl-C` in T1 to close the old bridge.
4. Optionally run group `12` in T3 to package logs.
5. Restart the menu script in T1 and T2.
6. Repeat `T1:4 -> T2:6->7 -> T3:8->13->23` from the beginning.

Never start group `7` again on an old T1 bridge. Deferred mapping, Homing state,
and PDO failures belong to that EtherCAT session.

## 0. Safety and Terminal Roles

- T1: Homing bridge, running continuously from Homing until CSP shutdown.
- T2: Homing services, then ros2_control.
- T3: controller/joint checks and trajectories.

Required precautions:

1. Keep emergency stop accessible and support the arm during first tests.
2. Never stop T1 after `home_all`; CSP must reuse the same bridge/master.
3. Never Home, move manually, or start a second bridge during CSP.
4. Never send targets with an inactive controller, WKC/following error, or wrong direction.
5. Support the arm before normal shutdown, which disables voltage.

The default mode is sensor Homing on Drives 0-2, fixed Drive 3 reference motion,
then four-axis CSP. Drive 3 does not search a sensor; it moves `+50000 counts`, uses
Method 37 to set zero, then participates in PDO checks and trajectory hold.
`ignore_spur_gear_in_csp:=true` exists only as an emergency three-axis fallback for
a Drive 3 hardware fault. Normal operation uses the default `false`.

### Drive 2 Internal Limits and Following-Error Test Settings

Drive 2 `statusword=0x3027` is a CSP following error. The previous drive values
`0x6065=32 counts` and `0x6066=48 ms` were too strict for the 196:1 reduction axis.
For the current session, group `4` writes and verifies only on Drive 2:

```text
0x6065 = 25000 counts  (approximately 0.0489 rad)
0x6066 = 250 ms
```

Protection remains active: approximately 2.8 degrees of error for 250 ms still
stops the drive. The code does not write `0x607B`, `0x607D`, or persistent-storage
object `0x1010`. Group `4` logs and group `6` diagnostics record current values.

If `0x607B` reports the full S32 range `[-2147483648,2147483647]`, no internal
position range is active. For a smaller range, compare it with
`CSP_SNAPSHOT D2(target=...)`; do not clear it immediately. Repeated following error
at the current thresholds means the physical error exceeded meaningful protection;
package logs instead of increasing the threshold.

If `0x2324.01` reports both `positive_limit_switch` and `negative_limit_switch`
while the PDO target remains within `0x607D`, inspect mappings before CSP:

```bash
bash ./rascl_debug.sh 18
```

The three axis sensors are Homing references, not bidirectional travel limits.
Group `7` therefore clears and verifies `0x2310:01/:02` after Homing. It does not
alter `0x2310:04`, input polarity, `0x607B/0x607D`, or persistent storage. For a
temporary rollback-only test, set
`RASCL_CLEAR_LIMIT_SWITCH_MAPPINGS_FOR_CSP=false` before group `4`; normal operation
must use `true`.

Group `18` also provides a read-only pre-CSP `0x6065/0x6066` snapshot. If Homing
restored old values such as `16384/48`, do not move. Group `7` must verify the final
CSP values `25000/250` through `CSP_FOLLOWING_ERROR_CONFIGURATION`.

Coordinate convention:

```text
URDF q=[0,0,0,0] TCP                   = [0.32840,-0.00177,0.043001] m
automatic Home q~=[0,+pi/2,+pi/2] TCP = [0.23840,-0.00177,0.293001] m
Drive 3 = +50000 counts after D0-D2 Home, then Method 37 zero
direction D0-D3 = [+1,+1,+1,-1]
nominal D0-D2 home_offset_counts = [0,-802816,-802816]
```

`homing_offsets=[0,0,0,0]` is drive object `0x607C`. Drive 3 Method 37 uses
`0x607C=0` to define the current position as zero. Do not use this object to
compensate URDF/TCP geometry. Drive 2 uses `direction=+1` with
`home_offset_counts=-802816`, so positive planned motion matches positive physical
motion while automatic Home appears as `+pi/2`.

The fixed `tcp_link` is at lowerarm local `[0.170,0,0.0179] m`; physical
`spur_gear_joint` remains at `[0.13916,0,0.0179] m`. Drive 3 and gripper geometry
did not move. IK and TF use the same TCP definition.

## 1. Start the Container

On the Ubuntu host, check Docker:

```bash
docker version
sudo systemctl start docker
```

Enter the repository and start T1 with a rebuilt image only when needed:

```bash
cd ~/RASCL_G8
SOFT_REBUILD=true bash ./rosws.sh
```

When the image is unchanged:

```bash
cd ~/RASCL_G8
bash ./rosws.sh
```

Open two more Ubuntu terminals and run the same command for T2 and T3. In all three
container terminals:

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=88
```

Use the repository menu script:

```bash
cd /root/ws
bash ./rascl_debug.sh
```

Enter a group number, or bypass the menu with, for example,
`bash ./rascl_debug.sh 4`. Short tasks return to the local menu; foreground tasks
occupy the terminal until `Ctrl-C`.

| Group | Terminal | Action |
|---|---|---|
| 1 | T1 | Build and run functional tests |
| 2 / 3 | T1 / T2 | Start and check fake hardware |
| 4 | T1 | Start the only Homing bridge and keep it running |
| 5 | T2 | Home Drives 0-2 individually |
| 6 | T2 | Run verified `home_all` sequence |
| 7 | T2 | Reuse the bridge, start CSP, and keep it running |
| 8 | T3 | Controller and joint-state hold check |
| 9 / 10 | T3 | Plan only / execute after checks |
| 11 / 12 | any | Process check / package complete ROS logs |
| 13 | T3 | Show live model TCP after CSP startup |
| 14 | T3 | Set target TCP and duration without moving |
| 15 | T3 | Close/open or custom nonzero relative Drive 3 counts |
| 16 | T3 | Show latest automatic CSP stall snapshot |
| 17 | T2/T3 | Read current absolute Drive 3 counts |
| 18 | T2 | Read pre-CSP input mappings and Drive 2 protection |
| 19 / 20 / 21 | T2 | Trim Drive 0 / 1 / 2 after `home_all`, before CSP |
| 22 | T2 | Set current Drive 0-2 pose as session Home |
| 23 | T3 | Enter, plan, and immediately execute a target |
| 24 / 25 / 26 / 27 | T3 | One-click Task 1 stages 1 / 2 / 3 / 4 |
| 28 | T3 | Execute all four Task 1 stages |
| 29 | T3 | Task 2 fixed-target pick-and-place from entered XY |

Normal order is
`T1:4 -> T2:6 -> (19/20/21->22 during calibration) -> 7 -> T3:8->13->28`.

## 2. Build and Software Tests

Run group `1` in T1. It verifies the `rascl_wp3_ss26_group8` package and `wp3_tsk1`
executable. Groups `3/9/10` are blocked without a valid installation.

With no old process running:

```bash
ps -ef | grep -E "ros2_control_node|rascl_faulhaber_bridge|wp3_tsk1" | grep -v grep
ss -ltnp | grep 15001
```

After confirming no output, build:

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
rm -rf build install log
colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON
source install/local_setup.bash
```

Reload T2 and T3:

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=88
```

### 2.1 Fast Rebuild After TCP/URDF-Only Changes

Stop the T1 bridge and T2 ros2_control first, then run:

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  rascl_description rascl_wp3_ss26_group8
source install/local_setup.bash
```

Reload T2/T3 with the preceding environment commands. A TCP/URDF change invalidates
the old robot description, TF, generated trajectory, and group `9` authorization.
Repeat `T1:4 -> T2:6->7 -> T3:8->13`, verify the new TCP, then use group `23` or
`14->9->10`. Never reuse an old CSV or authorization.

Run focused tests:

```bash
python3 -m pytest \
  src/rascl_wp3_ss26_group8/test/test_kinematics_calibration.py -q
ctest --test-dir build/rascl_description \
  -R '^test_robot_description_parameter$' \
  --output-on-failure
ctest --test-dir build/rascl_hardware_interface \
  -R '^(test_generic_system|test_faulhaber_bridge)$' \
  --output-on-failure
```

All four targets must pass. Full `colcon test` also runs style checks such as
`clang_format` and `cpplint`; use CTest functional-target results to assess hardware
readiness. Style or copyright-header failures do not by themselves prove a
functional failure.

Check bridge execute permission:

```bash
ls -l install/rascl_hardware_interface/lib/rascl_hardware_interface/rascl_faulhaber_bridge.py
```

## 3. Fake Hardware

Run group `2` in T1 and group `3` in T2, or use these commands.

T1:

```bash
ros2 launch rascl_description ros2_control.launch.py use_fake_hardware:=true
```

T2:

```bash
ros2 control list_controllers
ros2 topic echo --once /joint_states

ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
  -p target_x:=0.25 -p target_y:=0.00 -p target_z:=0.08 \
  -p duration:=4.0 -p rate_hz:=50.0 -p execute:=false

head -n 5 /tmp/rascl_wp3_tsk1_last_trajectory.csv
tail -n 5 /tmp/rascl_wp3_tsk1_last_trajectory.csv

ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
  -p target_x:=0.25 -p target_y:=0.00 -p target_z:=0.08 \
  -p duration:=4.0 -p rate_hz:=50.0 -p execute:=true
```

Controllers must be active, IK must succeed, and the CSV must contain no `nan`.
Press `Ctrl-C` in T1 when finished and wait for the shell prompt.

## 4. Physical-Hardware Preparation

Enable the EtherCAT interface on the Ubuntu host, not inside Docker:

```bash
ip link show enx3c18a0256deb
sudo ip link set enx3c18a0256deb up
ip link show enx3c18a0256deb
```

Reset the ROS graph from T3:

```bash
ros2 daemon stop
ros2 daemon start
```

Confirm sensors, directions, emergency stop, mechanical support, and free motion
space before applying power.

## 5. Homing: Never Stop T1

Run group `4` in T1. For first-time per-axis validation, run group `5` in T2;
after validation, use group `6`.

Equivalent T1 launch command:

```bash
ros2 launch rascl_description homing.launch.py \
  interface:=enx3c18a0256deb \
  homing_interval_max_travel_drive0_counts:=100000 \
  homing_interval_max_travel_drive1_counts:=300000 \
  homing_interval_max_travel_drive2_counts:=300000 \
  homing_interval_timeout_s:=120.0 \
  csp_torque_limit_per_mille:=1000 \
  spur_close_torque_limit_per_mille:=300 \
  spur_hold_torque_limit_per_mille:=100 \
  clear_limit_switch_mappings_for_csp:=true \
  drive2_following_error_window_counts:=25000 \
  drive2_following_error_timeout_ms:=250 \
  csp_stall_error_counts:=25000 \
  csp_stall_progress_counts:=100 \
  csp_stall_timeout_ms:=500 \
  spur_gear_reference_delta_counts:=50000 \
  spur_gear_reference_timeout_s:=30.0 \
  spur_gear_reference_tolerance_counts:=100 \
  spur_gear_reference_profile_velocity:=3000 \
  spur_gear_reference_profile_acceleration:=1000 \
  spur_gear_reference_profile_deceleration:=1000 \
  spur_gear_reference_following_error_confirm_s:=0.30 \
  skip_spur_gear_homing:=true
```

Drive 3 reference motion takes roughly 20 seconds. A following-error state shorter
than `0.30 s` is logged while recovery is awaited; a persistent state stops Drive 3
and fails `home_all`. Never hide a mechanical stall by increasing the confirmation
time or bypassing failure.

Expected T1 output:

```text
Homing-to-CSP session starts SDO-only in PRE-OP
PDO mapping is deferred until home_all succeeds
Drive 0-2 Homing uses the centre of the reference-input interval
TCP bridge listening on 127.0.0.1:15001
```

Read inputs from T2:

```bash
bash ./rascl_debug.sh 18
```

After first assembly or mechanical changes, Home Drives 0-2 individually:

```bash
ros2 param set /rascl_faulhaber_bridge test_drive_index 0
ros2 service call /rascl_faulhaber_bridge/home_one std_srvs/srv/Trigger "{}"

ros2 param set /rascl_faulhaber_bridge test_drive_index 1
ros2 service call /rascl_faulhaber_bridge/home_one std_srvs/srv/Trigger "{}"

ros2 param set /rascl_faulhaber_bridge test_drive_index 2
ros2 service call /rascl_faulhaber_bridge/home_one std_srvs/srv/Trigger "{}"
```

Per-axis sequence:

```text
native first-edge search at 1000
-> cross the active interval in the same direction at 200 with sinusoidal profile
-> second edge -> Halt
-> return to (entry+exit)/2 at 200
-> Method 37 sets the midpoint to 0 counts
```

`entry/exit` are edge counts in the same drive coordinate system. `width` is the
interval width, `midpoint` is the target, `reached` must be within `500 counts`, and
`abs(zero)` must be at most `500 counts`. With `0x607C=0`, the native method defines
the latched first edge as `entry=0`; the deceleration stop after that edge is not
used in the midpoint calculation.

Second-edge travel guards are D0/D1/D2 `100000/300000/300000 counts`. Measured D0
width was about `59657 counts`; D1 required more than the former 100000-count guard.
The native first-edge search still uses `motion_timeout_s=8 s`; bounded traversal
and midpoint return each use `120 s`. A missing second edge, fault/following error,
or missed midpoint leaves the drive un-Homed and blocks CSP.

After Drive 2 succeeds, the bridge automatically references and zeroes Drive 3.
Do not call `home_one` for Drive 3.

For a verified mechanism, use one `home_all` call and diagnostics:

```bash
ros2 service call /rascl_faulhaber_bridge/home_all \
  std_srvs/srv/Trigger "{}"

ros2 service call /rascl_faulhaber_bridge/read_spur_gear_counts \
  std_srvs/srv/Trigger "{}"

ros2 service call /rascl_faulhaber_bridge/read_drive2_diagnostics \
  std_srvs/srv/Trigger "{}"
```

Required output:

```text
success=True
Homing completed for required drives; CSP handoff armed ...
drive0_interval(entry=...,exit=...,width=...,midpoint=...,reached=...,zero=...,zero_tolerance=500)
drive1_interval(entry=...,exit=...,width=...,midpoint=...,reached=...,zero=...,zero_tolerance=500)
drive2_interval(entry=...,exit=...,width=...,midpoint=...,reached=...,zero=...,zero_tolerance=500)
drive3_reference(...zero=0,method=37)
Drive 3: absolute_counts=0 ... reference_complete=true
```

Do not press `Ctrl-C`, call `disable_all`, or stop T1.

### Fine-Trim Drives 0-2 After Homing

Use only to measure the difference from sensor midpoint to physical Home, after
group `6` and before group `7`:

```bash
bash ./rascl_debug.sh 19   # Drive 0
bash ./rascl_debug.sh 20   # Drive 1
bash ./rascl_debug.sh 21   # Drive 2
```

Enter a nonzero signed integer such as `500` or `-1200`. Motion starts from live
encoder feedback and uses relative Profile Position at `1000 counts/s`. Calls may
be repeated. Example output:

```text
Drive 1 Home fine adjustment completed:
source=500, delta=-1200, target=-700, actual=-692, target_error=+8,
correction_from_homed_zero=-692 counts
```

Record only final `correction_from_homed_zero`. This function does not apply
Method 37 and therefore does not change encoder zero. It rejects operation before
all Homing/reference work is complete or after PDO/CSP preparation. Before trim,
volatile `0x2310:01/:02` mappings are cleared to prevent stale sensor-region limit
flags from blocking Profile Position. Homing reference, polarity, and
`0x607B/0x607D` remain unchanged.

At the verified physical Home:

```bash
bash ./rascl_debug.sh 22
```

Required result:

```text
drive0_before=... drive1_before=... drive2_before=...
drive0_after=...  drive1_after=...  drive2_after=...
Drive 3 unchanged
```

Save the `before` values. Each `after` should be within 100 counts of zero. Group
`22` establishes only a session zero. To make future group `6` runs reach this
physical pose, add the measured correction motion after the interval midpoint and
before Method 37. Changing `home_offset_counts` alone changes encoder-to-URDF
mapping but does not move the physical Homing endpoint.

## 6. Optional One-Time home_offset_counts Calibration

Nominal values assume exactly 90 degrees at two joints. This operation requires
reliable support and actively disables drives. Afterward, repeat Homing from section
5; never enter CSP directly.

After `home_all`, in T2:

```bash
ros2 service call /rascl_faulhaber_bridge/disable_all \
  std_srvs/srv/Trigger "{}"
```

Support the arm, manually move to the previously verified physical URDF
`q=[0,0,0,0]` pose, then run:

```bash
python3 -c "import socket; s=socket.create_connection(('127.0.0.1',15001),2); s.sendall(b'GET_ALL\n'); print(s.recv(4096).decode().strip()); s.close()"
```

The container may not include `nc`; this Python standard-library command requires
no package installation. Run it before CSP starts. During CSP, ros2_control owns
the bridge's only TCP client connection and an external `GET_ALL` times out; use
`/joint_states` then.

Response format:

```text
OK <D0_raw> <D0_status> <D1_raw> <D1_status> \
   <D2_raw> <D2_status> <D3_raw> <D3_status>
```

Drive 3 raw is absolute counts relative to its Method 37 session zero. After CSP
starts, use group `17` instead of opening another TCP client. Record values, support
the arm, stop T1, return the mechanism to a verified Homing start region, and
repeat section 5.

## 7. Enter CSP in the Same EtherCAT Session

Keep T1 group `4` running and select group `7` in T2. Equivalent command:

```bash
ros2 launch rascl_description ros2_control.launch.py \
  interface:=enx3c18a0256deb \
  use_fake_hardware:=false \
  start_bridge:=false \
  lowerarm_direction:=1 \
  spur_gear_direction:=-1 \
  gripper_counts_per_revolution:=1323008 \
  shoulder_home_offset_counts:=0 \
  upperarm_home_offset_counts:=-802816 \
  lowerarm_home_offset_counts:=-802816 \
  spur_gear_home_offset_counts:=0
```

Expected T1 output:

```text
CSP interpolation 0x2332.00 configured to 200 x 100 us (20000000 ns PDO cycle)
assigning factory Position PDOs Rx=0x1601, Tx=0x1A01
Deferred process image mapped
Master reached OP state
Homing-to-CSP handoff completed without Shutdown/Disable controlwords
SPUR_REFERENCE drive3_reference(...delta=50000,...zero=0,method=37)
```

Expected T2 output:

```text
Activated real RASCL hardware in csp mode
```

For `not all required drives were homed`, lost Operation Enabled, WKC, following
error, or SAFE-OP + Error, do not retry motion. Follow section 10.

## 8. CSP Hold Check and TCP Inspection

Run group `8` in T3. Group `13` reads live `base_link -> tcp_link`; Translation is
in metres. Nominal Home is:

```text
[0.23840,-0.00177,0.293001] m
```

This is model FK, not an external measurement. Current `tcp_link` is lowerarm local
`[0.170,0,0.0179] m`; physical `spur_gear_joint` remains
`[0.13916,0,0.0179] m`. Measure the same ideal TCP reference at Home and other
poses. Current `base_link -> shoulder_joint` uses uncompensated CAD alignment
`[0,0,0.057441] m`. Diagnose pose-dependent errors through link, joint-origin, or
encoder-zero calibration; do not hide them with another shoulder translation.

Manual checks:

```bash
ros2 control list_controllers
ros2 topic echo --once /joint_states
ros2 topic hz /joint_states
```

Required:

```text
joint_state_broadcaster active
rascl_position_controller active
first three positions ~= [0,+1.5708,+1.5708]
```

Drive 3 should be near its Method 37 `0 counts`; group `17` provides exact readback.
Hold at least 10 seconds without targets. Physical Drives 0-2 and RViz must agree.
Stop only `topic hz` with `Ctrl-C` in T3.

## 9. Small Minimum-Jerk Trajectory

Use group `23` for normal plan-and-execute operation. For separate inspection, use
groups `14`, `9`, and `10`. Every target requires a new plan.

Plan-only example:

```bash
ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
  -p target_x:=0.2108 -p target_y:=-0.00177 -p target_z:=0.2913 \
  -p duration:=12.0 -p rate_hz:=50.0 -p execute:=false

head -n 5 /tmp/rascl_wp3_tsk1_last_trajectory.csv
tail -n 5 /tmp/rascl_wp3_tsk1_last_trajectory.csv
```

With joints near Home, expected IK is approximately `[0,1.5527,1.5550]`; the CSV
must contain no `nan`. After checking direction and space:

```bash
ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
  -p target_x:=0.2108 -p target_y:=-0.00177 -p target_z:=0.2913 \
  -p duration:=12.0 -p rate_hz:=50.0 -p execute:=true
```

Success requires `MOTION_RESULT reached=true`. Default acceptance is `0.03 rad`
per axis and `0.01 m` TCP error. An exceeded threshold returns nonzero and group
`10` reads the stall snapshot. Old target `[0.295,0,0.048]` is near URDF zero and
must not be the first move after automatic Home.

## 10. Faults and Shutdown

### Normal Shutdown

1. Stop trajectory publishers and confirm no new command.
2. Support the arm securely.
3. Press `Ctrl-C` in T2 and wait for ros2_control to exit and disable the drives.
4. Press `Ctrl-C` in T1 to close the EtherCAT bridge.
5. Check from T3:

```bash
ps -ef | grep -E "ros2_control_node|rascl_faulhaber_bridge|wp3_tsk1" | grep -v grep
ss -ltnp | grep 15001
```

There should be no output. Group `11` performs the same residual-process check.
Group `12` creates `/root/ws/ros_logs_<timestamp>.tar.gz` for direct submission.

### Critical Errors

- `not all required drives were homed`: stop T2 ros2_control, keep T1, complete
  missing Homing or repeat `home_all`, then enter CSP again.
- `lost Operation Enabled while selecting CSP`: support the arm or use emergency
  stop. Do not retry in a loop.
- Following error, WKC, or SAFE-OP: support the arm immediately and stop targets.
  The ros2_control log includes `CSP_SNAPSHOT` with PDO target, actual, error,
  status, and mode for every drive. The failed drive then logs read-only
  `DRIVE_DIAG`, including `0x2324.01`, `0x1001`, `0x1003`, input mappings,
  position/velocity, torque demand/actual, current, following error, current limits,
  and position-loop `0x2348.01`. Individual unavailable SDO reads do not erase the
  original PDO fault. `TORQUE_SNAPSHOT` then records all four drives. Stop T1/T2,
  run group `12`, and submit the archive.
- Stall without following error: the bridge detects error of at least
  `25000 counts` with less than `100 counts` progress over `500 ms`. It logs
  `CSP_STALL_DETECTED` and collects a rate-limited read-only
  `CSP_STALL_SNAPSHOT` without stopping the 50 Hz PDO loop. The snapshot includes
  cause, statusword, internal limits, `0x2324.01`, position demand/actual, velocity,
  torque, current, limits, `0x2329`, voltage thresholds, and measured voltages.
  Group `10` reads it automatically; otherwise run:

  ```bash
  bash ./rascl_debug.sh 16
  bash ./rascl_debug.sh 12
  ```

  Explicit flags include `TORQUE_LIMIT_REPORTED`,
  `VOLTAGE_OR_SUPPLY_LIMIT_REPORTED`, and `POSITION_OR_LIMIT_SWITCH_REPORTED`.
  `POSITION_LOOP_STALLED_WITHOUT_LIMIT_FLAG` means a confirmed stall without one
  drive-reported limiting cause; use torque/current demand and feedback to
  distinguish load, brake, and position-loop issues.

For inactive controllers:

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

For an old bridge or occupied port:

```bash
ps -ef | grep rascl_faulhaber_bridge | grep -v grep
ss -ltnp | grep 15001
```

For a missing or non-executable bridge:

```bash
ls -l install/rascl_hardware_interface/lib/rascl_hardware_interface/
colcon build --symlink-install --packages-select rascl_hardware_interface
source install/local_setup.bash
```

For IK failure, keep `execute:=false`, choose a target near the current TCP, and do
not execute the rejected target.

## 11. Acceptance Criteria

1. Software tests and fake hardware pass.
2. Drive 0-2 `home_one` or `home_all` succeeds with two edges, midpoint, and
   `abs(zero)<=500`; Drive 3 completes `+50000 counts + Method 37`, and group `17`
   reads near `0`.
3. The Homing bridge remains alive while deferred PDO mapping reaches OP/CSP.
4. Drives 0-2 hand off continuously; Drive 3 enters CSP after its fixed reference
   and Method 37 zeroing.
5. First-three-axis `/joint_states`, physical robot, and RViz agree and hold for 10
   seconds; Drive 3 has no PDO fault.
6. The 20 ms PDO loop has no WKC/following error, and motion ends with
   `MOTION_RESULT reached=true`.
7. Group `15` custom relative counts work during CSP, group `17` reads absolute
   counts, and later Task 1 motion holds the Drive 3 angle.
8. A 12-second minimum-jerk trajectory near Home succeeds.

## 12. Parameter Reference

| Item | Value |
|---|---|
| Drive / Joint | `0 shoulder`, `1 upperarm`, `2 lowerarm`, `3 spur_gear` |
| Drive 3 strategy | Skip sensor search; after D0-D2 move `+50000 counts`, then Method 37 zero |
| Homing method | D0-D2 first edge `[28,28,24]`; interval midpoint Method `37`; D3 current point Method `37` |
| Reference input | `[2,2,2,1]` |
| Drive `0x607C` | `[0,0,0,0]`; D3 Method 37 defines current position with zero offset |
| D0-D2 interval traversal | first edge at `1000`; second edge and midpoint at `200` with sinusoidal profile; guards `[100000,300000,300000] counts`; `120 s` per bounded stage; `500 counts` midpoint/zero tolerance |
| Nominal ROS direction | `[+1,+1,+1,-1]` |
| Nominal ROS offset | `[0,-802816,-802816,0]` |
| CSP mode | `8` |
| PDO cycle | `20 ms / 50 Hz` |
| CSP interpolation `0x2332:00` | `200` (`20 ms / 100 us`), written and read back by the bridge |
| RxPDO2 | `0x6040 + 0x607A`, 6 bytes |
| TxPDO2 | `0x6041 + 0x6064`, 6 bytes |
| TCP bridge | `127.0.0.1:15001` |
| ROS domain | `88` |
