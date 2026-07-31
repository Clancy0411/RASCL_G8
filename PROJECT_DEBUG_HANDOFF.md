# RASCL WP3 Project Debugging Handoff

> Use this document to continue debugging on another machine or in another Codex
> task. Unless stated otherwise, every path is relative to the Git repository root.
> Before changing anything, inspect the current Git state and preserve existing work.

## 1. Git Baseline

```text
branch: main
upstream commit before the Drive 3 direction reversal: 2d5c6d5
summary: gripper fixed-position calibration
remote: https://github.com/Clancy0411/RASCL_G8.git
```

Important historical baselines:

```text
7d9b55f5f33b8102b70863c0d4707d7ba6dded58
summary: coordinate-system changes; comparison point before the teammate update

214477ef7c9f4cca7f52b41106f4863b9f68442b
tag: hardware-verified
summary: reduced vibration
```

Commit `214477e` was verified on hardware to position Drives 0-3 at commanded
Cartesian targets and includes the CSP interpolation fix `0x2332:00=200`. Later
versions add Drive 3 open/close and custom-count control, Drive 2 diagnostics and
session parameters, and the current fixed TCP.

Run these commands first in a new environment:

```bash
git status --short
git branch --show-current
git rev-parse HEAD
```

Do not run `git reset --hard`, perform a bulk rollback, or overwrite the working
tree without explicit authorization. Preserve intentional deletions.

## 2. Project Goal and Current Priority

This is the RASCL Work Package 3 project. The final task is to pick up blocks with
the arm and place them at specified positions. The course requirements are in:

```text
Docs/RASCL_WP3_SS26_Tasksheet.pdf
Docs/RASCL_WP3_Intro.pdf
```

The required low-level motion mode is:

```text
Cyclic Synchronous Position (CSP) + EtherCAT PDO
```

CSP/PDO, automatic Homing, Cartesian IK, minimum-jerk trajectories, and Drive 3
CSP control are implemented. Current priorities are:

1. Rebuild and start the physical robot on a new machine.
2. Verify the current ideal TCP, measured 170 mm along lowerarm +X, while retaining
   group `15` shortcuts and custom relative-count control.
3. Compare the model TCP with an external measurement of the same physical point
   at Home and at additional poses.
4. If systematic error remains, use multi-pose results to distinguish a fixed TCP
   offset, base-frame error, link-geometry error, and encoder-zero error.

A repeated Drive 2 fault on 2026-07-24 showed `statusword=0x3827`, while
`0x2324.01` reported `following_error`, `positive_limit_switch`, and
`negative_limit_switch`. The current correction targets stale limit-input mappings
at `0x2310:01/:02`. Do not blindly increase torque, following-error, or position-loop
parameters.

## 3. Authoritative Documentation and Code

### 3.1 Physical-Hardware Debug Guide

```text
WP3_Task1_MinJerk_Debug_Guide.md
```

This is the single authoritative physical-hardware guide. The obsolete quick guide
was deleted; do not create a duplicate. Update this file whenever commands,
parameters, behavior, or workflow change. Keep it concise, accurate, and complete.

### 3.2 Hardware and Protocol Manuals

The top-level task PDFs are under `Docs/`. FAULHABER EtherCAT, commutation,
hardware, drive-function, and statusword manuals are in the hardware-manual
subdirectory under `Docs/`. This includes the ASCII-named files:

```text
EtherCat.pdf
Commution Manual.pdf
HardwareInformation.pdf
CANopenHelper_statusword.html
```

Consult these manuals first for FAULHABER object-dictionary entries, CiA-402,
Homing, PDO, torque, current, voltage, and statusword behavior.

### 3.3 Software Packages

Robot description, URDF, launch, and controllers:

```text
src/rascl_description/urdf/rascl.urdf
src/rascl_description/launch/homing.launch.py
src/rascl_description/launch/ros2_control.launch.py
src/rascl_description/config/controllers_csp.yaml
src/rascl_description/README.md
```

EtherCAT bridge and ros2_control hardware interface:

```text
src/rascl_hardware_interface/scripts/rascl_faulhaber_bridge.py
src/rascl_hardware_interface/src/rascl_hardware_interface.cpp
src/rascl_hardware_interface/test/test_faulhaber_bridge.py
src/rascl_hardware_interface/test/test_generic_system.cpp
src/rascl_hardware_interface/README.md
```

IK, trajectory generation, and WP3 application:

```text
src/rascl_wp3_ss26_group8/rascl_wp3_ss26_group8/kinematics.py
src/rascl_wp3_ss26_group8/rascl_wp3_ss26_group8/trajectory.py
src/rascl_wp3_ss26_group8/rascl_wp3_ss26_group8/wp3_tsk1.py
src/rascl_wp3_ss26_group8/launch/wp3_tsk1.launch.py
src/rascl_wp3_ss26_group8/docs/coordinate_convention.md
src/rascl_wp3_ss26_group8/README.md
```

Supporting files:

```text
rascl_debug.sh
rosws.sh
Log.md
```

`Log.md` contains historical work and may contain obsolete values. Prefer the
current code, the latest dated record, and the current debug guide.

## 4. Software Architecture

```text
rascl_position_controller
        | ROS 2 position command
C++ ros2_control hardware interface
        | local TCP 127.0.0.1:15001
Python pysoem EtherCAT bridge
        | EtherCAT PDO
FAULHABER Drives 0-3
```

The Python bridge exclusively owns the EtherCAT master. After Homing, CSP must
reuse the same bridge and master; never start a second bridge concurrently.

The WP3 application currently:

- solves numerical IK for TCP XYZ using the first three arm joints;
- does not constrain end-effector orientation;
- generates a 50 Hz joint-space minimum-jerk trajectory;
- publishes through the four-joint position controller;
- holds the current Drive 3 position during Cartesian motion;
- checks joint feedback and TCP error at the endpoint; and
- considers motion successful only when `MOTION_RESULT reached=true`.

## 5. Drive Mapping, Direction, and Software Zero

```text
Drive 0 = shoulder_joint
Drive 1 = upperarm_joint
Drive 2 = lowerarm_joint
Drive 3 = spur_gear_joint / gripper
```

Current mapping:

```text
direction = [+1,+1,+1,-1]
home_offset_counts = [0,-802816,-802816,0]
```

Conversion:

```text
q = direction * (raw_counts - home_offset_counts) / counts_per_rad
```

Drives 0-2:

```text
counts_per_revolution = 3211264
gear_ratio = 196
encoder_cpr = 4096
counts_per_rad ~= 511088.539
```

Drive 3:

```text
counts_per_revolution = 1323008
gear_ratio = 323
encoder_cpr = 4096
```

An experimental upperarm direction reversal was reverted by commit `e5ef61d`.
Both upperarm and lowerarm currently use `+1`. Do not flip a direction based only
on visual motion; compare raw counts, joint state, planned target, and physical
motion first.

An old comment near the beginning of `rascl.urdf` may mention the previous Drive 2
opposite-sign/positive-offset convention. The runtime values
`direction=+1` and `home_offset_counts=-802816` are authoritative.

Current URDF limits:

```text
shoulder_joint  = [-pi/2,+pi/2]
upperarm_joint  = [-pi,+pi]
lowerarm_joint  = [-pi,+pi]
spur_gear_joint = [-2*pi,+2*pi] = [-6.283185307,+6.283185307]
```

## 6. Automatic Homing and the Drive 3 Session Zero

Current sequence:

```text
Drives 0-2 use their native method to find the first reference-input edge.
Each drive continues across the active reference interval and records the second edge.
Each drive returns with a low-speed sinusoidal profile to (entry+exit)/2.
Method 37 sets that midpoint to 0 counts.
After Drives 0-2 arrive, Drive 3 moves +50000 counts from its live position.
Method 37 then sets the reached Drive 3 position to 0 counts.
All four drives participate in the following CSP/PDO session.
```

Parameters:

```text
skip_spur_gear_homing = true
ignore_spur_gear_in_csp = false
homing_methods = [28,28,24,24]
reference_inputs = [2,2,2,1]
homing_interval_max_travel_drive0_counts = 100000
homing_interval_max_travel_drive1_counts = 300000
homing_interval_max_travel_drive2_counts = 300000
homing_interval_timeout_s = 120.0
homing_interval_poll_s = 0.01
homing_midpoint_tolerance_counts = 500
second-edge traversal and midpoint return speed = homing_zero_speeds = [200,200,200]
second-edge traversal and midpoint return profile = 0x6086:00 = 1
spur_gear_reference_delta_counts = +50000
spur_gear_reference_timeout_s = 30.0
spur_gear_reference_tolerance_counts = 100
spur_gear_reference_profile_velocity = 3000 counts/s
spur_gear_reference_profile_acceleration = 1000
spur_gear_reference_profile_deceleration = 1000
spur_gear_reference_following_error_confirm_s = 0.30
drive 0x607C = [0,0,0,0]
```

`skip_spur_gear_homing=true` means only that Drive 3 does not search for a sensor
reference. It must still complete the fixed relative move and Method 37 zeroing.
The reference move must finish within 100 counts of the `+50000 counts` endpoint;
the zero readback must then be near `0 counts`. Otherwise, `home_all` fails and CSP
handoff is rejected.

The Drive 3 reference velocity was reduced from `10000` to `3000 counts/s`, and
acceleration/deceleration from `10000` to `1000`, to reduce instantaneous Profile
Position lag. A single following-error sample no longer aborts the sequence; the
state must persist for `0.30 s`. A persistent fault or timeout disables Drive 3
before returning failure. Never bypass that failure to force Method 37.

Nominal joint state after automatic Home:

```text
[shoulder,upperarm,lowerarm,spur] ~= [0,+pi/2,+pi/2,0]
```

Required invariants:

1. T1 starts the only Homing bridge. Each of Drives 0-2 must return
   `driveN_interval(entry,exit,width,midpoint,reached,zero,zero_tolerance=500)`,
   and `abs(zero)<=500` must hold.
2. Do not stop T1 after `home_all` succeeds.
3. CSP ros2_control in T2 must reuse the T1 bridge.
4. Do not close and recreate the EtherCAT master between Home and CSP.
5. Do not Home again while CSP is active.
6. Stopping ros2_control disables voltage; support the arm before shutdown.
7. Drive 3 skips sensor Homing but must not bypass `+50000 counts -> Method 37`.
8. `0x607C=0` is only the Drive 3 Method 37 zero definition. Do not use it to
   compensate URDF/TCP geometry.

## 7. CSP/PDO Configuration

```text
CSP mode = 8
PDO cycle = 20 ms / 50 Hz
PDO timeout = 5000 us
DC sync = false
```

PDO mapping:

```text
RxPDO2 = 0x6040 Controlword + 0x607A Target position, 6 bytes
TxPDO2 = 0x6041 Statusword + 0x6064 Actual position, 6 bytes
```

Before Drives 0-3 enter CSP, write and read back:

```text
0x2332:00 = 200
```

This is required because `20 ms / 100 us = 200`. It prevents each 20 ms target
from being interpreted as a roughly 100 us step. Do not remove this correction or
restore the default value `1`.

The bridge runs an independent continuous PDO loop. ROS read/write calls and
occasional TCP requests are not sufficient to keep process data alive.

## 8. Drive 3 / Gripper

Drive 3 enters CSP after its session zero is established. Group `17` reads current
absolute counts at any time. Group `15` accepts shortcuts or any nonzero signed
relative count value:

```text
close or c = exact relative -150000 counts
open  or o = exact relative +150000 counts
+2000       = add 2000 counts from the current position
-150000     = subtract 150000 counts from the current position
```

Group `15` input is never an absolute target. Use group `17` before and after a
move to inspect `absolute_counts` relative to the current Method 37 zero. This can
be used to determine physical open and closed positions experimentally.

Close, open, and custom-count commands are exact relative motions. Before motion,
the script restores `0x60E0/0x60E1=1000`, then publishes a 50 Hz minimum-jerk CSP
trajectory at `20000 counts/s`. Contact is not detected, motion is not stopped
early, and the close/hold torque guard is not called automatically.

Drive 3 limits in the URDF, ros2_control, kinematics, and script preflight all use
`[-2*pi,+2*pi]`. This does not modify drive objects `0x607B/0x607D`.

```text
close speed: 20000 counts/s
open/custom-count speed: 20000 counts/s
```

Duration is calculated from counts; `150000 counts` takes about 7.5 seconds.
Group `15` can alternate with Cartesian trajectories in the same CSP session, but
the two motions must never run concurrently. Both preflight and the motion node
use a 5-second timeout for complete `/joint_states` data. Node failures emit
`SPUR_TRACE failed` for later analysis through group `12`.

## 9. Drive 2 Session Parameters and Diagnostics

Drive 2 previously reported following error with `statusword=0x3027`. Current
session-only settings are:

```text
0x6065 = 25000 counts
0x6066 = 250 ms
```

This does not disable following-error protection.

The code reads but does not write:

```text
0x607B position range
0x607D software position limit
```

One physical readback was:

```text
0x607B = [-2147483648,2147483647]
0x607D = [-802816,802816]
```

At CSP handoff, Drives 0-3 use:

```text
0x60E0 = 1000
0x60E1 = 1000
```

The original Drive 2/3 peak currents `0x2329:03=220/81 mA` produced read-only
effective maximum torque values near `0x6072=200/150`. After Homing succeeds and
before CSP entry, the code changes only the current session:

```text
Drive 2 0x2329:03: 220 -> 1100 mA
Drive 3 0x2329:03:  81 ->  540 mA
```

Required readback:

```text
0x6072 >= 1000
0x60E0 = 1000
0x60E1 = 1000
```

No parameter is stored persistently. Drive 0/1 motor-current parameters are not
changed.

The 2026-07-24 log showed Drive 2 stopping while its PDO target was still inside
the `0x607D` range:

```text
statusword=0x3827
0x2324.01=0x070010FB
flags=following_error,positive_limit_switch,negative_limit_switch
target=106497 actual=85204 error=21293 velocity=0
```

Torque, current, and voltage were not saturated. After Homing and before CSP, the
code now reads and clears the Drive 0-3 lower/upper limit-input mappings at
`0x2310:01/:02`, then verifies the readback. It leaves the Homing reference at
`0x2310:04`, polarity at `0x2310:10`, and `0x607B/0x607D` unchanged. It does not
invoke persistent storage through `0x1010`. T1 should log
`CSP_LIMIT_SWITCH_CONFIGURATION`; group `18` reads the pre-CSP mapping.

Physical group `18` output confirmed `lower=0x01, upper=0x04` on Drives 0-2 and a
simultaneous Drive 1 `positive_limit_switch`. The same read also found Drive 2
`0x6065/0x6066=16384/48` after Homing instead of `25000/250`. CSP handoff therefore
logs `CSP_FOLLOWING_ERROR_CONFIGURATION`, rewrites the target values, and verifies
them. CSP entry is blocked on failed verification.

Git history shows that the Homing code's exclusive use of `0x2310:04` dates back
to commit `d56d695`. No `0x2310` or `REFERENCE_SWITCH_INPUT` change exists between
the verified `214477ef` baseline and the pre-fix HEAD. Teammate commit `4708444`
changed only Drive 3 reference-motion parameters and failure handling. The mapping
issue was an older latent configuration gap, possibly exposed by the newer path;
it was not introduced directly by that teammate change.

## 10. TCP Calibration History and Current 170 mm Ideal TCP

Before commit `7d9b55f`, at the same automatic-Home pose:

```text
model TCP    = (20.8,0,29.2) cm = (0.208,0,0.292) m
external TCP = (18.5,0,33.5) cm = (0.185,0,0.335) m
```

Single-pose correction:

```text
actual - model = (-0.023,0,+0.043) m
```

The old TCP was the `spur_gear_joint` origin:

```text
lowerarm local xyz = [0.13916,0,0.0179] m
```

The first fixed frame, independent of gripper motion, was:

```text
joint = tcp_fixed_joint
parent = lowerarm
child = tcp_link
xyz = [0.11616,0.043,0.0179] m
rpy = [0,-pi/2,0]
```

A second single-pose calibration used program target
`XYZ=[0.16,-0.16,0.05] m` and external measurement
`YXZ=[0.14,-0.16,0.05] m`. Under the project's physical-axis convention, the
corresponding numerical `base_link` error was `[-0.020,0,0] m`. Transformed into
`lowerarm`, the gear-surface reference was:

```text
xyz = [0.11478978,0.02881369,0.03193108] m
```

The TCP was temporarily extended by `0.020 m` along the gripper direction
(`lowerarm +X`) to represent the midpoint between the jaws. That local extension
was later removed after physical testing. During that stage the fixed TCP was:

```text
xyz = [0.13478978,0.02881369,0.03193108] m
```

On 2026-07-24, calibration against the same physical reference temporarily restored
`tcp_link` to the original `spur_gear_joint` center:

```text
xyz = [0.13916,0,0.0179] m
rpy = [0,-pi/2,0]
```

On 2026-07-27, hardware measurement showed 170 mm along the link from the reference
screw hole at the end of lowerarm opposite the TCP to the ideal TCP. The independent
17.9 mm vertical offset from the drawing remains. The current planned TCP is:

```text
tcp_fixed_joint xyz = [0.170,0,0.0179] m
spur_gear_joint xyz = [0.13916,0,0.0179] m  (physical axis unchanged)
separation along lowerarm +X = 0.03084 m
```

Only the massless `tcp_link` moved. The spur gear, gripper mesh, Drive 3, Homing,
CSP, and count mapping did not change.

A project-side physical-X compensation was temporarily added on 2026-07-23.
Because external readings were reported in `Y/X/Z` order, physical `+X` corresponded
to numerical `base_link +Y`. To correct a fixed physical-X shortfall of `0.020 m`,
the model transform was set to:

```text
base_link -> shoulder_joint xyz = [0,-0.020,0.057441] m
```

This was a model-coordinate correction, not a local TCP extension.

A later single-point observation associated model XY `[0.12,0.12] m` with physical
XY `[0.16,0.16] m`. Treating that as a fixed translation temporarily produced:

```text
base_link -> shoulder_joint xyz = [0.040,0.020,0.057441] m
```

Model inspection on 2026-07-27 showed that this single-point correction displaced
the shoulder rotation axis from the physical base center, so it was fully removed.
The current CAD alignment is uncompensated:

```text
base_link -> shoulder_joint xyz = [0,0,0.057441] m
```

Z, Homing, and drive mapping were unchanged. The physical spur-gear center retains
its CAD origin, while the planned TCP uses the measured 170 mm value. Future error
must be diagnosed with multi-pose data rather than moving the shoulder frame.

FK, IK, endpoint checking, TF queries, and debug group `13` all use `tcp_link`.
The physical `spur_gear_joint` URDF origin and Drive 3 control are unchanged.

At ideal joint angles:

```text
q=[0,0,0]
TCP=[0.32840,-0.00177,0.043001] m

q=[0,+pi/2,+pi/2]
TCP=[0.23840,-0.00177,0.293001] m
```

The current 170 mm TCP change must pass:

- URDF XML and fixed-frame checks;
- URDF/Python CAD-base alignment checks;
- FK zero-pose and nominal-Home checks;
- IK reconstruction of nominal Home;
- Python import and kinematics regressions; and
- `git diff --check`.

URDF, TF, FK, IK, and regression tests must remain numerically consistent. Unit
tests prove only internal software consistency. After physical changes, compare the
model `tcp_link` with an external measurement of the same ideal TCP reference at
Home and at least one additional pose.

## 11. Why Multi-Pose Validation Is Required

A single-pose offset cannot distinguish:

- fixed TCP installation offset;
- `base_link` origin or orientation error;
- link length or joint-origin error; and
- encoder or Home-offset error.

At the physical Home joint state, first check whether group `13` reports a TCP near:

```text
(0.23840,-0.00177,0.293001) m
```

Repeat the external measurement at two or three additional nonsingular,
collision-free poses. If error changes with pose, do not apply another fixed TCP
offset; calibrate the base frame, link parameters, or encoder zero instead.

A changed TCP maps the same old XYZ target to different joint angles. Do not reuse
old hardware-tested absolute XYZ targets without replanning and inspection.

## 12. New Machine and EtherCAT Interface

The ROS container workspace is normally:

```text
/root/ws
```

Runtime settings:

```text
ROS 2 Jazzy
ROS_DOMAIN_ID=88
TCP bridge=127.0.0.1:15001
```

The default EtherCAT interface in `rascl_debug.sh` belongs to the last workstation.
After changing machines, use the new workstation's real interface name. Determine
it on the Ubuntu host; do not rely on an `ip` command that is absent in the container.

Pass it explicitly to group `4`:

```bash
RASCL_INTERFACE=<new-interface-name> bash ./rascl_debug.sh 4
```

Alternatively, after confirmation, update the default `INTERFACE` in
`rascl_debug.sh` and this guide. Do not repeat unnecessary interface checks after
the user has provided the correct name.

## 13. Debug Script Groups

Script:

```text
rascl_debug.sh
```

Usage:

```bash
bash ./rascl_debug.sh
bash ./rascl_debug.sh <group-number>
```

Groups:

```text
1  Build + functional tests
2  Start fake ros2_control
3  Fake checks + plan + execute
4  Start the physical Homing bridge
5  Home Drives 0, 1, and 2 individually; then reference and zero Drive 3
6  home_all: midpoint-zero Drives 0-2, then Drive 3 +50000 counts and Method 37
7  Start physical CSP ros2_control with Drive 3 participating
8  Controller/joint-state hold check
9  Plan the physical minimum-jerk trajectory only
10 Execute the physical minimum-jerk trajectory
11 Check residual processes and TCP port
12 Package complete ROS logs
13 Query base_link -> tcp_link
14 Set the next target XYZ and motion duration
15 In CSP, close/open or enter any nonzero relative count value for Drive 3
16 Read the latest CSP_STALL_SNAPSHOT
17 Read current absolute Drive 3 counts relative to the session Method 37 zero
18 Before CSP, read Drive 0-3 input mappings and Drive 2 protection parameters
19 After Homing and before CSP, trim Drive 0 by relative counts
20 After Homing and before CSP, trim Drive 1 by relative counts
21 After Homing and before CSP, trim Drive 2 by relative counts
22 Use Method 37 to set the current Drive 0-2 pose as session Home; Drive 3 unchanged
23 Enter target XYZ/duration, plan, and execute immediately after success
24 Task 1 stage 1: move 1
25 Task 1 stage 2: move 2 -> temporary square
26 Task 1 stage 3: square 3 -> square 1
27 Task 1 stage 4: square 2 -> square 3
28 Task 1 full sequence: stages 1 -> 2 -> 3 -> 4
29 Task 2: enter start XY and complete the fixed-target pick-and-place
```

The script does not switch terminals automatically. Groups `4` and `7` are
foreground processes. Groups `19/20/21` are only for measuring the correction from
the sensor-interval midpoint to the physical Home. They accept repeated positive or
negative relative counts and return the live cumulative
`correction_from_homed_zero`; they do not reset zero or store permanent parameters.
Use them only after group `6` succeeds and before group `7` starts.

After all three axes reach the verified physical Home, group `22` preserves
`driveN_before` and requires `driveN_after` near zero. It does not move; it only
resets the D0-D2 session zero, leaving D3 unchanged. Running group `6` again
overwrites this manual Home.

Run Task 1 group `28` in T3 during one valid CSP session. It calls
`24 -> 25 -> 26 -> 27`. Waypoints are fixed. Cartesian and gripper CSP trajectories
take `5 s`, except explicitly marked `10 s` descents. Each action starts immediately
after the preceding one. Gripper actions reuse group `15` values
`close=-150000` and `open=+150000 counts`. There are no extra confirmations. Any
planning or execution failure stops the current stage.

Run Task 2 group `29` in T3 after CSP starts. It asks only for start `x/y`, calculates
`r=sqrt(x^2+y^2)`, and runs every action for `5 s`. Common pickup sequence:

```text
(x,y,0.10) -> (x,y,0.045) -> close -> (x,y,0.10)
```

Routes:

```text
0.17 <= r <= 0.20:
  (0.1812,-0.0336,0.10) -> (0.1812,-0.0336,0.045)

r < 0.17:
  (0.1517,-0.0282,0.10) -> (0.1517,-0.0282,0.045)
  -> (0.1812,-0.0336,0.045)

r > 0.20:
  (0.2107,-0.0391,0.10) -> (0.2107,-0.0391,0.045)
  -> (0.1812,-0.0336,0.045)

common finish:
  open -> (0.1812,-0.0336,0.10)
```

No delay or confirmation is inserted between Task 2 actions.

## 14. First Run on a New Machine

Enter the project container, stop all old physical-hardware processes, and build:

```bash
cd /root/ws
bash ./rascl_debug.sh 1
```

Use three container terminals.

### T1: Only Homing Bridge

```bash
cd /root/ws
RASCL_INTERFACE=<new-interface-name> bash ./rascl_debug.sh 4
```

Keep it running; do not press `Ctrl-C`.

### T2: Home, Then Enter CSP

```bash
cd /root/ws
bash ./rascl_debug.sh 6
```

Required output includes:

```text
success=True
Homing completed for required drives; CSP handoff armed:
drive0_interval(...) drive1_interval(...) drive2_interval(...)
drive3_reference(...delta=50000,...zero=0,method=37)
Drive 3: absolute_counts=0, ... reference_complete=true
```

For Home trim tests, repeat as needed in T2:

```bash
bash ./rascl_debug.sh 19   # Drive 0
bash ./rascl_debug.sh 20   # Drive 1
bash ./rascl_debug.sh 21   # Drive 2
bash ./rascl_debug.sh 22   # Set current D0-D2 pose as session Home
```

Record `drive0/1/2_before` from group `22`. To permanently change the physical
automatic-Homing endpoint, add these corrections after reaching the sensor midpoint
and before Method 37. Changing only `home_offset_counts` changes the ROS mapping,
not the physical automatic-Homing endpoint.

Then, still in T2:

```bash
bash ./rascl_debug.sh 7
```

Keep it running. Key success lines include:

```text
CSP interpolation 0x2332.00 ... 200
CSP directional torque limits verified for this session only
Master reached OP state
Activated real RASCL hardware in csp mode
```

### T3: Inspect TCP Before Motion

```bash
cd /root/ws
bash ./rascl_debug.sh 8
bash ./rascl_debug.sh 13
ros2 topic echo --once /joint_states
```

Record and remeasure the Home TCP before sending old motion targets. For a new
debugging task, provide:

1. Complete group `13` Translation output.
2. Complete simultaneous `/joint_states` output.
3. Externally measured physical TCP x/y/z and units.
4. Definition of the measured physical point.
5. Measurement origin and positive directions of all three axes.
6. EtherCAT interface name on the new workstation.

## 15. Motion After TCP Verification

For repeated targets, run:

```bash
bash ./rascl_debug.sh 23
```

Group `23` reads x/y/z/duration and executes immediately only after IK and CSV
checks pass. Planning failure sends no physical command. Groups `14`, `9`, and `10`
retain their separate behavior:

```bash
bash ./rascl_debug.sh 14
bash ./rascl_debug.sh 9
bash ./rascl_debug.sh 10
```

Required success output:

```text
MOTION_RESULT reached=true
```

Every new target requires:

```text
23  (or 14 -> 9 -> 10)
```

## 16. Faults and Logs

For Drive 2 following error, `internal_limit_active`, or a mid-trajectory stall:

```bash
bash ./rascl_debug.sh 16
bash ./rascl_debug.sh 12
```

Group `12` creates this archive in the shared workspace:

```text
ros_logs_YYYYMMDD_HHMMSS.tar.gz
```

Submit the `.tar.gz` directly; do not copy terminal output line by line. Search for:

```text
CSP_SNAPSHOT
DRIVE_DIAG
TORQUE_SNAPSHOT
CSP_STALL_DETECTED
CSP_STALL_SNAPSHOT
CSP_LIMIT_SWITCH_CONFIGURATION
CSP_FOLLOWING_ERROR_CONFIGURATION
MOTION_RESULT
```

## 17. Requirements for Future Changes

1. Preserve the `[28,28,24]` first-edge search directions for Drives 0-2. Cross
   the active interval, return to `(entry+exit)/2`, and mark Homed only after
   successful Method 37 midpoint zeroing.
2. Preserve Drive 3 behavior: no sensor search, then `+50000 counts`, Method 37,
   and participation in CSP.
3. Reuse the same EtherCAT master for Homing-to-CSP handoff.
4. Preserve `0x2332:00=200`.
5. Do not add `0x6060/0x6061` back to the cyclic PDO.
6. Do not run Drive 3 gripper motion concurrently with a Cartesian trajectory.
7. Do not change any axis direction without evidence.
8. Except for Drive 3 Method 37 zero, do not use `0x607C` to correct TCP geometry.
9. Do not clear or write `0x607B/0x607D` directly.
10. Keep test parameters session-only; do not store them persistently.
11. Update the single authoritative guide when commands, parameters, or workflow change.
12. Put confirmed configuration in code defaults; do not rely on hidden script overrides.
13. Collect joint state, raw/PDO data, and external measurements before changing kinematics.
14. Verify the Drive 2 `0x2310:01/:02` correction before loosening protection further.
15. Preserve existing Git work; do not commit, push, or perform destructive rollback
    without authorization.

## 18. First Work in a New Debugging Task

1. Read this file.
2. Read `WP3_Task1_MinJerk_Debug_Guide.md`.
3. Inspect `git status` and current HEAD.
4. Confirm the EtherCAT interface on the new machine.
5. Complete `1 -> T1:4 -> T2:6 -> T2:7 -> T3:8 -> T3:13`.
6. Collect the new TF, `/joint_states`, and external physical TCP.
7. Decide whether the current 170 mm ideal TCP is correct.
8. Validate additional poses and decide whether further kinematic calibration is needed.

Current critical state:

```text
The upstream baseline before the current Drive 3 direction reversal is commit 2d5c6d5.
The current tcp_link is the measured ideal point at lowerarm local
[0.170,0,0.0179] m. The physical spur_gear_joint center remains
[0.13916,0,0.0179] m.
Group 15 supports both close/open shortcuts and arbitrary nonzero relative counts.
Group 17 returns absolute counts relative to the current Drive 3 Method 37 zero.
The 2026-07-24 Drive 2 fault was reproduced. Verify that CSP handoff reports
CSP_LIMIT_SWITCH_CONFIGURATION with lower/upper=0x00/0x00.
```
