# RASCL Debug Script Guide

This is the complete operating reference for `rascl_debug.sh`. It describes
all command groups and their expected use. Run it inside the ROS container:

Use the repository `README.md` for the examiner-facing Task 1 and Task 2
reproduction procedure. This guide is the detailed command-group reference.

```bash
cd /root/ws
bash ./rascl_debug.sh
```

Select a number from the menu, or run one group directly:

```bash
bash ./rascl_debug.sh <group-number>
```

The script does not open, close, or switch terminals.

## Terminal Roles and Normal Order

- **T1** runs group `4`; keep it running for the entire Homing/CSP session.
- **T2** runs group `6`, then group `7`; keep group `7` running during motion.
- **T3** runs checks, manual motion, gripper commands, and Task 1.
- **T4** is needed only to publish `/goal_poses` while Task 2 group `29`
  occupies T3.

Start the same container from each Ubuntu terminal:

```bash
cd ~/RASCL_G8
bash ./rosws.sh
```

Wait for T1 to reach `rascl-container:~/ws$` before opening the other
terminals. They should report `Attaching to running container...`. Then run
`cd /root/ws` in each terminal.

For changed code:

```text
T1: 1 -> 4        keep 4 running
T2:      6 -> 7   keep 7 running
T3:               28 for Task 1, or 29 for Task 2
```

For an already built workspace, omit group `1`.

The default EtherCAT interface is `enx3c18a0256deb`. On another workstation,
set the real interface in T1 and T2:

```bash
export RASCL_INTERFACE=<actual-interface-name>
```

## Group Summary

| Group | Terminal | Motion | Function |
|---:|---|---|---|
| 1 | T1 | No | Build and run all project tests |
| 2 | T1 | Simulated | Start fake `ros2_control` |
| 3 | T2 | Simulated | Validate fake controllers and offline Task 1 CSV execution |
| 4 | T1 | Possible | Start the EtherCAT/Homing bridge |
| 5 | T2 | Yes | Home Drives 0-2 individually |
| 6 | T2 | Yes | Home Drives 0-2 together and reference Drive 3 |
| 7 | T2 | Yes | Start physical CSP `ros2_control` |
| 8 | T3 | No | Check controllers and `/joint_states` for 10 seconds |
| 9 | T3 | No | Generate and validate the offline Task 1 trajectory CSV |
| 10 | T3 | Yes | Load and execute the exact CSV authorized by group 9 |
| 11 | T3 | No | Show RASCL processes and TCP port 15001 |
| 12 | Any | No | Package `/root/.ros/log` |
| 13 | T3 | No | Show `base_link -> tcp_link` |
| 14 | T3 | No | Save a Cartesian target and duration |
| 15 | T3 | Yes | Close/open the gripper or move Drive 3 by relative counts |
| 16 | T3 | No | Show the latest automatic CSP stall snapshot |
| 17 | T2/T3 | No | Show absolute Drive 3 counts from the session zero |
| 18 | T2 | No | Read input mappings and Drive 2 protection settings |
| 19 | T2 | Yes | Trim Drive 0 by relative counts |
| 20 | T2 | Yes | Trim Drive 1 by relative counts |
| 21 | T2 | Yes | Trim Drive 2 by relative counts |
| 22 | T2 | No | Set the current Drive 0-2 pose as session Home |
| 23 | T3 | Yes | Enter, plan, and execute one Cartesian target |
| 24 | T3 | Yes | Run Task 1 stage 1 |
| 25 | T3 | Yes | Run Task 1 stage 2 |
| 26 | T3 | Yes | Run Task 1 stage 3 |
| 27 | T3 | Yes | Run Task 1 stage 4 |
| 28 | T3 | Yes | Run all Task 1 stages continuously |
| 29 | T3 | Yes | Start the online Task 2 `/goal_poses` node |

Groups `2`, `4`, `7`, and `29` occupy their terminal until `Ctrl-C`.

## Groups 1-3: Build and Fake Hardware

### Group 1 - Build and Test

```bash
bash ./rascl_debug.sh 1
```

This group builds with `colcon build --symlink-install`, confirms both
`wp3_tsk1` and `wp3_tsk2`, runs the WP3 Python tests, and runs the description
and hardware-interface tests. Stop all physical processes before rebuilding.

### Groups 2 and 3 - Fake-Hardware Check

```bash
# T1, keep running
bash ./rascl_debug.sh 2

# T2
bash ./rascl_debug.sh 3
```

Group `3` checks controllers and feedback, generates a Task 1 trajectory in
`src/rascl_wp3_ss26_group8/trajectories/task1_output.csv`, and executes that
same offline file on fake hardware. Stop group `2` before physical operation.

## Groups 4-8: Homing and CSP

### Group 4 - EtherCAT/Homing Bridge

Run in T1 and leave it active:

```bash
bash ./rascl_debug.sh 4
```

Required startup output includes:

```text
Found 4 slave(s)
TCP bridge listening on 127.0.0.1:15001
```

Drives 0-2 use reference-input interval-centre Homing: find the first edge,
cross the active interval, record the second edge, return to the midpoint, and
set that point to zero with Method 37. Drive 3 later moves `+50000` counts and
sets its reached position to the session zero.

At CSP startup the same bridge configures the 20 ms interpolation period,
maps Position PDOs, clears the volatile travel-limit input mappings, applies
the verified following-error and torque settings, and starts the 50 Hz PDO
loop.

### Group 5 - Home Drives Individually

```bash
bash ./rascl_debug.sh 5
```

This diagnostic alternative calls `home_one` for Drives 0, 1, and 2 and
requires a complete interval record for each drive. After the third arm drive,
Drive 3 performs its reference move. Use group `5` or group `6`, not both.

### Group 6 - Automatic Home All

```bash
bash ./rascl_debug.sh 6
```

This is the normal command. It reads drive inputs, calls `home_all`, checks all
three arm interval records, verifies Drive 3 reference completion, and prints
Drive 2 protection values. Continue only after output includes:

```text
success=True
drive0_interval(...)
drive1_interval(...)
drive2_interval(...)
reference_complete=true
```

### Group 7 - Start Physical CSP

Run in T2 and leave it active:

```bash
bash ./rascl_debug.sh 7
```

It starts `ros2_control` with `start_bridge:=false`, reusing the T1 bridge.
Continue after both controllers report `active`:

```text
joint_state_broadcaster
rascl_position_controller
```

### Group 8 - Controller and Feedback Check

```bash
bash ./rascl_debug.sh 8
```

This read-only group lists controllers/interfaces, reads one complete
`/joint_states` message, and measures its rate for 10 seconds. The nominal
automatic-Home pose is approximately `[0, +1.5708, +1.5708, 0] rad`.

## Groups 9, 10, 13, 14, and 23: Cartesian Motion

All coordinates are metres in `base_link`. The fixed-board XY correction is
applied before IK.

### Group 13 - Read Current TCP

```bash
bash ./rascl_debug.sh 13
```

Displays `base_link -> tcp_link` from live joint feedback for about three
seconds.

### Group 14 - Save Target

```bash
bash ./rascl_debug.sh 14
```

Enter X, Y, Z, and motion duration. This group saves the values but neither
plans nor moves.

### Group 9 - Generate Offline Trajectory

```bash
bash ./rascl_debug.sh 9
```

Group `9` verifies the current CSP session, reads live joints, runs IK, creates
a 50 Hz minimum-jerk trajectory, and writes:

```text
/root/ws/src/rascl_wp3_ss26_group8/trajectories/task1_output.csv
```

It checks the CSV for invalid values and records an authorization tied to the
current target and group `7` process. It publishes no motion command.

### Group 10 - Execute Offline Trajectory

```bash
bash ./rascl_debug.sh 10
```

Group `10` requires the matching group `9` authorization. It loads the exact
package CSV, verifies its joint order, timestamps, starting joint state, and
final target, then publishes its stored samples at their CSV timestamps. It
does not rerun IK or regenerate the trajectory. Final joint and TCP feedback
must pass before success is returned.

### Group 23 - Set, Plan, and Execute

```bash
bash ./rascl_debug.sh 23
```

This requests X, Y, Z, and duration and then calls the same group `9` and group
`10` functions. Repeat group `23` for another target in the same healthy CSP
session. Use `14 -> 9 -> 10` when the CSV must be inspected manually.

## Groups 15 and 17: Gripper

### Group 15 - Relative Drive 3 Motion

```bash
bash ./rascl_debug.sh 15
```

Accepted inputs are:

```text
close or c   exact relative -150000 counts
open or o    exact relative +150000 counts
+2000        increase the Drive 3 count by 2000
-5000        decrease the Drive 3 count by 5000
```

The first three axes hold their current positions while Drive 3 follows a 50 Hz
minimum-jerk trajectory. The default speed is `20000 counts/s`. Close/open are
fixed-distance moves and do not use following error as contact detection. The
requested target must remain within `[-2*pi, +2*pi] rad`.

Group `15` clears an old group `9` authorization. Alternating gripper and arm
motion is supported, for example `15 -> 23 -> 15 -> 23`, but never run two
command publishers concurrently.

### Group 17 - Read Absolute Drive 3 Counts

```bash
bash ./rascl_debug.sh 17
```

Reads Drive 3 counts relative to its current Method 37 session zero. It does
not command motion.

## Groups 19-22: Home Calibration

These groups are used after group `6` and before group `7`; they are not needed
for the validated assessed workflow.

### Groups 19, 20, and 21 - Fine Trim

```bash
bash ./rascl_debug.sh 19   # Drive 0
bash ./rascl_debug.sh 20   # Drive 1
bash ./rascl_debug.sh 21   # Drive 2
```

Enter a nonzero signed relative count. The selected drive makes a Profile
Position move and reports the accumulated correction. These groups do not
change zero by themselves.

### Group 22 - Set Current Arm Pose as Home

```bash
bash ./rascl_debug.sh 22
```

Runs Method 37 for Drives 0-2 without a position move and verifies the new
zero. Drive 3 is unchanged. This definition lasts only for the current
session.

## Groups 24-28: Task 1

The submission input record matching the fixed assessed action sequence is:

```text
/root/ws/src/rascl_wp3_ss26_group8/trajectories/task1_input.csv
```

| Group | Stage |
|---:|---|
| 24 | Move cube 1 to the tower position |
| 25 | Move the upper cube to the temporary position |
| 26 | Move the lower cube onto cube 1 |
| 27 | Move the temporary cube onto the tower |

Each Cartesian leg calls group `9` to generate the package output CSV and
group `10` to load and execute that same file. Gripper actions use the exact
group `15` count increments. Task 1 takes no runtime coordinate input. A
failure stops the stage.

The normal Task 1 command is:

```bash
bash ./rascl_debug.sh 28
```

Group `28` executes groups `24`-`27` continuously, without inserted waits.

## Group 29: Task 2

Start the online node in T3 after CSP is active:

```bash
bash ./rascl_debug.sh 29
```

In T4, attach to the same container and publish the runtime cube centre:

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
source /root/ws/install/local_setup.bash
export ROS_DOMAIN_ID=88
ros2 topic pub --once /goal_poses geometry_msgs/msg/Point \
  '{x: 0.16, y: 0.08, z: 0.0}'
```

`Point.x` and `Point.y` are the measured board coordinates in metres.
`Point.z` is ignored; the validated pickup height is configured in the node.

The fixed assessed destination is board coordinate `(40, 180)`. The start
radius only selects one of three validated actions:

- radius below `0.17 m`: inner alignment, then move outward;
- radius from `0.17 m` through `0.20 m`: direct transfer;
- radius above `0.20 m`: outer alignment, then move inward.

Each Cartesian waypoint is accepted only when online IK can solve it. The node queues repeated
`/goal_poses` messages and processes them sequentially.

Task 2 stores inputs and generated joint trajectories under:

```text
/root/ws/src/rascl_wp3_ss26_group8/trajectories/
```

Files use names such as `task2_job_0001_input.csv` and
`task2_job_0001_step_01_approach_pick.csv`.

## Groups 11, 12, 16, and 18: Diagnostics

### Group 11 - Process and Port Check

```bash
bash ./rascl_debug.sh 11
```

Lists the bridge, controller, task processes, and TCP port 15001. Use it when a
restart reports an existing process or occupied address.

### Group 12 - Package ROS Logs

```bash
bash ./rascl_debug.sh 12
```

Creates `/root/ws/ros_logs_YYYYMMDD_HHMMSS.tar.gz` from `/root/.ros/log`.
Run it immediately after a motion failure and copy the archive from the shared
workspace.

### Group 16 - CSP Stall Snapshot

```bash
bash ./rascl_debug.sh 16
```

Reads the most recent automatic `CSP_STALL_SNAPSHOT` from the existing bridge.
It does not open another EtherCAT client.

### Group 18 - Input and Drive 2 Readback

```bash
bash ./rascl_debug.sh 18
```

Use before CSP. It reads Drive 0-3 digital input mappings and Drive 2 position
range, software limits, and following-error settings without writing them.

## Main Configuration Overrides

The assessed workflow uses defaults. Set an override before its relevant group
only when the workstation or validated setup requires it.

| Variable | Default | Purpose |
|---|---:|---|
| `RASCL_WS` | `/root/ws` | Container workspace |
| `RASCL_INTERFACE` | `enx3c18a0256deb` | EtherCAT interface |
| `ROS_DOMAIN_ID` | `88` | ROS domain |
| `RASCL_TRAJECTORY_DIR` | package `trajectories/` | Task CSV directory |
| `RASCL_TASK1_OUTPUT_CSV` | `task1_output.csv` | Offline Task 1 joint trajectory |
| `RASCL_TASK2_OUTPUT_DIR` | package `trajectories/` | Task 2 CSV directory |
| `RASCL_LOWERARM_DIRECTION` | `1` | Drive 2 encoder-to-URDF sign |
| `RASCL_LOWERARM_HOME_OFFSET_COUNTS` | `-802816` | Drive 2 URDF offset |
| `RASCL_DRIVE2_FOLLOWING_ERROR_WINDOW_COUNTS` | `25000` | Drive 2 following window |
| `RASCL_DRIVE2_FOLLOWING_ERROR_TIMEOUT_MS` | `250` | Drive 2 following timeout |
| `RASCL_HOMING_INTERVAL_MAX_TRAVEL_DRIVE0_COUNTS` | `100000` | Drive 0 edge-search range |
| `RASCL_HOMING_INTERVAL_MAX_TRAVEL_DRIVE1_COUNTS` | `300000` | Drive 1 edge-search range |
| `RASCL_HOMING_INTERVAL_MAX_TRAVEL_DRIVE2_COUNTS` | `300000` | Drive 2 edge-search range |
| `RASCL_HOMING_INTERVAL_TIMEOUT_S` | `120.0` | Homing traversal timeout |
| `RASCL_CSP_TORQUE_LIMIT_PER_MILLE` | `1000` | Normal CSP torque limit |
| `RASCL_SPUR_GEAR_DIRECTION` | `-1` | Drive 3 encoder-to-URDF sign |
| `RASCL_SPUR_GEAR_COUNTS_PER_REVOLUTION` | `1323008` | Drive 3 conversion |
| `RASCL_SPUR_GEAR_REFERENCE_DELTA_COUNTS` | `50000` | Startup reference move |
| `RASCL_SPUR_GEAR_SPEED_COUNTS_PER_S` | `20000` | Group 15 speed |
| `RASCL_TARGET_X/Y/Z` | `0.2108/-0.00177/0.2913` | Initial manual target |
| `RASCL_DURATION` | `12.0` | Initial manual duration |
| `RASCL_STATE_DIR` | `/tmp/rascl_debug` | Session authorization state |

Other bridge diagnostic thresholds remain available in the script header.

## Failure, Restart, and Shutdown

Stop sending commands after any PDO/WKC error, drive fault, following error,
inactive controller, lost `/joint_states`, or `MOTION_RESULT reached=false`.
Package logs with group `12`, then restart:

1. stop group `29`, if active;
2. stop T2 group `7`;
3. stop T1 group `4`;
4. use group `11` if a process remains;
5. restart `T1:4 -> T2:6 -> T2:7`.

For normal shutdown, support the arm, stop group `29`, stop group `7`, wait for
`ros2_control` to exit, and finally stop group `4`.
