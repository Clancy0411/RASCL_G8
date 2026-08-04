# RASCL Group 8 - WP3 Motion Planning and Robot Control

This repository contains the Group 8 ROS 2 application for WP3 Task 1 and
Task 2. It combines a calibrated URDF, `ros2_control`, a custom FAULHABER
EtherCAT interface, Cyclic Synchronous Position (CSP) control, inverse
kinematics, and 50 Hz minimum-jerk joint trajectories.

The supported operator interface is [`rascl_debug.sh`](rascl_debug.sh). This
README gives the shortest examiner workflow. The complete description of all
29 command groups is in
[`WP3_Physical_Hardware_Debug_Guide.md`](WP3_Physical_Hardware_Debug_Guide.md).

## Submission Contents

The submission archive contains the source repository and separate Task 1 and
Task 2 demonstration videos. The Task 2 gripper pickup range demonstrated by
the project covers the complete coordinate board.

```text
RASCL_G8/
|-- README.md
|-- WP3_Physical_Hardware_Debug_Guide.md
|-- rascl_debug.sh
|-- rosws.sh
|-- Dockerfile
`-- src/
    |-- rascl_description/           URDF, meshes, controllers, launch files
    |-- rascl_hardware_interface/    ros2_control and EtherCAT bridge
    `-- rascl_wp3_ss26_group8/       Task nodes, IK, trajectories, tests
        |-- launch/
        |-- trajectories/            Task input and output CSV files
        `-- rascl_wp3_ss26_group8/
```

The commanded joint order is:

```text
[shoulder_joint, upperarm_joint, lowerarm_joint, spur_gear_joint]
```

Cartesian coordinates are metres in the URDF `base_link` frame. The planning
TCP is `tcp_link`, the measured point 170 mm along the lower arm.

## Task Implementation

### Task 1 - Offline Trajectory Execution

Task 1 stacks three cubes in the required order. The fixed assessed action list
is stored in
[`task1_input.csv`](src/rascl_wp3_ss26_group8/trajectories/task1_input.csv).
For every Cartesian leg, group `9` uses live feedback and IK to generate the
offline joint trajectory
[`task1_output.csv`](src/rascl_wp3_ss26_group8/trajectories/task1_output.csv).
Group `10` then loads, validates, and executes that exact CSV; it does not
replan during execution. Group `28` performs this plan/load/execute process for
all four assessed stages automatically.

### Task 2 - Online Pick and Place

The `wp3_tsk2` node receives each unknown cube centre at runtime from
`/goal_poses` as `geometry_msgs/msg/Point`. It plans from current joint
feedback and moves the cube to coordinate-board point `(40, 180)`.

The fixed validated internal destination remains `(0.1812, -0.0336) m`.
Planar radius is used only to select the validated inner, direct, or outer
transfer action; waypoint reachability is decided by IK. For an inner-to-outer
transfer, the cube briefly
contacts the smaller-radius board region; for an outer-to-inner transfer it
briefly contacts the larger-radius region. This reduces cube tilt before
release. Task 2 input and generated trajectory CSV files are written under the
package `trajectories/` directory.

## Requirements

- Ubuntu host with Docker
- powered RASCL robot and EtherCAT connection
- three terminals attached to the same running container; Task 2 publication
  uses one additional short-lived terminal
- clear robot workspace and coordinate board

The validated workstation interface is `enx3c18a0256deb`. On another
workstation, set the actual EtherCAT interface in T1 and T2 before starting:

```bash
export RASCL_INTERFACE=<actual-interface-name>
```

## 1. Start the Container

In T1 on the Ubuntu host:

```bash
cd ~/RASCL_G8
bash ./rosws.sh
```

Wait for `rascl-container:~/ws$`. Open T2 and T3 and run the same two commands;
they must report `Attaching to running container...`. In each container
terminal run:

```bash
cd /root/ws
```

The script loads ROS Jazzy, the workspace overlay, and `ROS_DOMAIN_ID=88`.

## 2. Build and Test

After extracting, pulling, or changing the submission, run in T1:

```bash
bash ./rascl_debug.sh 1
```

Group `1` builds the workspace and runs the application, description, and
hardware-interface tests. Do not rebuild while a physical bridge or controller
is active.

Optional fake-hardware validation:

```bash
# T1, keep running
bash ./rascl_debug.sh 2

# T2
bash ./rascl_debug.sh 3
```

Stop group `2` before physical operation.

## 3. Home and Enter CSP

In T1, start the EtherCAT/Homing bridge and leave it running:

```bash
bash ./rascl_debug.sh 4
```

Continue after it reports four slaves and
`TCP bridge listening on 127.0.0.1:15001`.

In T2, run automatic Homing:

```bash
bash ./rascl_debug.sh 6
```

Continue only after `success=True`, all three `driveN_interval(...)` records,
and `reference_complete=true` for Drive 3. Without stopping T1 group `4`, start
CSP in T2 and leave it running:

```bash
bash ./rascl_debug.sh 7
```

Wait until `joint_state_broadcaster` and `rascl_position_controller` are both
active. The normal startup order is:

```text
T1: 1 -> 4       keep 4 running
T2:      6 -> 7  keep 7 running
T3:              run a task
```

Group `1` may be omitted when the current workspace is already built.

## 4. Run Task 1

Place the cubes in the demonstrated Task 1 starting arrangement. In T3 run:

```bash
bash ./rascl_debug.sh 28
```

This one command executes all four Task 1 stages. Each arm leg is planned to
the package CSV and then executed from the same offline file. Any failed plan,
CSV validation, or endpoint check stops the sequence. Groups `24`-`27` expose
the same stages separately for diagnostics only.

## 5. Run Task 2

Task 2 can use the same healthy CSP session. In T3 start the online node and
leave it running:

```bash
bash ./rascl_debug.sh 29
```

Open another terminal, attach to the same container, and publish the measured
cube centre in metres:

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
source /root/ws/install/local_setup.bash
export ROS_DOMAIN_ID=88
ros2 topic pub --once /goal_poses geometry_msgs/msg/Point \
  '{x: 0.16, y: 0.08, z: 0.0}'
```

The node accepts repeated messages sequentially. It selects the appropriate
radial action and completes pickup, transfer to `(40, 180)`, release, and
retreat. Stop group `29` with `Ctrl-C` when Task 2 is complete.

## 6. Shutdown and Failure Handling

Support the arm before disabling drive voltage.

1. Stop Task 2 group `29`, if active.
2. Stop T2 group `7` and wait for `ros2_control` to exit.
3. Stop T1 group `4`.

After a PDO, WKC, following-error, controller, or endpoint failure, do not send
another motion. Run group `12` to package ROS logs, then restart the complete
Homing-to-CSP session.

For manual targets, gripper counts, calibration, diagnostics, log collection,
and every command group, see the
[`WP3 Physical Hardware Debug Guide`](WP3_Physical_Hardware_Debug_Guide.md).
