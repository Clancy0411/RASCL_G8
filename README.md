# RASCL Group 8 - WP3 Task 1 and Task 2

## Enter the Container

### 1. Check Docker First

Run the following on the Ubuntu host:

```bash
docker version
```

If Docker reports that it cannot connect to the Docker daemon, start Docker and
check again:

```bash
sudo systemctl start docker
docker version
```

Do not continue with the physical-robot workflow if `docker version` still fails.
Fix Docker first.

### 2. Start T1

Enter the project root. If the Docker image, `Dockerfile`, and dependencies have
not changed, run:

```bash
cd ~/RASCL_G8
bash ./rosws.sh
```

On the first run, or if the image does not exist locally, `rosws.sh` builds it
automatically. Use the following command only if the `Dockerfile`, container
dependencies, or image have changed and all physical-hardware processes have been
stopped:

```bash
cd ~/RASCL_G8
SOFT_REBUILD=true bash ./rosws.sh
```

Do not rebuild the image while group `4` or group `7` is still running. If a
container with the same name is already running, `rosws.sh` attaches to the old
container first; `SOFT_REBUILD=true` does not replace a running container. Before
rebuilding, safely stop the physical workflow and exit the old container.

Wait until T1 displays the following container prompt before opening the other two
terminals:

```text
rascl-container:~/ws$
```

T1 is the main terminal that starts this container and must remain open throughout
the workflow. Exiting T1 early stops the container, and T2/T3 can no longer attach.

If the image build fails, the container exits early, or this prompt does not
appear, do not proceed to T2/T3. Resolve the T1 error first.

### 3. Attach T2 and T3

After confirming that T1 has entered the container, open two more Ubuntu terminals
and run the following in each one:

```bash
cd ~/RASCL_G8
bash ./rosws.sh
```

Under normal conditions, the output should include:

```text
Attaching to running container...
```

If T2/T3 starts building the image again, the T1 container is not running
correctly. If a `docker exec` or container-not-running error appears, return to T1
and diagnose it. Do not continue to Homing or CSP.

The project root is mounted inside the container at:

```text
/root/ws
```

Run the following in all three container terminals, T1, T2, and T3:

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=88
```

Then open the debug-group menu with:

```bash
bash ./rascl_debug.sh
```

After changing code, pulling a new revision, or receiving a missing ROS package or
installation-file error from group `4`, first confirm that no physical-hardware
process is running, then run group `1` in T1. Do not rebuild or recompile at every
startup when the Docker image and `install/` tree have not changed.

## EtherCAT Interface for Group 4

Group `4` connects to the robot through the Ubuntu host's wired EtherCAT network
interface. The current default interface name in the script is:

```text
enx3c18a0256deb
```

An interface name is specific to its computer. On another workstation, using the
old interface name causes group `4` to fail because it cannot find the interface or
any EtherCAT slaves. Find and enable the interface on the Ubuntu host; do not rely
on an `ip` command that may be absent inside the container:

```bash
ip -br link
ip link show <actual-interface-name>
sudo ip link set <actual-interface-name> up
ip link show <actual-interface-name>
```

Select the interface physically connected to the robot's EtherCAT network, not the
Wi-Fi interface or an interface used for normal internet access. After entering the
container in T1, pass the interface explicitly when starting group `4`:

```bash
RASCL_INTERFACE=<actual-interface-name> bash ./rascl_debug.sh 4
```

Group `7` in T2 must use the same interface name. Because T1 and T2 are separate
shells, set the variable in both container terminals first:

```bash
export RASCL_INTERFACE=<actual-interface-name>
```

Then run group `4` in T1 and groups `6 -> 7` in T2. If the actual interface is
`enx3c18a0256deb`, the script's default value can be used directly.

After group `4` starts successfully, T1 must display:

```text
TCP bridge listening on 127.0.0.1:15001
```

If `No such device` appears, return to the Ubuntu host and verify the interface
name and state. If the interface opens but reports `Found 0 slave(s)`, check robot
power, the EtherCAT cable, and the selected interface. Do not run group `6` or group
`7` unless slaves are detected and the TCP bridge success message appears. Do not
try to bypass an error by repeatedly retrying motion.

This document covers only the debug groups required to complete WP3 Task 1 and
Task 2, the original block placement coordinates, and the purpose of every group in
`rascl_debug.sh`.

All coordinates are in metres. This document records the original block and target
coordinates, not values corrected through physical calibration.

## Usage

Run the following from the workspace root:

```bash
bash ./rascl_debug.sh <group-number>
```

Physical-robot operation requires three terminals:

- T1: build the workspace and keep the Homing/EtherCAT bridge running.
- T2: perform Homing, then keep CSP `ros2_control` running.
- T3: run Task 1, Task 2, and other checks.

> **Safety:** Groups `7`, `10`, `15`, `19`-`21`, and `23`-`29` may move the
> physical robot. Clear the workspace, prepare the emergency stop, and verify that
> the arm and gripper cannot collide before running them.

## Task 1

Task 1 requires stacking three blocks in the order `1 -> 2 -> 3` to form a stable
tower, with block 1 at the bottom, block 2 in the middle, and block 3 at the top.

### Groups Required for Task 1

Run the groups in this order:

```text
T1: 1 -> 4
T2: 6 -> 7
T3: 24 -> 25 -> 26 -> 27
```

Equivalent commands:

```bash
# T1: after group 1 finishes, start group 4 and keep it running
bash ./rascl_debug.sh 1
bash ./rascl_debug.sh 4

# T2: after group 6 succeeds, start group 7 and keep it running
bash ./rascl_debug.sh 6
bash ./rascl_debug.sh 7

# T3: complete the four Task 1 stages in order
bash ./rascl_debug.sh 24
bash ./rascl_debug.sh 25
bash ./rascl_debug.sh 26
bash ./rascl_debug.sh 27
```

Important requirements:

- Do not stop group `4` between groups `6` and `7`; CSP must reuse the same
  EtherCAT bridge.
- Run group `7` only after group `6` has successfully completed all Homing and the
  Drive 3 zeroing procedure.
- Run groups `24`-`27` in order. If any stage fails, do not continue to the next
  stage.

### Original Block Placement Positions

The following are the original XY coordinates of the blocks on the board. They do
not include physical-calibration corrections:

| Position | X [m] | Y [m] | Description |
|---|---:|---:|---|
| Block 1 start | 0.16 | 0.16 | Placed separately |
| Block 2/3 start | 0.17 | 0.03 | Block 2 at the bottom and block 3 on top |
| Block 3 temporary position | 0.18 | -0.04 | Move the top block 3 away first so that the bottom block 2 can be moved |
| Final tower target | 0.07 | -0.10 | Stack the blocks in the order 1, 2, 3 |

### Four Task 1 Stages

| Group | Stage | Action |
|---:|---|---|
| 24 | Stage 1 | Move block 1 from its start position to the final tower target |
| 25 | Stage 2 | Move the top block 3 from the block 2/3 start position to the temporary position |
| 26 | Stage 3 | Move the bottom block 2 onto block 1 |
| 27 | Stage 4 | Move block 3 from the temporary position onto block 2 to complete the three-level tower |

After completion, the three-level tower must remain stable for at least five
seconds.

## Task 2

During operation, Task 2 receives the XY start position of an unknown block and
moves the block to a fixed target.

### Task 2 ROS 2 Node

After completing groups `1 -> 4 -> 6 -> 7` and entering a healthy CSP session,
start the required long-running `wp3_tsk2` node with group `29` in T3:

```bash
bash ./rascl_debug.sh 29
```

Keep T3 running. From another container terminal, publish each cube centre during
runtime with the required `geometry_msgs/msg/Point` message:

```bash
ros2 topic pub --once /goal_poses geometry_msgs/msg/Point \
  "{x: 0.16, y: 0.08, z: 0.0}"
```

The node remains active and processes any additional valid messages sequentially.
It restores the original group `29` radial routing: `inner` for `r < 0.17 m`,
`middle` for `0.17 <= r <= 0.20 m`, and `outer` for `r > 0.20 m`. The inner route
descends 30 mm inside the fixed-goal radius and pushes outward at placement height;
the outer route descends 30 mm outside it and pulls inward. The middle route descends
directly at the goal. Every Cartesian waypoint is planned online from the latest
`/joint_states` feedback. Input and generated minimum-jerk CSV files are saved under
`/tmp/rascl_wp3_tsk2`. As in the original group `29`, every Cartesian and gripper
trajectory is requested with a duration of 5 seconds.

The submission-facing files and executable are:

```text
rascl_wp3_ss26_group8/wp3_tsk2.py
launch/wp3_tsk2.launch.py
ros2 run rascl_wp3_ss26_group8 wp3_tsk2
```

The configured radial region is `0.10 <= r <= 0.257099203 m`, with a shoulder angle
between `-pi/2` and `+pi/2`. The maximum is the farthest labelled centre across the
two physical box plates, `(0.250, 0.060) m`. Final collision-free reach still needs
physical validation for the robot-gripper configuration.

The maximum input radius is defined only by that farthest labelled box-plate point:

```text
r_max = sqrt(0.250^2 + 0.060^2) = 0.257099203 m
```

It is not derived from the fixed target radius. The maximum accepted cube radius and
the fixed target coordinate are configured independently; the original fixed target
remains `(0.18, -0.04) m`.

The routing behaviour relative to the original fixed goal is:

| Route | Input radius | Placement approach before XY compensation |
|---|---:|---|
| Inner | `r < 0.17 m` | Approach 30 mm inside the goal radius, descend, then push outward |
| Middle | `0.17 <= r <= 0.20 m` | Descend directly above the original goal `(0.18, -0.04)` |
| Outer | `0.20 < r <= 0.257099203 m` | Approach 30 mm outside the goal radius, descend, then pull inward |

### Original Task 2 Target

| Position | X [m] | Y [m] | Placement Z [m] | Retreat Z [m] |
|---|---:|---:|---:|---:|
| Fixed target | 0.18 | -0.04 | 0.045 | 0.10 |

These are the original target coordinates, not the motion coordinates corrected
through calibration.

The implementation applies the established target and board corrections internally
before IK; the coordinates documented here remain the original task coordinates.

## All Debug Groups

| Group | Terminal | Motion | Purpose |
|---:|---|---|---|
| 1 | T1 | No | Build the workspace and run functional tests |
| 2 | T1 | Simulation | Start fake `ros2_control` and keep the terminal occupied |
| 3 | T2 | Simulation | Check the fake controllers, then plan and execute a test trajectory |
| 4 | T1 | Possible | Start the physical Homing/EtherCAT bridge and keep the terminal occupied |
| 5 | T2 | Yes | Home Drives 0, 1, and 2 individually |
| 6 | T2 | Yes | Run `home_all` for Drives 0-2, then complete the Drive 3 reference move and zeroing |
| 7 | T2 | Yes | Start physical CSP `ros2_control` with Drive 3 participating and keep the terminal occupied |
| 8 | T3 | No | Check that the controllers and joint states remain stable for 10 seconds |
| 9 | T3 | No | Plan the physical minimum-jerk trajectory without executing it |
| 10 | T3 | Yes | Execute the physical trajectory successfully planned by group 9 |
| 11 | T3 | No | Check for residual processes and an occupied TCP port |
| 12 | Any | No | Package the complete ROS logs |
| 13 | T3 | No | Display the current model TCP coordinates |
| 14 | T3 | No | Set the next target TCP coordinates and motion duration |
| 15 | T3 | Yes | Open or close the gripper, or enter custom relative counts for Drive 3 |
| 16 | T3 | No | Display the latest automatic CSP stall snapshot |
| 17 | T2/T3 | No | Read the absolute Drive 3 counts relative to the Method 37 zero |
| 18 | T2 | No | Read the input mappings and Drive 2 protection parameters before CSP |
| 19 | T2 | Yes | Fine-trim Drive 0 with relative counts after Homing |
| 20 | T2 | Yes | Fine-trim Drive 1 with relative counts after Homing |
| 21 | T2 | Yes | Fine-trim Drive 2 with relative counts after Homing |
| 22 | T2 | No | Set the current Drive 0-2 pose as the Home pose for this session |
| 23 | T3 | Yes | Enter a target, then plan and execute it immediately |
| 24 | T3 | Yes | Task 1 stage 1: move block 1 |
| 25 | T3 | Yes | Task 1 stage 2: move block 3 to the temporary position |
| 26 | T3 | Yes | Task 1 stage 3: move block 2 onto block 1 |
| 27 | T3 | Yes | Task 1 stage 4: move block 3 to the top of the tower |
| 28 | T3 | Yes | Run Task 1 stages 1-4 continuously |
| 29 | T3 | Yes | Start the long-running `wp3_tsk2` node that receives cube centres on `/goal_poses` |

For the complete Homing, CSP, fault-handling, and safety procedures, see
`WP3_Physical_Hardware_Debug_Guide.md`.
