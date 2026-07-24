# rascl_description

This package contains the robot description, visualization setup, controller configuration, and launch files for the RASCL robot.

## Package Role

`rascl_description` defines the kinematic and visual structure of the RASCL robot and provides the configuration files required to start the robot with `ros2_control`.

It includes:

* the robot URDF/Xacro description,
* mesh files for visualization in RViz,
* controller configuration for `ros2_control`,
* launch files for visualization and hardware-control startup,
* an RViz configuration file.

## Main Files and Directories

```text
rascl_description/
├── config/
│   └── controllers.yaml
├── launch/
│   ├── display.launch.py
│   └── ros2_control.launch.py
├── meshes/
├── rviz/
│   └── urdf.rviz
└── urdf/
    └── rascl.urdf
```

### `urdf/rascl.urdf`

Defines the robot links, joints, transmission information, and the `ros2_control` hardware configuration block.

The command order used by the position controller is:

```text
[shoulder_joint, upperarm_joint, lowerarm_joint, spur_gear_joint]
```

All joint positions are represented in radians on the ROS side.

For real hardware, the raw reference-switch counts of Drive 0--2 are mapped to
the fixed URDF joint convention through per-joint `direction` and
`home_offset_counts`. The nominal arm-axis defaults are:

```text
[shoulder_joint, upperarm_joint, lowerarm_joint]
direction          = [+1, +1, +1]
home_offset_counts = [0, -802816, -802816] counts
```

Therefore raw zero after automatic Homing is represented as
`[0,+pi/2,+pi/2] rad` for those three axes. Drive 3 (`spur_gear_joint`) is
pre-installed and deliberately skips the sensor reference search. After Drives
0–2 Home, it moves `-50000` counts from its live position and uses Homing Method
37 to define the reached position as `0` counts before joining CSP/PDO.
Subsequent gripper commands remain relative encoder-count increments.
These parameters do not change URDF joint origins, the IK geometry, or fake
hardware's initial `q=[0,0,0,0]` pose. Drive 2 uses the paired
`lowerarm_direction:=1` and `lowerarm_home_offset_counts:=-802816` mapping so
that positive lowerarm commands agree with its physical motion. Override the
direction/offset pair only together when recalibrating Drive 2.

### `config/controllers.yaml`

Defines the controllers used by the system:

* `joint_state_broadcaster`
* `rascl_position_controller`

The `rascl_position_controller` is a forward command controller that accepts a `std_msgs/msg/Float64MultiArray` command.

### `launch/ros2_control.launch.py`

Starts the robot description, the hardware interface, the controller manager, and the required controllers.

For fake hardware:

```bash
ros2 launch rascl_description ros2_control.launch.py use_fake_hardware:=true
```

For the real robot:

```bash
# Keep homing.launch.py running after a successful home_all.
ros2 launch rascl_description ros2_control.launch.py \
  interface:=robot_interface \
  use_fake_hardware:=false \
  start_bridge:=false
```

If the EtherCAT network interface is not named `robot_interface`, replace it with the actual interface name.
`start_bridge:=false` reuses the Homing bridge and avoids a drive-disable gap.
This is the safe default. Use `start_bridge:=true` only for an explicit
standalone non-Homing diagnostic when no other EtherCAT master is running.

### `launch/display.launch.py`

Starts the robot model visualization without controlling the real hardware.

### `rviz/urdf.rviz`

RViz configuration for displaying the robot model and its joint-state-based motion.

## Basic Usage

After building and sourcing the workspace:

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=88
```

Start fake hardware:

```bash
ros2 launch rascl_description ros2_control.launch.py use_fake_hardware:=true
```

After `homing.launch.py` reports a successful `home_all`, keep it running and
start real hardware in another terminal:

```bash
ros2 launch rascl_description ros2_control.launch.py \
  interface:=robot_interface \
  use_fake_hardware:=false \
  start_bridge:=false
```

Open RViz in another terminal inside the same container:

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=88

rviz2 -d src/rascl_description/rviz/urdf.rviz
```

## Sending Joint Commands

Commands are sent to:

```bash
/rascl_position_controller/commands
```

After automatic Home, first read `/joint_states`. Drive 3 should be near zero
after its Method-37 reference, but preserve its measured position in every
manual four-joint command. Do not assume a fixed `0.0` if it has since moved. A
nominal arm-only example, with
`<current_spur_rad>` replaced by feedback, is:

```bash
ros2 topic pub --once /rascl_position_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 1.5708, 1.5708, <current_spur_rad>]}"
```

Do not send `[0,0,0,0]` as the first real-hardware command; it requests the
physically different arm URDF-zero pose.

The values correspond to:

```text
[shoulder_joint, upperarm_joint, lowerarm_joint, spur_gear_joint]
```

For the CSP session, `rascl_debug.sh` group `15` is the supported gripper
command. Entering ASCII `close` (or `c`) requests at most `+500000` counts and
stops early at object contact. Entering `open` (or `o`) requests an exact
`-200000`-count relative move. Only `close` uses tracking lag together with a
near-stationary encoder condition (`<=100 counts` progress for `0.10 s`) to hold
the measured Drive 3 position before expected contact becomes a following
error. This prevents ordinary minimum-jerk tracking lag from becoming a false
contact. `open` and signed non-zero integer commands request exact relative
increments and do not use contact termination. Every command starts from the
current spur joint state, then
publishes a 50 Hz minimum-jerk
four-joint CSP trajectory that preserves the measured arm pose. At the default
10000 counts/s, the duration is derived automatically from the requested
increment. It requires the session reference to have completed, but the command
itself remains relative. Repeating commands accumulates another move, subject
to the unified Drive 3 URDF/ros2_control limit
of `[-2*pi,+2*pi]` rad. This project limit does not overwrite drive objects
`0x607B` or `0x607D`. Do not send a direct Profile Position command while
ros2_control owns the CSP connection.

Group `17` reads the exact Drive 3 `absolute_counts` in the current Method-37
coordinate without commanding motion. Use it before and after small group `15`
increments to determine suitable absolute open and closed positions. Do not use
the force-based `close/c` shortcut during this calibration; it previously
damaged a gripper.

For Cartesian Task 1 moves, group `10` now reports success only after fresh
joint feedback satisfies the endpoint joint/TCP tolerances. If a drive remains
Operation Enabled but stops progressing, the bridge records a staged
`CSP_STALL_SNAPSHOT`; group `10` prints it automatically on failure and group
`16` returns the most recent snapshot without opening another EtherCAT client.

## Notes

This package does not implement the low-level EtherCAT communication itself. The actual hardware access is implemented in the `rascl_hardware_interface` package.
