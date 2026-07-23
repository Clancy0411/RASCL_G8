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
pre-installed and deliberately skips Homing: it retains its power-on measured
position, joins CSP/PDO, and is commanded by relative encoder-count increments.
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

After automatic Home, first read `/joint_states`. Drive 3 is intentionally not
homed in the normal workflow, so preserve its measured position in any manual
four-joint command. Do not use a fixed `0.0` for `spur_gear_joint` unless that
is its actual current position. A nominal arm-only example, with
`<current_spur_rad>` replaced by feedback, is:

```bash
ros2 topic pub --once /rascl_position_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 1.5708, 1.5708, <current_spur_rad>]}"
```

Do not send `[0,0,0,0]` as the first real-hardware command; it requests the
physically different URDF-zero pose and may also move the unhomed gripper.

The values correspond to:

```text
[shoulder_joint, upperarm_joint, lowerarm_joint, spur_gear_joint]
```

For the CSP session, `rascl_debug.sh` group `15` is the supported gripper
command. Entering ASCII `close` (or `c`) applies a relative `-110000` count
grip/close move; entering `open` (or `o`) applies a relative `+110000` count
release/open move. A signed non-zero integer instead requests that exact
relative Drive 3 increment in counts. Every command starts from the current
spur joint state, then publishes a 50 Hz minimum-jerk
four-joint CSP trajectory that preserves the measured arm pose. At the default
10000 counts/s, the duration is derived automatically from the requested
increment. It does not require Drive 3 Homing. Repeating commands accumulates
another relative move, subject to the URDF limit. Do not send a direct Profile Position
command while ros2_control owns the CSP connection.

For Cartesian Task 1 moves, group `10` now reports success only after fresh
joint feedback satisfies the endpoint joint/TCP tolerances. If a drive remains
Operation Enabled but stops progressing, the bridge records a staged
`CSP_STALL_SNAPSHOT`; group `10` prints it automatically on failure and group
`16` returns the most recent snapshot without opening another EtherCAT client.

## Notes

This package does not implement the low-level EtherCAT communication itself. The actual hardware access is implemented in the `rascl_hardware_interface` package.
