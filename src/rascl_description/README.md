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
ros2 launch rascl_description ros2_control.launch.py \
  interface:=robot_interface \
  use_fake_hardware:=false
```

If the EtherCAT network interface is not named `robot_interface`, replace it with the actual interface name.

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

Start real hardware:

```bash
ros2 launch rascl_description ros2_control.launch.py \
  interface:=robot_interface \
  use_fake_hardware:=false
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

Example:

```bash
ros2 topic pub --once /rascl_position_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0]}"
```

The values correspond to:

```text
[shoulder_joint, upperarm_joint, lowerarm_joint, spur_gear_joint]
```

## Notes

This package does not implement the low-level EtherCAT communication itself. The actual hardware access is implemented in the `rascl_hardware_interface` package.
