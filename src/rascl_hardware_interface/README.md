# rascl_hardware_interface

This package implements the custom `ros2_control` hardware interface for the RASCL robot.

## Package Role

`rascl_hardware_interface` connects the ROS 2 controller framework to the Faulhaber motion controllers used in the RASCL robot.

The package provides:

* a `ros2_control` hardware interface plugin,
* joint state and command interface export,
* conversion between motor encoder counts and ROS joint positions in radians,
* fake-hardware support for software-side testing,
* a Python TCP bridge for communication with the Faulhaber controllers via EtherCAT,
* a home service for defining the current robot pose as the software zero position,
* optional automated tests for the hardware interface lifecycle.

## Main Files and Directories

```text
rascl_hardware_interface/
├── include/
│   └── rascl_hardware_interface/
│       └── rascl_hardware_interface.hpp
├── src/
│   └── rascl_hardware_interface.cpp
├── scripts/
│   └── rascl_faulhaber_bridge.py
├── test/
│   └── test_generic_system.cpp
├── rascl_hardware_interface.xml
├── CMakeLists.txt
└── package.xml
```

### `include/rascl_hardware_interface/rascl_hardware_interface.hpp`

Declares the custom hardware interface class and its internal helper data structures.

The implemented `ros2_control` lifecycle and interface methods include:

* `on_init`
* `on_configure`
* `on_activate`
* `on_deactivate`
* `on_cleanup`
* `export_state_interfaces`
* `export_command_interfaces`
* `read`
* `write`

### `src/rascl_hardware_interface.cpp`

Implements the hardware interface logic.

Its main responsibilities are:

* reading hardware parameters from the URDF `ros2_control` block,
* creating state and command interfaces for all four joints,
* handling fake-hardware operation,
* connecting to the Faulhaber TCP bridge in real-hardware mode,
* reading actual joint positions,
* writing target joint positions,
* converting between encoder counts and radians,
* applying the configured home offset.

### `scripts/rascl_faulhaber_bridge.py`

Implements the Python bridge between the C++ hardware interface and the Faulhaber EtherCAT devices.

The C++ hardware interface communicates with this script through a local TCP socket.

The bridge is responsible for:

* opening the EtherCAT interface,
* enabling the Faulhaber drives,
* reading actual motor positions and status words,
* sending target positions,
* setting the current pose as the software home position.

Make sure the script is executable:

```bash
chmod +x src/rascl_hardware_interface/scripts/rascl_faulhaber_bridge.py
```

### `test/test_generic_system.cpp`

Contains optional automated tests for the hardware interface.

These tests only validate the software-side `ros2_control` behavior in fake-hardware mode. They do not connect to EtherCAT, do not start the Faulhaber bridge, do not enable any motor, and do not move the real robot.

## Building

Normal build for operation:

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash

rm -rf build install log
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF

source install/local_setup.bash
export ROS_DOMAIN_ID=88
```

## Running with Fake Hardware

Fake hardware can be started through the description package launch file:

```bash
ros2 launch rascl_description ros2_control.launch.py use_fake_hardware:=true
```

This mode is useful for checking the ROS 2 control stack without connecting to the physical robot.

## Running with Real Hardware

Real hardware mode requires the correct EtherCAT network interface:

```bash
ros2 launch rascl_description ros2_control.launch.py \
  interface:=robot_interface \
  use_fake_hardware:=false
```

If the network interface has a different name, replace `robot_interface` with the actual interface name.

## Home Service

The bridge provides a service for defining the current robot pose as the software home position:

```bash
ros2 service call /rascl_faulhaber_bridge/home_all \
  std_srvs/srv/Trigger "{}"
```

After calling this service, the current joint positions are treated as:

```text
[0.0, 0.0, 0.0, 0.0]
```

To move back to this software home position:

```bash
ros2 topic pub --once /rascl_position_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0]}"
```

## Optional Automated Test

To build and run the optional hardware interface test:

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash

rm -rf build install log
colcon build --symlink-install \
  --packages-select rascl_hardware_interface \
  --cmake-args -DBUILD_TESTING=ON

source install/local_setup.bash

colcon test --packages-select rascl_hardware_interface \
  --ctest-args -R test_generic_system --output-on-failure

colcon test-result --verbose
```

A successful result should report no errors or failures, for example:

```text
Summary: x tests, 0 errors, 0 failures, 0 skipped
```

## Notes

The automated test is not a replacement for real-hardware validation. It only checks the software-side hardware interface behavior. EtherCAT communication, motor direction, homing behavior, joint calibration, and mechanical safety must still be validated on the physical RASCL robot.
