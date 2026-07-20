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
* reference-switch homing plus a separate raw-count-to-URDF calibration,
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
* automatic reference-switch homing,
* CSP mode with cyclic EtherCAT Position PDO exchange.

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

## Reference-switch homing

Homing must run without an active CSP/PDO loop. Start the dedicated bridge:

```bash
ros2 launch rascl_description homing.launch.py interface:=robot_interface
```

The current launch defaults to temporary three-axis mode
(`ignore_spur_gear_in_csp:=true`). `home_all` homes only Drives 0–2; Drive 3
(`spur_gear_joint`) receives no Homing motion, is excluded from CSP admission
and state checks, and receives Disable Voltage in every PDO cycle. Do not rely
on it to hold a load. After its Homing fault is repaired, restore four-axis mode
with `ignore_spur_gear_in_csp:=false`.

Validate Drives 0–2 with `home_one`, or call `home_all` once. Keep the Homing
launch running until the complete CSP session has ended. For a gravity-loaded
arm, stopping it between Homing and CSP removes drive voltage.

## Running with Real Hardware (CSP/PDO)

After `home_all`, keep that bridge running and start ros2_control without a
second bridge:

```bash
ros2 launch rascl_description ros2_control.launch.py \
  interface:=robot_interface \
  use_fake_hardware:=false \
  start_bridge:=false
```

The existing bridge defers PDO mapping until this activation and initializes
every CSP target from `0x6064`. Required drives request OP using only Enable
Operation; an ignored Drive 3 stays at Disable Voltage. CSP is rejected if
Homing of a required drive did not finish or a required drive stopped being
Operation Enabled. Support the arm before stopping ros2_control because
shutdown disables the drives.

The defaults select `control_mode:=csp`, `controllers_csp.yaml`, a 20 ms PDO
cycle, and SM-Sync. Profile Position remains available only as a regression
fallback by explicitly selecting both the profile mode and controller config.

If the network interface has a different name, replace `robot_interface` with the actual interface name.

## Home Service

The bridge provides reference-switch homing services:

```bash
ros2 service call /rascl_faulhaber_bridge/home_all \
  std_srvs/srv/Trigger "{}"
```

After successful Homing in temporary three-axis mode, the first three joint
positions should be:

```text
[0.0, +1.5708, +1.5708]
```

Drive 3 is unhomed in this mode, so its reported position is not an acceptance
value and position commands for it are ignored by the bridge.

The drive-level `homing_offsets` (`0x607C`) remain `[0,0,0,0]`, preserving the
validated reference search. The ros2_control parameters
`direction=[+1,+1,+1,+1]` and `home_offset_counts=[0,-802816,-802816,0]`
map the switch pose to the URDF angles above. Drive 2's direction and offset
are paired so positive lowerarm commands match physical motion. The conversion is:

```text
q = direction * (raw_counts - home_offset_counts) / counts_per_rad
```

The nominal values assume exactly 90 degrees. For final calibration, record
each drive's raw `0x6064` value in the physical URDF zero pose and use those
counts as the corresponding `*_home_offset_counts` launch arguments. Keep the
Homing bridge running after `home_all`, call its `disable_all` service, support
the links while moving to the validated physical URDF-zero pose, and send
`GET_ALL` to `127.0.0.1:15001`. In temporary three-axis mode only the Drive 0–2
raw-position fields are valid calibration values; the unhomed Drive 3 value is
not. See the Chinese Debug Guide for the guarded procedure and response layout.

Do not call homing services while the CSP ros2_control stack is active. Also do
not publish `[0,0,0,0]` as the first command after automatic homing: that is the
old URDF zero pose and requires approximately 90-degree motions of Drive 1/2.

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

ctest --test-dir build/rascl_hardware_interface \
  -R '^(test_generic_system|test_faulhaber_bridge)$' \
  --output-on-failure
```

A successful functional result should report both selected targets as passed.
The package also defines optional clang-format/cpplint checks; style or missing
copyright-header findings are not real-hardware functional failures.

```text
100% tests passed, 0 tests failed out of 2
```

## Notes

The automated test is not a replacement for real-hardware validation. It only checks the software-side hardware interface behavior. EtherCAT communication, motor direction, homing behavior, joint calibration, and mechanical safety must still be validated on the physical RASCL robot.
