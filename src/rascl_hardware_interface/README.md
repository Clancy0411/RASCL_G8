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

The launch defaults to three-axis Homing plus four-axis CSP:
`skip_spur_gear_homing:=true` means `home_all` homes only Drives 0–2. The
pre-installed Drive 3 (`spur_gear_joint`) does not run a reference search, but
is explicitly brought through its CiA-402 enable sequence before CSP/PDO
activation and is validated like every other CSP drive. Keep
`ignore_spur_gear_in_csp:=false`; its `true` setting is only an emergency
three-axis fallback when Drive 3 has a hardware fault.

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
every CSP target from `0x6064`. Drives 0–2 preserve their Homing Operation
Enabled state; the non-homed Drive 3 is separately enabled before the handoff.
CSP is rejected if a Homing-required drive did not finish or any CSP drive
stopped being Operation Enabled. Support the arm before stopping ros2_control
because shutdown disables the drives.

The defaults select `control_mode:=csp`, `controllers_csp.yaml`, a 20 ms PDO
cycle, and SM-Sync. Before entering CSP, the bridge writes and reads back the
FAULHABER U16 `0x2332:00` Cyclic Mode Interpolation Rate as `200`
(`20 ms / 100 us`) for Drives 0-3. This
interpolates each 20 ms target update inside the drive instead of applying it
as a 100 us step. Profile Position remains available only as a regression
fallback by explicitly selecting both the profile mode and controller config.

For Drive 2 (`lowerarm_joint`), the Homing/CSP launch also reads `0x607B` and
`0x607D` and configures a finite session following-error monitor of `0x6065 =
25000` raw counts and `0x6066 = 250` ms. It does not write either position-limit
object or issue a parameter-store request. The `read_drive2_diagnostics`
service is available before CSP/PDO activation to report the values.
Transient PRE-OP mailbox WKC errors are retried. Failure to read the optional
`0x607B`/`0x607D` report no longer aborts bridge startup; the following-error
write/readback remains mandatory, and the service retries limit reporting
after Homing.

At the CSP handoff, every participating drive is configured with the symmetric
session-only directional limits `0x60E0 = 0x60E1 = 1000`, where `1000` is 100%
of rated motor torque. The MC5004 EtherCAT firmware used on the robot rejects
writes to `0x6072` as read-only, so the bridge observes that effective maximum
but never writes it. On this firmware, `0x6072` is derived from
`0x2329:03 peak_current / 0x2329:01 rated_current * 1000`. Drive 2 was
configured as `220 / 1100 * 1000 = 200`, so changing only `0x60E0/0x60E1`
could not raise its effective ceiling. At CSP handoff, the bridge now raises
only Drive 2's undersized `0x2329:03` to the current limit's required value
(`220 -> 1100 mA` for the default 1000-per-mille limit), then verifies both
that write and read-only `0x6072 >= 1000`. Drive 2's rated and continuous
currents are unchanged, as are all motor-current parameters on Drives 0, 1,
and 3. The correction is applied only after Homing, so the validated
reference-search behavior is unchanged. Any readback mismatch rejects CSP.
The limit is exposed as `csp_torque_limit_per_mille` (valid range 1--6000),
but the default stops at rated torque and no parameter-store request is issued.

When the PDO loop detects a drive fault or following error, it also captures a
best-effort read-only `DRIVE_DIAG` SDO snapshot before requesting SAFE-OP. It
includes `0x2324:01`, `0x1001`, `0x1003`, following error, actual velocity,
torque demand/actual value, actual current, torque/speed limits, the motor's
rated/continuous/peak current (`0x2329:01/:02/:03`), and the position-loop
gain `0x2348:01`. The
snapshot is appended to the existing CSP fault text, so normal log collection
captures it without a second TCP client. A `TORQUE_SNAPSHOT` is appended as
well, recording torque demand/actual value and all three torque limits for
Drives 0--3 at the same fault boundary.

If the network interface has a different name, replace `robot_interface` with the actual interface name.

## Home Service

The bridge provides reference-switch homing services:

```bash
ros2 service call /rascl_faulhaber_bridge/home_all \
  std_srvs/srv/Trigger "{}"
```

After successful Homing, the first three joint positions should be approximately:

```text
[0.0, +1.5708, +1.5708]
```

Drive 3 has no Homing zero in this workflow, but it participates in both CSP
state validation and position targets. `rascl_debug.sh` group `15` accepts a
signed **relative** Drive 3 encoder increment in counts. It adds that increment
to the current joint state using the configured direction and counts per
revolution, then publishes a 50 Hz minimum-jerk CSP trajectory through the
active position controller while holding the three arm joints at their current
positions. The default average speed is 10000 counts/s; group `15` derives a
safe minimum duration and records `SPUR_TRACE` feedback in the ROS log.

The drive-level `homing_offsets` (`0x607C`) remain `[0,0,0,0]`, preserving the
validated reference search for Drives 0--2; Drive 3 does not execute that
search. For the arm axes, the ros2_control parameters
`direction=[+1,+1,+1]` and `home_offset_counts=[0,-802816,-802816]` map the
switch pose to the URDF angles above. Drive 2's direction and offset
are paired so positive lowerarm commands match physical motion. The conversion is:

```text
q = direction * (raw_counts - home_offset_counts) / counts_per_rad
```

The nominal values assume exactly 90 degrees. For final calibration, record
each drive's raw `0x6064` value in the physical URDF zero pose and use those
counts as the corresponding `*_home_offset_counts` launch arguments. Keep the
Homing bridge running after `home_all`, call its `disable_all` service, support
the links while moving to the validated physical URDF-zero pose, and send
`GET_ALL` to `127.0.0.1:15001`. Because Drive 3 is not homed in this workflow,
its raw position is not a Homing calibration value. See the Chinese Debug Guide
for the guarded procedure and response layout.

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
