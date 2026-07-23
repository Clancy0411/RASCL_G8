# rascl_wp3_ss26_group8

This package contains the first WP3 application code for Group 8.

The current milestone focuses on Task 1 preparation:

- one Cartesian target is given in the `base_link` coordinate frame,
- the tool center point (TCP) is the calibrated fixed `tcp_link`,
- the node solves inverse kinematics for the three arm joints,
- it generates a joint-space minimum-jerk trajectory,
- it publishes the trajectory to `/rascl_position_controller/commands`,
- after execution it requires fresh `/joint_states` feedback and verifies the
  final four-joint and TCP errors before returning success.

This version does **not** yet control the gripper or constrain arbitrary
end-effector orientation. Joint position samples are executed through the
FAULHABER CSP mode and cyclic EtherCAT Position PDOs on real hardware.

## Coordinate convention

All target coordinates are expressed in the URDF `base_link` frame, in meters.
The TCP is the fixed `tcp_link` attached to `lowerarm`; it does not move when
`spur_gear_joint` opens or closes the gripper.  It is 40 mm outward from the
calibrated gear-surface reference along `lowerarm` +X: the prior 20 mm
grasp-center shift plus a second 20 mm user-requested X shift.

Calibration convention for real hardware:

1. Place the real robot in the validated safe starting region for the reference search.
2. Run the dedicated `homing.launch.py`, validate each axis with `home_one`, then call `home_all`.
3. Keep that bridge running and start ros2_control with `start_bridge:=false`.
4. Check that the calibrated automatic-Home pose reads approximately `[0,+pi/2,+pi/2,0] rad`.

The joint-coordinate convention from `3588dc98` is preserved:
`q=[0,0,0,0]` still corresponds to the physical URDF zero pose. With the
calibrated TCP definition, its nominal TCP is:

```text
base_link TCP = [0.31318978, -0.01580108, 0.07181469] m
```

The reference-switch pose is physically different. With nominal
`direction=[+1,+1,+1,+1]` and `home_offset_counts=[0,-802816,-802816,0]`, it is represented as
`q=[0,+pi/2,+pi/2,0]`, whose model TCP is:

```text
base_link auto-home TCP = [0.22318978, -0.01580108, 0.32181469] m
```

The current TCP uses a second single-pose calibration.  At the joint pose that
the former model reported as `[0.16, -0.16, 0.05] m`, the external Y/X/Z
measurement was `[0.14, -0.16, 0.05] m`; in the project's physical-axis
convention this is the numeric `base_link` XYZ correction `[-0.020, 0, 0] m`.
The resulting calibrated gear-surface origin in `lowerarm` is
`[0.11478978, 0.02881369, 0.03193108] m`.  The commanded TCP now adds
40 mm along the local jaw direction and is therefore
`[0.15478978, 0.02881369, 0.03193108] m`.  The surface calibration intentionally
prioritizes that measured pose and must be rechecked at Home and other arm poses.

The drive-level `0x607C homing_offsets` stay zero. Final real-hardware
calibration should replace the nominal count offsets with raw `0x6064` counts
measured in the physical URDF zero pose.

The positive/negative table directions of `base_link` should be checked in RViz
before using real hardware.

## Build

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
rm -rf build install log
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/local_setup.bash
export ROS_DOMAIN_ID=88
```

## Fake hardware validation

Start the robot stack with fake hardware:

```bash
ros2 launch rascl_description ros2_control.launch.py use_fake_hardware:=true
```

In a second terminal inside the same container:

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=88

ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
  -p target_x:=0.25 \
  -p target_y:=0.00 \
  -p target_z:=0.08 \
  -p duration:=4.0 \
  -p rate_hz:=50.0 \
  -p execute:=false
```

`execute:=false` only performs IK and writes the generated CSV.  After checking
the printed IK result and the CSV, run with `execute:=true`:

```bash
ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
  -p target_x:=0.25 \
  -p target_y:=0.00 \
  -p target_z:=0.08 \
  -p duration:=4.0 \
  -p rate_hz:=50.0 \
  -p execute:=true
```

Successful execution prints `MOTION_RESULT reached=true`. The default final
tolerances are `0.03 rad` per joint and `0.01 m` at the TCP. A publish loop that
finishes while one or more drives remain short of the endpoint exits non-zero;
`rascl_debug.sh` group `10` then retrieves the bridge's latest staged
`CSP_STALL_SNAPSHOT`, and group `16` can display it again.

## Combined launch

The WP3 launch file can also start the existing robot stack:

```bash
ros2 launch rascl_wp3_ss26_group8 wp3_tsk1.launch.py \
  start_robot:=true \
  use_fake_hardware:=true \
  target_x:=0.25 \
  target_y:=0.00 \
  target_z:=0.08 \
  duration:=4.0 \
  rate_hz:=50.0 \
  execute:=true
```

For real hardware, first validate in fake hardware. Run reference-switch Homing
with `rascl_description homing.launch.py` and do not stop it after `home_all`.
Start `ros2_control.launch.py` with `start_bridge:=false`; the same EtherCAT
master then maps PDOs and enters CSP without a Shutdown/Disable controlword.
Start with targets near the automatic-Home TCP.

## CSP/PDO execution path

```text
wp3_tsk1 minimum-jerk samples
  -> ForwardCommandController position interface
  -> RASCLHardwareInterface target-count cache
  -> rascl_faulhaber_bridge fixed 20 ms loop
  -> RxPDO2: 0x6040 Controlword + 0x607A Target Position
  <- TxPDO2: 0x6041 Statusword + 0x6064 Position Actual Value
```

The bridge uses the FAULHABER factory Position PDO mappings (`0x1601` and
`0x1A01`) and assigns only those PDOs to SyncManager 2/3. The default is
SM-Sync at 50 Hz; DC-Sync is optional after stable SM-Sync validation.
