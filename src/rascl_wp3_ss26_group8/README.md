# rascl_wp3_ss26_group8

This package contains the first WP3 application code for Group 8.

The current milestone focuses on Task 1 preparation:

- one Cartesian target is given in the `base_link` coordinate frame,
- the tool center point (TCP) is defined as the `spur_gear_joint` origin,
- the node solves inverse kinematics for the three arm joints,
- it generates a joint-space minimum-jerk trajectory,
- it publishes the trajectory to `/rascl_position_controller/commands`.

This version does **not** yet control the gripper or constrain arbitrary
end-effector orientation. Joint position samples are executed through the
FAULHABER CSP mode and cyclic EtherCAT Position PDOs on real hardware.

## Coordinate convention

All target coordinates are expressed in the URDF `base_link` frame, in meters.
The TCP is currently the `spur_gear_joint` origin.

Calibration convention for real hardware:

1. Place the real robot in the validated safe starting region for the reference search.
2. Run the dedicated `homing.launch.py`, validate each axis with `home_one`, then call `home_all`.
3. After the switches and offsets establish the URDF zero pose, check that all four joints read `0 rad`.

With this convention, `q=[0,0,0,0]` corresponds to the URDF zero pose.  The
nominal TCP position in that pose is approximately:

```text
base_link TCP = [0.29756, -0.00177, 0.043001] m
```

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

For real hardware, first validate in fake hardware. Run reference-switch homing
with `rascl_description homing.launch.py`, stop that launch, and then start
`ros2_control.launch.py`, whose default real-hardware mode is CSP with a 20 ms
Position PDO cycle. Start with conservative Cartesian targets.

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
