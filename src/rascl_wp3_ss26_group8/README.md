# rascl_wp3_ss26_group8

This package contains the first WP3 application code for Group 8.

The current milestone focuses on Task 1 preparation:

- one Cartesian target is given in the `base_link` coordinate frame,
- the tool center point (TCP) is defined as the `spur_gear_joint` origin,
- the node solves inverse kinematics for the three arm joints,
- it generates a joint-space minimum-jerk trajectory,
- it publishes the trajectory to `/rascl_position_controller/commands`.

This first version does **not** control the gripper and does **not** constrain the
end-effector orientation.  It is intended for step-by-step validation before the
full cube stacking sequence is assembled.

## Coordinate convention

All target coordinates are expressed in the URDF `base_link` frame, in meters.
The TCP is currently the `spur_gear_joint` origin.

Calibration convention for real hardware:

1. Place the real robot in the same physical pose as the URDF/RViz zero pose.
2. Call the existing `home_all` service.
3. Check that all four joints read `0 rad`.

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
  -p rate_hz:=10.0 \
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
  -p rate_hz:=10.0 \
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
  rate_hz:=10.0 \
  execute:=true
```

For real hardware, first validate in fake hardware.  Then use the existing real
hardware startup, calibrate the robot in the fixed zero pose, call `home_all`, and
run the same WP3 node with conservative targets and `rate_hz:=10.0`.

## Current limitation

This package generates and publishes minimum-jerk position trajectories on the
ROS side.  The lower-level WP2.2 bridge still uses the previous position command
path.  True motion-controller CSP mode will be added in a later step.
