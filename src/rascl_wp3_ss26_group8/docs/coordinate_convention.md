# WP3 Coordinate Convention

Frame: `base_link` from `rascl_description/urdf/rascl.urdf`.

Unit: meter.

TCP for the first milestone: `spur_gear_joint` origin.

Calibration for real hardware uses the validated reference-switch search from
the `auto_homing` branch. Start in its safe search region, validate each axis
with `home_one`, and then call `home_all`. After the switches and configured
offsets establish the URDF/RViz zero pose, the four ROS joint positions must be:

```text
[shoulder_joint, upperarm_joint, lowerarm_joint, spur_gear_joint] = [0, 0, 0, 0] rad
```

In this pose, the kinematic model gives:

```text
TCP in base_link = [0.29756, -0.00177, 0.043001] m
```

This switch/offset convention must remain fixed during WP3. Changing a homing
method, reference input, direction, or offset can invalidate the relationship
between Cartesian targets and the real robot.
