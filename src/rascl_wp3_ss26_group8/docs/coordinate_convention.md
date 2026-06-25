# WP3 Coordinate Convention

Frame: `base_link` from `rascl_description/urdf/rascl.urdf`.

Unit: meter.

TCP for the first milestone: `spur_gear_joint` origin.

Calibration pose for real hardware: the physical robot must be placed in the
same pose as the URDF/RViz zero pose.  After calling `home_all`, the four ROS
joint positions must be:

```text
[shoulder_joint, upperarm_joint, lowerarm_joint, spur_gear_joint] = [0, 0, 0, 0] rad
```

In this pose, the kinematic model gives:

```text
TCP in base_link = [0.29756, -0.00177, 0.043001] m
```

This convention must remain fixed during WP3.  If `home_all` is called in a
different physical pose, Cartesian targets will no longer match the model.
