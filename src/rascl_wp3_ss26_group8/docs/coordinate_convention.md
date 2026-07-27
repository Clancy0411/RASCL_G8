# WP3 Coordinate Convention

Frame: `base_link` from `rascl_description/urdf/rascl.urdf`.

Unit: meter.

This branch is the uncompensated measurement baseline.  Its TCP is the CAD
`spur_gear_joint` origin in `lowerarm`:

```text
[0.13916, 0, 0.0179] m
```

The former single-pose `tcp_link`, grasp-center extension, and all measured
base-frame XY translations have been removed. The latest calibrated shoulder
origin `[0.040, 0.020, 0.057441] m` is restored to the drawing value
`[0, 0, 0.057441] m`. External measurements may still be recorded as physical
Y/X/Z, but that convention is not converted into an automatic model correction
in this baseline.

Calibration for real hardware uses the validated reference-switch search from
the `auto_homing` branch. Start in its safe search region, validate each axis
with `home_one`, and then call `home_all`. The automatic-Home switch pose is not
the URDF zero pose. With the nominal software count calibration, it must read:

```text
[shoulder_joint, upperarm_joint, lowerarm_joint, spur_gear_joint]
= [0, +1.570796327, +1.570796327, 0] rad
```

In this automatic-Home pose, the kinematic model gives:

```text
TCP in base_link = [0.20756, -0.00177, 0.293001] m
```

The original physical URDF-zero joint-angle convention remains:

```text
q = [0, 0, 0, 0] rad
TCP in base_link = [0.29756, -0.00177, 0.043001] m
```

Use the same physical marker on the gripper for every measurement.  The CAD
joint origin is a reproducible software reference, but it is not automatically
the jaw contact center.  Record the raw discrepancy at Home and at several
additional poses before fitting a new base transform or TCP offset.

The bridge's drive-level `0x607C homing_offsets` remain zero. The hardware
interface applies nominal `direction=[+1,+1,+1,-1]` and
`home_offset_counts=[0,-802816,-802816,0]`; Drive 2's sign and offset are a
paired mapping so positive lowerarm commands match physical motion. Final values must be replaced by raw
counts measured in the physical URDF-zero pose after a successful Home, with
the drives disabled and the links supported. The guarded `GET_ALL` measurement
procedure is documented in `WP3_Task1_MinJerk_Debug_Guide_CN.md`.
Changing a homing method, reference input, direction, or either offset layer can
invalidate the relationship between Cartesian targets and the real robot.

The Homing bridge must remain running for the CSP transition. ros2_control is
started with `start_bridge:=false`, so it reuses the same EtherCAT master and
initializes CSP targets from the measured Home positions.
