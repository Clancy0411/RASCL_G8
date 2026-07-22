# WP3 Coordinate Convention

Frame: `base_link` from `rascl_description/urdf/rascl.urdf`.

Unit: meter.

TCP for the first milestone: calibrated fixed `tcp_link`, attached to
`lowerarm` and independent of `spur_gear_joint` motion.

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
TCP in base_link = [0.18318978, -0.01580108, 0.32181469] m
```

The original physical URDF-zero joint-angle convention remains; the listed TCP
uses the new calibrated `tcp_link`:

```text
q = [0, 0, 0, 0] rad
TCP in base_link = [0.27318978, -0.01580108, 0.07181469] m
```

The current 2026-07-22 TCP definition applies a second single-pose calibration.
At the joint pose that the former model reported as `[0.16, -0.16, 0.05] m`,
the external Y/X/Z measurement was `[0.14, -0.16, 0.05] m`.  Under the
project's physical-axis convention, the correction is `[-0.020, 0, 0] m` in
numeric `base_link` XYZ.  Transforming that correction into `lowerarm` gives
the fixed origin `[0.11478978, 0.02881369, 0.03193108] m`.  This calibration
prioritizes that measured pose; Home and additional poses must be rechecked.

The bridge's drive-level `0x607C homing_offsets` remain zero. The hardware
interface applies nominal `direction=[+1,+1,+1,+1]` and
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
