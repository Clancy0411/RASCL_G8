# WP3 Coordinate Convention

Frame: `base_link` from `rascl_description/urdf/rascl.urdf`.

Unit: meter.

TCP for the first milestone: fixed `tcp_link`, attached to `lowerarm` and
co-located with the historical `spur_gear_joint` origin at
`[0.13916, 0, 0.0179] m` in `lowerarm`. It remains independent of
`spur_gear_joint` motion.

Global physical-X calibration: external measurements use Y/X/Z, so physical
+X maps to numeric `base_link` +Y. The current project shifts the modeled arm
origin by `-0.020 m` in `base_link` Y. Solving IK for an unchanged requested
target therefore moves the real gripper `+0.020 m` in physical X. This fixed
base-frame translation must not be folded into the rotating local TCP vector.

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
TCP in base_link = [0.20756, -0.02177, 0.293001] m
```

The original physical URDF-zero joint-angle convention remains; the listed TCP
uses the new calibrated `tcp_link`:

```text
q = [0, 0, 0, 0] rad
TCP in base_link = [0.29756, -0.02177, 0.043001] m
```

The earlier single-pose gear-surface and grasp-center offsets are no longer
applied. Current calibration measurements refer directly to the fixed
`spur_gear_joint` center. The separate global base-frame translation above
remains active and must be evaluated independently.

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
