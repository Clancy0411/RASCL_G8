# WP3 Coordinate Convention

Frame: `base_link` from `rascl_description/urdf/rascl.urdf`.

Unit: meter.

TCP for the first milestone: fixed `tcp_link`, attached to `lowerarm` at
`[0.170, 0, 0.0179] m`. The measured ideal TCP is 170 mm along lowerarm +X;
the drawing's independent 17.9 mm perpendicular offset remains unchanged.
The physical `spur_gear_joint` stays at `[0.13916, 0, 0.0179] m`, so it is
30.84 mm behind the planning TCP and its motion remains independent.

Base-to-shoulder geometry uses the uncompensated CAD alignment:
`base_link -> shoulder_joint = [0, 0, 0.057441] m`. The former single-point
XY correction `[+0.040, +0.040, 0] m` has been removed so the shoulder
rotation axis remains centered on the base. Future physical calibration must
identify the actual geometric or encoder parameter instead of displacing this
joint in the model.

Calibration for real hardware starts with the validated reference-switch search
from the `auto_homing` branch, then refines zero to the sensor interval centre.
Each arm drive records the entry and exit counts, returns to
`(entry + exit) / 2`, and uses Method 37 to make that midpoint raw zero. Start
in the same safe search region, validate each axis with `home_one`, and then
call `home_all`. With the nominal software count calibration, this automatic
Home pose must read:

```text
[shoulder_joint, upperarm_joint, lowerarm_joint, spur_gear_joint]
= [0, +1.570796327, +1.570796327, 0] rad
```

In this automatic-Home pose, the kinematic model gives:

```text
TCP in base_link = [0.23840, -0.00177, 0.293001] m
```

The original physical URDF-zero joint-angle convention remains; the listed TCP
uses the new calibrated `tcp_link`:

```text
q = [0, 0, 0, 0] rad
TCP in base_link = [0.32840, -0.00177, 0.043001] m
```

The earlier single-pose gear-surface and grasp-center offsets are no longer
applied. Current calibration measurements refer to the measured ideal
`tcp_link`, not the physical spur-gear axis. No base-to-shoulder XY displacement
is active.

An optional fixed-board task correction is applied only when
`apply_board_xy_compensation:=true`. It transforms the requested board-plane
XY target before IK and leaves Z, Home, encoder offsets, and URDF geometry
unchanged. `rascl_debug.sh` groups `9` and `10` enable this correction with the
same fitted coefficients; direct `wp3_tsk1` and launch calls default to off.

The 2026-07-29 fit used the following measured command corrections in the board
plane. Coordinates and corrections in this table are millimetres:

| Board X | Board Y | Delta X | Delta Y | Note |
| ---: | ---: | ---: | ---: | --- |
| -30 | 170 | 0 | 0 | Three independent Home cycles |
| -90 | 210 | -2 | +2 | |
| +140 | 210 | +2 | +2 | |
| -160 | 40 | -2 | -2 | Repeatable closed-loop check |
| -160 | 160 | -6 | +2 | Repeatable closed-loop check |
| -230 | 50 | 0 | -5 | |
| +250 | 30 | +2 | -2 | |
| -20 | 250 | -4 | +2 | |
| +230 | 100 | 0 to +2 | 0 to +2 | Fit uses midpoint (+1,+1) |

The fitted correction, with metres as the unit, is:

```text
x_corrected = 1.0098577586*x - 0.0114794948*y + 0.0006327808
y_corrected = 0.0041074758*x + 1.0252229617*y - 0.0033871656
z_corrected = z
```

Its measured component-wise RMS residual is approximately 1.5 mm. It is a
continuous board-workspace correction, not a lookup table for only the listed
points. Do not reinterpret its rotation component as a shoulder Home offset.

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
