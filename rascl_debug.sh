#!/usr/bin/env bash

# Interactive command groups for the RASCL Homing -> CSP debug workflow.
# Run inside the ROS container: bash ./rascl_debug.sh

set -Eeuo pipefail

WORKSPACE="${RASCL_WS:-/root/ws}"
# EtherCAT NIC on the current workstation.  Override only when deliberately
# using another workstation: RASCL_INTERFACE=<nic> bash ./rascl_debug.sh.
INTERFACE="${RASCL_INTERFACE:-enx3c18a0256deb}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}"
# Drive 2's physical motion follows the positive lowerarm URDF direction.
# This paired offset preserves automatic Home at q=[0,+pi/2,+pi/2,0].
LOWERARM_DIRECTION="${RASCL_LOWERARM_DIRECTION:-1}"
LOWERARM_HOME_OFFSET_COUNTS="${RASCL_LOWERARM_HOME_OFFSET_COUNTS:--802816}"
DRIVE2_FOLLOWING_ERROR_WINDOW_COUNTS="${RASCL_DRIVE2_FOLLOWING_ERROR_WINDOW_COUNTS:-25000}"
DRIVE2_FOLLOWING_ERROR_TIMEOUT_MS="${RASCL_DRIVE2_FOLLOWING_ERROR_TIMEOUT_MS:-250}"
HOMING_INTERVAL_MAX_TRAVEL_DRIVE0_COUNTS="${RASCL_HOMING_INTERVAL_MAX_TRAVEL_DRIVE0_COUNTS:-100000}"
HOMING_INTERVAL_MAX_TRAVEL_DRIVE1_COUNTS="${RASCL_HOMING_INTERVAL_MAX_TRAVEL_DRIVE1_COUNTS:-300000}"
HOMING_INTERVAL_MAX_TRAVEL_DRIVE2_COUNTS="${RASCL_HOMING_INTERVAL_MAX_TRAVEL_DRIVE2_COUNTS:-300000}"
HOMING_INTERVAL_TIMEOUT_S="${RASCL_HOMING_INTERVAL_TIMEOUT_S:-120.0}"
CSP_TORQUE_LIMIT_PER_MILLE="${RASCL_CSP_TORQUE_LIMIT_PER_MILLE:-1000}"
SPUR_CLOSE_TORQUE_LIMIT_PER_MILLE="${RASCL_SPUR_CLOSE_TORQUE_LIMIT_PER_MILLE:-300}"
SPUR_HOLD_TORQUE_LIMIT_PER_MILLE="${RASCL_SPUR_HOLD_TORQUE_LIMIT_PER_MILLE:-100}"
CSP_STALL_ERROR_COUNTS="${RASCL_CSP_STALL_ERROR_COUNTS:-25000}"
CSP_STALL_PROGRESS_COUNTS="${RASCL_CSP_STALL_PROGRESS_COUNTS:-100}"
CSP_STALL_TIMEOUT_MS="${RASCL_CSP_STALL_TIMEOUT_MS:-500}"
CLEAR_LIMIT_SWITCH_MAPPINGS_FOR_CSP="${RASCL_CLEAR_LIMIT_SWITCH_MAPPINGS_FOR_CSP:-true}"
SPUR_GEAR_DIRECTION="${RASCL_SPUR_GEAR_DIRECTION:--1}"
SPUR_GEAR_HOME_OFFSET_COUNTS="${RASCL_SPUR_GEAR_HOME_OFFSET_COUNTS:-0}"
SPUR_GEAR_COUNTS_PER_REVOLUTION="${RASCL_SPUR_GEAR_COUNTS_PER_REVOLUTION:-1323008}"
SPUR_GEAR_REFERENCE_DELTA_COUNTS="${RASCL_SPUR_GEAR_REFERENCE_DELTA_COUNTS:-50000}"
SPUR_GEAR_REFERENCE_TIMEOUT_S="${RASCL_SPUR_GEAR_REFERENCE_TIMEOUT_S:-30.0}"
SPUR_GEAR_REFERENCE_TOLERANCE_COUNTS="${RASCL_SPUR_GEAR_REFERENCE_TOLERANCE_COUNTS:-100}"
SPUR_GEAR_REFERENCE_PROFILE_VELOCITY="${RASCL_SPUR_GEAR_REFERENCE_PROFILE_VELOCITY:-3000}"
SPUR_GEAR_REFERENCE_PROFILE_ACCELERATION="${RASCL_SPUR_GEAR_REFERENCE_PROFILE_ACCELERATION:-1000}"
SPUR_GEAR_REFERENCE_PROFILE_DECELERATION="${RASCL_SPUR_GEAR_REFERENCE_PROFILE_DECELERATION:-1000}"
SPUR_GEAR_REFERENCE_FOLLOWING_ERROR_CONFIRM_S="${RASCL_SPUR_GEAR_REFERENCE_FOLLOWING_ERROR_CONFIRM_S:-0.30}"
SPUR_GEAR_MIN_POSITION_RAD="${RASCL_SPUR_GEAR_MIN_POSITION_RAD:--6.283185307}"
SPUR_GEAR_MAX_POSITION_RAD="${RASCL_SPUR_GEAR_MAX_POSITION_RAD:-6.283185307}"
# Group 15 uses tested exact relative Drive 3 increments. Neither preset uses
# command/feedback-lag contact detection or an early-stop target.
GRIPPER_GRIP_DELTA_COUNTS=-150000
GRIPPER_RELEASE_DELTA_COUNTS=150000
SPUR_GEAR_SPEED_COUNTS_PER_S="${RASCL_SPUR_GEAR_SPEED_COUNTS_PER_S:-20000}"
SPUR_GEAR_MIN_MOTION_DURATION_S="${RASCL_SPUR_GEAR_MIN_MOTION_DURATION_S:-0.5}"
SPUR_GEAR_SETTLE_DURATION_S="${RASCL_SPUR_GEAR_SETTLE_DURATION_S:-1.0}"
SPUR_GEAR_FEEDBACK_TIMEOUT_S="${RASCL_SPUR_GEAR_FEEDBACK_TIMEOUT_S:-5.0}"
TARGET_X="${RASCL_TARGET_X:-0.2108}"
TARGET_Y="${RASCL_TARGET_Y:--0.00177}"
TARGET_Z="${RASCL_TARGET_Z:-0.2913}"
TRAJECTORY_DURATION="${RASCL_DURATION:-12.0}"
# Task deliverables stay inside the required ROS package instead of /tmp.
TRAJECTORY_DIR="${RASCL_TRAJECTORY_DIR:-$WORKSPACE/src/rascl_wp3_ss26_group8/trajectories}"
TASK1_OUTPUT_CSV="${RASCL_TASK1_OUTPUT_CSV:-$TRAJECTORY_DIR/task1_output.csv}"
TASK2_OUTPUT_DIR="${RASCL_TASK2_OUTPUT_DIR:-$TRAJECTORY_DIR}"
# These files bind a target and its plan to one live CSP process.
STATE_DIR="${RASCL_STATE_DIR:-/tmp/rascl_debug}"
TARGET_STATE_FILE="$STATE_DIR/target.state"
CSP_SESSION_FILE="$STATE_DIR/csp_session.state"
PLAN_STATE_FILE="$STATE_DIR/plan.state"
TASK1_SEQUENCE_ACTIVE=false

die() {
  echo "ERROR: $*" >&2
  exit 1
}

load_ros() {
  cd "$WORKSPACE"
  [[ -f /opt/ros/jazzy/setup.bash ]] || die "ROS Jazzy setup not found"
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  set -u
  if [[ -f install/local_setup.bash ]]; then
    set +u
    # shellcheck disable=SC1091
    source install/local_setup.bash
    set -u
  elif [[ "${1:-}" != "allow_unbuilt" ]]; then
    die "install/local_setup.bash not found; run group 1 first"
  fi
  export ROS_DOMAIN_ID
}

require_wp3_package() {
  if ! ros2 pkg prefix rascl_wp3_ss26_group8 >/dev/null 2>&1; then
    die "ROS package rascl_wp3_ss26_group8 was not found; stop the hardware workflow and complete group 1 in T1 first"
  fi
  if ! ros2 pkg executables rascl_wp3_ss26_group8 | grep -q 'wp3_tsk1'; then
    die "The package was found, but the wp3_tsk1 executable is not installed; run group 1 again and inspect the build output"
  fi
  if ! ros2 pkg executables rascl_wp3_ss26_group8 | grep -q 'wp3_tsk2'; then
    die "The package was found, but the wp3_tsk2 executable is not installed; run group 1 again and inspect the build output"
  fi
}

require_real_packages() {
  local package
  for package in rascl_description rascl_hardware_interface rascl_wp3_ss26_group8 tf2_ros; do
    ros2 pkg prefix "$package" >/dev/null 2>&1 ||
      die "ROS package $package was not found; complete group 1 while no hardware process is running"
  done
  require_wp3_package
  command -v timeout >/dev/null 2>&1 || die "The container has no timeout command; timed checks cannot run"
}

require_active_controllers() {
  local controllers
  controllers="$(ros2 control list_controllers)"
  printf '%s\n' "$controllers"
  grep -Eq '^[[:space:]]*joint_state_broadcaster[[:space:]].*[[:space:]]active([[:space:]]|$)' \
    <<<"$controllers" ||
    die "joint_state_broadcaster is not active; stopping"
  grep -Eq '^[[:space:]]*rascl_position_controller[[:space:]].*[[:space:]]active([[:space:]]|$)' \
    <<<"$controllers" ||
    die "rascl_position_controller is not active; stopping"
}

is_number() {
  [[ "$1" =~ ^[-+]?([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]]
}

is_integer() {
  [[ "$1" =~ ^[-+]?[0-9]+$ ]]
}

is_positive_number() {
  [[ "$1" =~ ^[+]?([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] || return 1
  local digits="${1#+}"
  digits="${digits//./}"
  [[ "$digits" =~ [1-9] ]]
}

ros_double_literal() {
  # ROS 2 parses an integer-looking command-line override as INTEGER.  All
  # wp3_tsk1 coordinate and duration parameters are declared as DOUBLE.
  if [[ "$1" == *.* ]]; then
    printf '%s' "$1"
  else
    printf '%s.0' "$1"
  fi
}

ensure_state_dir() {
  mkdir -p "$STATE_DIR"
}

target_signature() {
  printf '%s,%s,%s,%s' "$TARGET_X" "$TARGET_Y" "$TARGET_Z" "$TRAJECTORY_DURATION"
}

save_target_state() {
  ensure_state_dir
  printf '%s\n%s\n%s\n%s\n' \
    "$TARGET_X" "$TARGET_Y" "$TARGET_Z" "$TRAJECTORY_DURATION" >"$TARGET_STATE_FILE"
}

load_target_state() {
  [[ -f "$TARGET_STATE_FILE" ]] || return 0
  local values=()
  mapfile -t values <"$TARGET_STATE_FILE"
  if [[ "${#values[@]}" -ne 4 ]] ||
    ! is_number "${values[0]}" ||
    ! is_number "${values[1]}" ||
    ! is_number "${values[2]}" ||
    ! is_positive_number "${values[3]}"; then
    rm -f "$TARGET_STATE_FILE" "$PLAN_STATE_FILE"
    die "The target state file was invalid and has been cleared; run group 14 again"
  fi
  TARGET_X="${values[0]}"
  TARGET_Y="${values[1]}"
  TARGET_Z="${values[2]}"
  TRAJECTORY_DURATION="${values[3]}"
}

clear_plan_state() {
  rm -f "$PLAN_STATE_FILE"
}

require_csp_session() {
  [[ -f "$CSP_SESSION_FILE" ]] ||
    die "No valid group 7 CSP session exists; start it with T1:4 -> T2:6->7"
  local session_pid
  IFS= read -r session_pid <"$CSP_SESSION_FILE"
  [[ "$session_pid" =~ ^[0-9]+$ ]] && kill -0 "$session_pid" 2>/dev/null ||
    die "The group 7 CSP session has ended; do not reuse stale state, and restart the complete workflow"
}

require_no_active_wp3_motion() {
  local nodes
  nodes="$(ros2 node list 2>/dev/null || true)"
  if grep -Eq '^/wp3_tsk(1|2)(_[0-9]+)?$' <<<"$nodes"; then
    die "A WP3 trajectory node is still active; stop it before controlling Drive 3 independently"
  fi
}

read_csp_joint_snapshot() {
  python3 - "$SPUR_GEAR_FEEDBACK_TIMEOUT_S" <<'PY'
import sys
import time

import rclpy
from sensor_msgs.msg import JointState

JOINTS = ("shoulder_joint", "upperarm_joint", "lowerarm_joint", "spur_gear_joint")
latest = None
feedback_timeout_s = float(sys.argv[1])


def callback(message):
    global latest
    by_name = dict(zip(message.name, message.position))
    if all(name in by_name for name in JOINTS):
        latest = [float(by_name[name]) for name in JOINTS]


rclpy.init()
node = rclpy.create_node("rascl_spur_counts_snapshot")
subscription = node.create_subscription(JointState, "/joint_states", callback, 10)
deadline = time.monotonic() + feedback_timeout_s
try:
    while latest is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if latest is None:
        raise RuntimeError(
            f"No complete /joint_states received within {feedback_timeout_s:g} seconds"
        )
    print(" ".join(f"{value:.17g}" for value in latest))
finally:
    node.destroy_subscription(subscription)
    node.destroy_node()
    rclpy.shutdown()
PY
}

save_plan_state() {
  # Bind the authorized CSV segment to the current CSP process and target.
  local segment="${1:-manual}"
  ensure_state_dir
  local session_pid
  IFS= read -r session_pid <"$CSP_SESSION_FILE"
  printf '%s\n%s\n%s\n' "$session_pid" "$(target_signature)" "$segment" >"$PLAN_STATE_FILE"
}

require_matching_plan() {
  # Execution is valid only for the exact target planned in the current CSP session.
  local expected_segment="${1:-manual}"
  require_csp_session
  [[ -f "$PLAN_STATE_FILE" ]] || die "The current CSP session has no successful group 9 plan; execution is blocked"
  local planned=()
  mapfile -t planned <"$PLAN_STATE_FILE"
  [[ "${#planned[@]}" -eq 3 ]] || die "The planning authorization file is invalid; run group 9 again"
  local session_pid
  IFS= read -r session_pid <"$CSP_SESSION_FILE"
  [[ "${planned[0]}" == "$session_pid" ]] ||
    die "The plan belongs to an old CSP session; run group 9 again in the current session"
  [[ "${planned[1]}" == "$(target_signature)" ]] ||
    die "The current target differs from the group 9 planned target; run group 9 again"
  [[ "${planned[2]}" == "$expected_segment" ]] ||
    die "The authorized CSV segment differs from the requested segment; run group 9 again"
}

group_build_test() {
  load_ros allow_unbuilt
  echo "[1/2] Building workspace..."
  colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON
  set +u
  # shellcheck disable=SC1091
  source install/local_setup.bash
  set -u
  ros2 pkg prefix rascl_wp3_ss26_group8 >/dev/null
  ros2 pkg executables rascl_wp3_ss26_group8 | grep -q 'wp3_tsk1'
  echo "[2/2] Running kinematics, launch, and hardware-interface tests..."
  python3 -m pytest src/rascl_wp3_ss26_group8/test -q
  ctest --test-dir build/rascl_description \
    -R '^test_robot_description_parameter$' \
    --output-on-failure
  ctest --test-dir build/rascl_hardware_interface \
    -R '^(test_generic_system|test_faulhaber_bridge)$' \
    --output-on-failure
}

group_fake_launch() {
  load_ros
  echo "Fake ros2_control will keep this terminal occupied; press Ctrl-C to stop it."
  ros2 launch rascl_description ros2_control.launch.py use_fake_hardware:=true
}

group_fake_check() {
  load_ros
  require_wp3_package
  ros2 control list_controllers
  ros2 topic echo --once /joint_states
  ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
    -p target_x:=0.25 -p target_y:=0.00 -p target_z:=0.08 \
    -p duration:=4.0 -p rate_hz:=50.0 -p execute:=false \
    -p output_csv:="$TASK1_OUTPUT_CSV" \
    -p output_segment:=fake_check -p append_output_csv:=false
  head -n 5 "$TASK1_OUTPUT_CSV"
  tail -n 5 "$TASK1_OUTPUT_CSV"
  ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
    -p target_x:=0.25 -p target_y:=0.00 -p target_z:=0.08 \
    -p duration:=4.0 -p rate_hz:=50.0 -p execute:=true \
    -p save_csv:=false -p input_csv:="$TASK1_OUTPUT_CSV" \
    -p input_segment:=fake_check
}

group_homing_bridge() {
  load_ros
  require_real_packages
  is_integer "$SPUR_GEAR_REFERENCE_DELTA_COUNTS" &&
    [[ "$SPUR_GEAR_REFERENCE_DELTA_COUNTS" =~ [1-9] ]] ||
    die "RASCL_SPUR_GEAR_REFERENCE_DELTA_COUNTS must be a nonzero integer"
  is_positive_number "$SPUR_GEAR_REFERENCE_TIMEOUT_S" ||
    die "RASCL_SPUR_GEAR_REFERENCE_TIMEOUT_S must be positive"
  is_integer "$SPUR_GEAR_REFERENCE_TOLERANCE_COUNTS" &&
    (( SPUR_GEAR_REFERENCE_TOLERANCE_COUNTS >= 0 )) ||
    die "RASCL_SPUR_GEAR_REFERENCE_TOLERANCE_COUNTS must be a nonnegative integer"
  is_integer "$SPUR_GEAR_REFERENCE_PROFILE_VELOCITY" &&
    (( SPUR_GEAR_REFERENCE_PROFILE_VELOCITY > 0 )) ||
    die "RASCL_SPUR_GEAR_REFERENCE_PROFILE_VELOCITY must be a positive integer"
  is_integer "$SPUR_GEAR_REFERENCE_PROFILE_ACCELERATION" &&
    (( SPUR_GEAR_REFERENCE_PROFILE_ACCELERATION > 0 )) ||
    die "RASCL_SPUR_GEAR_REFERENCE_PROFILE_ACCELERATION must be a positive integer"
  is_integer "$SPUR_GEAR_REFERENCE_PROFILE_DECELERATION" &&
    (( SPUR_GEAR_REFERENCE_PROFILE_DECELERATION > 0 )) ||
    die "RASCL_SPUR_GEAR_REFERENCE_PROFILE_DECELERATION must be a positive integer"
  is_positive_number "$SPUR_GEAR_REFERENCE_FOLLOWING_ERROR_CONFIRM_S" ||
    die "RASCL_SPUR_GEAR_REFERENCE_FOLLOWING_ERROR_CONFIRM_S must be positive"
  local interval_guard
  for interval_guard in \
    "$HOMING_INTERVAL_MAX_TRAVEL_DRIVE0_COUNTS" \
    "$HOMING_INTERVAL_MAX_TRAVEL_DRIVE1_COUNTS" \
    "$HOMING_INTERVAL_MAX_TRAVEL_DRIVE2_COUNTS"; do
    is_integer "$interval_guard" && (( interval_guard > 0 )) ||
      die "The maximum Homing-interval search travel for Drives 0-2 must be a positive integer"
  done
  is_positive_number "$HOMING_INTERVAL_TIMEOUT_S" ||
    die "RASCL_HOMING_INTERVAL_TIMEOUT_S must be positive"
  is_integer "$CSP_TORQUE_LIMIT_PER_MILLE" &&
    (( CSP_TORQUE_LIMIT_PER_MILLE > 0 && CSP_TORQUE_LIMIT_PER_MILLE <= 6000 )) ||
    die "RASCL_CSP_TORQUE_LIMIT_PER_MILLE must be in 1..6000"
  is_integer "$SPUR_CLOSE_TORQUE_LIMIT_PER_MILLE" &&
    (( SPUR_CLOSE_TORQUE_LIMIT_PER_MILLE > 0 &&
       SPUR_CLOSE_TORQUE_LIMIT_PER_MILLE <= CSP_TORQUE_LIMIT_PER_MILLE )) ||
    die "RASCL_SPUR_CLOSE_TORQUE_LIMIT_PER_MILLE must be in 1..$CSP_TORQUE_LIMIT_PER_MILLE"
  is_integer "$SPUR_HOLD_TORQUE_LIMIT_PER_MILLE" &&
    (( SPUR_HOLD_TORQUE_LIMIT_PER_MILLE > 0 &&
       SPUR_HOLD_TORQUE_LIMIT_PER_MILLE <= SPUR_CLOSE_TORQUE_LIMIT_PER_MILLE )) ||
    die "RASCL_SPUR_HOLD_TORQUE_LIMIT_PER_MILLE must be in 1..$SPUR_CLOSE_TORQUE_LIMIT_PER_MILLE"
  [[ "$CLEAR_LIMIT_SWITCH_MAPPINGS_FOR_CSP" == "true" ||
    "$CLEAR_LIMIT_SWITCH_MAPPINGS_FOR_CSP" == "false" ]] ||
    die "RASCL_CLEAR_LIMIT_SWITCH_MAPPINGS_FOR_CSP must be true or false"
  ensure_state_dir
  rm -f "$CSP_SESSION_FILE" "$PLAN_STATE_FILE"
  echo "The Homing bridge will keep running in T1 until the entire CSP session ends."
  echo "Drives 0-2 automatically find both edges of their reference-input intervals, return to (entry+exit)/2 with a low-speed sinusoidal profile at 200, and set zero; the D0/D1/D2 second-edge travel limits are $HOMING_INTERVAL_MAX_TRAVEL_DRIVE0_COUNTS/$HOMING_INTERVAL_MAX_TRAVEL_DRIVE1_COUNTS/$HOMING_INTERVAL_MAX_TRAVEL_DRIVE2_COUNTS counts, and the traverse/return timeout is $HOMING_INTERVAL_TIMEOUT_S s."
  echo "Homing midpoint arrival and Method 37 zero readback share a 500-count tolerance; the overly strict 10-count check that could reject valid motion is no longer used."
  echo "After all three axes arrive, Drive 3 moves by $SPUR_GEAR_REFERENCE_DELTA_COUNTS counts and uses Method 37 to set the reached position to 0 counts."
  echo "Drive 3 reference motion: velocity $SPUR_GEAR_REFERENCE_PROFILE_VELOCITY counts/s, acceleration/deceleration $SPUR_GEAR_REFERENCE_PROFILE_ACCELERATION/$SPUR_GEAR_REFERENCE_PROFILE_DECELERATION, and abort only after following error persists for $SPUR_GEAR_REFERENCE_FOLLOWING_ERROR_CONFIRM_S s."
  echo "Drive 2 CSP following error: window $DRIVE2_FOLLOWING_ERROR_WINDOW_COUNTS counts, timeout $DRIVE2_FOLLOWING_ERROR_TIMEOUT_MS ms; 0x607B/0x607D software position limits are read only and are not modified."
  echo "The CSP handoff clears and verifies the Drive 0-3 positive/negative limit-input mappings at 0x2310:01/:02; the Homing reference input, polarity, and software position limits remain unchanged."
  echo "CSP stall diagnostics automatically capture a drive snapshot when error >= $CSP_STALL_ERROR_COUNTS counts and progress < $CSP_STALL_PROGRESS_COUNTS counts over $CSP_STALL_TIMEOUT_MS ms."
  echo "Before Drives 0-3 enter CSP, writable 0x60E0/0x60E1 are set to $CSP_TORQUE_LIMIT_PER_MILLE (1000=rated torque) and read back; read-only 0x6072 is logged only and is never stored persistently."
  echo "Group 15 close/open perform exact -150000/+150000-count relative moves; both and custom-count moves use normal CSP torque of $CSP_TORQUE_LIMIT_PER_MILLE per mille, with no contact-delta early stop."
  echo "At CSP handoff, Drives 2/3 raise an insufficient 0x2329:03 peak-current setting to the value required by the target torque (observed hardware values: 220->1100 mA and 81->540 mA); read-only 0x6072 must read back at least $CSP_TORQUE_LIMIT_PER_MILLE. Drive 0/1 current parameters are unchanged."
  ros2 launch rascl_description homing.launch.py \
    interface:="$INTERFACE" \
    homing_interval_max_travel_drive0_counts:="$HOMING_INTERVAL_MAX_TRAVEL_DRIVE0_COUNTS" \
    homing_interval_max_travel_drive1_counts:="$HOMING_INTERVAL_MAX_TRAVEL_DRIVE1_COUNTS" \
    homing_interval_max_travel_drive2_counts:="$HOMING_INTERVAL_MAX_TRAVEL_DRIVE2_COUNTS" \
    homing_interval_timeout_s:="$HOMING_INTERVAL_TIMEOUT_S" \
    csp_torque_limit_per_mille:="$CSP_TORQUE_LIMIT_PER_MILLE" \
    spur_close_torque_limit_per_mille:="$SPUR_CLOSE_TORQUE_LIMIT_PER_MILLE" \
    spur_hold_torque_limit_per_mille:="$SPUR_HOLD_TORQUE_LIMIT_PER_MILLE" \
    clear_limit_switch_mappings_for_csp:="$CLEAR_LIMIT_SWITCH_MAPPINGS_FOR_CSP" \
    drive2_following_error_window_counts:="$DRIVE2_FOLLOWING_ERROR_WINDOW_COUNTS" \
    drive2_following_error_timeout_ms:="$DRIVE2_FOLLOWING_ERROR_TIMEOUT_MS" \
    csp_stall_error_counts:="$CSP_STALL_ERROR_COUNTS" \
    csp_stall_progress_counts:="$CSP_STALL_PROGRESS_COUNTS" \
    csp_stall_timeout_ms:="$CSP_STALL_TIMEOUT_MS" \
    spur_gear_reference_delta_counts:="$SPUR_GEAR_REFERENCE_DELTA_COUNTS" \
    spur_gear_reference_timeout_s:="$SPUR_GEAR_REFERENCE_TIMEOUT_S" \
    spur_gear_reference_tolerance_counts:="$SPUR_GEAR_REFERENCE_TOLERANCE_COUNTS" \
    spur_gear_reference_profile_velocity:="$SPUR_GEAR_REFERENCE_PROFILE_VELOCITY" \
    spur_gear_reference_profile_acceleration:="$SPUR_GEAR_REFERENCE_PROFILE_ACCELERATION" \
    spur_gear_reference_profile_deceleration:="$SPUR_GEAR_REFERENCE_PROFILE_DECELERATION" \
    spur_gear_reference_following_error_confirm_s:="$SPUR_GEAR_REFERENCE_FOLLOWING_ERROR_CONFIRM_S" \
    skip_spur_gear_homing:=true
}

read_inputs() {
  ros2 service call /rascl_faulhaber_bridge/read_digital_inputs \
    std_srvs/srv/Trigger "{}"
}

read_drive2_diagnostics() {
  ros2 service call /rascl_faulhaber_bridge/read_drive2_diagnostics \
    std_srvs/srv/Trigger "{}"
}

read_csp_stall_snapshot() {
  timeout 5s ros2 service call /rascl_faulhaber_bridge/read_csp_stall_snapshot \
    std_srvs/srv/Trigger "{}"
}

home_one() {
  local drive="$1"
  local response
  ros2 param set /rascl_faulhaber_bridge test_drive_index "$drive"
  response="$(
    ros2 service call /rascl_faulhaber_bridge/home_one std_srvs/srv/Trigger "{}"
  )" || die "Drive $drive Homing service call failed"
  printf '%s\n' "$response"
  grep -q "success=True" <<<"$response" ||
    die "Drive $drive Homing did not succeed; stopping the workflow"
  grep -q "drive${drive}_interval(" <<<"$response" ||
    die "Drive $drive did not return both Homing-interval edges and the midpoint record; stopping the workflow"
}

group_home_individual() {
  load_ros
  read_inputs
  home_one 0
  home_one 1
  home_one 2
  echo "Drive 0-2 Homing is complete; after the last axis, Drive 3 automatically performs the $SPUR_GEAR_REFERENCE_DELTA_COUNTS-count reference move and sets zero."
}

group_home_all() {
  local response
  load_ros
  read_inputs
  echo "home_all first moves Drives 0-2 through their reference-input intervals, returns each to its midpoint, and sets zero; after success, Drive 3 automatically moves by $SPUR_GEAR_REFERENCE_DELTA_COUNTS counts and sets the reached position to 0 counts."
  response="$(
    ros2 service call /rascl_faulhaber_bridge/home_all std_srvs/srv/Trigger "{}"
  )" || die "home_all service call failed"
  printf '%s\n' "$response"
  grep -q "success=True" <<<"$response" ||
    die "home_all or the Drive 3 reference move/zeroing failed; CSP entry is blocked"
  for drive in 0 1 2; do
    grep -q "drive${drive}_interval(" <<<"$response" ||
      die "home_all has no Homing-interval record for Drive $drive; CSP entry is blocked"
  done
  group_spur_gear_counts
  read_drive2_diagnostics
}

group_adjust_home_counts() {
  local drive="$1"
  local delta response
  load_ros
  read -r -p "Drive $drive relative trim in counts (positive/negative integer, not 0): " delta
  delta="${delta//$'\r'/}"
  [[ "$delta" =~ ^[+-]?[0-9]+$ ]] ||
    die "counts must be a signed integer, for example 500 or -1200"
  (( delta != 0 )) || die "counts cannot be 0"
  ros2 param set /rascl_faulhaber_bridge test_drive_index "$drive" >/dev/null ||
    die "Drive $drive cannot be configured; confirm that group 4 is still running in T1"
  ros2 param set /rascl_faulhaber_bridge test_relative_counts "$delta" >/dev/null ||
    die "Relative counts cannot be set; confirm that the code was rebuilt and group 4 was restarted in T1"
  response="$(
    ros2 service call \
      /rascl_faulhaber_bridge/adjust_home_counts \
      std_srvs/srv/Trigger "{}"
  )" || die "Drive $drive Home-trim service call failed"
  printf '%s\n' "$response"
  grep -q "success=True" <<<"$response" ||
    die "Drive $drive Home trim failed; do not enter CSP"
  echo "This group may be repeated; correction_from_homed_zero is the actual cumulative count offset from the current Homing zero."
  echo "After confirming the physical Home of all three axes, record each value; to make the current pose the session Home, run group 22 and then group 7."
}

group_set_current_arm_home() {
  local response
  load_ros
  response="$(
    ros2 service call \
      /rascl_faulhaber_bridge/set_current_arm_home \
      std_srvs/srv/Trigger "{}"
  )" || die "The service call that sets the current pose as Home failed"
  printf '%s\n' "$response"
  grep -q "success=True" <<<"$response" ||
    die "The current pose was not fully set as Home; do not enter CSP"
  for drive in 0 1 2; do
    grep -q "drive${drive}_before=" <<<"$response" ||
      die "The pre-zero counts for Drive $drive are missing; do not enter CSP"
    grep -q "drive${drive}_after=" <<<"$response" ||
      die "The Method 37 readback for Drive $drive is missing; do not enter CSP"
  done
  grep -q "Drive 3 unchanged" <<<"$response" ||
    die "The service did not confirm that Drive 3 remained unchanged; do not enter CSP"
  echo "The current Drive 0-2 pose is now the session Home; the Drive 3 zero is unchanged. Group 7 may now be run."
  echo "Save drive0/1/2_before; running group 6 again will overwrite this manual Home."
}

group_csp_launch() {
  load_ros
  ensure_state_dir
  clear_plan_state
  printf '%s\n' "$$" >"$CSP_SESSION_FILE"
  cleanup_csp_state() {
    rm -f "$CSP_SESSION_FILE" "$PLAN_STATE_FILE"
  }
  trap cleanup_csp_state EXIT
  echo "Keep the Homing bridge running in T1; ros2_control will keep this terminal occupied."
  echo "Drive 2 mapping: direction=$LOWERARM_DIRECTION, home_offset_counts=$LOWERARM_HOME_OFFSET_COUNTS"
  echo "Drive 3 CSP mapping: direction=$SPUR_GEAR_DIRECTION, counts_per_revolution=$SPUR_GEAR_COUNTS_PER_REVOLUTION, Method 37 session zero=0 counts"
  echo "After entering CSP, lowerarm_joint at Home must remain near +1.5708 rad; otherwise, target transmission is blocked."
  set +e
  ros2 launch rascl_description ros2_control.launch.py \
    interface:="$INTERFACE" \
    use_fake_hardware:=false \
    start_bridge:=false \
    lowerarm_direction:="$LOWERARM_DIRECTION" \
    spur_gear_direction:="$SPUR_GEAR_DIRECTION" \
    gripper_counts_per_revolution:="$SPUR_GEAR_COUNTS_PER_REVOLUTION" \
    shoulder_home_offset_counts:=0 \
    upperarm_home_offset_counts:=-802816 \
    lowerarm_home_offset_counts:="$LOWERARM_HOME_OFFSET_COUNTS" \
    spur_gear_home_offset_counts:="$SPUR_GEAR_HOME_OFFSET_COUNTS"
  local launch_status=$?
  set -e
  cleanup_csp_state
  trap - EXIT
  return "$launch_status"
}

group_csp_check() {
  load_ros
  require_active_controllers
  ros2 control list_hardware_interfaces
  timeout 10s ros2 topic echo --once /joint_states
  echo "Measuring /joint_states frequency for 10 seconds..."
  timeout 10s ros2 topic hz /joint_states || [[ "$?" -eq 124 ]]
}

group_real_plan() {
  local segment="${1:-manual}"
  local append_output_csv="${2:-false}"
  load_ros
  require_wp3_package
  require_csp_session
  require_active_controllers
  clear_plan_state
  local ros_x ros_y ros_z ros_duration
  ros_x="$(ros_double_literal "$TARGET_X")"
  ros_y="$(ros_double_literal "$TARGET_Y")"
  ros_z="$(ros_double_literal "$TARGET_Z")"
  ros_duration="$(ros_double_literal "$TRAJECTORY_DURATION")"
  # A plan authorizes no motion until its finite package CSV has been checked.
  mkdir -p "$TRAJECTORY_DIR"
  if [[ "$append_output_csv" != "true" ]]; then
    rm -f "$TASK1_OUTPUT_CSV"
  fi
  if ! ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
    -p target_x:="$ros_x" -p target_y:="$ros_y" -p target_z:="$ros_z" \
    -p apply_board_xy_compensation:=true \
    -p duration:="$ros_duration" -p rate_hz:=50.0 -p execute:=false \
    -p output_csv:="$TASK1_OUTPUT_CSV" \
    -p output_segment:="$segment" \
    -p append_output_csv:="$append_output_csv"; then
    echo "The planning command failed; group 10 remains locked. When CSP/controller operation is normal, run groups 14 and 9 again." >&2
    return 0
  fi
  if [[ ! -s "$TASK1_OUTPUT_CSV" ]]; then
    echo "Planning did not generate a trajectory CSV; group 10 remains locked." >&2
    return 0
  fi
  if grep -Eiq '(^|,)(nan|[-+]?inf)(,|$)' "$TASK1_OUTPUT_CSV"; then
    echo "The trajectory CSV contains nan/inf; group 10 remains locked." >&2
    return 0
  fi
  head -n 5 "$TASK1_OUTPUT_CSV"
  tail -n 5 "$TASK1_OUTPUT_CSV"
  save_plan_state "$segment"
  echo "Planning passed (fixed-board XY compensation is enabled); group 10 may execute CSV segment '$segment' for target: [$TARGET_X, $TARGET_Y, $TARGET_Z] m"
}

group_real_execute() {
  local segment="${1:-manual}"
  load_ros
  require_wp3_package
  require_matching_plan "$segment"
  require_active_controllers
  [[ -s "$TASK1_OUTPUT_CSV" ]] ||
    die "Offline Task 1 trajectory is missing; run group 9 again"
  local ros_x ros_y ros_z ros_duration
  ros_x="$(ros_double_literal "$TARGET_X")"
  ros_y="$(ros_double_literal "$TARGET_Y")"
  ros_z="$(ros_double_literal "$TARGET_Z")"
  ros_duration="$(ros_double_literal "$TRAJECTORY_DURATION")"
  # Recheck live feedback immediately before publishing the authorized trajectory.
  timeout 3s ros2 topic echo --once /joint_states >/dev/null ||
    die "No /joint_states data arrived within 3 seconds; execution is blocked"
  echo "This command moves the physical robot; Drive 3 is also in CSP, and Task 1 will hold the current spur-gear angle."
  if ! ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
    -p target_x:="$ros_x" -p target_y:="$ros_y" -p target_z:="$ros_z" \
    -p apply_board_xy_compensation:=true \
    -p duration:="$ros_duration" -p rate_hz:=50.0 -p execute:=true \
    -p save_csv:=false -p input_csv:="$TASK1_OUTPUT_CSV" \
    -p input_segment:="$segment"; then
    clear_plan_state
    echo "Motion did not reach the planned endpoint; reading the CSP stall snapshot saved automatically by the bridge:" >&2
    read_csp_stall_snapshot || true
    die "Target transmission stopped; run group 12 immediately to package logs, then restart the complete EtherCAT session"
  fi
  clear_plan_state
  require_active_controllers
  timeout 3s ros2 topic echo --once /joint_states >/dev/null ||
    die "/joint_states was lost after motion; restart the complete EtherCAT session as described in the guide"
  echo "The motion command has ended; run groups 14, 9, and 10 again for the next target."
}

group_stall_snapshot() {
  load_ros
  read_csp_stall_snapshot
}

group_spur_gear_counts() {
  local response
  load_ros
  response="$(
    timeout 5s ros2 service call \
      /rascl_faulhaber_bridge/read_spur_gear_counts \
      std_srvs/srv/Trigger "{}"
  )" ||
    die "Drive 3 counts were not read within 5 seconds; confirm that group 4 is still running in T1"
  printf '%s\n' "$response"
  grep -q "success=True" <<<"$response" ||
    die "Drive 3 counts were read, but the current zero reference is incomplete; CSP entry is blocked"
}

group_input_limit_diagnostics() {
  load_ros
  echo "Reading Drive 0-3 input states and 0x2310 mappings (before CSP startup only):"
  read_inputs
  echo "Reading Drive 2 0x607B/0x607D and following-error parameters:"
  read_drive2_diagnostics
  echo "Note: group 18 is a read-only pre-CSP snapshot; the drive may restore 0x6065/0x6066 after Homing."
  echo "The group 7 handoff must report CSP_FOLLOWING_ERROR_CONFIGURATION and finally read back $DRIVE2_FOLLOWING_ERROR_WINDOW_COUNTS counts / $DRIVE2_FOLLOWING_ERROR_TIMEOUT_MS ms."
}

group_process_check() {
  cd "$WORKSPACE"
  echo "RASCL processes still running:"
  ps -ef | grep -E "ros2_control_node|rascl_faulhaber_bridge|wp3_tsk1|wp3_tsk2" | grep -v grep || true
  echo "TCP port 15001:"
  ss -ltnp | grep 15001 || true
}

group_pack_logs() {
  cd "$WORKSPACE"
  local output="$WORKSPACE/ros_logs_$(date +%Y%m%d_%H%M%S).tar.gz"
  [[ -d /root/.ros/log ]] || die "/root/.ros/log not found"
  tar -czf "$output" -C /root/.ros log
  echo "Log archive created: $output"
  echo "The tar.gz file can be dragged directly from the shared workspace; manual log-text copying is unnecessary."
}

group_tcp_pose() {
  load_ros
  echo "Current model TCP: base_link -> tcp_link (using live /joint_states)"
  echo "Translation x/y/z are in metres; output is displayed for approximately 3 seconds."
  timeout 3s ros2 run tf2_ros tf2_echo base_link tcp_link || [[ "$?" -eq 124 ]]
}

group_gripper_action() {
  load_ros
  require_csp_session
  require_active_controllers
  require_no_active_wp3_motion
  is_number "$SPUR_GEAR_DIRECTION" && [[ "$SPUR_GEAR_DIRECTION" != "0" ]] ||
    die "RASCL_SPUR_GEAR_DIRECTION must be a nonzero number"
  is_positive_number "$SPUR_GEAR_COUNTS_PER_REVOLUTION" ||
    die "RASCL_SPUR_GEAR_COUNTS_PER_REVOLUTION must be positive"
  is_positive_number "$SPUR_GEAR_SPEED_COUNTS_PER_S" ||
    die "RASCL_SPUR_GEAR_SPEED_COUNTS_PER_S must be positive"
  is_positive_number "$SPUR_GEAR_MIN_MOTION_DURATION_S" ||
    die "RASCL_SPUR_GEAR_MIN_MOTION_DURATION_S must be positive"
  is_positive_number "$SPUR_GEAR_SETTLE_DURATION_S" ||
    die "RASCL_SPUR_GEAR_SETTLE_DURATION_S must be positive"
  is_positive_number "$SPUR_GEAR_FEEDBACK_TIMEOUT_S" ||
    die "RASCL_SPUR_GEAR_FEEDBACK_TIMEOUT_S must be positive"
  is_number "$SPUR_GEAR_MIN_POSITION_RAD" && is_number "$SPUR_GEAR_MAX_POSITION_RAD" ||
    die "Drive 3 URDF limits must be numeric"

  local snapshot shoulder upperarm lowerarm spur gripper_action action_label
  local delta_counts target_rad minimum_duration motion_duration duration_override="${2:-}"
  local motion_speed torque_service torque_response
  if [[ -n "$duration_override" ]]; then
    is_positive_number "$duration_override" ||
      die "The specified Drive 3 trajectory duration must be an ordinary decimal number greater than 0"
  fi
  if ! snapshot="$(read_csp_joint_snapshot)"; then
    die "A complete /joint_states message was not received within $SPUR_GEAR_FEEDBACK_TIMEOUT_S seconds; Drive 3 control is blocked"
  fi
  read -r shoulder upperarm lowerarm spur <<<"$snapshot"
  echo "Current Drive 3 joint position = $spur rad; relative motion uses the current position, and absolute counts use the current Method 37 zero."
  if [[ $# -gt 0 ]]; then
    gripper_action="$1"
  else
    read -r -p "Gripper action [close/open] (c/o) or relative counts (positive/negative integer): " gripper_action
  fi
  gripper_action="${gripper_action,,}"
  case "$gripper_action" in
    close | c)
      action_label="close grip"
      delta_counts="$GRIPPER_GRIP_DELTA_COUNTS"
      ;;
    open | o)
      action_label="open release"
      delta_counts="$GRIPPER_RELEASE_DELTA_COUNTS"
      ;;
    *)
      is_integer "$gripper_action" ||
        die "Drive 3 motion was not executed: enter close/c, open/o, or a nonzero integer count value"
      [[ "$gripper_action" =~ [1-9] ]] || die "Relative counts cannot be 0"
      action_label="custom relative counts"
      delta_counts="$gripper_action"
      ;;
  esac
  motion_speed="$SPUR_GEAR_SPEED_COUNTS_PER_S"
  torque_service="/rascl_faulhaber_bridge/restore_spur_torque"
  # Convert the relative count request with the configured Drive 3 sign, then
  # reject it before publishing if the resulting joint angle exceeds the URDF limit.
  if ! read -r target_rad minimum_duration < <(python3 - "$spur" "$delta_counts" "$SPUR_GEAR_DIRECTION" "$SPUR_GEAR_COUNTS_PER_REVOLUTION" "$SPUR_GEAR_MIN_POSITION_RAD" "$SPUR_GEAR_MAX_POSITION_RAD" "$motion_speed" "$SPUR_GEAR_MIN_MOTION_DURATION_S" <<'PY'
import math
import sys
(
    current_rad,
    delta_counts,
    direction,
    counts_per_revolution,
    minimum,
    maximum,
    speed_counts_per_s,
    minimum_duration,
) = map(float, sys.argv[1:])
target_rad = current_rad + direction * delta_counts * 2.0 * math.pi / counts_per_revolution
if minimum > maximum:
    raise ValueError("minimum position exceeds maximum position")
if not minimum <= target_rad <= maximum:
    raise ValueError(
        f"relative move {int(delta_counts)} counts requests {target_rad:.6f} rad, outside "
        f"URDF range [{minimum:.6f}, {maximum:.6f}] rad"
    )
minimum_duration = max(abs(delta_counts) / speed_counts_per_s, minimum_duration)
print(f"{target_rad:.17g} {minimum_duration:.17g}")
PY
); then
    die "The gripper '$action_label' command was rejected by the Drive 3 URDF software limit"
  fi

  motion_duration="${duration_override:-$minimum_duration}"

  torque_response="$(
    timeout 5s ros2 service call "$torque_service" std_srvs/srv/Trigger "{}"
  )" || die "The Drive 3 torque-protection service did not respond; gripper motion was not executed"
  printf '%s\n' "$torque_response"
  grep -q "success=True" <<<"$torque_response" ||
    die "The Drive 3 torque limit was not written and read back successfully; gripper motion was not executed"

  clear_plan_state
  echo "Gripper action '$action_label': Drive 3 relative motion $delta_counts counts."
  echo "Using a 50 Hz minimum-jerk CSP trajectory with automatic duration $motion_duration s."
  echo "This action uses $motion_speed counts/s and normal CSP torque, requires an exact $delta_counts-count move, and does not enable contact-based early termination."
  echo "The first three axes hold their current joint states; this operation cleared the previous group 9 planning authorization."
  if ! python3 - "$shoulder" "$upperarm" "$lowerarm" "$spur" "$target_rad" \
    "$delta_counts" "$motion_duration" "$SPUR_GEAR_DIRECTION" \
    "$SPUR_GEAR_COUNTS_PER_REVOLUTION" "$SPUR_GEAR_HOME_OFFSET_COUNTS" \
    "$SPUR_GEAR_FEEDBACK_TIMEOUT_S" "$SPUR_GEAR_SETTLE_DURATION_S" <<'PY'
import math
import sys
import time

import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

(
    shoulder,
    upperarm,
    lowerarm,
    source_spur,
    target_spur,
    delta_counts,
    duration_s,
    direction,
    counts_per_revolution,
    home_offset_counts,
    feedback_timeout_s,
    settle_s,
) = map(float, sys.argv[1:])

JOINTS = ("shoulder_joint", "upperarm_joint", "lowerarm_joint", "spur_gear_joint")
TAU = 2.0 * math.pi
latest_spur = None
last_feedback_time = None


def rad_to_counts(angle):
    return int(round(home_offset_counts + direction * angle * counts_per_revolution / TAU))


def callback(message):
    global latest_spur, last_feedback_time
    by_name = dict(zip(message.name, message.position))
    if all(name in by_name for name in JOINTS):
        latest_spur = float(by_name["spur_gear_joint"])
        last_feedback_time = time.monotonic()


def publish(publisher, spur_command):
    message = Float64MultiArray()
    message.data = [shoulder, upperarm, lowerarm, spur_command]
    publisher.publish(message)


def log_feedback(logger, phase, reference_spur=None):
    if latest_spur is None:
        logger.warning(f"SPUR_TRACE {phase}: no /joint_states feedback yet")
        return
    if reference_spur is None:
        reference_spur = target_spur
    actual_counts = rad_to_counts(latest_spur)
    remaining_counts = rad_to_counts(reference_spur) - actual_counts
    logger.info(
        f"SPUR_TRACE {phase}: actual_rad={latest_spur:.6f} "
        f"actual_counts={actual_counts} remaining_counts={remaining_counts}"
    )


rclpy.init()
node = rclpy.create_node("rascl_spur_relative_motion")
publisher = node.create_publisher(Float64MultiArray, "/rascl_position_controller/commands", 10)
subscription = node.create_subscription(JointState, "/joint_states", callback, 10)
logger = node.get_logger()
source_counts = rad_to_counts(source_spur)
target_counts = rad_to_counts(target_spur)
logger.info(
    f"SPUR_TRACE start: delta_counts={int(delta_counts)} source_rad={source_spur:.6f} "
    f"source_counts={source_counts} target_rad={target_spur:.6f} "
    f"target_counts={target_counts} duration_s={duration_s:.3f} rate_hz=50"
)

try:
    feedback_deadline = time.monotonic() + feedback_timeout_s
    while latest_spur is None and time.monotonic() < feedback_deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if latest_spur is None:
        raise RuntimeError(
            "No Drive 3 /joint_states feedback within "
            f"{feedback_timeout_s:g} seconds before CSP motion"
        )

    # Use absolute monotonic deadlines so one late cycle does not accumulate drift.
    period_s = 0.02
    start = time.monotonic()
    next_tick = start
    next_log = start
    while True:
        now = time.monotonic()
        elapsed = now - start
        u = min(1.0, elapsed / duration_s)
        # Fifth-order minimum jerk: zero velocity and acceleration at both ends.
        blend = 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5
        command_spur = source_spur + (target_spur - source_spur) * blend
        publish(publisher, command_spur)
        rclpy.spin_once(node, timeout_sec=0.0)
        if now >= next_log:
            log_feedback(logger, "progress")
            next_log += 1.0
        if last_feedback_time is not None and now - last_feedback_time > 0.5:
            raise RuntimeError("/joint_states stopped during Drive 3 CSP motion")
        if u >= 1.0:
            break
        next_tick += period_s
        time.sleep(max(0.0, next_tick - time.monotonic()))

    settle_deadline = time.monotonic() + settle_s
    while time.monotonic() < settle_deadline:
        publish(publisher, target_spur)
        rclpy.spin_once(node, timeout_sec=0.0)
        if last_feedback_time is not None and time.monotonic() - last_feedback_time > 0.5:
            raise RuntimeError("/joint_states stopped while Drive 3 was settling")
        time.sleep(period_s)
    log_feedback(logger, "complete", target_spur)
    logger.info(
        "SPUR_RESULT outcome=target_reached "
        f"requested_target_counts={target_counts} "
        f"held_target_counts={rad_to_counts(target_spur)}"
    )
except Exception as exc:
    logger.error(f"SPUR_TRACE failed: {exc}")
    raise
finally:
    node.destroy_subscription(subscription)
    node.destroy_node()
    rclpy.shutdown()
PY
  then
    die "The Drive 3 CSP trajectory was interrupted; run group 12 immediately and submit the logs"
  fi
  require_active_controllers
  echo "Gripper action complete: $action_label, delta=$delta_counts counts. When group 9 runs next, Task 1 will hold this spur-gear angle."
}

group_set_target() {
  local x y z duration
  read -r -p "Target x [m] (current $TARGET_X): " x
  read -r -p "Target y [m] (current $TARGET_Y): " y
  read -r -p "Target z [m] (current $TARGET_Z): " z
  read -r -p "Motion duration [s] (current $TRAJECTORY_DURATION): " duration
  x="${x:-$TARGET_X}"
  y="${y:-$TARGET_Y}"
  z="${z:-$TARGET_Z}"
  duration="${duration:-$TRAJECTORY_DURATION}"
  is_number "$x" || die "x is not a valid number: $x"
  is_number "$y" || die "y is not a valid number: $y"
  is_number "$z" || die "z is not a valid number: $z"
  is_positive_number "$duration" || die "Motion duration must be an ordinary decimal number greater than 0"
  TARGET_X="$x"
  TARGET_Y="$y"
  TARGET_Z="$z"
  TRAJECTORY_DURATION="$duration"
  save_target_state
  clear_plan_state
  echo "Target set to [$TARGET_X, $TARGET_Y, $TARGET_Z] m with duration $TRAJECTORY_DURATION s."
  echo "Run group 9 next; group 10 is available only after planning succeeds."
}

group_target_plan_execute() {
  echo "Group 23: enter a target, then execute immediately after successful planning. Fixed-board XY compensation remains enabled."
  group_set_target
  group_real_plan
  if [[ ! -s "$PLAN_STATE_FILE" ]]; then
    echo "Planning failed; no physical motion will be executed. Correct the target and retry group 23." >&2
    return 0
  fi
  echo "Planning passed; executing the same target now."
  group_real_execute
}

task1_move_to() {
  local label="$1"
  local x="$2"
  local y="$3"
  local z="$4"
  local duration="$5"

  # Every waypoint is independently planned and verified before the stage continues.
  TARGET_X="$x"
  TARGET_Y="$y"
  TARGET_Z="$z"
  TRAJECTORY_DURATION="$duration"
  save_target_state
  clear_plan_state
  echo "Task 1 $label: move to [$TARGET_X, $TARGET_Y, $TARGET_Z] m in $TRAJECTORY_DURATION s."
  group_real_plan "$label" true
  [[ -s "$PLAN_STATE_FILE" ]] ||
    die "Task 1 $label planning failed; this stage stopped and no later action was executed"
  group_real_execute "$label"
}

task1_gripper_preset() {
  local action="$1"
  local duration="$2"
  echo "Task 1: gripper $action, $duration s."
  group_gripper_action "$action" "$duration"
}

reset_task1_combined_csv() {
  mkdir -p "$TRAJECTORY_DIR"
  rm -f "$TASK1_OUTPUT_CSV"
  clear_plan_state
  echo "Task 1 combined trajectory CSV reset: $TASK1_OUTPUT_CSV"
}

prepare_task1_stage_csv() {
  if [[ "$TASK1_SEQUENCE_ACTIVE" != "true" ]]; then
    reset_task1_combined_csv
  fi
}

group_task1_stage_1() {
  prepare_task1_stage_csv
  echo "Task 1 stage 1: move 1. Cartesian actions take 5 s; the marked descent takes 10 s."
  task1_move_to "stage1/1" 0.16 0.16 0.10 5
  task1_move_to "stage1/2" 0.16 0.16 0.05 5
  task1_gripper_preset close 5
  task1_move_to "stage1/3" 0.16 0.16 0.15 5
  task1_move_to "stage1/4" 0.0929 -0.1327 0.15 5
  task1_move_to "stage1/5" 0.0929 -0.1327 0.05 10
  task1_move_to "stage1/6" 0.07 -0.10 0.05 5
  task1_gripper_preset open 5
  task1_move_to "stage1/7" 0.07 -0.10 0.15 5
  echo "Task 1 stage 1 complete."
}

group_task1_stage_2() {
  prepare_task1_stage_csv
  echo "Task 1 stage 2: move 2 to the temporary square. Cartesian actions take 5 s; the marked descent takes 10 s."
  task1_move_to "stage2/1" 0.17 0.03 0.15 5
  task1_move_to "stage2/2" 0.17 0.03 0.085 5
  task1_gripper_preset close 5
  task1_move_to "stage2/3" 0.17 0.03 0.15 5
  task1_move_to "stage2/4" 0.18 -0.04 0.15 5
  task1_move_to "stage2/5" 0.18 -0.04 0.05 10
  task1_gripper_preset open 5
  task1_move_to "stage2/6" 0.18 -0.04 0.15 5
  echo "Task 1 stage 2 complete."
}

group_task1_stage_3() {
  prepare_task1_stage_csv
  echo "Task 1 stage 3: square 3 to square 1. Cartesian actions take 5 s; the marked descent takes 10 s."
  task1_move_to "stage3/1" 0.17 0.03 0.15 5
  task1_move_to "stage3/2" 0.17 0.03 0.045 5
  task1_gripper_preset close 5
  task1_move_to "stage3/3" 0.17 0.03 0.15 5
  task1_move_to "stage3/4" 0.0642 -0.0918 0.15 5
  task1_move_to "stage3/5" 0.0642 -0.0918 0.085 10
  task1_gripper_preset open 5
  task1_move_to "stage3/6" 0.0642 -0.0918 0.15 5
  echo "Task 1 stage 3 complete."
}

group_task1_stage_4() {
  prepare_task1_stage_csv
  echo "Task 1 stage 4: square 2 to square 3. Cartesian actions take 5 s; the marked descent takes 10 s."
  task1_move_to "stage4/1" 0.18 -0.04 0.15 5
  task1_move_to "stage4/2" 0.18 -0.04 0.045 5
  task1_gripper_preset close 5
  task1_move_to "stage4/3" 0.18 -0.04 0.15 5
  task1_move_to "stage4/4" 0.0642 -0.0918 0.15 5
  task1_move_to "stage4/5" 0.0642 -0.0918 0.125 10
  task1_gripper_preset open 5
  echo "Task 1 stage 4 complete."
}

group_task1_all_stages() {
  echo "Task 1 full sequence: stages 1 -> 2 -> 3 -> 4, with no delay between actions."
  reset_task1_combined_csv
  TASK1_SEQUENCE_ACTIVE=true
  group_task1_stage_1
  group_task1_stage_2
  group_task1_stage_3
  group_task1_stage_4
  TASK1_SEQUENCE_ACTIVE=false
  echo "Task 1 full sequence complete."
  echo "All 24 Cartesian arm segments remain in one CSV: $TASK1_OUTPUT_CSV"
}

group_task2_pick_and_place() {
  load_ros
  require_wp3_package
  require_csp_session
  require_active_controllers
  require_no_active_wp3_motion
  clear_plan_state
  mkdir -p "$TASK2_OUTPUT_DIR"
  echo "Starting the required wp3_tsk2 online node in T3. Keep this terminal running."
  echo "Publish each runtime cube centre from another container terminal, for example:"
  echo "ros2 topic pub --once /goal_poses geometry_msgs/msg/Point '{x: 0.16, y: 0.08, z: 0.0}'"
  echo "The node accepts repeated messages, processes them sequentially, and writes Task 2 CSV files under $TASK2_OUTPUT_DIR."
  ros2 run rascl_wp3_ss26_group8 wp3_tsk2 --ros-args \
    -p execute:=true \
    -p apply_board_xy_compensation:=true \
    -p require_torque_service:=true \
    -p output_directory:="$TASK2_OUTPUT_DIR"
}

print_menu() {
  printf '%s\n' \
    "" \
    "RASCL command groups" \
    "Workspace : $WORKSPACE" \
    "Interface : $INTERFACE" \
    "ROS domain: $ROS_DOMAIN_ID" \
    "Target TCP: [$TARGET_X, $TARGET_Y, $TARGET_Z] m / $TRAJECTORY_DURATION s" \
    "" \
    "  1  Build + functional tests                            [T1]" \
    "  2  Start fake ros2_control (foreground)                [T1]" \
    "  3  Fake checks + plan + execute                        [T2]" \
    "  4  Start physical Homing bridge                        [T1]" \
    "  5  Home Drives 0, 1, and 2 individually                [T2]" \
    "  6  home_all (Drives 0-2)                               [T2]" \
    "  7  Start physical CSP ros2_control (includes Drive 3)  [T2]" \
    "  8  Controller/joint-state hold check for 10 seconds    [T3]" \
    "  9  Generate offline minimum-jerk trajectory CSV        [T3]" \
    "     (fixed-board XY compensation)" \
    " 10  Load and execute the authorized offline CSV         [T3, moves]" \
    "     (fixed-board XY compensation)" \
    " 11  Check residual processes and TCP port                [T3]" \
    " 12  Package complete ROS logs in the shared workspace    [any]" \
    " 13  Show current model TCP coordinates                   [T3]" \
    " 14  Set the next physical TCP target and motion duration [T3]" \
    " 15  Gripper close(-150000)/open(+150000) or custom counts [T3, moves]" \
    " 16  Show the latest automatic CSP stall snapshot         [T3]" \
    " 17  Show current absolute Drive 3 counts (Method 37 zero) [T2/T3, read only]" \
    " 18  Show input mappings and Drive 2 protection settings  [T2, pre-CSP read only]" \
    " 19  Trim Drive 0 after Homing (relative counts)          [T2, moves]" \
    " 20  Trim Drive 1 after Homing (relative counts)          [T2, moves]" \
    " 21  Trim Drive 2 after Homing (relative counts)          [T2, moves]" \
    " 22  Set current Drive 0-2 pose as session Home           [T2, no motion]" \
    " 23  Enter target -> plan -> execute immediately          [T3, moves]" \
    "     (fixed-board XY compensation)" \
    " 24  Task 1 stage 1: move 1                               [T3, moves]" \
    " 25  Task 1 stage 2: move 2 -> temporary square           [T3, moves]" \
    " 26  Task 1 stage 3: square 3 -> square 1                 [T3, moves]" \
    " 27  Task 1 stage 4: square 2 -> square 3                 [T3, moves]" \
    " 28  Task 1 full sequence + one combined trajectory CSV  [T3, moves]" \
    " 29  Start Task 2 /goal_poses online pick-and-place node  [T3, moves]" \
    "  0  Exit" \
    "" \
    "Groups 2, 4, 7, and 29 keep their terminal occupied until Ctrl-C is pressed." \
    "CSP order: T1=4; T2=6->(19/20/21->22 during calibration)->7; T3=8->13->23 (or 14->9->10). Drive 3: T3=15 moves, 17 reads counts."
}

run_group() {
  case "$1" in
    1) group_build_test ;;
    2) group_fake_launch ;;
    3) group_fake_check ;;
    4) group_homing_bridge ;;
    5) group_home_individual ;;
    6) group_home_all ;;
    7) group_csp_launch ;;
    8) group_csp_check ;;
    9) group_real_plan ;;
    10) group_real_execute ;;
    11) group_process_check ;;
    12) group_pack_logs ;;
    13) group_tcp_pose ;;
    14) group_set_target ;;
    15) group_gripper_action ;;
    16) group_stall_snapshot ;;
    17) group_spur_gear_counts ;;
    18) group_input_limit_diagnostics ;;
    19) group_adjust_home_counts 0 ;;
    20) group_adjust_home_counts 1 ;;
    21) group_adjust_home_counts 2 ;;
    22) group_set_current_arm_home ;;
    23) group_target_plan_execute ;;
    24) group_task1_stage_1 ;;
    25) group_task1_stage_2 ;;
    26) group_task1_stage_3 ;;
    27) group_task1_stage_4 ;;
    28) group_task1_all_stages ;;
    29) group_task2_pick_and_place ;;
    0) exit 0 ;;
    *) die "Unknown group number: $1" ;;
  esac
}

if [[ $# -gt 0 ]]; then
  load_target_state
  run_group "$1"
else
  load_target_state
  while true; do
    print_menu
    read -r -p "Enter group number: " selection
    selection="${selection//$'\r'/}"
    run_group "$selection"
    echo "Command group $selection returned to the menu; use the output above to determine whether it succeeded."
  done
fi
