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
CSP_TORQUE_LIMIT_PER_MILLE="${RASCL_CSP_TORQUE_LIMIT_PER_MILLE:-1000}"
CSP_STALL_ERROR_COUNTS="${RASCL_CSP_STALL_ERROR_COUNTS:-25000}"
CSP_STALL_PROGRESS_COUNTS="${RASCL_CSP_STALL_PROGRESS_COUNTS:-100}"
CSP_STALL_TIMEOUT_MS="${RASCL_CSP_STALL_TIMEOUT_MS:-500}"
SPUR_GEAR_DIRECTION="${RASCL_SPUR_GEAR_DIRECTION:-1}"
SPUR_GEAR_HOME_OFFSET_COUNTS="${RASCL_SPUR_GEAR_HOME_OFFSET_COUNTS:-0}"
SPUR_GEAR_COUNTS_PER_REVOLUTION="${RASCL_SPUR_GEAR_COUNTS_PER_REVOLUTION:-1323008}"
SPUR_GEAR_MIN_POSITION_RAD="${RASCL_SPUR_GEAR_MIN_POSITION_RAD:--3.1415}"
SPUR_GEAR_MAX_POSITION_RAD="${RASCL_SPUR_GEAR_MAX_POSITION_RAD:-3.1415}"
# Group 15 uses opposite Drive 3 directions for closing and opening. Closing
# travels toward contact with a maximum positive increment; opening is an exact
# negative relative move.
GRIPPER_GRIP_DELTA_COUNTS=500000
GRIPPER_RELEASE_DELTA_COUNTS=-200000
SPUR_GEAR_SPEED_COUNTS_PER_S="${RASCL_SPUR_GEAR_SPEED_COUNTS_PER_S:-10000}"
SPUR_GEAR_MIN_MOTION_DURATION_S="${RASCL_SPUR_GEAR_MIN_MOTION_DURATION_S:-0.5}"
SPUR_GEAR_SETTLE_DURATION_S="${RASCL_SPUR_GEAR_SETTLE_DURATION_S:-1.0}"
SPUR_GEAR_FEEDBACK_TIMEOUT_S="${RASCL_SPUR_GEAR_FEEDBACK_TIMEOUT_S:-5.0}"
# close is a travel-until-contact shortcut. Stop it before the drive's
# following-error window is reached, then hold the measured contact position.
# open and custom signed counts remain exact relative moves.
GRIPPER_CONTACT_ERROR_COUNTS="${RASCL_GRIPPER_CONTACT_ERROR_COUNTS:-2000}"
GRIPPER_CONTACT_CONFIRM_S="${RASCL_GRIPPER_CONTACT_CONFIRM_S:-0.04}"
TARGET_X="${RASCL_TARGET_X:-0.2108}"
TARGET_Y="${RASCL_TARGET_Y:--0.00177}"
TARGET_Z="${RASCL_TARGET_Z:-0.2913}"
TRAJECTORY_DURATION="${RASCL_DURATION:-12.0}"
STATE_DIR="${RASCL_STATE_DIR:-/tmp/rascl_debug}"
TARGET_STATE_FILE="$STATE_DIR/target.state"
CSP_SESSION_FILE="$STATE_DIR/csp_session.state"
PLAN_STATE_FILE="$STATE_DIR/plan.state"

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
    die "找不到 ROS 包 rascl_wp3_ss26_group8；请停止实机流程，先在 T1 成功执行组 1"
  fi
  if ! ros2 pkg executables rascl_wp3_ss26_group8 | grep -q 'wp3_tsk1'; then
    die "包已找到，但未安装 wp3_tsk1 可执行入口；请重新执行组 1 并检查编译输出"
  fi
}

require_real_packages() {
  local package
  for package in rascl_description rascl_hardware_interface rascl_wp3_ss26_group8 tf2_ros; do
    ros2 pkg prefix "$package" >/dev/null 2>&1 ||
      die "找不到 ROS 包 $package；请在没有实机进程时先成功执行组 1"
  done
  require_wp3_package
  command -v timeout >/dev/null 2>&1 || die "容器缺少 timeout 命令，无法执行受限时检查"
}

require_active_controllers() {
  local controllers
  controllers="$(ros2 control list_controllers)"
  printf '%s\n' "$controllers"
  grep -Eq '^[[:space:]]*joint_state_broadcaster[[:space:]].*[[:space:]]active([[:space:]]|$)' \
    <<<"$controllers" ||
    die "joint_state_broadcaster 不是 active，禁止继续"
  grep -Eq '^[[:space:]]*rascl_position_controller[[:space:]].*[[:space:]]active([[:space:]]|$)' \
    <<<"$controllers" ||
    die "rascl_position_controller 不是 active，禁止继续"
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
    die "目标状态文件无效，已清除；请重新执行组 14"
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
    die "没有有效的组 7 CSP 会话；请按 T1:4 → T2:6→7 启动"
  local session_pid
  IFS= read -r session_pid <"$CSP_SESSION_FILE"
  [[ "$session_pid" =~ ^[0-9]+$ ]] && kill -0 "$session_pid" 2>/dev/null ||
    die "组 7 CSP 会话已结束；禁止复用旧状态，请完整重启"
}

require_no_active_wp3_motion() {
  local nodes
  nodes="$(ros2 node list 2>/dev/null || true)"
  if grep -Eq '^/wp3_tsk1(_[0-9]+)?$' <<<"$nodes"; then
    die "wp3_tsk1 轨迹节点仍在发布命令；等待组 10 完全结束后才能单独控制 Drive 3。"
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
  ensure_state_dir
  local session_pid
  IFS= read -r session_pid <"$CSP_SESSION_FILE"
  printf '%s\n%s\n' "$session_pid" "$(target_signature)" >"$PLAN_STATE_FILE"
}

require_matching_plan() {
  require_csp_session
  [[ -f "$PLAN_STATE_FILE" ]] || die "当前 CSP 会话没有通过组 9 的规划；禁止执行"
  local planned=()
  mapfile -t planned <"$PLAN_STATE_FILE"
  [[ "${#planned[@]}" -eq 2 ]] || die "规划授权文件无效；请重新执行组 9"
  local session_pid
  IFS= read -r session_pid <"$CSP_SESSION_FILE"
  [[ "${planned[0]}" == "$session_pid" ]] ||
    die "规划属于旧 CSP 会话；请在当前会话重新执行组 9"
  [[ "${planned[1]}" == "$(target_signature)" ]] ||
    die "当前目标与组 9 规划目标不同；请重新执行组 9"
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
  python3 -m pytest \
    src/rascl_wp3_ss26_group8/test/test_kinematics_calibration.py -q
  ctest --test-dir build/rascl_description \
    -R '^test_robot_description_parameter$' \
    --output-on-failure
  ctest --test-dir build/rascl_hardware_interface \
    -R '^(test_generic_system|test_faulhaber_bridge)$' \
    --output-on-failure
}

group_fake_launch() {
  load_ros
  echo "Fake ros2_control 将持续占用当前终端；按 Ctrl-C 停止。"
  ros2 launch rascl_description ros2_control.launch.py use_fake_hardware:=true
}

group_fake_check() {
  load_ros
  require_wp3_package
  ros2 control list_controllers
  ros2 topic echo --once /joint_states
  ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
    -p target_x:=0.25 -p target_y:=0.00 -p target_z:=0.08 \
    -p duration:=4.0 -p rate_hz:=50.0 -p execute:=false
  head -n 5 /tmp/rascl_wp3_tsk1_last_trajectory.csv
  tail -n 5 /tmp/rascl_wp3_tsk1_last_trajectory.csv
  ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
    -p target_x:=0.25 -p target_y:=0.00 -p target_z:=0.08 \
    -p duration:=4.0 -p rate_hz:=50.0 -p execute:=true
}

group_homing_bridge() {
  load_ros
  require_real_packages
  ensure_state_dir
  rm -f "$CSP_SESSION_FILE" "$PLAN_STATE_FILE"
  echo "Homing bridge 将在 T1 持续运行，直到整个 CSP 会话结束。"
  echo "Drive 0-2 自动 Homing；预装的 Drive 3 不 Homing，但会参与后续 CSP。"
  echo "Drive 2 CSP following-error：窗口 $DRIVE2_FOLLOWING_ERROR_WINDOW_COUNTS counts，超时 $DRIVE2_FOLLOWING_ERROR_TIMEOUT_MS ms；内部限位只读取、不改写。"
  echo "CSP 停滞诊断：误差 >= $CSP_STALL_ERROR_COUNTS counts 且 $CSP_STALL_TIMEOUT_MS ms 内进展 < $CSP_STALL_PROGRESS_COUNTS counts 时自动抓取驱动快照。"
  echo "Drive 0-3 进入 CSP 前会把可写的 0x60E0/0x60E1 设为 $CSP_TORQUE_LIMIT_PER_MILLE（1000=额定转矩）并回读；只读 0x6072 仅记录，不写入永久存储。"
  echo "Drive 2/3 在 CSP 交接时会把过低的 0x2329:03 峰值电流提高到满足目标转矩所需值（实机曾分别为 220→1100 mA、81→540 mA），并要求只读 0x6072 回读不低于 $CSP_TORQUE_LIMIT_PER_MILLE；Drive 0/1 电流参数不改。"
  ros2 launch rascl_description homing.launch.py \
    interface:="$INTERFACE" \
    csp_torque_limit_per_mille:="$CSP_TORQUE_LIMIT_PER_MILLE" \
    drive2_following_error_window_counts:="$DRIVE2_FOLLOWING_ERROR_WINDOW_COUNTS" \
    drive2_following_error_timeout_ms:="$DRIVE2_FOLLOWING_ERROR_TIMEOUT_MS" \
    csp_stall_error_counts:="$CSP_STALL_ERROR_COUNTS" \
    csp_stall_progress_counts:="$CSP_STALL_PROGRESS_COUNTS" \
    csp_stall_timeout_ms:="$CSP_STALL_TIMEOUT_MS" \
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
  ros2 param set /rascl_faulhaber_bridge test_drive_index "$drive"
  ros2 service call /rascl_faulhaber_bridge/home_one std_srvs/srv/Trigger "{}"
}

group_home_individual() {
  load_ros
  read_inputs
  home_one 0
  home_one 1
  home_one 2
  echo "Drive 0-2 Homing 已结束；Drive 3 不执行 Home。确认最后响应包含 CSP handoff armed。"
}

group_home_all() {
  load_ros
  read_inputs
  echo "home_all 只会运动 Drive 0-2；预装的 Drive 3 不执行 Home，随后仍会进入 CSP。"
  ros2 service call /rascl_faulhaber_bridge/home_all std_srvs/srv/Trigger "{}"
  read_drive2_diagnostics
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
  echo "保持 T1 的 Homing bridge 运行；ros2_control 将持续占用当前终端。"
  echo "Drive 2 映射：direction=$LOWERARM_DIRECTION，home_offset_counts=$LOWERARM_HOME_OFFSET_COUNTS"
  echo "Drive 3 CSP 映射：direction=$SPUR_GEAR_DIRECTION，counts_per_revolution=$SPUR_GEAR_COUNTS_PER_REVOLUTION（不执行 Home）"
  echo "进入 CSP 后，Home 的 lowerarm_joint 必须仍接近 +1.5708 rad；否则禁止发送目标。"
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
  echo "测量 /joint_states 频率 10 秒……"
  timeout 10s ros2 topic hz /joint_states || [[ "$?" -eq 124 ]]
}

group_real_plan() {
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
  rm -f /tmp/rascl_wp3_tsk1_last_trajectory.csv
  if ! ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
    -p target_x:="$ros_x" -p target_y:="$ros_y" -p target_z:="$ros_z" \
    -p duration:="$ros_duration" -p rate_hz:=50.0 -p execute:=false; then
    echo "规划命令失败；未解锁组 10。CSP/controller 正常时可重新选择组 14、9。" >&2
    return 0
  fi
  if [[ ! -s /tmp/rascl_wp3_tsk1_last_trajectory.csv ]]; then
    echo "规划未生成轨迹 CSV；未解锁组 10。" >&2
    return 0
  fi
  if grep -Eiq '(^|,)(nan|[-+]?inf)(,|$)' /tmp/rascl_wp3_tsk1_last_trajectory.csv; then
    echo "轨迹 CSV 包含 nan/inf；未解锁组 10。" >&2
    return 0
  fi
  head -n 5 /tmp/rascl_wp3_tsk1_last_trajectory.csv
  tail -n 5 /tmp/rascl_wp3_tsk1_last_trajectory.csv
  save_plan_state
  echo "规划已通过；当前目标可由组 10 执行：[$TARGET_X, $TARGET_Y, $TARGET_Z] m"
}

group_real_execute() {
  load_ros
  require_wp3_package
  require_matching_plan
  require_active_controllers
  local ros_x ros_y ros_z ros_duration
  ros_x="$(ros_double_literal "$TARGET_X")"
  ros_y="$(ros_double_literal "$TARGET_Y")"
  ros_z="$(ros_double_literal "$TARGET_Z")"
  ros_duration="$(ros_double_literal "$TRAJECTORY_DURATION")"
  timeout 3s ros2 topic echo --once /joint_states >/dev/null ||
    die "3 秒内没有 /joint_states，禁止执行"
  echo "该命令会运动实机；Drive 3 也处于 CSP，Task 1 会保持 spur gear 当前角度。"
  if ! ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
    -p target_x:="$ros_x" -p target_y:="$ros_y" -p target_z:="$ros_z" \
    -p duration:="$ros_duration" -p rate_hz:=50.0 -p execute:=true; then
    clear_plan_state
    echo "运动未到达规划终点；读取 bridge 自动保存的 CSP 停滞快照：" >&2
    read_csp_stall_snapshot || true
    die "停止发送目标；立即执行组 12 打包日志，再重启完整 EtherCAT 会话"
  fi
  clear_plan_state
  require_active_controllers
  timeout 3s ros2 topic echo --once /joint_states >/dev/null ||
    die "运动后 /joint_states 丢失；按指南执行完整 EtherCAT 会话重启"
  echo "运动命令结束；下一个目标必须重新执行组 14、9、10。"
}

group_stall_snapshot() {
  load_ros
  read_csp_stall_snapshot
}

group_process_check() {
  cd "$WORKSPACE"
  echo "仍在运行的 RASCL 进程："
  ps -ef | grep -E "ros2_control_node|rascl_faulhaber_bridge|wp3_tsk1" | grep -v grep || true
  echo "TCP 端口 15001："
  ss -ltnp | grep 15001 || true
}

group_pack_logs() {
  cd "$WORKSPACE"
  local output="$WORKSPACE/ros_logs_$(date +%Y%m%d_%H%M%S).tar.gz"
  [[ -d /root/.ros/log ]] || die "/root/.ros/log not found"
  tar -czf "$output" -C /root/.ros log
  echo "日志压缩包已生成：$output"
  echo "可直接从共享工作区拖出该 tar.gz 文件，无需手动复制日志文本。"
}

group_tcp_pose() {
  load_ros
  echo "当前模型 TCP：base_link -> tcp_link（读取实时 /joint_states）"
  echo "Translation 的 x/y/z 单位为米；以下显示约 3 秒。"
  timeout 3s ros2 run tf2_ros tf2_echo base_link tcp_link || [[ "$?" -eq 124 ]]
}

group_gripper_action() {
  load_ros
  require_csp_session
  require_active_controllers
  require_no_active_wp3_motion
  is_number "$SPUR_GEAR_DIRECTION" && [[ "$SPUR_GEAR_DIRECTION" != "0" ]] ||
    die "RASCL_SPUR_GEAR_DIRECTION 必须是非零数字"
  is_positive_number "$SPUR_GEAR_COUNTS_PER_REVOLUTION" ||
    die "RASCL_SPUR_GEAR_COUNTS_PER_REVOLUTION 必须是正数"
  is_positive_number "$SPUR_GEAR_SPEED_COUNTS_PER_S" ||
    die "RASCL_SPUR_GEAR_SPEED_COUNTS_PER_S 必须是正数"
  is_positive_number "$SPUR_GEAR_MIN_MOTION_DURATION_S" ||
    die "RASCL_SPUR_GEAR_MIN_MOTION_DURATION_S 必须是正数"
  is_positive_number "$SPUR_GEAR_SETTLE_DURATION_S" ||
    die "RASCL_SPUR_GEAR_SETTLE_DURATION_S 必须是正数"
  is_positive_number "$SPUR_GEAR_FEEDBACK_TIMEOUT_S" ||
    die "RASCL_SPUR_GEAR_FEEDBACK_TIMEOUT_S 必须是正数"
  is_integer "$GRIPPER_CONTACT_ERROR_COUNTS" &&
    (( GRIPPER_CONTACT_ERROR_COUNTS > 0 )) ||
    die "RASCL_GRIPPER_CONTACT_ERROR_COUNTS 必须是正整数"
  is_positive_number "$GRIPPER_CONTACT_CONFIRM_S" ||
    die "RASCL_GRIPPER_CONTACT_CONFIRM_S 必须是正数"
  is_number "$SPUR_GEAR_MIN_POSITION_RAD" && is_number "$SPUR_GEAR_MAX_POSITION_RAD" ||
    die "Drive 3 URDF 限位必须是数字"

  local snapshot shoulder upperarm lowerarm spur gripper_action action_label
  local delta_counts target_rad minimum_duration motion_duration stop_on_contact
  if ! snapshot="$(read_csp_joint_snapshot)"; then
    die "$SPUR_GEAR_FEEDBACK_TIMEOUT_S 秒内未收到完整 /joint_states；禁止控制 Drive 3"
  fi
  read -r shoulder upperarm lowerarm spur <<<"$snapshot"
  echo "Drive 3 当前 joint position = $spur rad；所有动作均以当前位置为基准，不依赖 Home。"
  read -r -p "Gripper action [close/open] (c/o) 或相对 counts（正/负整数）: " gripper_action
  gripper_action="${gripper_action,,}"
  case "$gripper_action" in
    close | c)
      action_label="收紧夹持"
      delta_counts="$GRIPPER_GRIP_DELTA_COUNTS"
      stop_on_contact=1
      ;;
    open | o)
      action_label="松开放下"
      delta_counts="$GRIPPER_RELEASE_DELTA_COUNTS"
      stop_on_contact=0
      ;;
    *)
      is_integer "$gripper_action" ||
        die "未执行 Drive 3 动作：请输入 close/c、open/o 或非零整数 counts"
      [[ "$gripper_action" =~ [1-9] ]] || die "相对 counts 不能为 0"
      action_label="自定义相对 counts"
      delta_counts="$gripper_action"
      stop_on_contact=0
      ;;
  esac
  if ! read -r target_rad minimum_duration < <(python3 - "$spur" "$delta_counts" "$SPUR_GEAR_DIRECTION" "$SPUR_GEAR_COUNTS_PER_REVOLUTION" "$SPUR_GEAR_MIN_POSITION_RAD" "$SPUR_GEAR_MAX_POSITION_RAD" "$SPUR_GEAR_SPEED_COUNTS_PER_S" "$SPUR_GEAR_MIN_MOTION_DURATION_S" <<'PY'
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
    die "抓夹 $action_label 指令被 Drive 3 URDF 软件限位拒绝"
  fi

  motion_duration="$minimum_duration"

  clear_plan_state
  echo "抓夹将执行“$action_label”：Drive 3 相对运动 $delta_counts counts。"
  echo "使用 50 Hz minimum-jerk CSP 轨迹，自动时长 $motion_duration s。"
  if (( stop_on_contact )); then
    echo "快捷动作将在持续跟踪误差达到 $GRIPPER_CONTACT_ERROR_COUNTS counts、维持 $GRIPPER_CONTACT_CONFIRM_S s 时判定接触/端点，立即保持实测位置；该结果按成功处理。"
  else
    echo "本动作要求精确运动 $delta_counts counts，不启用接触提前终止。"
  fi
  echo "前三轴保持当前 joint_state；此操作已清除旧组 9 规划授权。"
  if ! python3 - "$shoulder" "$upperarm" "$lowerarm" "$spur" "$target_rad" \
    "$delta_counts" "$motion_duration" "$SPUR_GEAR_DIRECTION" \
    "$SPUR_GEAR_COUNTS_PER_REVOLUTION" "$SPUR_GEAR_HOME_OFFSET_COUNTS" \
    "$SPUR_GEAR_FEEDBACK_TIMEOUT_S" "$SPUR_GEAR_SETTLE_DURATION_S" \
    "$stop_on_contact" "$GRIPPER_CONTACT_ERROR_COUNTS" \
    "$GRIPPER_CONTACT_CONFIRM_S" <<'PY'
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
    stop_on_contact,
    contact_error_counts,
    contact_confirm_s,
) = map(float, sys.argv[1:])
stop_on_contact = bool(int(stop_on_contact))
contact_error_counts = int(contact_error_counts)

JOINTS = ("shoulder_joint", "upperarm_joint", "lowerarm_joint", "spur_gear_joint")
TAU = 2.0 * math.pi
latest_spur = None
last_feedback_time = None
contact_error_since = None


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


def detect_contact(command_spur):
    global contact_error_since
    if not stop_on_contact or latest_spur is None:
        return None
    command_counts = rad_to_counts(command_spur)
    actual_counts = rad_to_counts(latest_spur)
    tracking_error = abs(command_counts - actual_counts)
    now = time.monotonic()
    if tracking_error < contact_error_counts:
        contact_error_since = None
        return None
    if contact_error_since is None:
        contact_error_since = now
        return None
    if now - contact_error_since < contact_confirm_s:
        return None

    logger.warning(
        "SPUR_CONTACT detected: "
        f"command_counts={command_counts} actual_counts={actual_counts} "
        f"tracking_error_counts={tracking_error} "
        f"threshold_counts={contact_error_counts} "
        f"confirm_s={contact_confirm_s:.3f}; "
        "holding measured Drive 3 position before following error"
    )
    return latest_spur


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

    period_s = 0.02
    start = time.monotonic()
    next_tick = start
    next_log = start
    outcome = "target_reached"
    final_target_spur = target_spur
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
        contact_hold_spur = detect_contact(command_spur)
        if contact_hold_spur is not None:
            outcome = "contact_or_endpoint"
            final_target_spur = contact_hold_spur
            publish(publisher, final_target_spur)
            break
        if u >= 1.0:
            break
        next_tick += period_s
        time.sleep(max(0.0, next_tick - time.monotonic()))

    settle_deadline = time.monotonic() + settle_s
    while time.monotonic() < settle_deadline:
        publish(publisher, final_target_spur)
        rclpy.spin_once(node, timeout_sec=0.0)
        if last_feedback_time is not None and time.monotonic() - last_feedback_time > 0.5:
            raise RuntimeError("/joint_states stopped while Drive 3 was settling")
        if outcome == "target_reached":
            contact_hold_spur = detect_contact(final_target_spur)
            if contact_hold_spur is not None:
                outcome = "contact_or_endpoint"
                final_target_spur = contact_hold_spur
                publish(publisher, final_target_spur)
        time.sleep(period_s)
    log_feedback(logger, "complete", final_target_spur)
    logger.info(
        f"SPUR_RESULT outcome={outcome} "
        f"requested_target_counts={target_counts} "
        f"held_target_counts={rad_to_counts(final_target_spur)}"
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
    die "Drive 3 CSP 轨迹中断；请立即执行组 12 并提交日志"
  fi
  require_active_controllers
  echo "抓夹动作完成：$action_label，delta=$delta_counts counts。随后执行组 9 时，Task 1 会保持此 spur gear 角度。"
}

group_set_target() {
  local x y z duration
  read -r -p "目标 x [m]（当前 $TARGET_X）：" x
  read -r -p "目标 y [m]（当前 $TARGET_Y）：" y
  read -r -p "目标 z [m]（当前 $TARGET_Z）：" z
  read -r -p "运动时间 [s]（当前 $TRAJECTORY_DURATION）：" duration
  x="${x:-$TARGET_X}"
  y="${y:-$TARGET_Y}"
  z="${z:-$TARGET_Z}"
  duration="${duration:-$TRAJECTORY_DURATION}"
  is_number "$x" || die "x 不是合法数字：$x"
  is_number "$y" || die "y 不是合法数字：$y"
  is_number "$z" || die "z 不是合法数字：$z"
  is_positive_number "$duration" || die "运动时间必须是大于 0 的普通十进制数字"
  TARGET_X="$x"
  TARGET_Y="$y"
  TARGET_Z="$z"
  TRAJECTORY_DURATION="$duration"
  save_target_state
  clear_plan_state
  echo "目标已设置为 [$TARGET_X, $TARGET_Y, $TARGET_Z] m，时间 $TRAJECTORY_DURATION s。"
  echo "下一步必须执行组 9，只规划成功后才能执行组 10。"
}

print_menu() {
  printf '%s\n' \
    "" \
    "RASCL 命令组" \
    "工作区   : $WORKSPACE" \
    "网卡     : $INTERFACE" \
    "ROS 域   : $ROS_DOMAIN_ID" \
    "目标 TCP : [$TARGET_X, $TARGET_Y, $TARGET_Z] m / $TRAJECTORY_DURATION s" \
    "" \
    "  1  编译 + 功能测试                                  [T1]" \
    "  2  启动 fake ros2_control（前台持续运行）            [T1]" \
    "  3  Fake 检查 + 规划 + 执行                           [T2]" \
    "  4  启动实机 Homing bridge（Drive 3 跳过 Home）        [T1]" \
    "  5  逐轴 Homing Drive 0、1、2                          [T2]" \
    "  6  home_all（执行 Drive 0-2）                         [T2]" \
    "  7  启动实机 CSP ros2_control（Drive 3 也参与）        [T2]" \
    "  8  Controller/joint state 保持检查 10 秒             [T3]" \
    "  9  只规划实机 minimum-jerk 轨迹                      [T3]" \
    " 10  执行实机 minimum-jerk 轨迹                        [T3，会运动]" \
    " 11  检查残留进程和 TCP 端口                           [T3]" \
    " 12  打包完整 ROS 日志到共享工作区                     [任意]" \
    " 13  查看当前模型 TCP 坐标                              [T3]" \
    " 14  设置下一次实机目标 TCP 和运动时间                  [T3]" \
    " 15  抓夹 close/open 或 Drive 3 自定义相对 counts      [T3，会运动]" \
    " 16  查看最近一次 CSP 停滞自动诊断快照                  [T3]" \
    "  0  退出" \
    "" \
    "组 2、4、7 会持续占用对应终端，直到按 Ctrl-C。" \
    "CSP 顺序：T1=4；T2=6→7；T3=8→13→14→9→10。Drive 3：T3=15。"
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
    0) exit 0 ;;
    *) die "未知组号: $1" ;;
  esac
}

if [[ $# -gt 0 ]]; then
  load_target_state
  run_group "$1"
else
  load_target_state
  while true; do
    print_menu
    read -r -p "请输入组号: " selection
    selection="${selection//$'\r'/}"
    run_group "$selection"
    echo "命令组 $selection 已返回菜单；是否成功以上方输出为准。"
  done
fi
