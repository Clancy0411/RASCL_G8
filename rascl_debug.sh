#!/usr/bin/env bash

# Interactive command groups for the RASCL Homing -> CSP debug workflow.
# Run inside the ROS container: bash ./rascl_debug.sh

set -Eeuo pipefail

WORKSPACE="${RASCL_WS:-/root/ws}"
# EtherCAT NIC on the current workstation.  Override only when deliberately
# using another workstation: RASCL_INTERFACE=<nic> bash ./rascl_debug.sh.
INTERFACE="${RASCL_INTERFACE:-enx3c18a0256deb}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}"
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

confirm_exact() {
  local expected="$1"
  local prompt="$2"
  local answer
  read -r -p "$prompt" answer
  [[ "$answer" == "$expected" ]] || die "Cancelled"
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
  echo "[2/2] Running functional hardware-interface tests..."
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
  echo "Drive 3 已忽略，并保持 Disable Voltage。"
  ros2 launch rascl_description homing.launch.py \
    interface:="$INTERFACE" \
    ignore_spur_gear_in_csp:=true
}

read_inputs() {
  ros2 service call /rascl_faulhaber_bridge/read_digital_inputs \
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
  confirm_exact HOME "检查传感器并支撑机械臂；输入 HOME 启动 Drive 0："
  home_one 0
  confirm_exact HOME "确认 Drive 0 成功；输入 HOME 启动 Drive 1："
  home_one 1
  confirm_exact HOME "确认 Drive 1 成功；输入 HOME 启动 Drive 2："
  home_one 2
  echo "Drive 0-2 Homing 已结束；确认最后响应包含 CSP handoff armed。"
}

group_home_all() {
  load_ros
  read_inputs
  echo "home_all 只会运动 Drive 0-2；Drive 3 不会运动。"
  confirm_exact HOME "检查传感器、支撑和空间；输入 HOME 继续："
  ros2 service call /rascl_faulhaber_bridge/home_all std_srvs/srv/Trigger "{}"
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
  set +e
  ros2 launch rascl_description ros2_control.launch.py \
    interface:="$INTERFACE" \
    use_fake_hardware:=false \
    start_bridge:=false \
    shoulder_home_offset_counts:=0 \
    upperarm_home_offset_counts:=-802816 \
    lowerarm_home_offset_counts:=-802816 \
    spur_gear_home_offset_counts:=0
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
  echo "该命令会运动实机；Drive 3 继续保持失能。"
  confirm_exact MOVE "确认目标 [$TARGET_X, $TARGET_Y, $TARGET_Z] m、支撑、空间和急停；输入 MOVE 执行："
  if ! ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
    -p target_x:="$ros_x" -p target_y:="$ros_y" -p target_z:="$ros_z" \
    -p duration:="$ros_duration" -p rate_hz:=50.0 -p execute:=true; then
    clear_plan_state
    die "运动节点失败；停止发送目标并按指南执行完整 EtherCAT 会话重启"
  fi
  clear_plan_state
  require_active_controllers
  timeout 3s ros2 topic echo --once /joint_states >/dev/null ||
    die "运动后 /joint_states 丢失；按指南执行完整 EtherCAT 会话重启"
  echo "运动命令结束；下一个目标必须重新执行组 14、9、10。"
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
  echo "当前模型 TCP：base_link -> spur_gear（读取实时 /joint_states）"
  echo "Translation 的 x/y/z 单位为米；以下显示约 3 秒。"
  timeout 3s ros2 run tf2_ros tf2_echo base_link spur_gear || [[ "$?" -eq 124 ]]
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
    "  4  启动实机 Homing bridge，忽略 Drive 3             [T1]" \
    "  5  逐轴 Homing Drive 0、1、2                         [T2]" \
    "  6  home_all（只执行 Drive 0-2）                      [T2]" \
    "  7  启动实机 CSP ros2_control（前台持续运行）         [T2]" \
    "  8  Controller/joint state 保持检查 10 秒             [T3]" \
    "  9  只规划实机 minimum-jerk 轨迹                      [T3]" \
    " 10  执行实机 minimum-jerk 轨迹                        [T3，会运动]" \
    " 11  检查残留进程和 TCP 端口                           [T3]" \
    " 12  打包完整 ROS 日志到共享工作区                     [任意]" \
    " 13  查看当前模型 TCP 坐标                              [T3]" \
    " 14  设置下一次实机目标 TCP 和运动时间                  [T3]" \
    "  0  退出" \
    "" \
    "组 2、4、7 会持续占用对应终端，直到按 Ctrl-C。" \
    "实机推荐顺序：T1=4；T2=6→7；T3=8→13→14→9→10。"
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
