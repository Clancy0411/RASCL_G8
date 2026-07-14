#!/usr/bin/env bash

# Interactive command groups for the RASCL Homing -> CSP debug workflow.
# Run inside the ROS container: bash ./rascl_debug.sh

set -Eeuo pipefail

WORKSPACE="${RASCL_WS:-/root/ws}"
INTERFACE="${RASCL_INTERFACE:-enx3c18a026488a}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}"

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

group_build_test() {
  load_ros allow_unbuilt
  echo "[1/2] Building workspace..."
  colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON
  set +u
  # shellcheck disable=SC1091
  source install/local_setup.bash
  set -u
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
  ip link show "$INTERFACE" || die "EtherCAT interface $INTERFACE not found"
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
  echo "保持 T1 的 Homing bridge 运行；ros2_control 将持续占用当前终端。"
  ros2 launch rascl_description ros2_control.launch.py \
    interface:="$INTERFACE" \
    use_fake_hardware:=false \
    start_bridge:=false \
    shoulder_home_offset_counts:=0 \
    upperarm_home_offset_counts:=-802816 \
    lowerarm_home_offset_counts:=-802816 \
    spur_gear_home_offset_counts:=0
}

group_csp_check() {
  load_ros
  ros2 control list_controllers
  ros2 control list_hardware_interfaces
  timeout 10s ros2 topic echo --once /joint_states
  echo "测量 /joint_states 频率 10 秒……"
  timeout 10s ros2 topic hz /joint_states || [[ "$?" -eq 124 ]]
}

group_real_plan() {
  load_ros
  ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
    -p target_x:=0.2108 -p target_y:=-0.00177 -p target_z:=0.2913 \
    -p duration:=12.0 -p rate_hz:=50.0 -p execute:=false
  head -n 5 /tmp/rascl_wp3_tsk1_last_trajectory.csv
  tail -n 5 /tmp/rascl_wp3_tsk1_last_trajectory.csv
}

group_real_execute() {
  load_ros
  echo "该命令会运动实机；Drive 3 继续保持失能。"
  confirm_exact MOVE "确认 IK、支撑、空间和急停；输入 MOVE 执行："
  ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
    -p target_x:=0.2108 -p target_y:=-0.00177 -p target_z:=0.2913 \
    -p duration:=12.0 -p rate_hz:=50.0 -p execute:=true
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

print_menu() {
  printf '%s\n' \
    "" \
    "RASCL 命令组" \
    "工作区   : $WORKSPACE" \
    "网卡     : $INTERFACE" \
    "ROS 域   : $ROS_DOMAIN_ID" \
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
    "  0  退出" \
    "" \
    "组 2、4、7 会持续占用对应终端，直到按 Ctrl-C。" \
    "实机推荐顺序：T1=4，T2=6，保持 T1，再 T2=7，最后 T3=8,9,10。"
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
    0) exit 0 ;;
    *) die "未知组号: $1" ;;
  esac
}

if [[ $# -gt 0 ]]; then
  run_group "$1"
else
  while true; do
    print_menu
    read -r -p "请输入组号: " selection
    selection="${selection//$'\r'/}"
    run_group "$selection"
    echo "命令组 $selection 已完成。"
  done
fi
