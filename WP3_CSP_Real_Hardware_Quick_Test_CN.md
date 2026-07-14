# WP3 Homing→CSP 实机快速流程

只用于代码、逐轴 Homing 和 offset 已验证后的实机测试。需要 `T1/T2/T3` 三个
容器终端。急停在手边，并使用能承重的机械支撑。

## 1. 三个终端初始化

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=88
```

## 2. T1：启动唯一 bridge

```bash
ros2 launch rascl_description homing.launch.py \
  interface:=enx94bdbe9565bc
```

确认显示 SDO-only PRE-OP，PDO mapping deferred。`T1` 从现在到停机不得关闭。

## 3. T2：Homing

```bash
ros2 service call /rascl_faulhaber_bridge/read_digital_inputs \
  std_srvs/srv/Trigger "{}"

ros2 service call /rascl_faulhaber_bridge/home_all \
  std_srvs/srv/Trigger "{}"
```

必须返回：

```text
success=True
Homing completed for all drives; CSP handoff armed
```

不要停止 `T1`，不要调用 `disable_all`。

## 4. T2：复用同一 bridge 启动 CSP

```bash
ros2 launch rascl_description ros2_control.launch.py \
  interface:=enx94bdbe9565bc \
  use_fake_hardware:=false \
  start_bridge:=false \
  shoulder_home_offset_counts:=0 \
  upperarm_home_offset_counts:=-802816 \
  lowerarm_home_offset_counts:=-802816 \
  spur_gear_home_offset_counts:=0
```

`T1` 必须出现：

```text
Deferred process image mapped
Master reached OP state
Homing-to-CSP handoff completed without Shutdown/Disable controlwords
```

## 5. T3：保持测试

```bash
ros2 control list_controllers
ros2 topic echo --once /joint_states
ros2 topic hz /joint_states
```

要求 controller active、关节约为 `[0,+1.5708,+1.5708,0]`、保持 10 秒无跳动，
且没有 WKC/following error。只在 `T3` 按 `Ctrl-C` 停止 `topic hz`。

## 6. T3：先规划，再执行

```bash
ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
  -p target_x:=0.2108 -p target_y:=-0.00177 -p target_z:=0.2913 \
  -p duration:=12.0 -p rate_hz:=50.0 -p execute:=false

head -n 5 /tmp/rascl_wp3_tsk1_last_trajectory.csv
tail -n 5 /tmp/rascl_wp3_tsk1_last_trajectory.csv
```

确认 IK success、结果接近 `[0,1.5527,1.5550]`、CSV 无 `nan`，再执行：

```bash
ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
  -p target_x:=0.2108 -p target_y:=-0.00177 -p target_z:=0.2913 \
  -p duration:=12.0 -p rate_hz:=50.0 -p execute:=true
```

## 7. 停机

先支撑机械臂，然后：

1. `T2` 按 `Ctrl-C`，等待 ros2_control 完全退出；该步骤会失能。
2. `T1` 按 `Ctrl-C`，关闭 bridge。
3. `T3` 检查：

```bash
ps -ef | grep -E "ros2_control_node|rascl_faulhaber_bridge|wp3_tsk1" | grep -v grep
ss -ltnp | grep 15001
```

若出现 `home_all has not completed`、`lost Operation Enabled`、WKC、following
error 或 SAFE-OP，禁止重试运动，立即支撑/急停并保留完整日志。
