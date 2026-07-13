# WP3 四轴 Homing + CSP 实机快速测试

本说明书用于已经确认四轴 Homing 方向、传感器和参数正确的机器人。流程只有：

```text
进入 Docker -> 编译 -> 四轴 home_all -> 停止 Homing
-> 启动 CSP/PDO -> 保持测试 -> 发送几毫米 minimum-jerk 运动
```

测试时急停必须在手边，机械臂周围不得有人或障碍物。

---

## 1. Terminal 1：启动 Docker 并编译

在实验室电脑的第一个 Ubuntu Terminal 中，进入仓库目录：

```bash
cd ~/RASCL_G8  # 实际目录不同时替换这一行
bash ./rosws.sh
```

进入容器、看到 `rascl-container:/root/ws$` 后执行：

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash

rm -rf build install log
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF

source install/local_setup.bash
export ROS_DOMAIN_ID=88
export ETHERCAT_IF=enx3c18a0264863
```

把 `enx3c18a0264863` 替换为实际 EtherCAT 网卡名。可用以下命令确认：

```bash
ip -br link
ip link show "$ETHERCAT_IF"
```

保持这个窗口打开，后面称为 `Terminal 1`。

---

## 2. Terminal 2：进入同一个 Docker

保持 Terminal 1 和容器运行，新开实验室电脑的第二个 Ubuntu Terminal：

```bash
cd ~/RASCL_G8  # 必须进入同一个仓库目录
bash ./rosws.sh
```

看到 `Attaching to running container...` 后执行：

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=88
```

保持这个窗口打开，后面称为 `Terminal 2`。

---

## 3. Terminal 1：启动四轴 Homing bridge

在 `Terminal 1`：

```bash
ros2 launch rascl_description homing.launch.py \
  interface:="$ETHERCAT_IF"
```

期望看到：

```text
Profile/Homing uses SDO-only PRE-OP; PDO mapping skipped
TCP bridge listening on 127.0.0.1:15001
```

保持该命令运行，不要关闭 Terminal 1。

---

## 4. Terminal 2：直接执行四轴 home_all

确认机器人位于已验证的安全 Homing 起始区域，然后在 `Terminal 2`：

```bash
ros2 service call /rascl_faulhaber_bridge/home_all \
  std_srvs/srv/Trigger "{}"
```

四轴按 Drive 0 -> 1 -> 2 -> 3 顺序 Homing。必须等待返回：

```text
success=True
```

任何轴方向错误、碰到限位、出现 fault/Homing Error 或找不到传感器时，立即急停，不要继续 CSP。

---

## 5. Terminal 1：停止 Homing

`home_all` 成功后，回到 `Terminal 1` 按一次 `Ctrl-C`。

等待重新出现：

```text
rascl-container:/root/ws$
```

bridge 会执行 Disable Operation / Disable Voltage。不要关闭 Docker。

在 `Terminal 2` 确认旧 bridge 已退出：

```bash
ps -ef | grep rascl_faulhaber_bridge | grep -v grep
ss -ltnp | grep 15001
```

正常情况两条命令都没有输出。

---

## 6. Terminal 1：启动 CSP/PDO

仍在同一个 `Terminal 1`：

```bash
ros2 launch rascl_description ros2_control.launch.py \
  interface:="$ETHERCAT_IF" \
  use_fake_hardware:=false
```

期望日志包含：

```text
assigning factory Position PDOs Rx=0x1601, Tx=0x1A01
SM2 cycle monitoring configured for 20000000 ns
Process image mapped
SM-Sync selected with cycle 20000000 ns
Master reached OP state
Activated real RASCL hardware in csp mode
```

如果出现 `WkcError`、`CSP/PDO loop stopped`、following error 或 SAFE-OP + Error，停止测试，不要发送运动命令。

---

## 7. Terminal 2：检查 CSP 保持状态

在 `Terminal 2`：

```bash
ros2 control list_controllers
ros2 topic echo --once /joint_states
```

期望：

```text
joint_state_broadcaster active
rascl_position_controller active
```

四轴位置应接近 `0 rad`。保持 10 秒，不发送运动目标；机械臂不应跳动，Terminal 1 不应出现 PDO/WKC/following error。

---

## 8. Terminal 2：先计算几毫米轨迹

在 `Terminal 2`：

```bash
ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
  -p target_x:=0.295 \
  -p target_y:=0.000 \
  -p target_z:=0.048 \
  -p duration:=6.0 \
  -p rate_hz:=50.0 \
  -p execute:=false
```

必须看到：

```text
IK result: success=True
```

快速检查轨迹：

```bash
head -n 5 /tmp/rascl_wp3_tsk1_last_trajectory.csv
tail -n 5 /tmp/rascl_wp3_tsk1_last_trajectory.csv
```

确认没有 `nan`，目标方向和空间安全后再执行下一步。

---

## 9. Terminal 2：发送真实 CSP 运动指令

在 `Terminal 2`：

```bash
ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
  -p target_x:=0.295 \
  -p target_y:=0.000 \
  -p target_z:=0.048 \
  -p duration:=6.0 \
  -p rate_hz:=50.0 \
  -p execute:=true
```

运动期间观察 `Terminal 1`。成功标准：

1. 机械臂平滑移动，没有突然跳变；
2. Terminal 1 没有 WKC、following error 或 PDO loop 错误；
3. 轨迹结束后命令自动退出；
4. `/joint_states` 持续更新。

运动结束后检查：

```bash
ros2 topic echo --once /joint_states
```

---

## 10. 结束测试

回到 `Terminal 1` 按 `Ctrl-C`，等待 CSP launch 完全退出。

先在 `Terminal 2`：

```bash
exit
```

最后在 `Terminal 1`：

```bash
exit
```

不要在 CSP 或 Homing 仍运行时直接关闭 Terminal 1。
