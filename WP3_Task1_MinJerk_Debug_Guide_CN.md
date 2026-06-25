# WP3 Task 1 Minimum-Jerk 调试指南（Group 8）

本文档用于调试当前版本的 `rascl_wp3_ss26_group8` 包。  
这一版的目标是先完成一个最小闭环：

```text
给定一个 base_link 坐标系下的空间目标点 x/y/z
→ 计算 IK
→ 生成 joint-space minimum-jerk 轨迹
→ 连续发送到 /rascl_position_controller/commands
→ 在 fake hardware / RViz 中验证
→ 再进行实机小幅测试
```

当前版本**暂时不控制 gripper**，也**暂时不改 CSP**。  
底层仍然沿用 WP2.2 已经调通的 position command 通道。

---

## 1. 当前版本功能范围

当前版本支持：

```text
1. 新增 ROS2 package: rascl_wp3_ss26_group8
2. 新增 node: wp3_tsk1
3. 支持命令行输入目标空间坐标 target_x / target_y / target_z
4. TCP 暂时定义为 spur_gear_joint 的原点
5. 只控制 TCP 的空间位置 x/y/z
6. 不控制任意末端方向
7. 根据当前 joint state 计算 IK
8. 从当前 joint position 到目标 joint position 生成 minimum-jerk 轨迹
9. 将轨迹连续发布到 /rascl_position_controller/commands
10. 可用 execute:=false 只规划不运动
11. 可用 execute:=true 执行轨迹
12. 自动保存最近一次生成的轨迹 CSV
```

当前版本不支持：

```text
1. gripper 开合控制
2. 完整 pick-and-place sequence
3. cube stacking
4. 完整末端姿态控制
5. 真正的 CSP mode
6. PDO cyclic process data streaming
```

---

## 2. 坐标系规定

当前版本使用的目标坐标系为：

```text
frame_id = base_link
unit = meter
```

也就是说，输入：

```text
target_x = 0.25
target_y = 0.00
target_z = 0.08
```

表示：

```text
目标 TCP 位置在 base_link 坐标系下为：
x = 0.25 m
y = 0.00 m
z = 0.08 m
```

注意：这里的 x/y/z 方向完全按照 URDF / RViz 中 `base_link` 的坐标轴定义，不一定等同于你肉眼看到的“左/右/前/后”。

调试时一定要先在 RViz 里确认 `base_link` 坐标轴方向。

---

## 3. TCP 定义

当前版本将 TCP 暂时定义为：

```text
spur_gear_joint 的原点
```

这不是最终夹爪抓取中心。  
后续真正做 cube grasping 时，建议把 TCP 改成两个 jaw 中间、真正接触 cube 的夹爪中心点。

当前版本这么做的原因是：

```text
1. spur_gear_joint 在 URDF 中已有明确位置
2. 不需要额外测量 gripper 几何
3. 适合先验证 IK + minimum-jerk 运动链路
```

---

## 4. Calibration pose / Home 规定

这一点非常重要。

WP3 中不能随便在任意姿态调用 `home_all`，否则空间坐标会失去意义。

当前版本规定：

```text
真实机器人应先摆到 URDF / RViz 中的零位姿态；
然后调用 home_all；
此时四个 joint 都应该读成 0 rad。
```

也就是说，calibration pose 定义为：

```text
[shoulder_joint, upperarm_joint, lowerarm_joint, spur_gear_joint]
=
[0.0, 0.0, 0.0, 0.0]
```

在这个 calibration pose 下，当前版本根据 URDF 估计的 TCP 位置约为：

```text
TCP in base_link:
x = 0.29756 m
y = -0.00177 m
z = 0.043001 m
```

所以第一次实机测试时，不要给很远的目标点。  
应该先给一个非常接近这个初始 TCP 的目标，例如：

```text
x = 0.29
y = 0.00
z = 0.05
```

---

## 5. 编译流程

进入 container 后执行：

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash

rm -rf build install log
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF

source install/local_setup.bash
export ROS_DOMAIN_ID=88
```

如果 build 成功，说明三个 package 都能被 colcon 找到并安装：

```text
rascl_description
rascl_hardware_interface
rascl_wp3_ss26_group8
```

可以检查 package 是否存在：

```bash
ros2 pkg list | grep rascl
```

期望至少看到：

```text
rascl_description
rascl_hardware_interface
rascl_wp3_ss26_group8
```

---

## 6. Fake hardware 调试流程

### 6.1 Terminal 1：启动 fake hardware

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=88

ros2 launch rascl_description ros2_control.launch.py use_fake_hardware:=true
```

看到 controller 激活成功即可：

```text
joint_state_broadcaster active
rascl_position_controller active
Successfully switched controllers
```

如果想打开 RViz，在另一个 terminal 中执行：

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=88

rviz2 -d src/rascl_description/rviz/urdf.rviz
```

---

### 6.2 Terminal 2：只规划，不运动

先用 `execute:=false`。  
这一步只计算 IK 和 minimum-jerk 轨迹，不会向 controller 发送运动命令。

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=88

ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
  -p target_x:=0.25 \
  -p target_y:=0.00 \
  -p target_z:=0.08 \
  -p duration:=4.0 \
  -p rate_hz:=10.0 \
  -p execute:=false
```

这一步用于确认：

```text
1. node 能正常启动
2. 当前 /joint_states 能读到
3. IK 能收敛
4. 目标点在可达范围内
5. 轨迹 CSV 能生成
```

成功后会生成最近一次轨迹文件：

```text
/tmp/rascl_wp3_tsk1_last_trajectory.csv
```

可以查看前几行：

```bash
head /tmp/rascl_wp3_tsk1_last_trajectory.csv
```

---

### 6.3 Terminal 2：执行 minimum-jerk 轨迹

确认 `execute:=false` 没问题后，再执行：

```bash
ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
  -p target_x:=0.25 \
  -p target_y:=0.00 \
  -p target_z:=0.08 \
  -p duration:=4.0 \
  -p rate_hz:=10.0 \
  -p execute:=true
```

这一步会连续发布 joint command 到：

```text
/rascl_position_controller/commands
```

在 RViz 中应该看到机械臂平滑运动。

---

## 7. 使用 launch 一起启动 fake hardware 和 wp3_tsk1

也可以用 WP3 launch 文件启动：

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=88

ros2 launch rascl_wp3_ss26_group8 wp3_tsk1.launch.py \
  start_robot:=true \
  use_fake_hardware:=true \
  target_x:=0.25 \
  target_y:=0.00 \
  target_z:=0.08 \
  duration:=4.0 \
  rate_hz:=10.0 \
  execute:=true
```

参数说明：

```text
start_robot:=true
  同时启动 rascl_description 的 ros2_control launch

use_fake_hardware:=true
  使用 fake hardware，不连接真实机器人

target_x / target_y / target_z
  base_link 坐标系下的目标 TCP 坐标，单位 meter

duration
  从当前 joint state 运动到目标 joint state 的总时间，单位 second

rate_hz
  发送 trajectory setpoints 的频率

execute
  false = 只规划不运动
  true  = 执行轨迹
```

---

## 8. 检查 /joint_states

查看当前 joint 状态：

```bash
ros2 topic echo --once /joint_states
```

也可以按固定顺序打印四个 joint：

```bash
python3 - <<'PY'
import rclpy
from sensor_msgs.msg import JointState

order = ["shoulder_joint", "upperarm_joint", "lowerarm_joint", "spur_gear_joint"]

rclpy.init()
node = rclpy.create_node("print_current_order")

def cb(msg):
    d = dict(zip(msg.name, msg.position))
    print([d.get(j, None) for j in order])
    rclpy.shutdown()

node.create_subscription(JointState, "/joint_states", cb, 10)
rclpy.spin(node)
PY
```

期望输出类似：

```text
[0.0, 0.0, 0.0, 0.0]
```

如果刚做完 calibration pose 和 `home_all`，这四个值应该接近 0。

---

## 9. 检查 controller 和 topic

查看 controller 状态：

```bash
ros2 control list_controllers
```

期望看到：

```text
joint_state_broadcaster active
rascl_position_controller active
```

查看 command topic 是否存在：

```bash
ros2 topic list | grep rascl_position_controller
```

查看 `/joint_states` 发布者：

```bash
ros2 topic info /joint_states -v
```

如果 warning 一直刷：

```text
Moved backwards in time, re-publishing joint transforms!
```

需要检查是否有多个 `/joint_states` publisher 或旧 ROS 节点残留。

---

## 10. 实机调试流程

实机调试不要直接大范围运动。  
必须先用 fake hardware 验证，再上实机小幅测试。

### 10.1 Terminal 1：启动真实硬件

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=88

ros2 launch rascl_description ros2_control.launch.py \
  interface:=robot_interface \
  use_fake_hardware:=false
```

如果真实网卡名不是 `robot_interface`，改成实际网卡名，例如：

```bash
ip link
```

找到类似：

```text
enx3c18a0256e51
```

则启动命令改为：

```bash
ros2 launch rascl_description ros2_control.launch.py \
  interface:=enx3c18a0256e51 \
  use_fake_hardware:=false
```

---

### 10.2 设置 calibration pose / home

将真实机器人摆到你规定的 URDF 零位姿态，然后调用：

```bash
ros2 service call /rascl_faulhaber_bridge/home_all \
  std_srvs/srv/Trigger "{}"
```

检查 joint state：

```bash
ros2 topic echo --once /joint_states
```

确认四个 joint 接近：

```text
[0.0, 0.0, 0.0, 0.0]
```

---

### 10.3 实机先只规划，不运动

实机第一步仍然用 `execute:=false`：

```bash
ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
  -p target_x:=0.29 \
  -p target_y:=0.00 \
  -p target_z:=0.05 \
  -p duration:=5.0 \
  -p rate_hz:=5.0 \
  -p execute:=false
```

确认：

```text
1. IK 成功
2. 没有明显超限
3. 目标点离当前 TCP 很近
4. 轨迹文件生成正常
```

---

### 10.4 实机小幅执行

确认上一步没问题后，再执行：

```bash
ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
  -p target_x:=0.29 \
  -p target_y:=0.00 \
  -p target_z:=0.05 \
  -p duration:=5.0 \
  -p rate_hz:=5.0 \
  -p execute:=true
```

实机第一步建议：

```text
duration >= 5.0
rate_hz = 5 或 10
目标点只比当前 TCP 移动几毫米到一两厘米
随时准备急停
```

不要一开始就用：

```text
rate_hz = 50
duration = 1
大范围 target
```

---

## 11. 常见目标点建议

Calibration pose 下，当前 TCP 大约是：

```text
[0.29756, -0.00177, 0.043001]
```

fake hardware 可先试：

```bash
ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
  -p target_x:=0.25 \
  -p target_y:=0.00 \
  -p target_z:=0.08 \
  -p duration:=4.0 \
  -p rate_hz:=10.0 \
  -p execute:=true
```

实机建议先试更小的位移：

```bash
ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
  -p target_x:=0.29 \
  -p target_y:=0.00 \
  -p target_z:=0.05 \
  -p duration:=5.0 \
  -p rate_hz:=5.0 \
  -p execute:=true
```

---

## 12. 清理旧 ROS 进程

如果 launch 异常、warning 一直刷、topic 混乱，可以先清理：

```bash
pkill -9 -f rviz2
pkill -9 -f robot_state_publisher
pkill -9 -f ros2_control_node
pkill -9 -f controller_manager
pkill -9 -f spawner
pkill -9 -f rascl_faulhaber_bridge.py

ros2 daemon stop
ros2 daemon start
```

如果怀疑多个 container 残留，在 WSL 或宿主机中查看：

```bash
docker ps
```

停掉不用的 container：

```bash
docker stop <container_name_or_id>
```

最彻底方式是在 Windows PowerShell 中：

```powershell
wsl --shutdown
```

然后重新打开 WSL 和 container。

---

## 13. 常见问题判断

### 13.1 Package not found

错误：

```text
Package 'rascl_wp3_ss26_group8' not found
```

原因通常是没有 source：

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=88
```

或者 build 没成功。

---

### 13.2 找不到 /joint_states

检查 fake/real hardware 是否已经启动：

```bash
ros2 topic list | grep joint_states
```

检查 controller：

```bash
ros2 control list_controllers
```

---

### 13.3 IK failed

可能原因：

```text
1. 目标点超出机械臂可达范围
2. target_z 太低，接近桌面或低于机械结构可达范围
3. target_x / target_y 对当前模型不可达
4. calibration pose 不符合规定
```

先用接近初始 TCP 的点测试。

---

### 13.4 RViz 方向和直觉相反

这不一定是错。当前坐标以 URDF 的 `base_link` 为准。

先查看 RViz 里的坐标轴：

```text
Red   = X
Green = Y
Blue  = Z
```

确认之后再决定 target_x / target_y 正负方向。

---

### 13.5 轨迹不平滑

检查参数：

```text
duration 太短 → 运动太急
rate_hz 太低 → 采样点太少
目标点太远 → joint 变化太大
```

建议：

```text
fake hardware:
  duration = 4.0
  rate_hz = 10 或 20

real hardware first test:
  duration = 5.0
  rate_hz = 5 或 10
```

---

## 14. 当前版本成功标准

在进入 CSP 修改前，当前版本至少应满足：

```text
1. colcon build 成功
2. fake hardware launch 成功
3. wp3_tsk1 execute:=false 能成功生成轨迹
4. /tmp/rascl_wp3_tsk1_last_trajectory.csv 存在且内容合理
5. wp3_tsk1 execute:=true 时 RViz 中机械臂平滑运动
6. 换几个近距离 target 后，运动方向和 base_link 坐标系一致
7. 实机小位移测试中，机器人没有异常跳动、反向、撞击
```

---

## 15. 下一步：CSP 修改方向

当前版本暂时没有改 CSP。  
如果本阶段验证通过，下一步应修改底层执行方式：

```text
当前：
  WP3 node 发布 minimum-jerk joint samples
  → /rascl_position_controller/commands
  → hardware_interface::write()
  → MOVE_ALL
  → bridge
  → Profile Position Mode

目标：
  WP3 node 发布 minimum-jerk joint samples
  → /rascl_position_controller/commands
  → hardware_interface::write()
  → CSP_SETPOINT_ALL
  → bridge
  → Cyclic Synchronous Position Mode, mode = 8
```

建议分阶段：

```text
1. bridge 中新增 MODE_CYCLIC_SYNC_POSITION = 8
2. 新增 ENTER_CSP，不运动，只确认 drive 能安全进入 CSP mode
3. 进入 CSP 前先把 target position 设置为当前 actual position，避免跳动
4. 新增 CSP_SETPOINT_ALL count0 count1 count2 count3
5. C++ hardware_interface 新增 control_mode 参数
6. control_mode:=profile 时保留旧 MOVE_ALL
7. control_mode:=csp 时发送 CSP_SETPOINT_ALL
8. 实机先做单关节极小幅测试
9. 再执行当前 wp3_tsk1 的 x/y/z minimum-jerk 轨迹
```

注意：严格意义上的 CSP 最好通过 PDO cyclic process data 实现，而不是 SDO 高频写入。  
但工程调试可以先做最小 CSP 版本，再逐步规范化。
