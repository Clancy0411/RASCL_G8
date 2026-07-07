# RASCL Group 8 — WP3 CSP/PDO 调试说明

本文档记录当前包在 WP2.2 + WP3 Task 1 基础上的 CSP/PDO 修改和调试流程。当前目标是：保留原有 Profile Position 功能，同时新增 `control_mode:=csp`，让 WP3 的 minimum-jerk 轨迹通过 motion controller 的 Cyclic Synchronous Position mode 和 PDO 周期 setpoint 执行。

---

## 1. 当前版本改了什么

这次只围绕 CSP/PDO 做修改，不改变 IK、minimum-jerk 轨迹生成、URDF 几何、joint 名字和原来的 Profile Position 路径。

主要修改文件：

```text
src/rascl_hardware_interface/scripts/rascl_faulhaber_bridge.py
src/rascl_hardware_interface/src/rascl_hardware_interface.cpp
src/rascl_hardware_interface/include/rascl_hardware_interface/rascl_hardware_interface.hpp
src/rascl_description/urdf/rascl.urdf
src/rascl_description/launch/ros2_control.launch.py
src/rascl_description/config/controllers_csp.yaml
src/rascl_wp3_ss26_group8/launch/wp3_tsk1.launch.py
README_RASCL_Group8.md
```

新增/保留两种底层控制模式：

```text
control_mode:=profile   原来的 Profile Position / MOVE_ALL 路径
control_mode:=csp       新增的 CSP + PDO / CSP_SETPOINT_ALL 路径
```

Profile 模式仍然保留，用于回退和对照测试。

---

## 2. CSP/PDO 实现逻辑

### 2.1 旧链路

原来的 WP2.2 链路是：

```text
ROS position command
→ rascl_hardware_interface::write()
→ TCP: MOVE_ALL count0 count1 count2 count3
→ rascl_faulhaber_bridge.py
→ Profile Position Mode, mode = 1
→ SDO 写 0x607A Target Position
→ Controlword bit 4 触发 motion
```

这能动，但不是 WP3 要求的 CSP trajectory streaming。

### 2.2 新链路

现在新增 CSP 链路：

```text
ROS position command / minimum-jerk samples
→ rascl_hardware_interface::write()
→ TCP: CSP_SETPOINT_ALL count0 count1 count2 count3
→ rascl_faulhaber_bridge.py
→ PDO 写 RxPDO
→ Faulhaber drive 处于 CSP mode, mode = 8
```

硬件接口在 `control_mode:=csp` 下每个 write cycle 都发送一次 PDO setpoint，即使目标暂时不变，也继续保持周期通信。

### 2.3 PDO mapping

bridge 启动时会在 `config_map()` 之前尝试配置每个选中 slave 的 PDO mapping：

RxPDO `0x1600`：

```text
0x6040:00 Controlword       16 bit
0x607A:00 Target position   32 bit
0x6060:00 Mode of operation  8 bit
```

TxPDO `0x1A00`：

```text
0x6041:00 Statusword                16 bit
0x6064:00 Position actual value     32 bit
0x6061:00 Mode of operation display  8 bit
```

也就是说，代码假定 PDO 字节布局是：

```text
RxPDO: <uint16 controlword, int32 target_position, int8 mode>
TxPDO: <uint16 statusword, int32 actual_position, int8 mode_display>
```

这一步是 CSP/PDO 能否工作的关键。如果某台机器上的驱动器拒绝 PDO remapping，需要先单独排查 PDO mapping，而不是直接运行 WP3 运动。

---

## 3. build

进入 container 后：

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
rm -rf build install log
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/local_setup.bash
export ROS_DOMAIN_ID=88
```

如果 build 失败，先不要上实机。把完整 build log 保存下来。

---

## 4. fake hardware 回归测试

CSP 是实机 EtherCAT/PDO 相关功能，fake hardware 不会真的走 PDO，但要先确认本次修改没有破坏原来的 fake hardware 和 WP3 node。

启动 fake hardware：

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=88

ros2 launch rascl_description ros2_control.launch.py use_fake_hardware:=true
```

第二个 terminal 运行 WP3 规划但不运动：

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
  -p rate_hz:=50.0 \
  -p execute:=false
```

确认 IK 成功后再执行：

```bash
ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
  -p target_x:=0.25 \
  -p target_y:=0.00 \
  -p target_z:=0.08 \
  -p duration:=4.0 \
  -p rate_hz:=50.0 \
  -p execute:=true
```

如果 fake hardware/RViz 不正常，不要继续实机 CSP。

---

## 5. 原 Profile Position 模式回归测试

实机前建议确认旧功能仍然可用。启动 profile 模式：

```bash
ros2 launch rascl_description ros2_control.launch.py \
  interface:=robot_interface \
  use_fake_hardware:=false \
  control_mode:=profile \
  controller_config:=controllers.yaml
```

如果网卡不是 `robot_interface`，换成实际网卡名，例如：

```bash
interface:=enx3c18a0256e51
```

另一个 terminal 检查 joint states：

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=88

ros2 topic echo --once /joint_states
```

小幅命令测试，例如保持或轻微移动一个 joint：

```bash
ros2 topic pub --once /rascl_position_controller/commands std_msgs/msg/Float64MultiArray \
"{data: [0.0, 0.0, 0.0, 0.0]}"
```

确认旧模式没坏后，再测 CSP。

---

## 6. CSP/PDO 低层调试：只启动 bridge

这一步用于验证 EtherCAT、PDO mapping、OP state、CSP mode，不经过 ros2_control。

先不要启动 `ros2_control.launch.py`。只启动 bridge：

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=88

ros2 run rascl_hardware_interface rascl_faulhaber_bridge.py --ros-args \
  -p interface:=robot_interface \
  -p configure_pdo_mapping:=true \
  -p enable_dc_sync:=false \
  -p dc_cycle_ns:=20000000 \
  -p pdo_timeout_us:=20000
```

正常日志应该包括类似：

```text
[EtherCAT] Opening interface: ...
[EtherCAT] Found ... slave(s)
[EtherCAT] Configuring CSP PDO mapping for slave ...
[EtherCAT] PDO mapping configured
[EtherCAT] Master reached OP state
TCP bridge listening on 127.0.0.1:15001
```

如果这里报错，先不要继续。常见问题见本文档第 11 节。

---

## 7. TCP 命令验证 CSP

保持 bridge 运行，另开 terminal：

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=88
```

测试 PING / GET_ALL：

```bash
python3 - <<'PY'
import socket
s = socket.create_connection(("127.0.0.1", 15001), timeout=5)
for cmd in ["PING", "GET_ALL", "GET_MODE_ALL"]:
    s.sendall((cmd + "\n").encode())
    print(cmd, "=>", s.recv(4096).decode().strip())
s.close()
PY
```

进入 CSP，但不要求运动：

```bash
python3 - <<'PY'
import socket, time
s = socket.create_connection(("127.0.0.1", 15001), timeout=5)
for cmd in ["GET_ALL", "ENTER_CSP_ALL", "GET_ALL", "GET_MODE_ALL"]:
    s.sendall((cmd + "\n").encode())
    print(cmd, "=>", s.recv(4096).decode().strip())
    time.sleep(0.2)
s.close()
PY
```

成功时，`ENTER_CSP_ALL` 的响应应该形如：

```text
OK actual0 status0 mode0 actual1 status1 mode1 actual2 status2 mode2 actual3 status3 mode3
```

其中每个 mode 应该是：

```text
8
```

statusword 中应该包含 Operation Enabled。常见状态可能是 `0x0027` 或带有其它状态位的 `0x0427` / `0x1427`，只要没有 fault bit，一般不是故障。

退出 CSP：

```bash
python3 - <<'PY'
import socket
s = socket.create_connection(("127.0.0.1", 15001), timeout=5)
s.sendall(b"EXIT_CSP_ALL\n")
print(s.recv(4096).decode().strip())
s.close()
PY
```

---

## 8. CSP 下保持当前位置

这个脚本读取当前 counts，然后用 `CSP_SETPOINT_ALL` 反复发送当前位置。理论上机器人不应明显运动。

```bash
python3 - <<'PY'
import socket, time

s = socket.create_connection(("127.0.0.1", 15001), timeout=5)

def cmd(c):
    s.sendall((c + "\n").encode())
    return s.recv(4096).decode().strip()

print("ENTER", cmd("ENTER_CSP_ALL"))
reply = cmd("GET_ALL")
print("GET", reply)
parts = reply.split()
counts = [int(parts[i]) for i in range(1, len(parts), 2)]
print("counts", counts)

for i in range(50):
    reply = cmd("CSP_SETPOINT_ALL " + " ".join(str(c) for c in counts))
    if i % 10 == 0:
        print(i, reply)
    time.sleep(0.02)

print("EXIT", cmd("EXIT_CSP_ALL"))
s.close()
PY
```

这一步若出现突然运动，立刻停止，不要继续。

---

## 9. ros2_control CSP 模式启动

低层 bridge 验证没问题后，再启动完整 ros2_control CSP 模式。

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=88

ros2 launch rascl_description ros2_control.launch.py \
  interface:=robot_interface \
  use_fake_hardware:=false \
  control_mode:=csp \
  controller_config:=controllers_csp.yaml
```

这里的关键参数是：

```text
control_mode:=csp
controller_config:=controllers_csp.yaml
```

`controllers_csp.yaml` 把 controller manager 的 update rate 设为 50 Hz。不要一开始就追求 100 Hz 或更高。

另一个 terminal 检查：

```bash
ros2 topic echo --once /joint_states
ros2 control list_controllers
```

如果 controller 都 active，并且 `/joint_states` 正常刷新，说明 ros2_control 和 CSP/PDO 桥接至少已经进入闭环。

---

## 10. WP3 minimum-jerk 通过 CSP 执行

先只规划，不运动：

```bash
ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
  -p target_x:=0.29 \
  -p target_y:=0.00 \
  -p target_z:=0.05 \
  -p duration:=5.0 \
  -p rate_hz:=50.0 \
  -p execute:=false
```

确认 IK 成功、目标很近、CSV 生成正常后，再执行：

```bash
ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
  -p target_x:=0.29 \
  -p target_y:=0.00 \
  -p target_z:=0.05 \
  -p duration:=5.0 \
  -p rate_hz:=50.0 \
  -p execute:=true
```

实机第一次测试建议：

```text
目标点离当前 TCP 很近
rate_hz = 50
运动时间 duration >= 5 s
不要同时大幅改变多个 joint
手靠近急停
```

---

## 11. 常见问题排查

### 11.1 PDO mapping 报错

如果 bridge 启动时在 `Configuring CSP PDO mapping` 附近报错，说明驱动器可能拒绝修改 `0x1600/0x1A00/0x1C12/0x1C13`。

先不要运行 WP3。可以试着启动 bridge 时关闭自动 remap 做诊断：

```bash
ros2 run rascl_hardware_interface rascl_faulhaber_bridge.py --ros-args \
  -p interface:=robot_interface \
  -p configure_pdo_mapping:=false
```

但注意：当前 CSP 代码假定 PDO layout 是本文第 2.3 节的 7-byte layout。若关闭 remap 但默认 PDO layout 不一致，`ENTER_CSP_ALL` / `CSP_SETPOINT_ALL` 可能无法正常工作。此时需要读取实际 PDO mapping 后再改代码。

### 11.2 EtherCAT 进不了 OP

如果出现：

```text
EtherCAT master did not reach OP state
```

检查：

```bash
ip link
```

确认网卡名正确。然后清理旧进程：

```bash
pkill -9 -f rascl_faulhaber_bridge.py
pkill -9 -f ros2_control_node
pkill -9 -f controller_manager
pkill -9 -f spawner
ros2 daemon stop
ros2 daemon start
```

再重新插电/上电，让 EtherCAT 从干净状态启动。

### 11.3 mode display 不是 8

`ENTER_CSP_ALL` 后如果 mode 不是 8，说明驱动器没有进入 CSP。不要继续发送轨迹。先确认 `0x6060` 是否支持 mode 8，以及驱动器状态机是否已经 Operation Enabled。

### 11.4 statusword 有 fault

如果 statusword fault bit 置位，停止测试，重启驱动/电源。不要在 fault 状态反复发送 CSP setpoints。

### 11.5 ROS topic 发了但机器人不动

检查 controller：

```bash
ros2 control list_controllers
ros2 topic echo --once /rascl_position_controller/commands
ros2 topic echo --once /joint_states
```

检查是否使用了 CSP 模式：

```bash
ros2 launch rascl_description ros2_control.launch.py ... control_mode:=csp controller_config:=controllers_csp.yaml
```

### 11.6 机器人方向反了

不要先改 CSP。方向问题仍然由 URDF 中每个 joint 的 `direction` 参数和运动学坐标约定决定。先回到 fake hardware / profile mode 确认：

```text
shoulder_direction
upperarm_direction
lowerarm_direction
gripper_direction
```

---

## 12. 回退到旧模式

任何时候需要回退到旧 Profile Position 路径，用：

```bash
ros2 launch rascl_description ros2_control.launch.py \
  interface:=robot_interface \
  use_fake_hardware:=false \
  control_mode:=profile \
  controller_config:=controllers.yaml
```

此时 C++ hardware interface 重新发送：

```text
MOVE_ALL count0 count1 count2 count3
```

bridge 重新使用：

```text
Profile Position Mode, mode = 1
```

---

## 13. 本版本的边界

当前代码已经实现：

```text
CSP mode = 8
PDO remapping
EtherCAT OP request
RxPDO target position / controlword / mode
TxPDO actual position / statusword / mode display
C++ control_mode 参数切换
CSP_SETPOINT_ALL 周期 setpoint 命令
controllers_csp.yaml 50 Hz 调试配置
```

当前代码没有强制启用 distributed clocks，`enable_dc_sync` 默认是 false。需要更严格同步时，可以在 bridge 参数里打开：

```bash
-p enable_dc_sync:=true -p dc_cycle_ns:=20000000
```

但建议先在 `enable_dc_sync:=false` 下完成基本 CSP/PDO 调试，再打开 DC sync。
