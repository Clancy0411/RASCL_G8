# WP3 Task 1 Minimum-Jerk + CSP/PDO 完整操作与调试手册（Group 8）

本文档对应当前仓库代码，目标是让第一次接触本项目的人也能按照固定顺序完成：

```text
启动 Docker
-> 编译和软件测试
-> fake hardware 测试
-> 实机逐轴 Homing
-> 实机 CSP/PDO 保持测试
-> 几毫米 minimum-jerk 轨迹测试
```

最终真实硬件链路为：

```text
base_link 目标坐标
-> IK
-> 预计算 joint-space minimum-jerk 轨迹
-> ROS position controller
-> FAULHABER CSP mode 8
-> 20 ms EtherCAT Position PDO
```

Task Sheet 要求 Task 1 的运动控制器使用 CSP，并向驱动器发送位置轨迹。不要跳过本手册中的 fake hardware、逐轴 Homing 和 CSP 保持测试。

---

## 0. 开始前必须理解的 Terminal 规则

本手册使用以下名称：

| 名称 | 是什么 | 用途 |
|---|---|---|
| `Ubuntu-A` | 实验室电脑上的第一个 Ubuntu Terminal | 启动 Docker；进入容器后变成 `Container-T1` |
| `Container-T1` | 容器的主终端 | 编译；启动 fake、Homing、CSP 等需要持续运行的 launch |
| `Ubuntu-B` | 实验室电脑上的第二个 Ubuntu Terminal | 连接已经运行的容器；进入后变成 `Container-T2` |
| `Container-T2` | 容器的第二终端 | 调 service、检查 topic/controller、运行 WP3 轨迹节点 |

必须遵守：

1. `rosws.sh` 在实验室电脑的 Ubuntu 主机 Terminal 中运行，不在容器内部运行。
2. `Container-T1` 是容器主终端。整个调试结束前不要在这里输入 `exit`。
3. launch 正在 `Container-T1` 运行时，该窗口不能输入其他命令；检查命令全部在 `Container-T2` 运行。
4. 停止 launch 时，在 `Container-T1` 按一次 `Ctrl-C`。看到 shell 提示符重新出现，表示 launch 已停止，但容器仍在。
5. 不同测试阶段不能同时运行。例如 Homing launch 没停止时，不能启动 CSP launch。
6. 新开的每个容器 Terminal 都必须执行 ROS 环境初始化命令。一个 Terminal 中的 `source` 和 `export` 不会自动传给另一个 Terminal。
7. 如果 `Container-T1` 被关闭或输入 `exit`，`rosws.sh` 使用的 `--rm` 容器会停止并删除，`Container-T2` 也会断开。

判断自己在哪里：

```bash
pwd
```

- 输出 `/root/ws`：当前在容器内。
- 输出 `/home/.../RASCL_G8`：当前在 Ubuntu 主机的仓库目录。

---

## 1. 在实验室 Ubuntu 电脑上使用 rosws.sh 启动 Docker

### 1.1 检查 Ubuntu Docker Engine

操作位置：`Ubuntu-A`。

打开实验室电脑上的第一个 Ubuntu Terminal，执行：

```bash
docker version
```

期望同时看到 `Client` 和 `Server` 信息。如果出现 `Cannot connect to the Docker daemon`，执行：

```bash
sudo systemctl start docker
docker version
```

如果出现 Docker socket 的 `Permission denied`，执行：

```bash
sudo usermod -aG docker "$USER"
```

然后注销当前 Ubuntu 用户并重新登录。不要用 `sudo bash ./rosws.sh` 长期绕过权限，否则仓库中的 build 文件可能变成主机 root 所有。

### 1.2 进入仓库目录

仍在 `Ubuntu-A`。进入实验室电脑上的仓库目录。例如仓库位于 home 下时：

```bash
cd ~/RASCL_G8
pwd
ls rosws.sh Dockerfile
```

如果实际目录不同，只替换 `cd ~/RASCL_G8` 这一行。也可以先查找：

```bash
find "$HOME" -name rosws.sh -type f 2>/dev/null
```

最后一条 `ls` 命令必须能看到 `rosws.sh` 和 `Dockerfile`。

### 1.3 第一次启动或 Dockerfile 更新后启动

操作位置：`Ubuntu-A`。

本版本修改了 Dockerfile，因此第一次必须执行普通重新构建：

```bash
SOFT_REBUILD=true bash ./rosws.sh
```

脚本会：

1. 构建镜像 `ros2-irs-rascl-wp22`；
2. 创建同名容器；
3. 把当前仓库挂载到容器的 `/root/ws`；
4. 使用 host network、privileged 和 EtherCAT 所需 capability；
5. 直接进入容器 shell。

构建可能需要几分钟。完成后提示符类似：

```text
rascl-container:/root/ws$
```

从这一刻开始，这个窗口叫 `Container-T1`。不要再次在这个窗口运行 `rosws.sh`。

如果镜像已经是最新版本，以后可以直接使用：

```bash
bash ./rosws.sh
```

只有怀疑 Docker cache 或依赖损坏时才使用无缓存重建：

```bash
REBUILD=true bash ./rosws.sh
```

注意：重新构建镜像前必须先结束正在运行的旧容器，否则脚本只会 attach 到旧容器。

### 1.4 打开第二个容器 Terminal

先保持 `Container-T1` 开着。新开实验室电脑上的第二个 Ubuntu Terminal，这个窗口叫 `Ubuntu-B`。

在 `Ubuntu-B` 进入同一个仓库目录并执行：

```bash
cd ~/RASCL_G8  # 实际目录不同时替换这一行
bash ./rosws.sh
```

脚本应显示：

```text
Attaching to running container...
```

随后提示符变为：

```text
rascl-container:/root/ws$
```

这个窗口现在叫 `Container-T2`。如果它开始重新 build 镜像而不是显示 `Attaching`，说明原容器没有运行；返回检查 `Container-T1` 是否已经退出。

### 1.5 每个容器 Terminal 的环境初始化

下面这组命令在 `Container-T1` 和 `Container-T2` 中都要分别执行一次：

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=88
```

首次编译前还没有 `install`，因此暂时不要执行 `source install/local_setup.bash`。编译完成后，本手册会明确要求两个 Terminal 再分别 source。

---

## 2. 编译当前代码

### 2.1 关闭状态要求

操作位置：`Container-T1`。

此时不应运行任何 ROS launch。`Container-T2` 可以开着，但不要在那里运行 ROS 节点。

先检查：

```bash
ps -ef | grep -E "ros2_control_node|rascl_faulhaber_bridge|wp3_tsk1" | grep -v grep
```

正常情况没有输出。

### 2.2 清理并完整编译

在 `Container-T1` 执行：

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
rm -rf build install log

colcon build --symlink-install \
  --cmake-args -DBUILD_TESTING=ON
```

不要在编译过程中启动第二次编译。成功时最后应看到所有 package 为 `Finished`，且没有 `Failed`。

### 2.3 两个 Terminal 都重新加载编译结果

编译完成后，在 `Container-T1` 执行：

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=88
```

然后在 `Container-T2` 也执行同样的命令：

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=88
```

检查 package。可在任意一个容器 Terminal 执行：

```bash
ros2 pkg list | grep -E "rascl_description|rascl_hardware_interface|rascl_wp3_ss26_group8"
```

必须看到：

```text
rascl_description
rascl_hardware_interface
rascl_wp3_ss26_group8
```

### 2.4 检查 bridge 安装权限

操作位置：`Container-T1`。此时仍然不启动 ROS launch。

```bash
ls -l build/rascl_hardware_interface/scripts/rascl_faulhaber_bridge.py
ls -l install/rascl_hardware_interface/lib/rascl_hardware_interface/rascl_faulhaber_bridge.py
```

权限中应包含 `x`，例如：

```text
-rwxr-xr-x
```

如果没有执行权限，不要继续 launch，转到故障排查 11.3。

---

## 3. 软件单元测试（不连接电机）

### 3.1 Terminal 状态

- 使用：`Container-T1`。
- `Container-T2`：保持在 shell，不运行任何节点。
- 真实机器人：不需要上电，不需要连接 EtherCAT。
- 上一步编译使用了 `BUILD_TESTING=ON`，所以不用再次编译。

### 3.2 执行测试

在 `Container-T1` 执行：

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=88

colcon test --packages-select rascl_hardware_interface \
  --event-handlers console_direct+

colcon test-result --verbose
```

bridge PDO 测试会检查：

1. Position PDO payload 为 6 bytes、小端序；
2. 使用 RxPDO2 `0x1601` / TxPDO2 `0x1A01`；
3. 不写只读 PDO mapping count；
4. SM-Sync 周期合法并写入 `0x1C32:02`；
5. Profile/Homing 保持 SDO-only PRE-OP，不调用 `config_map()`；
6. 固定周期 PDO loop 可以重复交换目标和状态。

只有测试结果没有 failure 才进入 fake hardware。

---

## 4. Fake hardware 完整测试

Fake hardware 不连接 FAULHABER，不会驱动真实电机。它验证 URDF、ros2_control、IK、CSV 和 minimum-jerk 发布流程。

### 4.1 在 Terminal 1 启动 fake hardware

操作位置：`Container-T1`。

确认前一个 `colcon test` 已结束并已经回到 shell，然后执行：

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=88

ros2 launch rascl_description ros2_control.launch.py \
  use_fake_hardware:=true
```

这条命令会持续运行。保持 `Container-T1` 不动，不要在这里输入后续命令。

### 4.2 在 Terminal 2 检查 controller

操作位置：`Container-T2`。`Container-T1` 的 fake launch 必须继续运行。

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=88

ros2 control list_controllers
ros2 topic echo --once /joint_states
```

期望 controller 至少包含：

```text
joint_state_broadcaster active
rascl_position_controller active
```

`/joint_states` 应包含四个关节。若没有，先解决 controller 问题，不要继续轨迹测试。

### 4.3 在 Terminal 2 只规划、不运动

仍在 `Container-T2`，不要关闭或重启 `Container-T1`：

```bash
ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
  -p target_x:=0.25 \
  -p target_y:=0.00 \
  -p target_z:=0.08 \
  -p duration:=4.0 \
  -p rate_hz:=50.0 \
  -p execute:=false
```

该命令完成 IK、生成整条 minimum-jerk 轨迹和 CSV，然后自动退出。检查终端中是否出现 `IK result: success=True`。

命令退出后，仍在 `Container-T2` 检查 CSV：

```bash
head -n 5 /tmp/rascl_wp3_tsk1_last_trajectory.csv
tail -n 5 /tmp/rascl_wp3_tsk1_last_trajectory.csv
```

CSV 应有连续时间和四个关节的位置，不应出现 `nan`。

### 4.4 在 Terminal 2 执行 fake 轨迹

只有 4.3 成功后，在同一个 `Container-T2` 执行：

```bash
ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
  -p target_x:=0.25 \
  -p target_y:=0.00 \
  -p target_z:=0.08 \
  -p duration:=4.0 \
  -p rate_hz:=50.0 \
  -p execute:=true
```

等待命令自动结束，然后检查：

```bash
ros2 topic echo --once /joint_states
```

### 4.5 停止 fake hardware，但不要退出 Docker

回到 `Container-T1`，按一次：

```text
Ctrl-C
```

等待重新出现：

```text
rascl-container:/root/ws$
```

此时：

- `Container-T1`：保留，已经回到 shell；
- `Container-T2`：保留，已经回到 shell；
- 不要在任何窗口输入 `exit`；
- 下面继续做实机准备。

---

## 5. 实机前检查和 EtherCAT 网卡确认

### 5.1 物理安全检查

执行任何实机命令前：

1. 急停按钮必须在操作人员手边；
2. 机械臂周围无人、无线缆或工具阻挡；
3. 确认每个轴的 reference switch 和运动方向已经在 `auto_homing` 版本单轴验证；
4. 第一次 CSP 轨迹只允许几毫米位移；
5. 不要同时运行 Homing 和 CSP。

### 5.2 在 Ubuntu 主机启用 EtherCAT 网卡

`ip link` 是 Ubuntu 主机命令，不在 Docker 容器内执行。保持 `Container-T1` 和 `Container-T2` 开着，另外新开一个普通 Ubuntu 主机 Terminal；不要在这个新窗口运行 `rosws.sh`。

实验室电脑上连接 EtherCAT 驱动链的专用网卡已经确认为：

```text
enx94bdbe9565bc
```

在这个 Ubuntu 主机 Terminal 执行：

```bash
ip link show enx94bdbe9565bc
sudo ip link set enx94bdbe9565bc up
ip link show enx94bdbe9565bc
```

最后一次输出应显示网卡为 `UP`。如果提示 `Device does not exist`，不要继续 Homing。检查完成后可以关闭这个额外的 Ubuntu 主机 Terminal；两个 Docker Terminal 继续保留。

### 5.3 清理 ROS graph 缓存

操作位置：`Container-T2`。`Container-T1` 此时没有运行 launch。

```bash
source /opt/ros/jazzy/setup.bash
source /root/ws/install/local_setup.bash
export ROS_DOMAIN_ID=88

ros2 daemon stop
ros2 daemon start
```

---

## 6. 实机 Homing：先逐轴，再四轴

Homing 使用 CiA 402 mode 6 和 SDO。当前专用 Homing launch 不启动 ros2_control，不配置 PDO，也不进入 EtherCAT OP；这保持了 `auto_homing` 已验证的 SDO-only PRE-OP 行为。

当前已验证参数：

```text
homing_methods       = [28, 28, 24, 24]
reference_inputs     = [2, 2, 2, 1]
homing_offsets       = [0, 0, 0, 0]
homing_search_speeds = [1000, 1000, 1000, 1000]
homing_zero_speeds   = [200, 200, 200, 200]
homing_accelerations = [1000, 1000, 1000, 1000]
motion_timeout_s     = 8.0
```

对应关系：

| Drive | Joint | Method | Reference input |
|---:|---|---:|---:|
| 0 | `shoulder_joint` | 28 | DigIn2 |
| 1 | `upperarm_joint` | 28 | DigIn2 |
| 2 | `lowerarm_joint` | 24 | DigIn2 |
| 3 | `spur_gear_joint` | 24 | DigIn1 |

### 6.1 Terminal 1 启动 Homing bridge

操作位置：`Container-T1`。直接使用已经确认的实机网卡名启动：

```bash
ros2 launch rascl_description homing.launch.py \
  interface:=enx94bdbe9565bc
```

保持 `Container-T1` 运行。期望看到：

```text
Profile/Homing uses SDO-only PRE-OP; PDO mapping skipped
TCP bridge listening on 127.0.0.1:15001
```

如果提示找不到 slave、网卡不存在或权限错误，不要在 `Container-T2` 调 Homing service。

### 6.2 Terminal 2 检查数字输入

操作位置：`Container-T2`。`Container-T1` 的 Homing bridge 必须继续运行。

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=88

ros2 service list | grep rascl_faulhaber_bridge
ros2 service call /rascl_faulhaber_bridge/read_digital_inputs \
  std_srvs/srv/Trigger "{}"
```

应看到四个 Drive 的 physical、logical 和 polarity。必要时人工触发对应传感器，再次运行同一条 service 命令，确认输入确实变化。

### 6.3 Terminal 2 单独 Homing Drive 0

仍在 `Container-T2`：

```bash
ros2 param set /rascl_faulhaber_bridge test_drive_index 0
ros2 service call /rascl_faulhaber_bridge/home_one \
  std_srvs/srv/Trigger "{}"
```

等待 service 返回后再做下一轴。成功示例：

```text
success=True
message='Drive 0 homing completed; actual_position=...'
```

如果运动方向不对、碰到机械限位、出现 fault/Homing Error 或 8 秒内找不到开关：立即急停，不要继续 Drive 1，也不要调用 `home_all`。

### 6.4 Terminal 2 单独 Homing Drive 1

Drive 0 成功后，在同一 `Container-T2` 执行：

```bash
ros2 param set /rascl_faulhaber_bridge test_drive_index 1
ros2 service call /rascl_faulhaber_bridge/home_one \
  std_srvs/srv/Trigger "{}"
```

等待成功返回，再继续。

### 6.5 Terminal 2 单独 Homing Drive 2

Drive 1 成功后执行：

```bash
ros2 param set /rascl_faulhaber_bridge test_drive_index 2
ros2 service call /rascl_faulhaber_bridge/home_one \
  std_srvs/srv/Trigger "{}"
```

等待成功返回，再继续。

### 6.6 Terminal 2 单独 Homing Drive 3

Drive 2 成功后执行：

```bash
ros2 param set /rascl_faulhaber_bridge test_drive_index 3
ros2 service call /rascl_faulhaber_bridge/home_one \
  std_srvs/srv/Trigger "{}"
```

只有四个 `home_one` 都成功，才进入下一步。

### 6.7 Terminal 2 执行完整 home_all

仍然保持 `Container-T1` 的同一个 Homing bridge，不要重启它。在 `Container-T2` 执行：

```bash
ros2 service call /rascl_faulhaber_bridge/home_all \
  std_srvs/srv/Trigger "{}"
```

`home_all` 会按 Drive 0 -> 1 -> 2 -> 3 顺序再次执行 Homing。必须等待整个 service 返回 `success=True`。

### 6.8 停止 Homing bridge

完成后回到 `Container-T1`，按一次 `Ctrl-C`。bridge 会尝试对每个驱动执行 Disable Operation 和 Disable Voltage，然后关闭 EtherCAT master。

必须等待 `Container-T1` 回到 shell。不要直接关窗口。

然后在 `Container-T2` 检查旧进程和端口：

```bash
ps -ef | grep rascl_faulhaber_bridge | grep -v grep
ss -ltnp | grep 15001
```

正常情况两条命令都没有输出。如果仍有进程，先处理故障排查 11.4，不要启动 CSP。

此时两个容器 Terminal 都继续保留，不输入 `exit`。

---

## 7. 实机 CSP/PDO 启动和保持测试

### 7.1 Terminal 1 启动 CSP

操作位置：刚刚停止 Homing 的同一个 `Container-T1`。

启动 CSP：

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=88

ros2 launch rascl_description ros2_control.launch.py \
  interface:=enx94bdbe9565bc \
  use_fake_hardware:=false
```

这条命令持续运行。不要在 `Container-T1` 输入其他命令。

默认值为：

```text
control_mode = csp
controller_config = controllers_csp.yaml
update_rate = 50 Hz
pdo_cycle_ns = 20000000
enable_dc_sync = false
```

每个 Drive 应出现类似日志：

```text
assigning factory Position PDOs Rx=0x1601, Tx=0x1A01
SM2 cycle monitoring configured for 20000000 ns
```

随后应看到：

```text
Process image mapped
SM-Sync selected with cycle 20000000 ns
Master reached OP state
Activated real RASCL hardware in csp mode
```

不应出现：

```text
pysoem.WkcError
CSP/PDO loop stopped
following error
SAFE-OP + Error
```

### 7.2 Terminal 2 检查 controller

操作位置：`Container-T2`。保持 `Container-T1` 的 CSP launch 运行。

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=88

ros2 control list_controllers
```

期望：

```text
joint_state_broadcaster active
rascl_position_controller active
```

如果任一 controller 为 inactive，不要发送运动目标。

### 7.3 Terminal 2 检查 Homing 后零位

仍在 `Container-T2`：

```bash
ros2 topic echo --once /joint_states
```

四个关节应接近：

```text
[0.0, 0.0, 0.0, 0.0]
```

少量编码器/换算误差可以记录，但明显偏离零位时不要继续。

### 7.4 保持 10 秒，不发送目标

现在不要运行 `wp3_tsk1`，也不要发布 controller command。

先同时观察：

- `Container-T1`：日志中不应出现 PDO/WKC/following error；
- 机械臂：激活时不应跳动；
- `Container-T2`：确认 joint state 持续更新。

在 `Container-T2` 执行：

```bash
ros2 topic hz /joint_states
```

观察至少 10 秒，然后只在 `Container-T2` 按 `Ctrl-C` 停止 `topic hz`。不要在 `Container-T1` 按 Ctrl-C，因为 CSP launch 仍要继续。

保持测试必须同时满足：

1. 机械臂无突然跳动；
2. `/joint_states` 持续更新；
3. `Container-T1` 无 `CSP/PDO loop stopped`；
4. 无 following error；
5. EtherCAT 没有退回 SAFE-OP + Error。

只有全部满足才进入轨迹测试。

---

## 8. 实机 minimum-jerk 几毫米轨迹

### 8.1 保持哪些窗口

- `Container-T1`：继续运行第 7 节的同一个 CSP launch，不关闭、不重启。
- `Container-T2`：运行规划和轨迹命令。
- 不需要新开第三个 Terminal。

### 8.2 Terminal 2 先只规划

在 `Container-T2` 执行：

```bash
ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
  -p target_x:=0.295 \
  -p target_y:=0.000 \
  -p target_z:=0.048 \
  -p duration:=6.0 \
  -p rate_hz:=50.0 \
  -p execute:=false
```

该命令自动退出后，检查：

```bash
head -n 5 /tmp/rascl_wp3_tsk1_last_trajectory.csv
tail -n 5 /tmp/rascl_wp3_tsk1_last_trajectory.csv
```

必须确认：

1. `IK result: success=True`；
2. 目标点与当前零位 TCP 很接近；
3. CSV 没有 `nan`；
4. duration 为 6 秒，不是突然运动；
5. 操作人员确认机械方向和空间安全。

### 8.3 Terminal 2 执行真实轨迹

确认 8.2 全部通过后，在同一个 `Container-T2` 执行：

```bash
ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
  -p target_x:=0.295 \
  -p target_y:=0.000 \
  -p target_z:=0.048 \
  -p duration:=6.0 \
  -p rate_hz:=50.0 \
  -p execute:=true
```

运动期间：

- 手放在急停附近；
- 观察 `Container-T1` 的 PDO/WKC/following error；
- 不要在另一个 Terminal 同时启动第二个 `wp3_tsk1`；
- 任何方向异常立即急停。

轨迹命令自动退出后，在 `Container-T2` 检查：

```bash
ros2 topic echo --once /joint_states
```

第一次只验证几毫米运动。成功后才逐步扩大范围，不要同时增大距离并缩短 duration。

### 8.4 完成后停止 CSP

回到 `Container-T1`，按一次 `Ctrl-C`。等待回到 shell。

然后在 `Container-T2` 执行：

```bash
ps -ef | grep -E "ros2_control_node|rascl_faulhaber_bridge" | grep -v grep
ss -ltnp | grep 15001
```

正常情况无输出。两个 Terminal 仍然可以保留，用于后续可选测试。

---

## 9. 可选测试：Profile Position 回归模式

该模式只用于排查旧 SDO 路径，不用于 WP3 CSP 验收。

开始条件：第 7/8 节的 CSP launch 已经在 `Container-T1` 用 Ctrl-C 停止，端口 15001 没有占用。

### 9.1 Terminal 1 启动 Profile

在 `Container-T1`：

```bash
ros2 launch rascl_description ros2_control.launch.py \
  interface:=enx94bdbe9565bc \
  use_fake_hardware:=false \
  control_mode:=profile \
  controller_config:=controllers.yaml
```

期望看到：

```text
Profile/Homing uses SDO-only PRE-OP; PDO mapping skipped
```

不应看到 Position PDO assignment 或 PDO loop。

### 9.2 Terminal 2 检查后停止

保持 Profile launch，在 `Container-T2` 执行：

```bash
ros2 control list_controllers
ros2 topic echo --once /joint_states
```

检查完成后回到 `Container-T1` 按 `Ctrl-C`。不要让 Profile 与后面的 CSP/DC 测试同时运行。

---

## 10. 可选测试：DC-Sync

只有 SM-Sync 的 Homing、10 秒保持和几毫米轨迹全部稳定后才测试 DC-Sync。Task 1 要求 CSP+PDO，不强制 DC-Sync。

开始条件：所有旧 launch 已停止。

### 10.1 Terminal 1 启动 DC-Sync CSP

在 `Container-T1`：

```bash
ros2 launch rascl_description ros2_control.launch.py \
  interface:=enx94bdbe9565bc \
  use_fake_hardware:=false \
  enable_dc_sync:=true \
  pdo_cycle_ns:=20000000
```

### 10.2 Terminal 2 只做保持测试

先不要执行轨迹。在 `Container-T2`：

```bash
ros2 control list_controllers
ros2 topic hz /joint_states
```

观察 10 秒后在 `Container-T2` 按 Ctrl-C。若 `Container-T1` 出现 Sync0 monitoring、周期抖动或 SAFE-OP 错误，立即在 `Container-T1` 按 Ctrl-C 停止，并恢复默认 SM-Sync。

Python 用户态调度不是硬实时环境，因此 DC-Sync 不稳定时不要勉强继续。

---

## 11. 故障排查：按发生阶段操作

### 11.1 rosws.sh 无法启动 Docker

操作位置：Ubuntu 主机 Terminal，不是容器。

```bash
docker version
docker ps -a --filter name=ros2-irs-rascl-wp22
```

处理顺序：

1. 启动 Ubuntu Docker service：

```bash
sudo systemctl start docker
```

2. 确认当前用户有 Docker 权限；
3. 若存在已经停止但未删除的同名容器：

```bash
docker rm ros2-irs-rascl-wp22
```

4. 回到仓库目录重新运行：

```bash
bash ./rosws.sh
```

不要删除其他项目的容器。

### 11.2 修改 Dockerfile 后容器仍使用旧依赖

先在所有容器 Terminal 停止 launch。`Container-T2` 输入 `exit`，最后在 `Container-T1` 输入 `exit`。回到 Ubuntu 主机 Terminal 后：

```bash
cd ~/RASCL_G8  # 实际目录不同时替换这一行
SOFT_REBUILD=true bash ./rosws.sh
```

如果仍怀疑 cache：

```bash
REBUILD=true bash ./rosws.sh
```

### 11.3 `executable not found` 或 `Permission denied`

先停止所有 launch。在 `Container-T1` 执行：

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
rm -rf build/rascl_hardware_interface install/rascl_hardware_interface

colcon build --symlink-install \
  --packages-select rascl_hardware_interface

source install/local_setup.bash
ls -l build/rascl_hardware_interface/scripts/rascl_faulhaber_bridge.py
ls -l install/rascl_hardware_interface/lib/rascl_hardware_interface/rascl_faulhaber_bridge.py
```

当前 CMake 会生成 `0755` 的 build 副本。两个路径都应有 `x` 权限。

### 11.4 旧 bridge 或端口 15001 没有退出

优先回到启动 launch 的 `Container-T1` 按 Ctrl-C。只有该窗口已经丢失时，才在仍可用的容器 Terminal 执行：

```bash
ps -ef | grep rascl_faulhaber_bridge | grep -v grep
ss -ltnp | grep 15001
pkill -TERM -f rascl_faulhaber_bridge.py
sleep 2
ps -ef | grep rascl_faulhaber_bridge | grep -v grep
```

不要一开始就使用 `pkill -9`。如果进程仍不能结束，先让机器人安全断使能，再考虑强制结束。

### 11.5 `pysoem.WkcError` 出现在 `0x1600:00`

这表示运行了旧版本。当前代码不写 `0x1600:00`。

先在 `Container-T1` 停止 launch，然后执行：

```bash
cd /root/ws
grep -R -n "PDO_RX_MAPPING\|sdo_write.*0x1600" \
  src/rascl_hardware_interface \
  install/rascl_hardware_interface 2>/dev/null
```

若 install 中存在旧代码，清理重编：

```bash
rm -rf build/rascl_hardware_interface install/rascl_hardware_interface
colcon build --symlink-install --packages-select rascl_hardware_interface
source install/local_setup.bash
```

然后从第 6 节重新 Homing，不要直接恢复轨迹。

### 11.6 Factory Position PDO mapping 不一致

该错误发生在 `Container-T1` 启动 CSP 时。不要继续在 `Container-T2` 发布命令。

记录错误中的 Rx/Tx entries、驱动 firmware 和 revision。代码期望：

```text
RxPDO2 0x1601 = 0x6040:16 + 0x607A:32
TxPDO2 0x1A01 = 0x6041:16 + 0x6064:32
```

不要强行覆盖只读 mapping count；与 FAULHABER ESI/手册或老师确认实际固件。

### 11.7 EtherCAT 无法进入 OP

错误显示在 `Container-T1`。此时 `Container-T2` 不要发送运动目标。

在 `Container-T1` 按 Ctrl-C。然后新开一个 Ubuntu 主机 Terminal（不要进入 Docker）检查：

```bash
ip link show enx94bdbe9565bc
```

然后重新检查：

1. 是否找到四个 slave；
2. 四个 Position PDO assignment 是否都成功；
3. 是否先进入 SAFE-OP；
4. 初始 process-data working counter 是否大于 0；
5. 是否有旧 bridge 占用网卡或端口。

不要反复激活电机。修正原因后从第 6 节 Homing 重新开始。

### 11.8 `CSP/PDO loop stopped`

该错误显示在 `Container-T1`。立即停止轨迹；机械运动异常时先急停。

常见原因：

1. working counter 为 0 或不是 expected WKC；
2. Drive 离开 Operation Enabled；
3. Statusword bit 13 following error；
4. PDO 长度不是 6 bytes；
5. EtherCAT 周期/同步错误。

在 `Container-T1` 按 Ctrl-C 停止 CSP。不要在同一激活状态下直接重发轨迹。检查机械阻挡、目标步长和跟随误差后，从 Homing/保持测试重新开始。

### 11.9 Mode display 不是 8

错误发生在 CSP 激活阶段，bridge 不会启动轨迹。停止 `Container-T1` 的 CSP launch，检查是否有其他软件修改 `0x6060`，以及驱动是否支持 CSP。不要通过禁用 mode 检查来绕过错误。

### 11.10 Homing 被拒绝

如果 `Container-T2` 返回：

```text
Cannot home while CSP/PDO is active
```

说明 `Container-T1` 仍运行 CSP。正确操作：

1. 回 `Container-T1` 按 Ctrl-C；
2. 等待回到 shell；
3. 检查端口 15001 已释放；
4. 从第 6.1 节启动专用 Homing bridge。

不要绕过该保护。

### 11.11 `/joint_states` 没有数据

保持 `Container-T1` 当前 launch 运行，在 `Container-T2` 执行：

```bash
ros2 control list_controllers
ros2 node list
ros2 topic list | grep joint_states
```

如果 controller inactive，先看 `Container-T1` 的启动错误。如果 bridge/PDO 已报错，停止 launch；不要只增加 `joint_state_timeout_s`。

### 11.12 IK failed

该错误出现在 `Container-T2` 的 `wp3_tsk1`。它不会发布轨迹。保持 `Container-T1` 不动，改用更接近零位 TCP 的目标，再执行 `execute:=false`：

```text
零位 TCP ~= [0.29756, -0.00177, 0.043001] m
```

不要在 IK 失败后直接改成 `execute:=true`。

---

## 12. 正确结束 Docker

### 12.1 先停止 ROS

1. 如果 `Container-T1` 仍有 launch，先按 Ctrl-C；
2. 等待 shell 提示符回来；
3. 在 `Container-T2` 检查没有 bridge/control node：

```bash
ps -ef | grep -E "ros2_control_node|rascl_faulhaber_bridge|wp3_tsk1" | grep -v grep
```

### 12.2 先退出 Terminal 2

在 `Container-T2`：

```bash
exit
```

这只关闭第二个 `docker exec` shell，主容器仍由 `Container-T1` 保持。

### 12.3 最后退出 Terminal 1

确认机器人已断使能，然后在 `Container-T1`：

```bash
exit
```

主 shell 退出后，容器停止并因 `--rm` 自动删除。仓库、build、install 和 log 都在挂载目录 `/root/ws`，不会随容器删除；Docker 镜像也会保留。

回到 Ubuntu 主机 Terminal 检查：

```bash
docker ps --filter name=ros2-irs-rascl-wp22
```

正常情况没有运行中的同名容器。

---

## 13. 当前阶段成功标准

必须按顺序全部满足：

1. `rosws.sh` 能创建容器，两个 Terminal 能同时进入同一容器；
2. `colcon build` 和 `colcon test` 通过；
3. bridge 安装文件有执行权限；
4. fake hardware 的 controller、IK、CSV 和 50 Hz minimum-jerk 正常；
5. 四个 Drive 分别通过 `home_one`；
6. `home_all` 按 0 -> 1 -> 2 -> 3 成功；
7. Homing bridge 完全停止后才能启动 CSP；
8. 四个 Drive 都进入 mode 8；
9. CSP 激活时 target=actual，机械臂不跳动；
10. Position PDO 以 20 ms 周期持续交换，无 WKC/following error；
11. `/joint_states` 持续更新；
12. 几毫米、6 秒 minimum-jerk 实机轨迹成功；
13. 成功后才逐步扩大运动范围；
14. Task 1 每个 major movement 都使用预计算 minimum-jerk 位置轨迹。

---

## 14. CSP/PDO 技术参数速查

| 项目 | 当前值 |
|---|---|
| Operation mode | CSP = 8 |
| ROS controller rate | 50 Hz |
| PDO cycle | 20 ms / 20000000 ns |
| Default sync | SM-Sync |
| RxPDO | `0x1601`: `0x6040 + 0x607A` |
| TxPDO | `0x1A01`: `0x6041 + 0x6064` |
| Rx/Tx length | 6 bytes each |
| SM assignment | `0x1C12` / `0x1C13` |
| SM2 cycle monitor | `0x1C32:02 = 20000000` |
| TCP | `127.0.0.1:15001` |
| ROS domain | 88 |

代码只分配 FAULHABER 出厂 Position PDO，不写 `0x1600:00` 或 `0x1601:00` 等只读 mapping count。Mode 8 通过 SDO 在 CSP 激活前设置一次；周期 PDO 只传 Controlword/Target Position 和 Statusword/Actual Position。

---

## 15. 文档依据

1. `Docs/RASCL_WP3_SS26_Tasksheet.pdf` 第 2 页：Task 1 要求 CSP；
2. `Docs/RASCL_WP3_Intro.pdf` 第 5 页：MC CSP + position trajectory，不使用顶层 effort interface；
3. `Docs/Codex可能需要的规格说明和功能手册/驱动功能.pdf` 第 117-120 页：CSP mode 8、周期更新 `0x607A`、Statusword bit 12/13；
4. FAULHABER Communications Manual EtherCAT 7000.05051：RxPDO2/TxPDO2、SM/DC 周期和 SyncManager assignment；
5. 仓库 `auto_homing` 分支：已验证的 reference input、Homing method、speed、acceleration、timeout 和 `home_done` 行为。
