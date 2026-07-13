# WP3 Task 1 Minimum-Jerk + CSP/PDO 调试指南（Group 8）

本文档对应当前仓库代码，用于按安全顺序验证：

```text
base_link 目标坐标 x/y/z
-> IK
-> joint-space minimum-jerk 离线采样
-> /rascl_position_controller/commands
-> ros2_control position interface
-> FAULHABER CSP mode = 8
-> EtherCAT cyclic Position PDO
```

Task sheet 和 WP3 Intro 的关键要求是：运动控制器必须使用 CSP，并由位置轨迹驱动；顶层不实现 effort interface。当前实现仍然只向 ROS position interface 发送关节位置。

---

## 1. 当前功能范围

已实现：

1. `wp3_tsk1` 接受 `base_link` 下的 `target_x/y/z`。
2. 根据 `/joint_states` 计算 IK。
3. 生成并保存 joint-space minimum-jerk CSV。
4. 以 50 Hz 默认采样率发送位置轨迹。
5. fake hardware / RViz 验证。
6. 四轴 reference-switch 自动 Homing，保留 `home_one` 与 `home_all`。
7. FAULHABER CSP（`0x6060 = 8`）。
8. RxPDO2/TxPDO2 周期过程数据交换。
9. 进入 CSP 前用 actual position 初始化 target，防止激活跳变。
10. CSP 状态、mode display、following error 和 PDO working counter 检查。
11. Profile Position 回归模式仍可显式启用，但不属于 WP3 最终执行模式。

尚未实现：

1. 完整抓取/堆叠 sequence。
2. gripper 开合动作编排。
3. 任意末端姿态约束。
4. Task 2 在线规划。

---

## 2. CSP/PDO 实现说明

当前真实硬件链路为：

```text
wp3_tsk1 minimum-jerk samples
  -> ForwardCommandController (position)
  -> C++ hardware_interface 更新目标 count 缓存
  -> Python bridge 固定 20 ms PDO 线程
  -> RxPDO2: 0x6040 Controlword + 0x607A Target Position
  <- TxPDO2: 0x6041 Statusword + 0x6064 Position Actual Value
```

使用 FAULHABER 出厂 Position PDO：

| 方向 | PDO | Mapping object | 内容 | 长度 |
|---|---|---:|---|---:|
| Master -> Drive | RxPDO2 | `0x1601` | `0x6040:16 + 0x607A:32` | 6 bytes |
| Drive -> Master | TxPDO2 | `0x1A01` | `0x6041:16 + 0x6064:32` | 6 bytes |

代码只把 `0x1601` / `0x1A01` 分配到 SyncManager `0x1C12` / `0x1C13`，不再写 `0x1600:00`。FAULHABER EtherCAT 手册中 `0x1600:00`、`0x1601:00` 等 mapping count 是只读项；旧实现写这些对象会产生 `pysoem.WkcError`。

默认同步方式：

```text
SM-Sync
pdo_cycle_ns = 20000000 ns
cycle = 20 ms = 50 Hz
0x1C32:02 = 20000000  # PRE-OP 写入并读回校验
```

SM-Sync 也是 EtherCAT 支持的同步过程数据方式，满足 CSP 周期位置更新。bridge 会在 `config_map()` 之前配置驱动端的 SM2 到达时间监控，使其与主站实际周期一致。DC-Sync 的周期由 ESC/SYNC0 配置，不使用该对象；可在 SM-Sync 实机稳定后单独测试，不应作为第一步。

重要：PDO 周期由 bridge 的独立线程维持。`GET_ALL` 只读取缓存，`CSP_SETPOINT_ALL` 只更新目标缓存，不会额外发送 EtherCAT frame，因此 ros2_control 的 read/write 不会把一个周期拆成两个不规则 PDO 周期。

---

## 3. 坐标系和 TCP

```text
frame_id = base_link
unit = meter
TCP = spur_gear_joint 原点
```

URDF 零位姿态下：

```text
TCP in base_link ~= [0.29756, -0.00177, 0.043001] m
```

输入示例：

```text
target_x = 0.25
target_y = 0.00
target_z = 0.08
```

方向以 RViz 中 `base_link` 坐标轴为准：Red=X，Green=Y，Blue=Z。

---

## 4. 编译

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash

rm -rf build install log
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF

source install/local_setup.bash
export ROS_DOMAIN_ID=88
```

检查包：

```bash
ros2 pkg list | grep rascl
```

应至少包含：

```text
rascl_description
rascl_hardware_interface
rascl_wp3_ss26_group8
```

---

## 5. 软件测试

### 5.1 bridge PDO 单元测试

该测试不连接 EtherCAT，也不会使能电机：

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash

# 第 4 节使用 BUILD_TESTING=OFF 时不会生成测试目标；测试前单独重编该包。
colcon build --symlink-install \
  --packages-select rascl_hardware_interface \
  --cmake-args -DBUILD_TESTING=ON
source install/local_setup.bash

colcon test --packages-select rascl_hardware_interface \
  --ctest-args --output-on-failure
colcon test-result --verbose
```

它检查：

1. CSP Position PDO payload 为 6 bytes、小端序。
2. 只写 SyncManager assignment。
3. 不写只读的 Position PDO mapping count。
4. 非法同步周期被拒绝。
5. SM-Sync 周期写入 `0x1C32:02` 并正确读回。
6. C++ fake hardware lifecycle 和 interface 声明。

### 5.2 fake hardware

Terminal 1：

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=88

ros2 launch rascl_description ros2_control.launch.py \
  use_fake_hardware:=true
```

fake hardware 不连接 FAULHABER，因此它验证 ROS position interface、IK、轨迹和 controller，不证明真实 PDO 通信。

Terminal 2，只规划：

```bash
ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
  -p target_x:=0.25 \
  -p target_y:=0.00 \
  -p target_z:=0.08 \
  -p duration:=4.0 \
  -p rate_hz:=50.0 \
  -p execute:=false
```

检查：

```bash
head /tmp/rascl_wp3_tsk1_last_trajectory.csv
```

确认 IK 和 CSV 后执行：

```bash
ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
  -p target_x:=0.25 \
  -p target_y:=0.00 \
  -p target_z:=0.08 \
  -p duration:=4.0 \
  -p rate_hz:=50.0 \
  -p execute:=true
```

组合启动：

```bash
ros2 launch rascl_wp3_ss26_group8 wp3_tsk1.launch.py \
  start_robot:=true \
  use_fake_hardware:=true \
  target_x:=0.25 \
  target_y:=0.00 \
  target_z:=0.08 \
  duration:=4.0 \
  rate_hz:=50.0 \
  execute:=true
```

---

## 6. 实机 Homing：必须先于 CSP

Homing 使用 mode 6 和 SDO controlword；CSP 线程使用 mode 8 和 PDO controlword。两者不能同时控制驱动器。

因此当前安全流程是：

```text
homing.launch.py（只启动 bridge，不启动 ros2_control）
-> home_one / home_all
-> Ctrl-C 停止 homing launch
-> ros2_control.launch.py（CSP/PDO）
```

不要在 CSP ros2_control 已启动后调用 `home_all`。bridge 会拒绝该操作。

### 6.1 启动专用 Homing bridge

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=88

ros2 launch rascl_description homing.launch.py \
  interface:=enx3c18a0264863
```

按实际 `ip link` 结果替换网卡名。

当前参数：

```text
homing_methods       = [28, 28, 24, 24]
reference_inputs     = [2, 2, 2, 1]
homing_offsets       = [0, 0, 0, 0]
homing_search_speeds = [1000, 1000, 1000, 1000]
homing_zero_speeds   = [200, 200, 200, 200]
homing_accelerations = [1000, 1000, 1000, 1000]
motion_timeout_s     = 8.0
```

这两组值与 `auto_homing` 分支中实际启动文件覆盖后的、已实机验证的参数一致。专用 Homing bridge 保持 SDO-only PRE-OP，不调用 `config_map()`，以免改变已经验证的归零通信条件。

Reference 输入：

| Drive | Joint | Reference input |
|---:|---|---:|
| 0 | `shoulder_joint` | DigIn2 |
| 1 | `upperarm_joint` | DigIn2 |
| 2 | `lowerarm_joint` | DigIn2 |
| 3 | `spur_gear_joint` | DigIn1 |

### 6.2 检查数字输入

```bash
ros2 service call /rascl_faulhaber_bridge/read_digital_inputs \
  std_srvs/srv/Trigger "{}"
```

### 6.3 单轴 Homing

每个轴第一次都必须单独验证：

```bash
ros2 param set /rascl_faulhaber_bridge test_drive_index 0
ros2 service call /rascl_faulhaber_bridge/home_one \
  std_srvs/srv/Trigger "{}"
```

依次测试 `0, 1, 2, 3`。成功示例：

```text
success=True
message='Drive 0 homing completed; actual_position=...'
```

若出现 fault、Homing Error、方向错误或持续找不到开关，立即急停/断使能，不要直接继续 `home_all`。

### 6.4 四轴 Homing

四轴都通过 `home_one` 后：

```bash
ros2 service call /rascl_faulhaber_bridge/home_all \
  std_srvs/srv/Trigger "{}"
```

`home_all` 按 Drive 0 -> 1 -> 2 -> 3 顺序执行。

完成后按 `Ctrl-C` 停止 `homing.launch.py`；bridge 会依次执行 Disable Operation / Disable Voltage。确认 bridge 已退出，再进入 CSP 启动。

---

## 7. 实机 CSP/PDO 启动

Terminal 1：

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=88

ros2 launch rascl_description ros2_control.launch.py \
  interface:=enx3c18a0264863 \
  use_fake_hardware:=false
```

默认参数已经是：

```text
control_mode = csp
controller_config = controllers_csp.yaml
update_rate = 50 Hz
pdo_cycle_ns = 20000000
enable_dc_sync = false  # SM-Sync
```

期望日志顺序包含：

```text
assigning factory Position PDOs Rx=0x1601, Tx=0x1A01
SM2 cycle monitoring configured for 20000000 ns
Process image mapped
SM-Sync selected with cycle 20000000 ns
Master reached OP state
Activated real RASCL hardware in csp mode
```

不应再出现：

```text
Configuring CSP PDO mapping ...
sdo_write(0x1600, 0, ...)
pysoem.WkcError at PDO_RX_MAPPING subindex 0
```

### 7.1 检查 controller 和 joint state

```bash
ros2 control list_controllers
ros2 topic echo --once /joint_states
```

期望：

```text
joint_state_broadcaster active
rascl_position_controller active
```

Homing 后四轴应接近：

```text
[0.0, 0.0, 0.0, 0.0]
```

### 7.2 CSP 激活保持测试

启动后先不要发布运动目标，观察至少 10 秒：

1. 机械臂不应跳动。
2. `/joint_states` 应持续更新。
3. 不应出现 `CSP/PDO loop stopped`。
4. 不应出现 following error。
5. EtherCAT 不应掉到 SAFE-OP + Error。

只有保持测试通过后才执行轨迹。

---

## 8. 实机 minimum-jerk 轨迹

Terminal 2 先只规划：

```bash
ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
  -p target_x:=0.295 \
  -p target_y:=0.000 \
  -p target_z:=0.048 \
  -p duration:=6.0 \
  -p rate_hz:=50.0 \
  -p execute:=false
```

第一次 CSP 实机测试应只移动几毫米。确认 IK、CSV 和方向后：

```bash
ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
  -p target_x:=0.295 \
  -p target_y:=0.000 \
  -p target_z:=0.048 \
  -p duration:=6.0 \
  -p rate_hz:=50.0 \
  -p execute:=true
```

逐步扩大范围，不要第一次就使用短 duration 或大位移。

---

## 9. DC-Sync 可选测试

只有默认 SM-Sync 稳定后才测试：

```bash
ros2 launch rascl_description ros2_control.launch.py \
  interface:=enx3c18a0264863 \
  use_fake_hardware:=false \
  enable_dc_sync:=true \
  pdo_cycle_ns:=20000000
```

Python 用户态调度不是硬实时环境。若出现 Sync0 monitoring、周期抖动或 SAFE-OP 错误，恢复默认 SM-Sync；Task 1 的核心要求是 CSP + 周期 Position PDO，不要求强制 DC-Sync。

---

## 10. Profile Position 回归模式

仅用于确认旧链路或排查硬件，不用于最终 WP3 CSP 验收：

```bash
ros2 launch rascl_description ros2_control.launch.py \
  interface:=enx3c18a0264863 \
  use_fake_hardware:=false \
  control_mode:=profile \
  controller_config:=controllers.yaml
```

Profile 模式不会配置 CSP PDO assignment，也不会运行周期 PDO 线程。

---

## 11. 常见故障

### 11.1 `pysoem.WkcError` 出现在 `0x1600:00`

说明运行的是旧安装文件。当前代码不写该对象。

```bash
grep -R -n "PDO_RX_MAPPING\|0x1600" \
  src/rascl_hardware_interface install/rascl_hardware_interface 2>/dev/null
```

清理并重新编译、重新 source。

### 11.2 Factory Position PDO mapping 不一致

错误会列出读取到的 Rx/Tx entries。不要猜测 payload 布局，也不要强行覆盖只读 mapping count。记录 drive firmware/revision 和实际对象值，再与老师或 FAULHABER ESI/手册确认。

### 11.3 EtherCAT 无法进入 OP

检查：

1. 四个 slave 是否全部找到。
2. 网卡名是否正确。
3. Position PDO assignment 是否成功。
4. 是否先到 SAFE-OP。
5. 初始 process-data frame 是否成功，working counter 是否大于 0。

### 11.4 `CSP/PDO loop stopped`

bridge 会把原因写在同一行。常见原因：

1. PDO working counter 为 0。
2. Drive 离开 Operation Enabled。
3. Statusword bit 13 following error。
4. PDO 长度不是 6 bytes。
5. EtherCAT 同步/周期错误。

发生后停止轨迹、急停或断使能，检查机械阻挡、目标步长和跟随误差参数。

### 11.5 Mode display 不是 8

当前 bridge 在 CSP 激活时读取 `0x6061`。若不是 8，CSP 激活失败，不会开始轨迹。检查 drive 是否支持 CSP、是否有其他接口同时修改模式。

### 11.6 Homing 被拒绝

如果返回：

```text
Cannot home while CSP/PDO is active
```

停止 `ros2_control.launch.py`，使用 `homing.launch.py`，不要绕过保护。

### 11.7 `/joint_states` 中断

检查 bridge 是否仍在运行、controller 是否 active、PDO loop 是否报错。不要只提高 `joint_state_timeout_s` 掩盖 EtherCAT 故障。

### 11.8 IK failed

目标可能不可达、接近奇异位形或超出关节限位。先使用接近零位 TCP 的目标。

### 11.9 bridge `executable not found` / `Permission denied`

`auto_homing` 分支中的脚本 Git mode 是 `100755`，main 曾退回 `100644`。当前 CMake 会先在 build 目录生成权限为 `0755` 的副本，再安装该副本，因此也兼容 `--symlink-install`。重新 build 后检查：

```bash
ls -l /root/ws/build/rascl_hardware_interface/scripts/rascl_faulhaber_bridge.py
ls -l /root/ws/install/rascl_hardware_interface/lib/rascl_hardware_interface/rascl_faulhaber_bridge.py
```

两者都应具有执行权限；如果 install 中仍指向旧文件，清理该包的 build/install 后重新编译。

---

## 12. 清理旧进程

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

重新启动前确认没有第二个 bridge 占用 EtherCAT 网卡或 TCP 端口 15001。

---

## 13. 当前阶段成功标准

1. `colcon build` 和软件测试通过。
2. fake hardware 下 IK、CSV、50 Hz minimum-jerk 执行正常。
3. 四个轴分别通过 `home_one`，再通过 `home_all`。
4. Homing launch 停止后，CSP launch 成功进入 OP。
5. 四个 drive mode display 均为 8。
6. 启动后 target=actual，不发生跳动。
7. Position PDO 以固定 20 ms 周期持续交换。
8. `/joint_states` 持续更新，无 following error / WKC error。
9. 实机几毫米 minimum-jerk 轨迹成功，再逐步扩大范围。
10. 最终 Task 1 的每个 major movement 都使用预计算 minimum-jerk 位置轨迹。

---

## 14. 本实现依据

仓库内文档：

1. `Docs/RASCL_WP3_SS26_Tasksheet.pdf` 第 2 页：Task 1 要求 CSP。
2. `Docs/RASCL_WP3_Intro.pdf` 第 5 页：MC CSP + position trajectory，不使用顶层 effort interface。
3. `Docs/Codex可能需要的规格说明和功能手册/驱动功能.pdf` 第 117-120 页：CSP mode 8、周期写 `0x607A`、状态字 bit 12/13。
4. FAULHABER Communications Manual EtherCAT（7000.05051）：RxPDO2/TxPDO2、SM/DC 同步周期、SyncManager assignment。
