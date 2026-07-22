# RASCL WP3 项目调试接手说明

> 本文件用于在新的机器或新的 Codex 窗口中继续调试。所有文件位置均相对于 Git
> 仓库根目录。开始工作前先检查当前 Git 状态，不要覆盖用户已有修改。

## 1. 当前 Git 基线

```text
branch: main
base commit: 7d9b55f5f33b8102b70863c0d4707d7ba6dded58
commit message: 坐标系变化
remote: https://github.com/Clancy0411/RASCL_G8.git
```

重要历史基线：

```text
214477ef7c9f4cca7f52b41106f4863b9f68442b
tag: 已经验证
commit message: 减少抖动
```

`214477e` 曾在实机验证 Drive 0–3 能正确定位到指定空间点，并包含
`0x2332:00=200` 的 CSP 插值修复。当前版本在它的基础上继续加入了 Drive 3 相对
counts 控制、Drive 2 参数与故障诊断，以及最新 TCP 偏移修正。

进入新环境后首先执行：

```bash
git status --short
git branch --show-current
git rev-parse HEAD
```

不要未经用户允许执行 `git reset --hard`、批量回滚或覆盖工作区。当前工作区可能包含
用户主动删除的旧文件；保留该删除，不要自行恢复。

## 2. 项目目标与当前优先级

这是 RASCL Work Package 3 项目，最终任务是使用机械臂抓取方块并放到指定位置。
官方要求见：

```text
Docs/RASCL_WP3_SS26_Tasksheet.pdf
Docs/RASCL_WP3_Intro.pdf
```

课程要求的底层运动模式是：

```text
Cyclic Synchronous Position (CSP) + EtherCAT PDO
```

当前 CSP/PDO、自动 Homing、Cartesian IK、minimum-jerk 轨迹和 Drive 3 CSP 控制均
已实现。现在的首要目标是：

1. 在新机器上重新编译并启动实机；
2. 验证 commit `7d9b55f` 的最新 TCP 偏移修正；
3. 在 Home 和额外姿态中比较模型 TCP 与外部真实测量；
4. 根据多姿态结果判断剩余误差是固定 TCP offset、base frame、连杆几何还是 encoder
   零位问题。

Drive 2 停滞问题目前视为已经解决，后续先忽略，不要主动继续调整它的转矩、限位、
following-error 或位置环参数。只有同类故障再次真实复现时，才启用现有诊断流程。

## 3. 权威文档与代码位置

### 3.1 当前唯一中文实机指南

```text
WP3_Task1_MinJerk_Debug_Guide_CN.md
```

这是当前唯一权威中文调试指南。以前的 quick 版本已经删除，不要创建第二份重复指南。
凡是修改功能、参数、命令或流程，都要同步更新这一份文件。文字应简洁、准确，并覆盖
所有实际需要执行的命令。

### 3.2 硬件与协议手册

```text
Docs/Codex可能需要的规格说明和功能手册/EtherCat.pdf
Docs/Codex可能需要的规格说明和功能手册/Commution Manual.pdf
Docs/Codex可能需要的规格说明和功能手册/HardwareInformation.pdf
Docs/Codex可能需要的规格说明和功能手册/技术手册.pdf
Docs/Codex可能需要的规格说明和功能手册/驱动功能.pdf
Docs/Codex可能需要的规格说明和功能手册/CANopenHelper_statusword.html
```

涉及 FAULHABER 对象字典、CiA-402、Homing、PDO、转矩、电流、电压和 statusword
时，应优先查阅这些资料。

### 3.3 软件包

机器人描述、URDF、launch 与 controller：

```text
src/rascl_description/urdf/rascl.urdf
src/rascl_description/launch/homing.launch.py
src/rascl_description/launch/ros2_control.launch.py
src/rascl_description/config/controllers_csp.yaml
src/rascl_description/README.md
```

EtherCAT bridge 与 ros2_control hardware interface：

```text
src/rascl_hardware_interface/scripts/rascl_faulhaber_bridge.py
src/rascl_hardware_interface/src/rascl_hardware_interface.cpp
src/rascl_hardware_interface/test/test_faulhaber_bridge.py
src/rascl_hardware_interface/test/test_generic_system.cpp
src/rascl_hardware_interface/README.md
```

IK、trajectory 和 WP3 application：

```text
src/rascl_wp3_ss26_group8/rascl_wp3_ss26_group8/kinematics.py
src/rascl_wp3_ss26_group8/rascl_wp3_ss26_group8/trajectory.py
src/rascl_wp3_ss26_group8/rascl_wp3_ss26_group8/wp3_tsk1.py
src/rascl_wp3_ss26_group8/launch/wp3_tsk1.launch.py
src/rascl_wp3_ss26_group8/docs/coordinate_convention.md
src/rascl_wp3_ss26_group8/README.md
```

辅助文件：

```text
rascl_debug.sh
rosws.sh
Log.md
```

`Log.md` 包含历史过程，旧数值可能已过时；当前代码、最新日期记录和当前中文指南优先。

## 4. 软件架构

```text
rascl_position_controller
        ↓ ROS 2 position command
C++ ros2_control hardware interface
        ↓ local TCP 127.0.0.1:15001
Python pysoem EtherCAT bridge
        ↓ EtherCAT PDO
FAULHABER Drive 0–3
```

Python bridge 独占 EtherCAT master。Homing 后 CSP 必须复用同一 bridge 和同一 master，
不能同时启动第二个 bridge。

WP3 application 目前：

- 对前三个机械臂关节求解 TCP XYZ 的数值 IK；
- 不约束末端 orientation；
- 生成 50 Hz joint-space minimum-jerk trajectory；
- 通过四关节 position controller 发布命令；
- Cartesian 轨迹保持当前 Drive 3 位置；
- 结束时检查 joint feedback 与 TCP error；
- 只有 `MOTION_RESULT reached=true` 才算真正到达。

## 5. Drive、方向和软件零位

```text
Drive 0 = shoulder_joint
Drive 1 = upperarm_joint
Drive 2 = lowerarm_joint
Drive 3 = spur_gear_joint / gripper
```

当前实际采用：

```text
direction = [+1,+1,+1,+1]
home_offset_counts = [0,-802816,-802816,0]
```

换算关系：

```text
q = direction * (raw_counts - home_offset_counts) / counts_per_rad
```

Drive 0–2：

```text
counts_per_revolution = 3211264
gear_ratio = 196
encoder_cpr = 4096
counts_per_rad ≈ 511088.539
```

Drive 3：

```text
counts_per_revolution = 1323008
gear_ratio = 323
encoder_cpr = 4096
```

历史上 `upperarm_joint` 曾被试验性反向，但已经由
`e5ef61d Revert "upper arm modify"` 回滚。当前 upperarm 和 lowerarm 都使用 `+1`。
不要仅凭运动视觉再次翻转方向，应先比较 raw counts、joint state、规划目标和真实运动。

`rascl.urdf` 开头仍可能有一段旧注释提及 Drive 2 opposite sign/positive offset；当前
运行数值 `direction=+1`、`home_offset_counts=-802816` 才是权威值，不要按旧注释改动。

当前 URDF 限位：

```text
shoulder_joint  = [-pi/2,+pi/2]
upperarm_joint  = [-pi,+pi]
lowerarm_joint  = [-pi,+pi]
spur_gear_joint = [-3.1415,+3.1415]
```

## 6. 自动 Homing——必须保持不变

当前策略：

```text
Drive 0–2 自动 Homing
Drive 3 不 Homing
Drive 0–3 全部参与随后 CSP/PDO
```

参数：

```text
skip_spur_gear_homing = true
ignore_spur_gear_in_csp = false
homing_methods = [28,28,24,24]
reference_inputs = [2,2,2,1]
drive 0x607C = [0,0,0,0]
```

自动 Home 后名义 joint state：

```text
[shoulder,upperarm,lowerarm,spur]
≈ [0,+pi/2,+pi/2,Drive 3当前上电位置]
```

不可破坏的约束：

1. T1 启动唯一 Homing bridge；
2. `home_all` 成功后 T1 不能停止；
3. T2 的 CSP ros2_control 必须复用 T1 bridge；
4. Home 与 CSP 之间不能关闭并重建 EtherCAT master；
5. CSP 运行时不能再次 Homing；
6. 停止 ros2_control 会 Disable Voltage，停机前必须支撑机械臂；
7. 不要重新加入 Drive 3 Homing；
8. 不要用驱动器 `0x607C` 补偿 URDF/TCP 几何误差。

## 7. CSP/PDO 当前配置

```text
CSP mode = 8
PDO cycle = 20 ms / 50 Hz
PDO timeout = 5000 us
DC sync = false
```

PDO mapping：

```text
RxPDO2 = 0x6040 Controlword + 0x607A Target position, 6 bytes
TxPDO2 = 0x6041 Statusword + 0x6064 Actual position, 6 bytes
```

Drive 0–3 进入 CSP 前必须写入并回读：

```text
0x2332:00 = 200
```

因为 `20 ms / 100 us = 200`。该设置解决了每个 20 ms 目标被驱动器当作约 100 us
突变而造成的抖动，不能删除或退回默认值 `1`。

PDO 由 bridge 内部独立循环持续发送，不能只依赖 ROS read/write 或偶发 TCP 请求。

## 8. Drive 3 / gripper

Drive 3 不 Homing，但正常进入 CSP。调试脚本组 `15` 接受有符号相对 counts：

```text
+2000 = 从当前位置正向增加 2000 counts
-2000 = 从当前位置反向减少 2000 counts
```

它不是绝对 encoder 目标。默认速度为：

```text
10000 counts/s
```

组 `15` 使用 50 Hz minimum-jerk 轨迹，同时保持 Drive 0–2 当前状态。它可以与
Cartesian 轨迹在同一 CSP 会话中交替使用，但不能在 `wp3_tsk1` 正在发布时并发执行。

## 9. Drive 2 当前参数

Drive 2 曾出现 `statusword=0x3027` following error。当前 session-only 设置：

```text
0x6065 = 25000 counts
0x6066 = 250 ms
```

这不是关闭 following-error 保护。

当前代码只读取、不改写：

```text
0x607B position range
0x607D software position limit
```

最近一次实机回读曾为：

```text
0x607B = [-2147483648,2147483647]
0x607D = [-802816,802816]
```

CSP 交接时 Drive 0–3：

```text
0x60E0 = 1000
0x60E1 = 1000
```

Drive 2 原峰值电流 `0x2329:03=220 mA` 导致只读有效最大转矩 `0x6072≈200`。
当前代码只在 Homing 成功、进入 CSP 前将 Drive 2：

```text
0x2329:03: 220 → 1100 mA
```

并要求回读：

```text
0x6072 >= 1000
0x60E0 = 1000
0x60E1 = 1000
```

这些修改仅用于当前上电会话，不保存到永久存储。Drive 0、1、3 的电机电流参数不改。

当前用户认为 Drive 2 停滞问题已经解决，所以不要继续主动调参。若再次复现，使用现有
`CSP_STALL_SNAPSHOT` 和打包日志分析，不要先增大阈值。

## 10. 最新 TCP 修正

commit `7d9b55f` 之前，在同一个自动 Home 姿态：

```text
模型认为的 TCP = (20.8,0,29.2) cm = (0.208,0,0.292) m
外部实际测量   = (18.5,0,33.5) cm = (0.185,0,0.335) m
```

单姿态修正量：

```text
actual - model = (-0.023,0,+0.043) m
```

旧 TCP 是 `spur_gear_joint` origin：

```text
lowerarm local xyz = [0.13916,0,0.0179] m
```

现在新增与 gripper 开合无关的固定 frame：

```text
joint = tcp_fixed_joint
parent = lowerarm
child = tcp_link
xyz = [0.11616,0.043,0.0179] m
rpy = [0,-pi/2,0]
```

FK、IK、轨迹终点检查、TF 查询和调试组 `13` 全部改用 `tcp_link`。没有移动
`spur_gear_joint` 的实体 URDF 原点，也没有修改 Drive 3 控制。

理想关节角下：

```text
q=[0,0,0]
TCP=[0.27456,-0.00177,0.086001] m

q=[0,+pi/2,+pi/2]
TCP=[0.18456,-0.00177,0.336001] m
```

最新 TCP 修改已通过：

- URDF XML 与 fixed frame 检查；
- FK 零位和 nominal Home 检查；
- IK 对 nominal Home 的回算；
- Python `py_compile`；
- `bash -n rascl_debug.sh`；
- `git diff --check`。

但它尚未完成修改后的实机重新测量，这是当前第一调试目标。

## 11. 为什么必须做多姿态验证

本次只有一个 Home 姿态的外部测量。一个姿态不能区分：

- 固定 TCP 安装偏移；
- `base_link` 原点/方向误差；
- 连杆长度或 joint origin 误差；
- encoder/home offset 误差。

首先在同一实际 Home joint state 检查新 TF 是否从约：

```text
(0.208,0,0.292) m
```

变为接近：

```text
(0.185,0,0.335) m
```

随后至少在另外 2–3 个不过奇异、不碰撞的姿态重复外部测量。若误差随姿态变化，就不能
继续用一个固定 TCP offset 修正，而应标定 base frame、连杆参数或 encoder 零位。

TCP 改变后，同一个旧 XYZ 目标会对应新的关节角。不要未经重新规划和检查，直接复用
以前实机验证过的绝对 XYZ 目标。

## 12. 新机器与 EtherCAT 网卡

ROS 容器内工作区通常为：

```text
/root/ws
```

运行参数：

```text
ROS 2 Jazzy
ROS_DOMAIN_ID=88
TCP bridge=127.0.0.1:15001
```

`rascl_debug.sh` 当前保存的默认 EtherCAT 网卡名来自上一台工作站。换机器后必须使用
新工作站的真实网卡名。网卡应在 Ubuntu 主机确认，不要依赖容器内不存在的 `ip` 命令。

新网卡名确认后，可在每次启动组 `4` 时显式传入：

```bash
RASCL_INTERFACE=<新网卡名> bash ./rascl_debug.sh 4
```

或者经用户确认后统一修改 `rascl_debug.sh` 的默认 `INTERFACE`，并同步更新中文指南。
在用户已经明确告诉网卡名后，不要重复进行没有必要的网卡检查。

## 13. 调试脚本组号

脚本：

```text
rascl_debug.sh
```

使用方式：

```bash
bash ./rascl_debug.sh
bash ./rascl_debug.sh <组号>
```

组号：

```text
1  编译 + 功能测试
2  启动 fake ros2_control
3  Fake 检查 + 规划 + 执行
4  启动实机 Homing bridge，Drive 3 跳过 Home
5  逐轴 Homing Drive 0、1、2
6  home_all，执行 Drive 0–2
7  启动实机 CSP ros2_control，Drive 3 参与
8  Controller/joint state 保持检查
9  只规划实机 minimum-jerk
10 执行实机 minimum-jerk
11 检查残留进程和 TCP 端口
12 打包完整 ROS 日志
13 查询 base_link -> tcp_link
14 设置下一目标 XYZ 与运动时间
15 CSP 下 Drive 3 相对 counts
16 读取最近 CSP_STALL_SNAPSHOT
```

脚本不会自动跨终端操作。组 `4` 和组 `7` 是前台持续进程。

## 14. 新机器第一次运行

先进入项目提供的容器，然后在容器工作区执行。确保所有旧实机进程已停止，再编译：

```bash
cd /root/ws
bash ./rascl_debug.sh 1
```

使用三个容器终端：

### T1：唯一 Homing bridge

```bash
cd /root/ws
RASCL_INTERFACE=<新网卡名> bash ./rascl_debug.sh 4
```

保持运行，不得 `Ctrl-C`。

### T2：Home，然后切换 CSP

```bash
cd /root/ws
bash ./rascl_debug.sh 6
```

必须看到：

```text
success=True
Homing completed for required drives; CSP handoff armed
```

随后仍在 T2：

```bash
bash ./rascl_debug.sh 7
```

保持运行。

关键成功信息包括：

```text
CSP interpolation 0x2332.00 ... 200
CSP directional torque limits verified for this session only
Master reached OP state
Activated real RASCL hardware in csp mode
```

### T3：检查 TCP，不先运动

```bash
cd /root/ws
bash ./rascl_debug.sh 8
bash ./rascl_debug.sh 13
ros2 topic echo --once /joint_states
```

先记录和重新测量 Home TCP，不要立即发送旧运动目标。

需要向新窗口提供：

1. 组 `13` 的完整 `Translation`；
2. 同时刻完整 `/joint_states`；
3. 外部测量的真实 TCP `x/y/z` 和单位；
4. 被测量的实际物理点定义；
5. 实际测量原点和三轴正方向定义；
6. 新工作站使用的 EtherCAT 网卡名。

## 15. TCP 验证后的运动流程

设置目标：

```bash
bash ./rascl_debug.sh 14
```

只规划：

```bash
bash ./rascl_debug.sh 9
```

必须确认 `IK result: success=True` 和“规划已通过”。然后执行：

```bash
bash ./rascl_debug.sh 10
```

必须看到：

```text
MOTION_RESULT reached=true
```

每一个新目标都必须重新执行：

```text
14 → 9 → 10
```

## 16. 故障与日志

Drive 2 问题当前先忽略。只有故障再次复现时才执行：

```bash
bash ./rascl_debug.sh 16
bash ./rascl_debug.sh 12
```

组 `12` 会在共享工作区生成：

```text
ros_logs_YYYYMMDD_HHMMSS.tar.gz
```

直接把 `.tar.gz` 提交给 Codex，不需要逐行复制终端。分析时重点搜索：

```text
CSP_SNAPSHOT
DRIVE_DIAG
TORQUE_SNAPSHOT
CSP_STALL_DETECTED
CSP_STALL_SNAPSHOT
MOTION_RESULT
```

## 17. 后续修改要求

1. 不破坏 Drive 0–2 已验证的自动 Homing；
2. Drive 3 保持“不 Homing、参与 CSP”；
3. Homing→CSP 必须复用同一 EtherCAT master；
4. 保留 `0x2332:00=200`；
5. 不把 `0x6060/0x6061` 重新加入周期 PDO；
6. 不让 Drive 3 相对运动与 Cartesian trajectory 并发冲突；
7. 不无依据改变四轴 direction；
8. 不使用 `0x607C` 修正 TCP 几何；
9. 不直接清除或改写 `0x607B/0x607D`；
10. 驱动测试参数保持 session-only，不执行永久存储；
11. 修改命令、参数或流程时同步更新唯一中文指南；
12. 代码默认值应反映最终确认配置，不依赖隐藏的临时脚本覆盖；
13. 先收集 joint state、raw/PDO 数据和外部实测，再修改运动学；
14. 当前先验证 TCP，不主动继续调试 Drive 2；
15. 保留用户现有 Git 修改，不进行无授权的提交、推送或破坏性回滚。

## 18. 新窗口的第一项工作

新窗口接手后应先：

1. 阅读本文件；
2. 阅读 `WP3_Task1_MinJerk_Debug_Guide_CN.md`；
3. 检查 `git status` 和当前 HEAD；
4. 确认新机器的 EtherCAT 网卡名；
5. 指导完成组 `1 → T1:4 → T2:6 → T2:7 → T3:8 → T3:13`；
6. 获取新 TF、`/joint_states` 和外部真实 TCP；
7. 判断单姿态 TCP offset 是否正确；
8. 验证额外姿态，决定是否需要进一步运动学标定。

当前最重要的状态：

```text
最新 TCP 修正已经进入 commit 7d9b55f，但尚未实机复测。
Drive 2 停滞目前视为已经解决，除非复现，否则不再主动调整。
```
