# RASCL WP3 项目调试接手说明

> 本文件用于在新的机器或新的 Codex 窗口中继续调试。所有文件位置均相对于 Git
> 仓库根目录。开始工作前先检查当前 Git 状态，不要覆盖用户已有修改。

## 1. 当前 Git 基线

```text
branch: main
upstream commit before the Drive 3 direction reversal: 2d5c6d5
commit message: Gripper运动到固定位置标定
remote: https://github.com/Clancy0411/RASCL_G8.git
```

重要历史基线：

```text
7d9b55f5f33b8102b70863c0d4707d7ba6dded58
commit message: 坐标系变化（本轮队友修改前的比较基线）

214477ef7c9f4cca7f52b41106f4863b9f68442b
tag: 已经验证
commit message: 减少抖动
```

`214477e` 曾在实机验证 Drive 0–3 能正确定位到指定空间点，并包含
`0x2332:00=200` 的 CSP 插值修复。当前版本在它的基础上继续加入了 Drive 3 抓夹
收放及自定义 counts 控制、Drive 2 参数与故障诊断，以及当前抓夹中心 TCP。

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
2. 验证当前抓夹中心 TCP，并使用组 `15` 的固定绝对开合位置；
3. 在 Home 和额外姿态中比较模型抓夹中心与外部真实测量；
4. 如仍有系统误差，根据多姿态结果判断来源是固定 TCP offset、base frame、连杆几何还是 encoder
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
direction = [+1,+1,+1,-1]
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
spur_gear_joint = [-2*pi,+2*pi] = [-6.283185307,+6.283185307]
```

## 6. 自动 Homing 与 Drive 3 会话零位

当前策略：

```text
Drive 0–2 自动 Homing
Drive 0–2 全部到位后，Drive 3 从实时位置相对运动 +50000 counts
Drive 3 到位后用 Homing Method 37 将当前位置设为 0 counts
Drive 0–3 全部参与随后 CSP/PDO
```

参数：

```text
skip_spur_gear_homing = true
ignore_spur_gear_in_csp = false
homing_methods = [28,28,24,24]
reference_inputs = [2,2,2,1]
spur_gear_reference_delta_counts = +50000
spur_gear_reference_timeout_s = 30.0
spur_gear_reference_tolerance_counts = 100
spur_gear_reference_profile_velocity = 3000 counts/s
spur_gear_reference_profile_acceleration = 1000
spur_gear_reference_profile_deceleration = 1000
spur_gear_reference_following_error_confirm_s = 0.30
drive 0x607C = [0,0,0,0]
```

这里的 `skip_spur_gear_homing=true` 只表示 Drive 3 不进行传感器寻零；它仍必须执行固定
相对运动和 Method 37 当前位置置零。参考运动必须在 `+50000 counts` 终点误差不超过
`100 counts` 后才允许置零；置零回读必须接近 `0 counts`，否则 `home_all` 失败且 CSP
handoff 被拒绝。参考速度已由 `10000` 降到 `3000 counts/s`，加减速度由 `10000`
降到 `1000`，用于减小 Profile Position 的瞬时跟随滞后。单次 following-error 不再立即
打断流程；只有状态连续保持 `0.30 s` 才判定为持续错误。持续错误或超时会先 Disable
Drive 3，再返回失败；不得通过跳过该失败来强制执行 Method 37。

自动 Home 后名义 joint state：

```text
[shoulder,upperarm,lowerarm,spur]
≈ [0,+pi/2,+pi/2,0]
```

不可破坏的约束：

1. T1 启动唯一 Homing bridge；
2. `home_all` 成功后 T1 不能停止；
3. T2 的 CSP ros2_control 必须复用 T1 bridge；
4. Home 与 CSP 之间不能关闭并重建 EtherCAT master；
5. CSP 运行时不能再次 Homing；
6. 停止 ros2_control 会 Disable Voltage，停机前必须支撑机械臂；
7. Drive 3 不做传感器寻零，但不得绕过 `+50000 counts → Method 37 置零`；
8. `0x607C=0` 只用于 Drive 3 的 Method 37 零位定义，不得用于补偿 URDF/TCP 几何误差。

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

Drive 3 完成会话零位后正常进入 CSP。组 `17` 随时只读当前的绝对 counts；组 `15`
只接受 ASCII 开合动作，并运动到本次 Method 37 零位下的固定绝对位置：

```text
close 或 c = 绝对 -122000 counts
open  或 o = 绝对 +122000 counts
```

组 `15` 会先把实时 `spur_gear_joint` 角度换算成本次 Method 37 坐标中的当前 counts，再
生成到固定绝对目标的 50 Hz minimum-jerk CSP 轨迹。因此重复执行 `close` 或 `open` 不会
继续累计行程；直接输入自定义 counts 已禁用。Drive 3 的 URDF、ros2_control、运动学和
脚本预检限位仍统一为 `[-2*pi,+2*pi]`；这不会修改驱动器对象 `0x607B/0x607D`。默认速度为：

```text
10000 counts/s
```

运动时间按当前绝对位置与目标的差值自动计算：从零位到任一目标约 `12.2 s`，从一个开合
端点到另一个约 `24.4 s`。组 `15` 同时保持 Drive 0–2 当前状态，可以与 Cartesian 轨迹
在同一 CSP 会话中交替使用，但不能在 `wp3_tsk1` 正在发布时并发执行。固定位置控制不是
力控制，也不检测物体接触；稳定后绝对位置误差必须不超过默认 `500 counts`，否则返回失败。
失败前脚本会把目标收回到实测位置，避免继续保持不可达目标。`-122000` 仅适用于本次标定
的夹持条件，机械受阻时不得继续执行。
预检查和实际运动节点取得完整 `/joint_states` 的默认超时均为 5 秒；运动节点异常会以
`SPUR_TRACE failed` 写入 ROS 日志，便于组 `12` 打包分析。

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

Drive 2/3 原峰值电流分别为 `0x2329:03=220/81 mA`，导致只读有效最大转矩
`0x6072≈200/150`。当前代码只在 Homing 成功、进入 CSP 前将：

```text
Drive 2 0x2329:03: 220 → 1100 mA
Drive 3 0x2329:03:  81 →  540 mA
```

并要求回读：

```text
0x6072 >= 1000
0x60E0 = 1000
0x60E1 = 1000
```

这些修改仅用于当前上电会话，不保存到永久存储。Drive 0、1 的电机电流参数不改。

当前用户认为 Drive 2 停滞问题已经解决，所以不要继续主动调参。若再次复现，使用现有
`CSP_STALL_SNAPSHOT` 和打包日志分析，不要先增大阈值。

## 10. TCP 标定历史与当前抓夹中心

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

最初新增与 gripper 开合无关的固定 frame：

```text
joint = tcp_fixed_joint
parent = lowerarm
child = tcp_link
xyz = [0.11616,0.043,0.0179] m
rpy = [0,-pi/2,0]
```

随后根据程序 `XYZ=[0.16,-0.16,0.05] m`、外部实测
`YXZ=[0.14,-0.16,0.05] m` 做了第二次单姿态标定。按项目实体轴约定，对应
`base_link` 数值误差为 `[-0.020,0,0] m`；转换到 `lowerarm` 后，齿轮表面参考点为：

```text
xyz = [0.11478978,0.02881369,0.03193108] m
```

随后按用户定义，把 TCP 沿夹爪伸出方向（`lowerarm +X`）向外移动 `0.020 m`，
作为两爪长度中间的悬空抓取中心。之后追加的局部 X `0.020 m` 已根据实机测试撤销。
当前固定 TCP 为：

```text
xyz = [0.13478978,0.02881369,0.03193108] m
```

2026-07-23 又加入了项目级实体 X 补偿。用户确认外部读数顺序为 `Y/X/Z`，因此
实体 `+X` 对应数值 `base_link +Y`。为消除“实际位置在实体 X 负方向少 `0.020 m`”
的固定误差，`base_link -> shoulder_joint` 的模型平移设为：

```text
xyz = [0,-0.020,0.057441] m
```

该负号属于模型坐标修正：用户目标保持不变，IK 会把实机抓夹沿实体 `+X` 多移动
`0.020 m`。这不是 `lowerarm` 局部 TCP 延伸，因此不会随姿态转成主要的 Z 偏移。

FK、IK、轨迹终点检查、TF 查询和调试组 `13` 全部改用 `tcp_link`。没有移动
`spur_gear_joint` 的实体 URDF 原点，也没有修改 Drive 3 控制。

理想关节角下：

```text
q=[0,0,0]
TCP=[0.29318978,-0.03580108,0.07181469] m

q=[0,+pi/2,+pi/2]
TCP=[0.20318978,-0.03580108,0.32181469] m
```

当前抓夹中心修改已通过：

- URDF XML 与 fixed frame 检查；
- URDF 与 Python 的全局基座标定一致性检查；
- FK 零位和 nominal Home 检查；
- IK 对 nominal Home 的回算；
- Python `py_compile`；
- `bash -n rascl_debug.sh`；
- `git diff --check`。

当前代码中的 URDF、TF、FK、IK 与回归测试在数值上保持一致。单元测试只能证明软件内部
一致，不能证明 `20 mm` 的物理方向和长度一定等于实机目标点；实机调整后仍应在 Home
和至少一个额外姿态比较模型 `tcp_link` 与外部测得的目标点。

## 11. 为什么必须做多姿态验证

当前表面标定和抓夹中心延伸仍主要来自单姿态测量与实体尺寸。一个姿态不能区分：

- 固定 TCP 安装偏移；
- `base_link` 原点/方向误差；
- 连杆长度或 joint origin 误差；
- encoder/home offset 误差。

首先在实际 Home joint state 检查组 `13` 的抓夹中心是否接近当前名义值：

```text
(0.20318978,-0.03580108,0.32181469) m
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
4  启动实机 Homing bridge
5  逐轴 Homing Drive 0、1、2；最后自动执行 Drive 3 参考运动和置零
6  home_all：Drive 0–2 Homing，再执行 Drive 3 +50000 counts 和 Method 37 置零
7  启动实机 CSP ros2_control，Drive 3 参与
8  Controller/joint state 保持检查
9  只规划实机 minimum-jerk
10 执行实机 minimum-jerk
11 检查残留进程和 TCP 端口
12 打包完整 ROS 日志
13 查询 base_link -> tcp_link
14 设置下一目标 XYZ 与运动时间
15 CSP 下输入 close/open，到达绝对 -122000/+122000 counts
16 读取最近 CSP_STALL_SNAPSHOT
17 读取 Drive 3 当前绝对 counts（本次 Method 37 零位）
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
Homing completed for required drives; CSP handoff armed: ... drive3_reference(...delta=50000,...zero=0,method=37)
Drive 3: absolute_counts=0, ... reference_complete=true
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
2. Drive 3 保持“不做传感器寻零、完成 +50000 counts 与 Method 37 置零后参与 CSP”；
3. Homing→CSP 必须复用同一 EtherCAT master；
4. 保留 `0x2332:00=200`；
5. 不把 `0x6060/0x6061` 重新加入周期 PDO；
6. 不让 Drive 3 抓夹收放与 Cartesian trajectory 并发冲突；
7. 不无依据改变四轴 direction；
8. 除 Drive 3 Method 37 的零偏移外，不使用 `0x607C` 修正 TCP 几何；
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
7. 判断当前抓夹中心 TCP 是否正确；
8. 验证额外姿态，决定是否需要进一步运动学标定。

当前最重要的状态：

```text
本轮 Drive 3 方向反转前的上游基线是 commit 2d5c6d5，TCP 已改为固定抓夹中心。
组 15 只支持 close/open，目标为 Method 37 零位下的固定绝对 counts。
组 17 返回以本次 Drive 3 Method 37 零位为基准的绝对 counts。
Drive 2 停滞目前视为已经解决，除非复现，否则不再主动调整。
```
