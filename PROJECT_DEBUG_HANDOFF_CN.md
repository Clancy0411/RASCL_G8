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
收放及自定义 counts 控制、Drive 2 参数与故障诊断，以及当前固定 TCP。

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
2. 验证沿 lowerarm +X 170 mm 的当前理想 TCP，并保留组 `15` 快捷动作及自定义相对 counts；
3. 在 Home 和额外姿态中比较模型 TCP 与同一实体参考点的外部真实测量；
4. 如仍有系统误差，根据多姿态结果判断来源是固定 TCP offset、base frame、连杆几何还是 encoder
   零位问题。

2026-07-24 同类 Drive 2 故障再次复现：`statusword=0x3827`，且
`0x2324.01` 同时报 `following_error`、`positive_limit_switch` 和
`negative_limit_switch`。当前修复聚焦 `0x2310:01/:02` 的旧限位输入映射；不要继续
盲目增大转矩、following-error 或位置环参数。

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
Drive 0–2 原生方法找到第一边沿后，继续穿过参考输入有效区间
记录第二边沿，以低速正弦曲线反向回到 (entry+exit)/2，并以 Method 37 将中点设为 0 counts
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
homing_interval_max_travel_drive0_counts = 100000
homing_interval_max_travel_drive1_counts = 300000
homing_interval_max_travel_drive2_counts = 300000
homing_interval_timeout_s = 120.0
homing_interval_poll_s = 0.01
homing_midpoint_tolerance_counts = 500
第二边沿穿越和回中点速度 = homing_zero_speeds = [200,200,200]
第二边沿穿越和回中点曲线 = 0x6086:00 = 1（正弦加速度，会话内临时设置）
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

1. T1 启动唯一 Homing bridge；Drive 0–2 必须各自返回
   `driveN_interval(entry,exit,width,midpoint,reached,zero,zero_tolerance=500)`，
   并要求 `abs(zero)<=500`；
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
保留快捷动作或任意非零的有符号相对 counts：

```text
close 或 c = 最多相对 -500000 counts，夹住物体后提前停止并保持
open  或 o = 固定相对 +200000 counts，要求完整到位
+2000       = 从当前位置正向增加 2000 counts
-500000     = 从当前位置反向减少 500000 counts
```

组 `15` 的输入仍然不是绝对目标；执行前后用组 `17` 读取以本次 Method 37 零位为基准
的 `absolute_counts`，可据此实验确定开、合位置。只有 `close` 是接触感知快捷动作：
先把 Drive 3 会话内 `0x60E0/0x60E1` 从正常 `1000` 降到行程转矩 `300`，再以
`20000 counts/s` 克服滑槽摩擦。跟踪误差达到 `300 counts`，并且连续 `0.06 s` 内
编码器进度不超过 `50 counts` 时，立即降到保持转矩 `100`，并只预压 `100 counts`。
日志包含 `SPUR_CONTACT`、
`SPUR_RESULT outcome=contact_or_endpoint` 和分步 SDO 采集的
`SPUR_CONTACT_SNAPSHOT`。`-500000 counts`
是最大闭合行程，不保证走满。`open=+200000 counts` 和直接输入的有符号 counts 均要求
精确相对运动、不启用接触提前终止，并在执行前恢复 `0x60E0/0x60E1=1000`。Drive 3
的 URDF、ros2_control、运动学和脚本预检限位已统一为 `[-2*pi,+2*pi]`；这不会修改
驱动器对象 `0x607B/0x607D`。默认速度为：

```text
close: 20000 counts/s
open / 自定义 counts: 20000 counts/s
```

运动时间由 counts 自动计算；`close` 若未提前接触最长约 25 秒，`open` 约 10 秒。组 `15` 使用 50 Hz
minimum-jerk 轨迹，同时保持 Drive 0–2 当前状态。它可以与
Cartesian 轨迹在同一 CSP 会话中交替使用，但不能在 `wp3_tsk1` 正在发布时并发执行。
若 `300‰` 仍不足以克服空载摩擦，只在完整重启时提高
`RASCL_SPUR_CLOSE_TORQUE_LIMIT_PER_MILLE`；保持
`RASCL_SPUR_HOLD_TORQUE_LIMIT_PER_MILLE=100` 不变。
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

2026-07-24 的新日志中，Drive 2 在 PDO 目标仍位于 `0x607D` 范围内时停止：

```text
statusword=0x3827
0x2324.01=0x070010FB
flags=following_error,positive_limit_switch,negative_limit_switch
target=106497 actual=85204 error=21293 velocity=0
```

转矩、电流和电压均未饱和。代码现于 Homing 完成、进入 CSP 前读取并清零
Drive 0–3 的 `0x2310:01/:02` 下/上限位输入映射，再回读确认；`0x2310:04`
Homing reference、`0x2310:10` polarity 与 `0x607B/0x607D` 保持不变，不执行
`0x1010` 永久存储。T1 应记录 `CSP_LIMIT_SWITCH_CONFIGURATION`；组 `18` 可在 CSP
前只读修复前映射。

实机组 `18` 已确认 Drive 0–2 均有 `lower=0x01, upper=0x04`，且 Drive 1 当时已上报
`positive_limit_switch`。同一次读取还发现 Drive 2 在 Homing 后为
`0x6065/0x6066=16384/48`，而不是目标 `25000/250`。因此当前代码在 CSP 交接点另行
记录 `CSP_FOLLOWING_ERROR_CONFIGURATION`，重新写入并回读目标值；未通过时禁止 CSP。

Git 追溯显示，Homing 代码只设置 `0x2310:04` 的行为来自较早的 commit `d56d695`；
从已验证基线 `214477ef` 到本次修改前 HEAD 的差异中，也没有任何
`0x2310`/`REFERENCE_SWITCH_INPUT` 改动。最近队友的 `4708444` 只改了 Drive 3
参考运动参数与失败处理。因此这是旧的潜在配置缺口；新坐标/路径可能触发它，但不是
最近队友改动直接引入。

## 10. TCP 标定历史与当前 170 mm 理想 TCP

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

随后曾按用户定义，把 TCP 沿夹爪伸出方向（`lowerarm +X`）向外移动 `0.020 m`，
作为两爪长度中间的悬空抓取中心。之后追加的局部 X `0.020 m` 已根据实机测试撤销。
该阶段的固定 TCP 为：

```text
xyz = [0.13478978,0.02881369,0.03193108] m
```

2026-07-24 为按同一实体参考点重新校正，当时 `tcp_link` 曾还原到最早的
`spur_gear_joint` 中心：

```text
xyz = [0.13916,0,0.0179] m
rpy = [0,-pi/2,0]
```

2026-07-27 根据实机测量，从 lowerarm 远离 TCP 端的参考螺丝孔到理想 TCP
沿杆方向为 `170 mm`。图纸中的 `17.9 mm` 是独立垂直偏置，继续保留。当前
规划 TCP 因而改为：

```text
tcp_fixed_joint xyz = [0.170,0,0.0179] m
spur_gear_joint xyz = [0.13916,0,0.0179] m（实体轴位置不变）
两者沿 lowerarm +X 相差 0.03084 m
```

此次只移动无实体质量的 `tcp_link`；spur gear、gripper mesh、Drive 3、Homing、
CSP 和 counts 映射均未改动。

2026-07-23 曾加入项目级实体 X 补偿。用户确认外部读数顺序为 `Y/X/Z`，因此
实体 `+X` 对应数值 `base_link +Y`。为消除“实际位置在实体 X 负方向少 `0.020 m`”
的固定误差，`base_link -> shoulder_joint` 的模型平移设为：

```text
xyz = [0,-0.020,0.057441] m
```

该负号属于模型坐标修正：用户目标保持不变，IK 会把实机抓夹沿实体 `+X` 多移动
`0.020 m`。这不是 `lowerarm` 局部 TCP 延伸，因此不会随姿态转成主要的 Z 偏移。

2026-07-24 后续单点观测显示，模型 XY `[0.12,0.12] m` 对应实体
`[0.16,0.16] m`。当时曾将该差值作为固定 XY 平移处理，在上一版基础上把整个机械臂
模型的 X、Y 各增加 `0.040 m`；该阶段 `base_link -> shoulder_joint` 为：

```text
xyz = [0.040,0.020,0.057441] m
```

2026-07-27 根据模型检查确认，这个单点 XY 平移会使 shoulder 旋转轴与底座实体
中心发生线性错位，因此已完全撤销。当前恢复无补偿 CAD 对齐：

```text
base_link -> shoulder_joint xyz = [0,0,0.057441] m
```

Z、Homing 和驱动映射均未修改。物理 spur gear 中心仍保持 CAD 原点；当前规划
TCP 继续使用上面的 170 mm 实测值。后续误差应通过多姿态测量定位实际几何或 encoder
参数，不能再次移动 shoulder 掩盖。

FK、IK、轨迹终点检查、TF 查询和调试组 `13` 全部改用 `tcp_link`。没有移动
`spur_gear_joint` 的实体 URDF 原点，也没有修改 Drive 3 控制。

理想关节角下：

```text
q=[0,0,0]
TCP=[0.32840,-0.00177,0.043001] m

q=[0,+pi/2,+pi/2]
TCP=[0.23840,-0.00177,0.293001] m
```

当前 170 mm 理想 TCP 修改必须通过：

- URDF XML 与 fixed frame 检查；
- URDF 与 Python 的 CAD 基座对齐一致性检查；
- FK 零位和 nominal Home 检查；
- IK 对 nominal Home 的回算；
- Python 导入与运动学回归检查；
- `git diff --check`。

当前代码中的 URDF、TF、FK、IK 与回归测试必须在数值上保持一致。单元测试只能证明软件
内部一致；实机调整后仍应在 Home 和至少一个额外姿态比较模型 `tcp_link` 与外部测得的
同一个理想 TCP 实体参考点。

## 11. 为什么必须做多姿态验证

此前的单姿态偏移不能区分：

- 固定 TCP 安装偏移；
- `base_link` 原点/方向误差；
- 连杆长度或 joint origin 误差；
- encoder/home offset 误差。

首先在实际 Home joint state 检查组 `13` 的理想 TCP 是否接近当前名义值：

```text
(0.23840,-0.00177,0.293001) m
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
5  逐轴执行 Drive 0、1、2 区间中点 Homing；最后自动执行 Drive 3 参考运动和置零
6  home_all：Drive 0–2 区间中点置零，再执行 Drive 3 +50000 counts 和 Method 37 置零
7  启动实机 CSP ros2_control，Drive 3 参与
8  Controller/joint state 保持检查
9  只规划实机 minimum-jerk
10 执行实机 minimum-jerk
11 检查残留进程和 TCP 端口
12 打包完整 ROS 日志
13 查询 base_link -> tcp_link
14 设置下一目标 XYZ 与运动时间
15 CSP 下输入 close/open，或输入任意非零相对 counts 控制 Drive 3
16 读取最近 CSP_STALL_SNAPSHOT
17 读取 Drive 3 当前绝对 counts（本次 Method 37 零位）
18 CSP 前只读 Drive 0–3 输入映射及 Drive 2 保护参数
19 Homing 后、CSP 前相对微调 Drive 0（输入 counts）
20 Homing 后、CSP 前相对微调 Drive 1（输入 counts）
21 Homing 后、CSP 前相对微调 Drive 2（输入 counts）
```

脚本不会自动跨终端操作。组 `4` 和组 `7` 是前台持续进程。
组 `19/20/21` 仅用于标定传感器区间中点到真实 Home 的差值：可反复输入正/负
相对 counts，返回的 `correction_from_homed_zero` 是实时累计值；不会重设零点或
写入永久参数。必须在组 `6` 成功后、组 `7` 之前使用。

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
Homing completed for required drives; CSP handoff armed:
drive0_interval(...) drive1_interval(...) drive2_interval(...)
drive3_reference(...delta=50000,...zero=0,method=37)
Drive 3: absolute_counts=0, ... reference_complete=true
```

需要测试 Home 微调时，仍在 T2 按需重复执行：

```bash
bash ./rascl_debug.sh 19   # Drive 0
bash ./rascl_debug.sh 20   # Drive 1
bash ./rascl_debug.sh 21   # Drive 2
```

记录每轴最终 `correction_from_homed_zero`。永久标定公式为
`new_home_offset_counts = current_home_offset_counts + correction_from_homed_zero`；
改代码并重新编译前，不要把临时微调误认为已经保存。

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

Drive 2 出现 following error、`internal_limit_active` 或中途停滞时执行：

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
CSP_LIMIT_SWITCH_CONFIGURATION
CSP_FOLLOWING_ERROR_CONFIGURATION
MOTION_RESULT
```

## 17. 后续修改要求

1. Drive 0–2 保持 `[28,28,24]` 的第一边沿搜索方向，并在穿出有效区间后回到
   `(entry+exit)/2`，Method 37 中点置零成功后才标记 Homed；
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
14. Drive 2 先验证 `0x2310:01/:02` 修复，不盲目继续调大保护参数；
15. 保留用户现有 Git 修改，不进行无授权的提交、推送或破坏性回滚。

## 18. 新窗口的第一项工作

新窗口接手后应先：

1. 阅读本文件；
2. 阅读 `WP3_Task1_MinJerk_Debug_Guide_CN.md`；
3. 检查 `git status` 和当前 HEAD；
4. 确认新机器的 EtherCAT 网卡名；
5. 指导完成组 `1 → T1:4 → T2:6 → T2:7 → T3:8 → T3:13`；
6. 获取新 TF、`/joint_states` 和外部真实 TCP；
7. 判断当前 170 mm 理想 TCP 是否正确；
8. 验证额外姿态，决定是否需要进一步运动学标定。

当前最重要的状态：

```text
本轮 Drive 3 方向反转前的上游基线是 commit 2d5c6d5；当前 `tcp_link` 为
lowerarm 局部 `[0.170,0,0.0179] m` 的实测理想点，物理
`spur_gear_joint` 中心仍为 `[0.13916,0,0.0179] m`。
组 15 同时支持 close/open 快捷动作和任意非零相对 counts。
组 17 返回以本次 Drive 3 Method 37 零位为基准的绝对 counts。
Drive 2 的 2026-07-24 故障已复现；当前应验证 CSP 交接时
CSP_LIMIT_SWITCH_CONFIGURATION 的 lower/upper=0x00/0x00 回读。
```
