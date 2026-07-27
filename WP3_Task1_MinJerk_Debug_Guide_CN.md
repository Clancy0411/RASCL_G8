# WP3 Task 1：Homing + CSP/PDO 调试指南

## 实机最短流程（每次完整重启从这里开始）

这条流程只包含启动实机、Homing、进入 CSP 和发送 TCP 坐标所必需的步骤。
代码、Docker 镜像和 `install/` 没有变化时，不要在流程中插入编译、测试或 fake
hardware。只有修改代码、重新拉取仓库，或组 `4` 明确提示缺包时，才在所有实机
进程关闭后单独执行一次组 `1`。

### A. 打开三个窗口

在三个 Ubuntu Terminal 中分别执行：

```bash
cd ~/RASCL_G8
bash ./rosws.sh
```

看到 `rascl-container:~/ws$` 后，三个容器终端分别命名为 T1、T2、T3，并各执行：

```bash
cd /root/ws
bash ./rascl_debug.sh
```

脚本不会跨终端操作。组 `4` 和组 `7` 是前台持续进程，必须分别占住 T1、T2。

### B. 一步到位启动实机

严格按以下窗口和组号执行：

```text
T1：4
    ↓ bridge 保持运行，不能 Ctrl-C
T2：6
    ↓ D0–D2 回到各自传感区间中点后，D3 自动 +50000 counts 并置零；必须显示 success=True
T2：可选 19 / 20 / 21
    ↓ 仅在标定时，以相对 counts 分别微调 D0 / D1 / D2；记录 correction_from_homed_zero
T2：标定时执行 22
    ↓ 不运动；以 Method 37 将当前 D0–D2 姿态设为本次会话 Home，D3 不变
T2：7
    ↓ ros2_control 保持运行，不能 Ctrl-C
T3：8 → 13 → 14 → 9 → 10
```

详细检查点：

1. **T1 组 4：启动唯一 EtherCAT/Homing bridge。**
   - 当前工作站使用网卡 `enx3c18a0256deb`。脚本组 `4` 会自动使用该默认值；只有切换到另一台工作站时，才通过 `RASCL_INTERFACE=<网卡名>` 覆盖。
   - 启动前只做必需的软件存在性检查；缺包时会在机械臂动作前停止并提示组 `1`。
   - Drive 0–2 使用传感器区间中点 Homing：原生方法以 `1000` 找到第一边沿，
     随后改用 `200` 的低速和正弦加速度曲线沿同方向穿出有效区间，记录第二边沿，
     Halt 后以相同低速反向到 `(entry+exit)/2`，最后用 Method 37 把中点设为
     `0 counts`。D0/D1 穿越负方向，D2 穿越正方向；不改变关节 direction 或
     URDF offset。
   - 三轴到位后，Drive 3 从实时位置相对运动
     `+50000 counts`，再用 FAULHABER Homing Method 37 把到达位置设为 `0 counts`。
   - 启动日志必须包含 `Drive 2 CSP following-error monitor changed`。本次测试会将
     Drive 2 的 `0x6065/0x6066` 设为 `25000 counts / 250 ms`，并尝试读出
     `0x607B/0x607D`；不会改写这两项软件位置限位。若 PRE-OP 第一次读取出现临时
     `WkcError`，bridge 会短重试；仍失败时只警告并继续，组 `6` 会再次读取。
   - CSP 转矩配置不会影响这一阶段的 Homing。组 `7` 交接 CSP 时，Drive 0–3
     可写的正/负方向限制 `0x60E0/0x60E1` 才会设为 `1000`（100% 额定转矩）并
     回读。Drive 2/3 还会把过低的峰值电流 `0x2329:03` 分别从实机曾测得的
     `220/81 mA` 提高到各自额定电流 `1100/540 mA`；额定和持续电流
     `0x2329:01/:02` 不改。
     这些修改只在当前上电会话使用，不执行永久参数存储。
   - 等到出现 `TCP bridge listening on 127.0.0.1:15001`。
   - 此后直到停机或故障重启，禁止关闭 T1，禁止启动第二个 bridge。
2. **T2 组 6：执行 `home_all`。**
   - 脚本不再要求输入二次确认，选择组 `6` 后立即开始 Home。
   - 开始时会打印每个 Drive 的输入状态及 `0x2310 lower/upper/option/reference`，
     用于记录 CSP 修复前的限位输入映射。
   - 先运动 Drive 0–2。每轴都必须打印
     `driveN_interval(entry=...,exit=...,width=...,midpoint=...,reached=...,zero=...,zero_tolerance=500)`；
     `abs(zero)` 必须不超过 `500 counts`。缺少任一轴记录时脚本会停止。随后
     Drive 3 执行 `+50000 counts` 并置零。
   - 必须返回 `success=True`、`CSP handoff armed`，且消息中包含
     `drive3_reference(...delta=50000,...zero=0,method=37)`。
   - 随后脚本自动调用 counts 查询；必须显示
     `absolute_counts` 接近 `0`、`reference_complete=true`。
   - 随后脚本自动打印 `Drive 2 protection`。若 `0x607B` 或 `0x607D` 不是完整 S32
     范围，先保存该行输出；不要自行清除限位，提交后再判断是否与计划目标冲突。
   - 未看到这两项时禁止进入组 7。
3. **仅在检查 Home 偏差时，T2 选择组 19/20/21。**
   - `19/20/21` 分别微调 Drive `0/1/2`。输入带正负号的相对 counts；同组可反复执行。
   - 输出的 `correction_from_homed_zero` 是该轴相对本次 Method 37 Homing 零点的实时累计
     counts，不需要手工累加。三个组都不会重新 Homing、不会重设零点、不会移动 Drive 3。
   - 找到正确物理 Home 后记录三个累计值，再执行组 `22`。它们是标定数据，不会因本次
     试验自动写入代码。
4. **标定时仍在 T2 选择组 22：将当前姿态设为本次会话 Home。**
   - Drive 0–2 依次用 Method 37 把当前位置定义为 `0 counts`，不会产生主动运动；
     Drive 3 的夹爪零位不变。
   - 必须保存输出中的 `drive0_before/drive1_before/drive2_before`。它们是将来固化自动
     Homing 所需的三轴修正量；三个 `driveN_after` 应接近 `0`。
   - 该 Home 只对当前上电/EtherCAT 会话有效。重新执行组 `6` 会再次以传感器区间中点
     置零并覆盖它。
5. **仍在 T2 选择组 7：启动 CSP ros2_control。**
   - 组 7 必须复用仍在 T1 运行的 bridge，不能另开 bridge。
   - T1 必须出现 `CSP_LIMIT_SWITCH_CONFIGURATION`。D0–D3 的
     `lower/upper` 最终都必须是 `0x00/0x00`，且 `device` 不再含
     `positive_limit_switch` 或 `negative_limit_switch`。bridge 只清除
     `0x2310:01/:02` 的易失映射；不会改变 Homing 的 `reference`、输入 `polarity`
     或 `0x607B/0x607D`。回读不符时组 `7` 会拒绝进入 CSP。
   - T1 必须出现 `CSP_FOLLOWING_ERROR_CONFIGURATION`，且最终显示
     `0x6065 ... -> 25000 counts`、`0x6066 ... -> 250 ms`。部分驱动会在 Homing
     模式切换后恢复旧值，因此 bridge 在 CSP 交接点重新写入并验证；失败时拒绝进入 CSP。
   - T1 必须先出现 `CSP directional torque limits verified for this session only`。
     D0–D3 的每项输出格式为 `max/pos/neg 原值 -> 新值`；新值中的 `pos/neg`
     必须是 `1000/1000`。首次修正时 Drive 2 应显示近似
     `max/pos/neg 200/200/200 -> 1000/1000/1000`，并显示
     `motor_mA(rated/continuous/peak) 1100/1100/220 -> 1100/1100/1100`。Drive 3
     应显示近似 `max/pos/neg 150/150/150 -> 1000/1000/1000` 和
     `motor_mA(rated/continuous/peak) 540/540/81 -> 540/540/540`。若同一
     上电会话已经修正，原值也可能已经等于相应最终值。最终值必须分别为
     `1000/1000/1000`、`1100/1100/1100` 和 `540/540/540`。Drive 0、1 的只读
     `max` 可以保持原值，其电流参数不会被修改。
   - `0x6072` 是只读值；bridge 通过修正 Drive 2/3 的 `0x2329:03` 使它们分别从
     `200/150` 变为 `1000`。任一峰值电流写入、回读或 `0x6072 >= 1000` 检查
     失败，组 `7` 会直接拒绝进入 CSP，此时禁止发送运动目标。
   - T1 必须出现 `Master reached OP state` 和 Homing-to-CSP handoff 成功信息。
   - T2 必须出现 `Activated real RASCL hardware in csp mode`；组 7 随后持续占住 T2。
6. **T3 组 8：检查 controller 和 joint state。**
   - `joint_state_broadcaster` 与 `rascl_position_controller` 必须都是 `active`。
   - `/joint_states` 必须连续输出；四轴应接近 `[0,+1.5708,+1.5708,0]`。
   - 任一项不满足，禁止执行组 9/10。
7. **T3 组 13：查看当前模型 TCP。**
   - 读取 TF `base_link -> tcp_link`；当前 TCP 是沿 lowerarm +X 测得的
     170 mm 理想点，名义 Home 应接近
     `[0.23840,-0.00177,0.293001] m`。记录输出并测量同一实体参考点。
8. **T3 组 14：输入下一目标。**
   - 依次输入 TCP 的 `x/y/z`（米）和运动时间（秒）。
   - 整数或小数都可，例如输入 `5` 会自动作为 `5.0` 秒发送给 ROS。
   - 只设置目标，不会运动。
9. **T3 组 9：只规划。**
   - 必须 IK success、命令正常结束，且脚本确认 CSV 不含 `nan/inf`。
   - 成功后会保存“当前 CSP 会话 + 当前目标”的执行授权；可在同一菜单或另一次
     `bash ./rascl_debug.sh 10` 中执行。
10. **T3 组 10：执行。**
   - 脚本再次确认两个 controller 为 `active` 且 `/joint_states` 可用。
   - 组 `10` 不再要求输入二次确认，执行后立即开始已规划的轨迹。
   - 结束时必须出现 `MOTION_RESULT reached=true`。节点会核对最终四轴反馈和 TCP；仅把
     命令发布完、但实机中途停住会返回失败，不再显示为运动成功。
   - 失败时脚本自动打印最近一次 `CSP_STALL_SNAPSHOT`；随后立即执行组 `12` 打包日志，
     不要继续发送目标。组 `16` 可再次查看同一快照。
   - 每次执行后规划授权自动清除；下一个坐标必须重新执行 `14 → 9 → 10`。

### Drive 3 / gripper 的收放与自定义 counts

完成 Drive 0–2 Home 和组 `7` 后，在 T3 运行组 `15`。可以输入 ASCII 快捷动作，也可以
直接输入任意非零的有符号整数 counts：

```text
close 或 c = 最多从当前位置相对 -500000 counts；夹住物体后提前停止并保持
open  或 o = 从当前位置固定相对 +200000 counts；要求完整到位
+2000       = 从当前位置正向增加 2000 counts
-500000     = 从当前位置反向减少 500000 counts
```

Drive 3 的绝对 counts 以本次 Method 37 零位为基准；组 `15` 的数值输入仍是相对当前位置
的增量。它的 URDF、ros2_control、运动学和脚本预检限位统一为 `[-2*pi,+2*pi]`；越界
目标仍会被拒绝。只有 `close` 是接触感知快捷动作。执行前，bridge 会在保持 50 Hz PDO
的同时逐周期写入并回读 Drive 3 `0x60E0/0x60E1=300`（额定转矩 30%），以
`20000 counts/s` 克服滑槽摩擦。命令/反馈误差达到 `300 counts`，且连续 `0.06 s` 内
编码器进度不超过 `50 counts` 时判定接触；bridge 立即把转矩降至 `100`（10%），目标只
再向闭合方向预压 `100 counts`。
脚本记录 `SPUR_CONTACT`、`SPUR_RESULT` 和包含实际转矩/电流的
`SPUR_CONTACT_SNAPSHOT`。它的 `-500000 counts` 是最大闭合
行程，不保证走满。`open` 固定精确运动 `+200000 counts`，不启用接触提前终止；直接
输入的有符号 counts 同样是精确相对运动。`open` 或自定义 counts 会先恢复正常
`0x60E0/0x60E1=1000` 和 `20000 counts/s`；精确目标不可达时仍会失败。

脚本以 50 Hz minimum-jerk CSP 轨迹平滑发送，前三轴使用刚读到的 `/joint_states`
保持不动。`close`、`open` 和自定义 counts 均默认 `20000 counts/s`；完整
`500000 counts` 最长约 25 秒，`open` 约 10 秒，close 通常会在接触时提前结束。
运动时间由 counts 自动计算，不再单独询问。
组 `15` 的预检查和实际运动节点都会等待最多 5 秒取得完整 `/joint_states`，避免新建
DDS 节点时 1 秒发现窗口过短；仍未收到反馈才禁止运动。可通过
`RASCL_SPUR_GEAR_FEEDBACK_TIMEOUT_S` 有意覆盖该超时。每次组 `15` 会等待运动和
1 秒稳定期结束后才返回。

任意一次相对运动后，在 T3 执行组 `17`：

```bash
bash ./rascl_debug.sh 17
```

输出中的 `absolute_counts` 是相对本次 Drive 3 零位的驱动器实际位置，可直接记录为候选
打开/闭合位置。组 `17` 只读且可在 CSP 中使用。新的 `close` 不再等待 drive following
error 才停止：行程转矩为 `300‰`，接触后立即降到 `100‰` 保持。若 `300‰` 仍不能克服
空载滑槽摩擦，只调整行程参数 `RASCL_SPUR_CLOSE_TORQUE_LIMIT_PER_MILLE`；保持参数
`RASCL_SPUR_HOLD_TORQUE_LIMIT_PER_MILLE=100` 不随之提高。

每次组 `15` 会在 ROS 日志中写入 `SPUR_TRACE`：相对 counts、源/目标估算 raw counts、
每秒实际位置和剩余 counts、完成时误差及最终 `SPUR_RESULT`。快捷动作检测到接触时
还会写入 `SPUR_CONTACT`；动作后 bridge 会记录 `SPUR_CONTACT_SNAPSHOT`，包括
`0x6074/0x6077` 转矩需求/实际值、`0x6078` 实际电流、位置误差和限位状态。组 `12`
打包的日志包含这些记录；若再次故障，同时会有 bridge 的
`CSP_SNAPSHOT D3(target=...,actual=...,error=...,status=...)`。

组 `15` 只能在组 `7` 的 controller 都为 `active`、且 `wp3_tsk1` 没有执行时使用。
它会清除旧组 `9` 的规划授权，因此之后运行 Task 1 必须重新执行 `14 → 9 → 10`。
Task 1 会从实时 `/joint_states` 读取新的 spur gear 位置，并在其 Cartesian 轨迹中保持
该位置。这样可在同一 CSP 会话中交替执行：`15 → 14 → 9 → 10 → 15`。

### C. CSP 正常时反复发送新坐标

T1 的组 `4` 和 T2 的组 `7` 保持原样运行。只在 T3 重复：

```text
14 → 9 → 10
```

组 `14`、`9`、`10` 可以分别用 `bash ./rascl_debug.sh <组号>` 运行；目标和规划授权会
保存在容器临时状态中。授权只对仍在运行的当前组 `7` 会话和完全相同的目标有效；组 `7`
退出、改目标或完成一次运动后，授权都会自动失效。

IK/规划失败但 T1/T2 没有 PDO、WKC、following error，且两个 controller 仍为
`active` 时，不需要重启 CSP；换一个安全目标，重新执行 `14 → 9`。禁止对规划失败
的目标执行组 `10`。

### D. CSP 或 controller 失败后的完整重启

出现 PDO/WKC/following error、`MOTION_RESULT reached=false`、SAFE-OP、controller
inactive、组 `7` 退出或 T1/T2 报错时，旧 EtherCAT 会话不得直接重试：

1. 停止发送目标，立即支撑机械臂；必要时急停。
2. 若 T2 仍在运行，在 T2 按 `Ctrl-C`，等待 ros2_control 完全退出。
3. 在 T1 按 `Ctrl-C`，关闭旧 bridge。
4. T3 可选择组 `12` 打包日志。
5. T1、T2 重新运行 `bash ./rascl_debug.sh`。
6. 从 `T1:4 → T2:6→7 → T3:8→13→14→9→10` 完整重来。

禁止在旧 T1 bridge 上再次选择组 `7`；延迟 PDO mapping、Homing 完成状态和 PDO
错误都属于该 EtherCAT 会话，必须重新启动 bridge 并重新 Homing。

## 0. 安全与窗口

实机使用三个容器终端：

- `T1`：Homing bridge；从 Homing 到 CSP 结束始终运行。
- `T2`：Homing service；Home 完成后启动 ros2_control。
- `T3`：检查 controller、joint state 和执行轨迹。

必须遵守：

1. 急停在手边，首次测试必须有能承受机械臂重量的支撑。
2. `home_all` 后禁止停止 `T1`；CSP 必须复用同一个 bridge/master。
3. CSP 运行时禁止 Homing、手动移动和启动第二个 bridge。
4. 任一 controller inactive、WKC/following error、方向异常时禁止发目标。
5. 停机前先支撑机械臂；正常停机会 Disable Voltage。

当前默认是“三轴传感器 Homing + Drive 3 定长参考 + 四轴 CSP”模式：Drive 0–2
使用外部开关 Homing；Drive 3 不搜索传感器，而是在三轴完成后运动 `+50000 counts`，
再以 Method 37 把当前位置设为零，随后参与 PDO 状态检查和轨迹保持。
`ignore_spur_gear_in_csp:=true` 仅保留为 Drive 3 硬件故障时的紧急三轴回退；正常运行
必须保持其默认值 `false`。

### Drive 2 内部限位与 following-error 测试设置

Drive 2（`lowerarm_joint`）发生的 `statusword=0x3027` 是 CSP following error。驱动器
默认 `0x6065=32 counts`、`0x6066=48 ms`，对本机械臂的 196:1 减速轴过紧。本版本仅在
每次 T1 组 `4` 启动时，以 SDO 写入并回读 **Drive 2** 的：

```text
0x6065 = 25000 counts  (约 0.0489 rad)
0x6066 = 250 ms
```

这不是关闭保护：实际偏差超过约 2.8°并持续 250 ms 仍会停机。代码不向 `0x607B`
（position range）或 `0x607D`（software position limit）写值，也不发送 `0x1010`
保存命令；其当前值由组 `4` 日志及组 `6` 末尾的
`/rascl_faulhaber_bridge/read_drive2_diagnostics` 输出记录。

若输出为完整 S32 范围 `[-2147483648, 2147483647]`，Drive 2 没有启用内部位置区间。
若限位较小，先比较它与故障日志的 `CSP_SNAPSHOT D2(target=...)`，不要直接清除限位。
新阈值下仍发生 following error，说明实际偏差已超过这条有限保护，应打包日志而不是继续
增加阈值。

若 `0x2324.01` 同时报告 `positive_limit_switch` 与
`negative_limit_switch`，而 PDO 目标仍在 `0x607D` 范围内，优先检查输入映射：

```bash
# T1 组 4 运行后、T2 组 7 之前；只读
bash ./rascl_debug.sh 18
```

输出中的 `0x2310 lower/upper` 是物理输入到下/上限位的映射。当前机械臂的三轴传感器
仅用于 `reference` Homing，不是双向行程限位。因此组 `7` 交接 CSP 时会将
`0x2310:01/:02` 清零并回读；这一步发生在 Homing 完成之后，不改变自动寻零过程，也
不写入永久存储。若需只为回退验证保留旧映射，可在组 `4` 前临时设置
`RASCL_CLEAR_LIMIT_SWITCH_MAPPINGS_FOR_CSP=false`，正常运行必须使用默认 `true`。

组 `18` 的 `0x6065/0x6066` 也是 CSP 前只读快照。若在 Homing 后看到驱动器旧值
（例如 `16384/48`），不要发送目标；组 `7` 必须通过
`CSP_FOLLOWING_ERROR_CONFIGURATION` 将最终 CSP 值重新验证为 `25000/250`。

坐标约定：

```text
URDF q=[0,0,0,0] TCP                    = [0.32840,-0.00177,0.043001] m
自动 Home（D0–D2）q~=[0,+pi/2,+pi/2] TCP = [0.23840,-0.00177,0.293001] m
Drive 3                                  = D0–D2 Home 后相对 +50000 counts，再以 Method 37 置零
direction（D0–D3）                       = [+1,+1,+1,-1]
home_offset_counts（D0–D2）名义值         = [0,-802816,-802816]
```

`homing_offsets=[0,0,0,0]` 是驱动器 `0x607C`。Drive 3 Method 37 使用 `0x607C=0`
把当时的当前位置定义为零；仍不得用它补偿 URDF/TCP 几何误差。Drive 2
使用 `direction=+1` 与 `home_offset_counts=-802816` 的配对，使正向规划对应实机正向，
同时自动 Home 仍显示为 `+pi/2`。固定 `tcp_link` 使用
`lowerarm` 局部 `[0.170,0,0.0179] m`；实体 `spur_gear_joint` 仍在
`[0.13916,0,0.0179] m`，Drive 3 与夹爪几何没有移动。IK 与 TF 使用相同 TCP 定义。

## 1. 启动容器

Ubuntu 主机检查 Docker：

```bash
docker version
sudo systemctl start docker
```

进入仓库并启动 `T1`：

```bash
cd ~/RASCL_G8
SOFT_REBUILD=true bash ./rosws.sh
```

镜像未变化时以后使用：

```bash
cd ~/RASCL_G8
bash ./rosws.sh
```

分别打开两个新 Ubuntu Terminal，执行同一命令，连接为 `T2`、`T3`：

```bash
cd ~/RASCL_G8
bash ./rosws.sh
```

三个容器终端都执行：

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=88
```

后续优先使用仓库根目录的菜单脚本：

```bash
cd /root/ws
bash ./rascl_debug.sh
```

直接输入组号即可，也可以跳过菜单，例如 `bash ./rascl_debug.sh 4`。脚本不会自动
控制其他终端；应在 T1、T2、T3 各运行一次。短任务结束后会返回本终端菜单；
持续任务会占用当前终端，按 `Ctrl-C` 才会停止。

| 组 | 终端 | 内容 |
|---|---|---|
| 1 | T1 | 编译并运行功能测试 |
| 2 / 3 | T1 / T2 | 启动并检查 fake hardware |
| 4 | T1 | 启动唯一 Homing bridge，保持运行 |
| 5 | T2 | 首次逐轴 Homing Drive 0–2 |
| 6 | T2 | 已验证后一次执行 `home_all` |
| 7 | T2 | 复用 bridge 启动 CSP，保持运行 |
| 8 | T3 | Controller 与 joint state 保持检查 |
| 9 / 10 | T3 | 只规划 / 检查后执行 |
| 11 / 12 | 任意 | 进程检查 / 打包完整 ROS 日志 |
| 13 | T3 | CSP 启动后查看实时模型 TCP |
| 14 | T3 | 设置目标 TCP 和运动时间，不运动 |
| 15 | T3 | 输入 `close`/`open`，或输入任意非零相对 counts 控制 Drive 3 |
| 16 | T3 | 查看最近一次 CSP 停滞自动诊断快照 |
| 17 | T2/T3 | 只读 Drive 3 当前绝对 counts（本次 Method 37 零位） |
| 18 | T2 | CSP 前只读输入映射和 Drive 2 保护参数 |
| 19 / 20 / 21 | T2 | `home_all` 后、CSP 前分别相对微调 Drive 0 / 1 / 2 |
| 22 | T2 | 将 Drive 0–2 当前姿态设为本次会话 Home；Drive 3 不变 |

日常实机顺序以本指南最前面的
`T1:4 → T2:6→（标定时 19/20/21→22）→7 → T3:8→13→14→9→10`
为准。下面保留原始命令用于排错。

## 2. 编译与软件测试

脚本：`T1` 选择组 `1`。

组 `1` 还会确认 ROS 能找到 `rascl_wp3_ss26_group8` 和执行入口 `wp3_tsk1`；
未完成该组时，后面的组 `3/9/10` 不能运行。

在 `T1` 确认没有旧进程：

```bash
ps -ef | grep -E "ros2_control_node|rascl_faulhaber_bridge|wp3_tsk1" | grep -v grep
ss -ltnp | grep 15001
```

无输出后编译：

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
rm -rf build install log
colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON
source install/local_setup.bash
```

`T2`、`T3` 重新加载：

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=88
```

### 2.1 只修改 TCP/URDF 后快速重新编译

如果只修改了夹爪 TCP、URDF 或 `rascl_wp3_ss26_group8` 运动学代码，不必删除整个
`build/install/log`。先停止 T1 的 Homing bridge 和 T2 的 ros2_control，确认没有旧实机
进程继续占用 EtherCAT，然后在容器 T1 执行：

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  rascl_description rascl_wp3_ss26_group8
source install/local_setup.bash
```

T2、T3 也必须重新加载新的安装环境：

```bash
cd /root/ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=88
```

TCP/URDF 改变后，旧 robot description、TF、已生成轨迹和组 `9` 的执行授权全部失效。
重新执行 `T1:4 → T2:6→7 → T3:8→13`，确认新 TCP 后，再执行
`T3:14→9→10`。不得复用修改前的 CSV 或直接执行旧目标授权。

在 `T1` 运行 TCP 标定、robot description 参数类型和两个硬件功能测试目标：

```bash
python3 -m pytest \
  src/rascl_wp3_ss26_group8/test/test_kinematics_calibration.py -q
ctest --test-dir build/rascl_description \
  -R '^test_robot_description_parameter$' \
  --output-on-failure
ctest --test-dir build/rascl_hardware_interface \
  -R '^(test_generic_system|test_faulhaber_bridge)$' \
  --output-on-failure
```

四个目标必须通过。TCP 测试确保 URDF/FK 基准和当前实测标定一致，并验证 IK 能对
原 Cartesian 目标重新规划。robot description 测试确保 xacro 输出始终以字符串参数
传入，不会被 ROS 2 launch 误当作 YAML；另外两个目标覆盖硬件接口换算、PDO 字节布局、
固定周期循环、SDO-only Homing、延迟 mapping、必需轴未 Home 禁止 CSP，以及 Drive 3
固定相对参考运动、Method 37 置零、置零未完成禁止四轴 CSP 准入。

完整 `colcon test` 还会运行 `clang_format`、`cpplint` 等代码风格检查。这些检查
可以在提交前处理，但格式或版权头失败不阻塞实机调试。判断时看 CTest 的测试
目标结果，不要把单个格式差异数量当成功能 failure。

检查 bridge 可执行权限：

```bash
ls -l install/rascl_hardware_interface/lib/rascl_hardware_interface/rascl_faulhaber_bridge.py
```

## 3. Fake hardware

脚本：`T1` 选择组 `2`，`T2` 选择组 `3`。

`T1`：

```bash
ros2 launch rascl_description ros2_control.launch.py use_fake_hardware:=true
```

`T2`：

```bash
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
```

要求 controller active、IK success、CSV 无 `nan`。完成后在 `T1` 按
`Ctrl-C`，等待返回 shell。

## 4. 实机准备

Ubuntu **主机**启用 EtherCAT 网卡（不要在 Docker 容器内执行 `ip`）：

```bash
ip link show enx3c18a0256deb
sudo ip link set enx3c18a0256deb up
ip link show enx3c18a0256deb
```

`T3` 清理 ROS graph：

```bash
ros2 daemon stop
ros2 daemon start
```

确认传感器、方向、急停、机械支撑和活动空间后再上电。

## 5. Homing（T1 全程不得停止）

脚本：`T1` 选择组 `4`；首次逐轴验证时 `T2` 选组 `5`，已验证后选组 `6`。

`T1` 启动唯一 bridge：

```bash
ros2 launch rascl_description homing.launch.py \
  interface:=enx3c18a0256deb \
  homing_interval_max_travel_drive0_counts:=100000 \
  homing_interval_max_travel_drive1_counts:=300000 \
  homing_interval_max_travel_drive2_counts:=300000 \
  homing_interval_timeout_s:=120.0 \
  csp_torque_limit_per_mille:=1000 \
  spur_close_torque_limit_per_mille:=300 \
  spur_hold_torque_limit_per_mille:=100 \
  clear_limit_switch_mappings_for_csp:=true \
  drive2_following_error_window_counts:=25000 \
  drive2_following_error_timeout_ms:=250 \
  csp_stall_error_counts:=25000 \
  csp_stall_progress_counts:=100 \
  csp_stall_timeout_ms:=500 \
  spur_gear_reference_delta_counts:=50000 \
  spur_gear_reference_timeout_s:=30.0 \
  spur_gear_reference_tolerance_counts:=100 \
  spur_gear_reference_profile_velocity:=3000 \
  spur_gear_reference_profile_acceleration:=1000 \
  spur_gear_reference_profile_deceleration:=1000 \
  spur_gear_reference_following_error_confirm_s:=0.30 \
  skip_spur_gear_homing:=true
```

Drive 3 的 `+50000 counts` 参考运动约需 20 秒。短于 `0.30 s` 的 following-error
仅记录并继续等待恢复；持续错误仍会停止 Drive 3 并使 `home_all` 失败。不要通过增大
确认时间或跳过失败来绕过机械卡滞。

应看到：

```text
Homing-to-CSP session starts SDO-only in PRE-OP
PDO mapping is deferred until home_all succeeds
Drive 0-2 Homing uses the centre of the reference-input interval
TCP bridge listening on 127.0.0.1:15001
```

`T2` 检查输入：

```bash
bash ./rascl_debug.sh 18
```

首次或机械结构变化后逐轴执行 Drive 0–2；每轴成功后再继续：

```bash
ros2 param set /rascl_faulhaber_bridge test_drive_index 0
ros2 service call /rascl_faulhaber_bridge/home_one std_srvs/srv/Trigger "{}"

ros2 param set /rascl_faulhaber_bridge test_drive_index 1
ros2 service call /rascl_faulhaber_bridge/home_one std_srvs/srv/Trigger "{}"

ros2 param set /rascl_faulhaber_bridge test_drive_index 2
ros2 service call /rascl_faulhaber_bridge/home_one std_srvs/srv/Trigger "{}"

```

每轴动作顺序固定为：

```text
以 1000 原生寻找第一边沿 → 以 200 正弦曲线同方向穿过有效区间
→ 第二边沿 → Halt → 以 200 反向到 (entry+exit)/2
→ Method 37 将中点设为 0 counts
```

服务返回中的 `entry/exit` 是同一驱动坐标系下的两边沿 counts；`width` 是区间宽度，
`midpoint` 是计算目标，`reached` 必须与它相差不超过 `500 counts`，`zero` 必须为
`0` 附近且绝对值不超过 `500 counts`。当前 `0x607C=0`，所以原生 Homing 锁存的第一边沿严格定义为
`entry=0`；边沿后的减速停稳读数不参与中点计算。
第二边沿从第一边沿继续搜索的默认上限为 D0/D1/D2
`100000/300000/300000 counts`。D0 已实测区间宽度约 `59657 counts`；D1 的
`100000-count` 上限不足，因此只放宽 D1/D2 的守卫。第一边沿的原生搜索仍受
`motion_timeout_s=8 s` 限制；有限距离的穿越和回中点分别使用 `120 s` 超时。未找到
第二边沿、出现 fault/following error，或中点未到达时，
该轴不会标记为 Homed，也不能交接 CSP。

Drive 2 成功后，bridge 会自动让 Drive 3 相对运动 `+50000 counts` 并执行 Method 37
置零；应看到 `drive3_reference(...zero=0,method=37)` 和 `CSP handoff armed`。
不要单独对 Drive 3 执行 `home_one`。

Drive 0–2 已经逐轴验证过时，也可以在新的 bridge 会话中用一次 `home_all`
代替上面三组 `home_one`。该服务在三轴完成后自动执行 Drive 3 参考流程：

```bash
ros2 service call /rascl_faulhaber_bridge/home_all \
  std_srvs/srv/Trigger "{}"

ros2 service call /rascl_faulhaber_bridge/read_spur_gear_counts \
  std_srvs/srv/Trigger "{}"

ros2 service call /rascl_faulhaber_bridge/read_drive2_diagnostics \
  std_srvs/srv/Trigger "{}"
```

必须返回：

```text
success=True
Homing completed for required drives; CSP handoff armed ...
drive0_interval(entry=...,exit=...,width=...,midpoint=...,reached=...,zero=...,zero_tolerance=500)
drive1_interval(entry=...,exit=...,width=...,midpoint=...,reached=...,zero=...,zero_tolerance=500)
drive2_interval(entry=...,exit=...,width=...,midpoint=...,reached=...,zero=...,zero_tolerance=500)
drive3_reference(...zero=0,method=37)
Drive 3: absolute_counts=0 ... reference_complete=true
```

此时不要按 `Ctrl-C`，不要调用 `disable_all`，不要关闭 `T1`。

### Homing 后以 counts 微调 Drive 0–2

只在测量传感器区间中点到真实 Home 的差值时使用。完成组 `6` 后、启动组 `7` 前：

```bash
bash ./rascl_debug.sh 19   # Drive 0
bash ./rascl_debug.sh 20   # Drive 1
bash ./rascl_debug.sh 21   # Drive 2
```

每次输入非零有符号整数，例如 `500` 或 `-1200`。命令以实时 encoder feedback 为起点，
按 `1000 counts/s` 执行相对 Profile Position 运动，并等待实际到位。同一轴可以反复
调用。成功输出示例：

```text
Drive 1 Home fine adjustment completed:
source=500, delta=-1200, target=-700, actual=-692, target_error=+8,
correction_from_homed_zero=-692 counts
```

最终只需记录每轴 `correction_from_homed_zero`；它已经是相对本次 Homing 零点的累计
实际值。此功能不执行 Method 37，因此不会改变编码器零点。未完成全部 D0–D2 Homing、
Drive 3 参考未完成、PDO 已准备或 CSP 已启动时，服务会拒绝动作。
微调前会清除该轴易失的 `0x2310:01/:02` 限位输入映射，避免传感器区域内遗留的
`positive/negative_limit_switch` 阻挡 Profile Position；`0x2310:04` Homing reference、
输入 polarity 和 `0x607B/0x607D` 均不改变，组 `7` 仍会对全部轴统一复核。

三轴到达确认过的物理 Home 后执行：

```bash
bash ./rascl_debug.sh 22
```

组 `22` 不发送位置目标，只读取 D0–D2 当前 counts，然后逐轴运行 Method 37。成功消息
必须同时包含：

```text
drive0_before=... drive1_before=... drive2_before=...
drive0_after=...  drive1_after=...  drive2_after=...
Drive 3 unchanged
```

保存三个 `before` 值；`after` 应在 `100 counts` 内接近 `0`。此后可以直接执行组 `7`，
当前物理姿态会按现有映射表示为名义 Home `[0,+pi/2,+pi/2]`。组 `22` 只建立会话零点，
不会永久改变下一次自动 Homing。

要让以后每次组 `6` 都自动到达该物理姿态，正确的永久方案是在“区间中点”之后按三轴
`before` 修正量移动，再执行 Method 37。只修改 `home_offset_counts` 只会改变
encoder-to-URDF 映射，不会让机械臂物理移动到修正后的 Home；因此先把三个实测值发回，
再统一固化自动 Homing 流程。

## 6. 可选：一次性精标 home_offset_counts

名义值只适用于两个关节恰好 90°。精标需要可靠支撑，并会主动失能；执行后
必须重新从第 5 节开始 Homing，不能直接进入 CSP。

完成 `home_all` 后，在 `T2`：

```bash
ros2 service call /rascl_faulhaber_bridge/disable_all \
  std_srvs/srv/Trigger "{}"
```

支撑机械臂并手动移动到旧版本验证过的物理 URDF `q=[0,0,0,0]` 姿态，然后：

```bash
python3 -c "import socket; s=socket.create_connection(('127.0.0.1',15001),2); s.sendall(b'GET_ALL\\n'); print(s.recv(4096).decode().strip()); s.close()"
```

该容器不保证安装 `nc`；以上 Python 标准库命令不需要额外安装软件。
必须在启动第 7 节 CSP 前执行：CSP 运行时 ros2_control 会持续占用 bridge 唯一的 TCP
客户端连接，外部 `GET_ALL` 查询会超时；此时只用 `/joint_states` 读取反馈。

响应格式：

```text
OK <D0_raw> <D0_status> <D1_raw> <D1_status> \
   <D2_raw> <D2_status> <D3_raw> <D3_status>
```

Drive 3 此时已经以 Method 37 建立会话零位，因此其 raw 值就是相对该零位的绝对 counts。
进入 CSP 后不要再新建 TCP 客户端；直接执行组 `17`，它通过 bridge 的只读 ROS service
读取 PDO 缓存。记录后支撑机械臂，在 `T1` 按 `Ctrl-C`；重新启动前放回已验证的安全
Homing 起始区域，再从第 5 节执行。第 7 节用 D0–D2 实测值替换示例数字。

## 7. 同一 EtherCAT 会话切换 CSP

脚本：保持 `T1` 的组 `4` 运行，在 `T2` 选择组 `7`。

保持 `T1` 的 Homing bridge 原样运行。在 `T2` 启动 ros2_control，明确禁止
创建第二个 bridge：

```bash
ros2 launch rascl_description ros2_control.launch.py \
  interface:=enx3c18a0256deb \
  use_fake_hardware:=false \
  start_bridge:=false \
  lowerarm_direction:=1 \
  spur_gear_direction:=-1 \
  gripper_counts_per_revolution:=1323008 \
  shoulder_home_offset_counts:=0 \
  upperarm_home_offset_counts:=-802816 \
  lowerarm_home_offset_counts:=-802816 \
  spur_gear_home_offset_counts:=0
```

`T1` 应依次看到：

```text
CSP interpolation 0x2332.00 configured to 200 x 100 us (20000000 ns PDO cycle)
assigning factory Position PDOs Rx=0x1601, Tx=0x1A01
Deferred process image mapped
Master reached OP state
Homing-to-CSP handoff completed without Shutdown/Disable controlwords
SPUR_REFERENCE drive3_reference(...delta=50000,...zero=0,method=37)
```

`T2` 应看到：

```text
Activated real RASCL hardware in csp mode
```

若出现 `not all required drives were homed`、`lost Operation Enabled`、WKC、following
error 或 SAFE-OP + Error，禁止重试运动；按第 10 节处理。

## 8. CSP 保持检查

脚本：`T3` 选择组 `8`。

Homing 后选择组 `13` 可直接查看实时模型 TCP。脚本读取 TF
`base_link -> tcp_link`；其中 `Translation` 的 `x/y/z` 单位为米。名义 Home 值为：

```text
[0.23840, -0.00177, 0.293001] m
```

该值由实时 joint state 和 URDF 正运动学计算，不是外部测量。当前 `tcp_link` 使用
`lowerarm` 局部 `[0.170,0,0.0179] m`：沿杆方向 170 mm，垂直偏置仍为 17.9 mm。
实体 `spur_gear_joint` 保持 `[0.13916,0,0.0179] m`。Home 和其他姿态仍须测量
同一个理想 TCP 实体参考点，不能用模型 TF 自身证明实体位置正确。当前
`base_link -> shoulder_joint` 已恢复无补偿 CAD 对齐
`[0,0,0.057441] m`，不再使用曾经的单点 XY 平移。后续位置误差应通过多姿态测量
定位实际连杆、joint origin 或 encoder 零位参数，不能再次用 shoulder 平移掩盖。

`T3`：

```bash
ros2 control list_controllers
ros2 topic echo --once /joint_states
ros2 topic hz /joint_states
```

要求：

```text
joint_state_broadcaster active
rascl_position_controller active
前三轴 positions ~= [0,+1.5708,+1.5708]
```

Drive 3 此时应接近本次 Method 37 的 `0 counts`；可用组 `17` 精确查看，但机械夹爪
姿态仍需实物确认。它必须处于 CSP Operation Enabled，且无 PDO/WKC/following error。
保持至少 10 秒，不发送目标。前三轴实机与
RViz 必须一致。结束 `topic hz` 时只在 `T3` 按 `Ctrl-C`。

## 9. 小幅 minimum-jerk 轨迹

脚本：`T3` 先选择组 `14` 设置目标，再选择组 `9`；检查结果后选择组 `10`。
每个新目标都必须重新执行 `14 → 9 → 10`。

`T3` 先只规划：

```bash
ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
  -p target_x:=0.2108 -p target_y:=-0.00177 -p target_z:=0.2913 \
  -p duration:=12.0 -p rate_hz:=50.0 -p execute:=false

head -n 5 /tmp/rascl_wp3_tsk1_last_trajectory.csv
tail -n 5 /tmp/rascl_wp3_tsk1_last_trajectory.csv
```

要求：当前关节接近 Home，IK 约为 `[0,1.5527,1.5550]`，CSV 无 `nan`。
确认方向和空间安全后执行：

```bash
ros2 run rascl_wp3_ss26_group8 wp3_tsk1 --ros-args \
  -p target_x:=0.2108 -p target_y:=-0.00177 -p target_z:=0.2913 \
  -p duration:=12.0 -p rate_hz:=50.0 -p execute:=true
```

成功必须看到 `MOTION_RESULT reached=true`。默认验收阈值为每轴 `0.03 rad`、TCP
`0.01 m`；超出任一项，命令以非零状态退出，脚本组 `10` 会自动读取停滞快照。

旧目标 `[0.295,0,0.048]` 接近 URDF 零位，不能作为自动 Home 后的首次运动。

## 10. 故障与停机

### 正常停机

1. 停止轨迹节点，确认没有新命令。
2. 可靠支撑机械臂。
3. 在 `T2` 按 `Ctrl-C`，等待 ros2_control 返回 shell；这一步会退出 CSP 并失能。
4. 再在 `T1` 按 `Ctrl-C`，关闭唯一 EtherCAT bridge。
5. `T3` 检查：

```bash
ps -ef | grep -E "ros2_control_node|rascl_faulhaber_bridge|wp3_tsk1" | grep -v grep
ss -ltnp | grep 15001
```

应无输出。

也可用脚本组 `11` 检查残留进程。需要提交完整日志时选择组 `12`，脚本会在
`/root/ws` 生成 `ros_logs_时间.tar.gz`，可直接从共享目录拖出。

### 关键错误

- `not all required drives were homed`：停止 `T2` 的 ros2_control launch；保持 `T1`，完成
  缺少的逐轴 Homing，或重新执行 `home_all`，再启动第 7 节。
- `lost Operation Enabled while selecting CSP`：驱动状态不支持当前无失能交接。
  立即支撑/急停，不得循环重试。
- `following error`、WKC、SAFE-OP：controller 可能自动失能，立即支撑/急停；
  不再发送命令。following error 的 `ros2_control_node` 日志会自动含
  `CSP_SNAPSHOT`：每个 Drive 的 PDO `target`、`actual`、`error=target-actual`、
  `status` 和 `mode`，均为原始 counts。故障 Drive 还会紧接着记录 `DRIVE_DIAG`：
  `0x2324.01`（软件/硬件限位、转矩/电压/速度限制、温度等状态）、`0x1001`、
  `0x1003`、`0x2310:01/:02/:03/:04/:10` 输入映射、
  `0x2311:01/:02` 逻辑/物理输入、位置/速度、`0x6074/0x6077` 转矩需求/实际值、`0x6078` 实际电流、
  `0x60F4` 跟随偏差、转矩/速度限值、`0x2329.01/.02/.03` 额定/持续/峰值电流及
  `0x2348.01` 位置环 Kv。快照为只读 SDO，任何单项读取失败都会标为
  `unavailable`，不会覆盖原始 PDO 故障。随后还会记录 `TORQUE_SNAPSHOT`，一次
  列出 D0–D3 的转矩需求/实际值及 `0x6072/0x60E0/0x60E1` 回读值。停止 T1/T2 后选择
  组 `12` 打包并提交该
  `tar.gz`；无需手动复制终端。
- **无 following error 但中途停住**：bridge 监测每轴 PDO `target-actual`。误差至少
  `25000 counts`，且连续 `500 ms` 的编码器进展不足 `100 counts` 时，会先记录
  `CSP_STALL_DETECTED`，再以每个 PDO 周期最多一次只读 SDO 的方式生成
  `CSP_STALL_SNAPSHOT`，不停止 50 Hz PDO。快照包含 `cause`、statusword 的
  `internal_limit_active`、`0x2324.01`、位置需求/实际值、速度、转矩、电流、位置/速度/
  following-error 限值、`0x2329` 电流参数，以及 `0x2325.01-.07` 电压阈值和两路
  实际电压（单位 `10 mV`）。组 `10` 会自动读取；也可在 T3 执行：

  ```bash
  bash ./rascl_debug.sh 16
  bash ./rascl_debug.sh 12
  ```

  `TORQUE_LIMIT_REPORTED`、`VOLTAGE_OR_SUPPLY_LIMIT_REPORTED`、
  `POSITION_OR_LIMIT_SWITCH_REPORTED` 等是驱动器明确上报；
  `POSITION_LOOP_STALLED_WITHOUT_LIMIT_FLAG` 表示已确认停滞但驱动未给出单一限制标志，
  此时仍需结合快照中的 demand/actual torque/current 区分负载、制动或位置环问题。
- controller inactive：

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

- 旧 bridge/端口占用：

```bash
ps -ef | grep rascl_faulhaber_bridge | grep -v grep
ss -ltnp | grep 15001
```

- bridge 找不到或无权限：

```bash
ls -l install/rascl_hardware_interface/lib/rascl_hardware_interface/
colcon build --symlink-install --packages-select rascl_hardware_interface
source install/local_setup.bash
```

- IK failed：保持 `execute:=false`，使用接近当前 TCP 的目标；禁止直接执行。

## 11. 验收标准

1. 软件测试和 fake hardware 全部通过。
2. Drive 0–2 的 `home_one` 或一次 `home_all` 成功；三轴均返回两边沿、中点和
   `abs(zero)<=500` 记录；Drive 3 随后完成
   `+50000 counts + Method 37` 置零，组 `17` 回读接近 `0`。
3. Homing bridge 未重启，延迟 PDO mapping 后进入 OP/CSP。
4. Drive 0–2 连续交接；Drive 3 完成固定参考与 Method 37 置零后进入 CSP。
5. 前三轴 `/joint_states`、实机和 RViz 一致，保持 10 秒无跳动；Drive 3 无 PDO 故障。
6. 20 ms PDO 循环无 WKC/following error，运动结束有 `MOTION_RESULT reached=true`。
7. Drive 3 的组 `15` 小步自定义相对 counts 可在 CSP 中执行，组 `17` 能读回绝对
   counts，且随后 Task 1 保持该角度；在重新标定前不验收 `close/c`。
8. Home 附近 12 秒 minimum-jerk 小轨迹成功。

## 12. 参数速查

| 项目 | 值 |
|---|---|
| Drive / Joint | `0 shoulder`, `1 upperarm`, `2 lowerarm`, `3 spur_gear` |
| Drive 3 策略 | 跳过传感器搜索；D0–D2 完成后相对 `+50000 counts`，Method 37 置零 |
| Homing method | D0–D2 第一边沿 `[28,28,24]`；区间中点再用 `37` 置零；Drive 3 当前点置零 `37` |
| Reference input | `[2,2,2,1]` |
| Drive 0x607C | `[0,0,0,0]`；D3 Method 37 以零偏置定义当前位置 |
| D0–D2 区间穿越 | 第一边沿速度 `1000`；第二边沿与回中点速度 `200`、正弦曲线；最大 `[100000,300000,300000] counts`；每阶段 `120 s`；中点/置零回读容差 `500 counts` |
| ROS direction（名义） | `[+1,+1,+1,-1]` |
| ROS offset（名义） | `[0,-802816,-802816,0]` |
| CSP mode | 8 |
| PDO cycle | `20 ms / 50 Hz` |
| CSP interpolation `0x2332:00` | `200` (`20 ms / 100 us`，由 bridge 写入并回读) |
| RxPDO2 | `0x6040 + 0x607A`, 6 bytes |
| TxPDO2 | `0x6041 + 0x6064`, 6 bytes |
| TCP bridge | `127.0.0.1:15001` |
| ROS domain | 88 |
