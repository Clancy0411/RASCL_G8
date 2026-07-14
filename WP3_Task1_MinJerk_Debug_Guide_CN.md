# WP3 Task 1：Homing + CSP/PDO 调试指南

本指南对应当前代码。实机顺序固定为：

```text
编译/测试 -> Fake hardware -> SDO Homing -> 同一 EtherCAT 会话切换 CSP
-> 保持测试 -> 小幅 minimum-jerk 轨迹 -> 支撑后停机
```

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

当前临时使用三轴模式：Drive 3 `spur_gear_joint` 不执行 Homing，不参与 CSP
准入和状态检查，并在每个 PDO 周期保持 Disable Voltage。不得向该关节施加载荷或
依赖其保持位置。修复后用 `ignore_spur_gear_in_csp:=false` 恢复四轴模式。

坐标约定：

```text
URDF q=[0,0,0,0] TCP              = [0.29756,-0.00177,0.043001] m
自动 Home q~=[0,+pi/2,+pi/2,0] TCP = [0.20756,-0.00177,0.293001] m
home_offset_counts 名义值          = [0,-802816,-802816,0]
```

`homing_offsets=[0,0,0,0]` 是驱动器 `0x607C`，不得用来补偿 URDF 零位。

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
| 8 / 9 / 10 | T3 | 保持检查 / 只规划 / 确认后执行 |
| 11 / 12 | 任意 | 进程检查 / 打包完整 ROS 日志 |

实机终端协作方式：

1. T1 输入 `4`，保持 Homing bridge 前台运行，不再操作 T1。
2. T2 输入 `6`；完成且显示 `CSP handoff armed` 后，菜单会回来，再输入 `7`。
3. T2 的 CSP launch 保持前台运行；T3 依次输入 `8`、`9`、`10`。
4. T1/T2 分别保留 bridge 和 ros2_control 日志；脚本不会跨终端发送命令。

下面保留展开后的原始命令用于排错。

## 2. 编译与软件测试

脚本：`T1` 选择组 `1`。

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

在 `T1` 只运行两个功能测试目标：

```bash
ctest --test-dir build/rascl_hardware_interface \
  -R '^(test_generic_system|test_faulhaber_bridge)$' \
  --output-on-failure
```

这两个目标必须通过。它们覆盖硬件接口换算、PDO 字节布局、固定周期循环、
SDO-only Homing、延迟 mapping、必需轴未 Home 禁止 CSP，以及三轴临时模式下
Drive 3 始终 Disable Voltage。

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

Ubuntu 主机启用 EtherCAT 网卡：

```bash
ip link show enx3c18a026488a
sudo ip link set enx3c18a026488a up
ip link show enx3c18a026488a
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
  interface:=enx3c18a026488a \
  ignore_spur_gear_in_csp:=true
```

应看到：

```text
Homing-to-CSP session starts SDO-only in PRE-OP
PDO mapping is deferred until home_all succeeds
TCP bridge listening on 127.0.0.1:15001
```

`T2` 检查输入：

```bash
ros2 service call /rascl_faulhaber_bridge/read_digital_inputs \
  std_srvs/srv/Trigger "{}"
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

Drive 2 成功后应看到 `CSP handoff armed`，不需要再重复执行 `home_all`。

Drive 0–2 已经逐轴验证过时，也可以在新的 bridge 会话中用一次 `home_all`
代替上面三组 `home_one`。该服务不会命令 Drive 3：

```bash
ros2 service call /rascl_faulhaber_bridge/home_all \
  std_srvs/srv/Trigger "{}"
```

必须返回：

```text
success=True
Homing completed for required drives; CSP handoff armed
```

此时不要按 `Ctrl-C`，不要调用 `disable_all`，不要关闭 `T1`。

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
printf 'GET_ALL\n' | nc 127.0.0.1 15001
```

响应格式：

```text
OK <D0_raw> <D0_status> <D1_raw> <D1_status> \
   <D2_raw> <D2_status> <D3_raw> <D3_status>
```

三轴临时模式下只有 D0–D2 raw 可作为 offset。D3 未 Homing，其 raw 不得用于
最终标定。记录后支撑机械臂，在 `T1` 按 `Ctrl-C`；重新启动前放回已验证的安全
Homing 起始区域，再从第 5 节执行。第 7 节用 D0–D2 实测值替换示例数字。

## 7. 同一 EtherCAT 会话切换 CSP

脚本：保持 `T1` 的组 `4` 运行，在 `T2` 选择组 `7`。

保持 `T1` 的 Homing bridge 原样运行。在 `T2` 启动 ros2_control，明确禁止
创建第二个 bridge：

```bash
ros2 launch rascl_description ros2_control.launch.py \
  interface:=enx3c18a026488a \
  use_fake_hardware:=false \
  start_bridge:=false \
  shoulder_home_offset_counts:=0 \
  upperarm_home_offset_counts:=-802816 \
  lowerarm_home_offset_counts:=-802816 \
  spur_gear_home_offset_counts:=0
```

`T1` 应依次看到：

```text
assigning factory Position PDOs Rx=0x1601, Tx=0x1A01
Deferred process image mapped
Master reached OP state
Homing-to-CSP handoff completed for required drives; ignored drives remain Disable Voltage
```

`T2` 应看到：

```text
Activated real RASCL hardware in csp mode
```

若出现 `not all required drives were homed`、`lost Operation Enabled`、WKC、following
error 或 SAFE-OP + Error，禁止重试运动；按第 10 节处理。

## 8. CSP 保持检查

脚本：`T3` 选择组 `8`。

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

Drive 3 的状态值此时未标定，不作为验收依据；它必须不动作且保持失能。保持至少
10 秒，不发送目标。前三轴实机与 RViz 必须一致，`T1/T2` 无 PDO、WKC 或
following error。结束 `topic hz` 时只在 `T3` 按 `Ctrl-C`。

## 9. 小幅 minimum-jerk 轨迹

脚本：`T3` 先选择组 `9`；检查结果后再选择组 `10` 并输入 `MOVE` 确认。

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
  记录 `T1/T2` 完整日志，不再发送命令。
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
2. Drive 0–2 的 `home_one` 或一次 `home_all` 成功；Drive 3 未动作。
3. Homing bridge 未重启，延迟 PDO mapping 后进入 OP/CSP。
4. Drive 0–2 交接只发送 Enable Operation；Drive 3 始终 Disable Voltage。
5. 前三轴 `/joint_states`、实机和 RViz 一致，保持 10 秒无跳动。
6. 20 ms PDO 循环无 WKC/following error。
7. Home 附近 12 秒 minimum-jerk 小轨迹成功。

## 12. 参数速查

| 项目 | 值 |
|---|---|
| Drive / Joint | `0 shoulder`, `1 upperarm`, `2 lowerarm`, `3 spur_gear` |
| 临时三轴开关 | `ignore_spur_gear_in_csp:=true`（Drive 3 不 Homing、保持失能） |
| Homing method | `[28,28,24,24]` |
| Reference input | `[2,2,2,1]` |
| Drive 0x607C | `[0,0,0,0]` |
| ROS offset（名义） | `[0,-802816,-802816,0]` |
| CSP mode | 8 |
| PDO cycle | `20 ms / 50 Hz` |
| RxPDO2 | `0x6040 + 0x607A`, 6 bytes |
| TxPDO2 | `0x6041 + 0x6064`, 6 bytes |
| TCP bridge | `127.0.0.1:15001` |
| ROS domain | 88 |
