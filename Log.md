## 7.7初始位置的调整还需要再研究一下 目前没有实装 改了CSP+PDO 需要老师们实机验证一下是不是对
新的调试说明在README_RASCL_Group8.md





## 6.2测试成功生成了轨迹文件。

### 输出结果为：
head /tmp/rascl_wp3_tsk1_last_trajectory.csv
time_from_start,shoulder_joint,upperarm_joint,lowerarm_joint,spur_gear_joint
0.000000,0.000000000,0.000000000,0.000000000,0.000000000
0.100000,-0.000001057,-0.000037122,-0.000123951,0.000000000
0.200000,-0.000008138,-0.000285761,-0.000954150,0.000000000
0.300000,-0.000026411,-0.000927358,-0.003096434,0.000000000
0.400000,-0.000060153,-0.002112131,-0.007052369,0.000000000
0.500000,-0.000112803,-0.003960800,-0.013225041,0.000000000
0.600000,-0.000187008,-0.006566328,-0.021924854,0.000000000
0.700000,-0.000284675,-0.009995654,-0.033375314,0.000000000
0.800000,-0.000407019,-0.014291428,-0.047718830,0.000000000


从输出可以看到，轨迹从 0 秒开始，初始关节角度都是 0。
之后 shoulder_joint、upperarm_joint 和 lowerarm_joint 的数值逐渐变化，说明程序确实生成了一条连续平滑的轨迹。

因为 execute:=false，所以这一步没有让真实机器人运动，只验证了规划、IK 和 CSV 文件生成是否正常。
x轴是机械臂延申方向 正方向沿机械臂朝外
y轴是水平方向 正方向是左边
z轴是垂直方向 正方向朝上


## 6.2在 RViz 中应该看到机械臂平滑运动。

今天我继续在真实 RASCL 机器人上测试 wp3_tsk1 minimum-jerk 轨迹程序。 

目前已经确认，真实硬件启动后两个 controller 都可以正常 active，/joint_states 也可以读取。通过 /rascl_position_controller/commands 发送小的 joint command 时，机器人可以执行小幅运动，说明 ROS 2 command topic 到 controller、hardware interface、EtherCAT/Faulhaber bridge、真实电机这一整条链路是通的。 

之后我测试了 WP3 程序本身。程序可以正常读取当前 joint state，计算当前 TCP 位置，完成 IK，生成 minimum-jerk trajectory，并保存 CSV 文件。对于很小的目标点，例如 IK 结果大约为： 

q_arm = [-0.00022, -0.00144, -0.01722] 

这种最大关节变化只有约 1°，实机测试比较稳定。 

但是当我测试稍大的目标点，例如： 

target = (0.29, 0.00, 0.05) 

IK 结果变成： 

q_arm = [-0.00608, -0.06696, -0.19718] 

其中 lowerarm_joint 需要运动约 -0.197 rad，大约是 -11.3°。执行这种较大的轨迹时，真实硬件的 ros2_control interface 会报错：  

之后 /joint_states 也收不到，需要停止 launch、清理进程并重新启动硬件。 

因此，后续应该先在 fake hardware / RViz 中继续验证较大的目标点。如果 fake hardware 中可以正常运行，则说明 WP3 程序本身没有明显问题；真实机器人部分则应该继续使用更小的位移、更长的 duration 和更低的 rate_hz 逐步测试。 



x=0.5 failed
The requested Cartesian target is probably outside the reachable workspace, too close to a singularity, or blocked by the current joint limits. Best error was 0.1992 m.


x=0.29 y=0 z=0.1 work successlly
but after that give command again, always meets,
[ERROR] [1782836702.566606570] [wp3_tsk1]: No joint state received within 5.0 s
solution:change controllers.yaml ---update_rate to 50 or 100

>> TO DO
change original state in RVIZ 
automatical homing with green light sensor






2026-07-07


## Error ：pysoem WkcError during SDO write

### 错误信息

```text
pysoem.pysoem.WkcError
```

完整上下文：

```text
[rascl_faulhaber_bridge]: Connecting EtherCAT on interface: enx3c18a0264863
[EtherCAT] Opening interface: enx3c18a0264863
[EtherCAT] Found 4 slave(s)
[EtherCAT] Configuring CSP PDO mapping for slave 0
[EtherCAT] Configuring CSP PDO mapping for slave 1
Traceback (most recent call last):
  File "/root/ws/install/rascl_hardware_interface/lib/rascl_hardware_interface/rascl_faulhaber_bridge.py", line 465, in __init__
    self.bus.connect()
  File "/root/ws/install/rascl_hardware_interface/lib/rascl_hardware_interface/rascl_faulhaber_bridge.py", line 256, in connect
    self.configure_csp_pdo_mapping(self.master.slaves[slave_index], slave_index)
  File "/root/ws/install/rascl_hardware_interface/lib/rascl_hardware_interface/rascl_faulhaber_bridge.py", line 293, in configure_csp_pdo_mapping
    self._sdo_write_int_raw(slave, PDO_RX_MAPPING, 0, 0, size=1)
  File "/root/ws/install/rascl_hardware_interface/lib/rascl_hardware_interface/rascl_faulhaber_bridge.py", line 284, in _sdo_write_int_raw
    slave.sdo_write(index, subindex, int(value).to_bytes(size, "little", signed=signed))
  File "src/pysoem/pysoem.pyx", line 972, in pysoem.pysoem.CdefSlave.sdo_write
pysoem.pysoem.WkcError
```

### 原因判断

这不是找不到 slave。因为已经有：

```text
Found 4 slave(s)
```

真正问题是：程序在配置 CSP PDO mapping 时，对 slave 执行 SDO write 失败。

失败位置是：

```python
self._sdo_write_int_raw(slave, PDO_RX_MAPPING, 0, 0, size=1)
```

这个操作的含义是：尝试清空 RxPDO mapping 的 subindex 0，为后续重新映射做准备。

可能原因：

1. drive 当前状态不允许改 PDO mapping。
2. drive 需要处于 PRE-OP 才能 remap PDO。
3. 某些 Faulhaber drive 不支持当前 object dictionary 的写法。
4. slave 1 状态异常，因为 log 显示 slave 0 似乎通过，slave 1 崩。
5. CSP PDO mapping 不应该在 profile 回归测试中执行。

---

## 8. 和 Error 3 相关的代码总结

### 8.1 connect() 中的关键代码

文件：

```text
src/rascl_hardware_interface/scripts/rascl_faulhaber_bridge.py
```

今天看到的代码：

```python
def connect(self) -> None:
    # Create and configure the EtherCAT master before constructing drive wrappers.
    self.master = pysoem.Master()
    print(f"[EtherCAT] Opening interface: {self.interface}")
    self.master.open(self.interface)

    if self.master.config_init() <= 0:
        raise RuntimeError("No EtherCAT slaves found")

    print(f"[EtherCAT] Found {len(self.master.slaves)} slave(s)")

    if self.configure_pdo_mapping:
        for slave_index in self.slave_indices:
            if slave_index >= len(self.master.slaves):
                raise RuntimeError(
                    f"slave index {slave_index} requested, but only {len(self.master.slaves)} slave(s) found"
                )
            self.configure_csp_pdo_mapping(self.master.slaves[slave_index], slave_index)

    self.master.config_map()
    print("[EtherCAT] PDO mapping configured")
```

### 8.2 问题点

这里的关键判断是：

```python
if self.configure_pdo_mapping:
```

只要 `self.configure_pdo_mapping` 是 `True`，就会执行：

```python
self.configure_csp_pdo_mapping(...)
```

而今天的错误正是发生在 `configure_csp_pdo_mapping()` 里面。

### 8.3 configure_csp_pdo_mapping() 中失败位置

今天看到的代码：

```python
def configure_csp_pdo_mapping(self, slave, slave_index: int) -> None:
    # Mapping is done in PRE-OP before config_map().  If a lab drive rejects
    # remapping, launch with configure_pdo_mapping:=false and inspect the default
    # PDO layout before retrying.
    print(f"[EtherCAT] Configuring CSP PDO mapping for slave {slave_index}")

    # RxPDO 0x1600
    self._sdo_write_int_raw(slave, PDO_RX_MAPPING, 0, 0, size=1)
    self._sdo_write_int_raw(slave, PDO_RX_MAPPING, 1, 0x60400010, size=4)
    self._sdo_write_int_raw(slave, PDO_RX_MAPPING, 2, 0x607A0020, size=4)
    self._sdo_write_int_raw(slave, PDO_RX_MAPPING, 3, 0x60600008, size=4)
    self._sdo_write_int_raw(slave, PDO_RX_MAPPING, 0, 3, size=1)

    # TxPDO 0x1A00
    self._sdo_write_int_raw(slave, PDO_TX_MAPPING, 0, 0, size=1)
    self._sdo_write_int_raw(slave, PDO_TX_MAPPING, 1, 0x60410010, size=4)
    self._sdo_write_int_raw(slave, PDO_TX_MAPPING, 2, 0x60640020, size=4)
    self._sdo_write_int_raw(slave, PDO_TX_MAPPING, 3, 0x60610008, size=4)
    self._sdo_write_int_raw(slave, PDO_TX_MAPPING, 0, 3, size=1)

    # Assign the single RxPDO and TxPDO.
    self._sdo_write_int_raw(slave, PDO_RX_ASSIGNMENT, 0, 0, size=1)
    self._sdo_write_int_raw(slave, PDO_RX_ASSIGNMENT, 1, PDO_RX_MAPPING, size=2)
    self._sdo_write_int_raw(slave, PDO_RX_ASSIGNMENT, 0, 1, size=1)

    self._sdo_write_int_raw(slave, PDO_TX_ASSIGNMENT, 0, 0, size=1)
    self._sdo_write_int_raw(slave, PDO_TX_ASSIGNMENT, 1, PDO_TX_MAPPING, size=2)
    self._sdo_write_int_raw(slave, PDO_TX_ASSIGNMENT, 0, 1, size=1)
```

今天实际失败在第一行 SDO write：

```python
self._sdo_write_int_raw(slave, PDO_RX_MAPPING, 0, 0, size=1)
```

---

## 9. 当前最重要结论

### 结论 1

fake hardware 和 WP3 Task 1 上层 minimum-jerk node 是成功的。

### 结论 2

真实 EtherCAT 网卡应该使用：

```text
enx3c18a0264863
```

因为该网卡能找到：

```text
Found 4 slave(s)
```

### 结论 3

真实硬件 Profile Position 回归测试失败，不是因为找不到 slave，而是因为 bridge 在 profile 模式启动时仍然尝试配置 CSP PDO mapping。

### 结论 4

`configure_pdo_mapping:=false` 当前没有生效。因为 launch 后仍然出现：

```text
Configuring CSP PDO mapping for slave 0
Configuring CSP PDO mapping for slave 1
```

这说明：

1. launch argument 可能没有正确传给 bridge；或
2. bridge 里面 `configure_pdo_mapping` 默认值还是 True；或
3. install 目录还是旧代码；或
4. launch 文件虽然接受参数，但没有把参数写入 bridge node 的 parameters。

---

## 10. 下次继续调试建议顺序

## Step 1：确认 configure_pdo_mapping 参数在哪里定义和传递

```bash
cd /root/ws
grep -R -n "configure_pdo_mapping" src/rascl_description src/rascl_hardware_interface
```

重点检查：

```text
src/rascl_description/launch/ros2_control.launch.py
src/rascl_hardware_interface/scripts/rascl_faulhaber_bridge.py
```

---

## Step 2：检查 launch 文件是否声明并传参

```bash
nl -ba src/rascl_description/launch/ros2_control.launch.py | sed -n '1,220p'
```

需要确认 launch 文件中有类似：

```python
DeclareLaunchArgument(
    "configure_pdo_mapping",
    default_value="true",
)
```

并且 bridge node 的 parameters 里面有：

```python
"configure_pdo_mapping": LaunchConfiguration("configure_pdo_mapping"),
```

如果只有 DeclareLaunchArgument，但没有传给 Node，则命令行参数不会进入 bridge。

---

## Step 3：临时粗暴修复方案

如果只是为了先完成 Profile Position 回归测试，可以临时把 bridge 默认值改为 False。

搜索：

```bash
grep -n "configure_pdo_mapping" src/rascl_hardware_interface/scripts/rascl_faulhaber_bridge.py
```

找到类似：

```python
self.declare_parameter("configure_pdo_mapping", True)
```

临时改成：

```python
self.declare_parameter("configure_pdo_mapping", False)
```

或者如果是：

```python
self.configure_pdo_mapping = True
```

临时改成：

```python
self.configure_pdo_mapping = False
```

然后重新 build。