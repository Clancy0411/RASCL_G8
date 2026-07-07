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
