已完成：不再对 Drive 3 / spur_gear_joint 执行 Homing；但它仍完整参与四轴 CSP/PDO，能与已验证的空间点轨迹在同一 T3 交替使用。URDF、IK 和空间轨迹逻辑未改动。
使用流程：
# T1，代码更新后首次
bash ./rascl_debug.sh 1

# T1，保持运行
bash ./rascl_debug.sh 4

# T2
bash ./rascl_debug.sh 6    # 输入 HOME；只 Home Drive 0–2
bash ./rascl_debug.sh 7    # 保持运行

# T3
bash ./rascl_debug.sh 8
bash ./rascl_debug.sh 15   # 输入 2000，再输入 SPUR
组 15 中：
2000 = 在当前 Drive 3 位置基础上移动 +2000 counts
-2000 = 反向移动 2000 counts
不使用 Drive 3 的 Home 或绝对零位
三个机械臂关节保持当前值
通过现有 /rascl_position_controller/commands 发送，因此不会和 CSP 循环抢占
交替使用方式为：
15 → 14 → 9 → 10 → 15
每次组 15 后，旧的空间轨迹规划会被主动作废；若要再执行空间轨迹，必须重新运行 14 → 9 → 10。
已更新完整指南 [WP3_Task1_MinJerk_Debug_Guide_CN.md](C:/Users/Xin/Desktop/第三学期/RASCL/WP3/RASCL_G8/WP3_Task1_MinJerk_Debug_Guide_CN.md)、两个包 README 和 [rascl_debug.sh](C:/Users/Xin/Desktop/第三学期/RASCL/WP3/RASCL_G8/rascl_debug.sh)，并删除重复的简版 debug_bash.md。
验证完成：bridge 单元测试 18 项通过、Python 编译检查通过、git diff --check 通过。尚未提交或 push；上实机前在容器中执行一次组 1。