已加入 Drive 3 独立夹爪调试：组 15。它不需要 Homing，也不会运动 Drive 0–2 或进入 CSP。
使用流程：
先确保没有运行 CSP：停止旧的 T2 组 7 和 T1 bridge。
因为这是代码更新，先在无实机进程时执行一次组 1。
T1 启动 bridge：
bash ./rascl_debug.sh 4
等待出现 TCP bridge listening on 127.0.0.1:15001。
在 T2 或 T3 执行：
bash ./rascl_debug.sh 15
首次输入 2000，确认时输入 SPUR。观察夹爪实际方向后，再运行一次组 15，输入 -2000 验证反向。
该数值是 Drive 3 原始 encoder counts。每次执行逻辑为：
当前编码器值 + 输入步长 → 等待到位 → 自动 Disable Voltage
成功会显示类似：
OK drive=3 start=... target=... actual=... disabled=true
默认单次上限是 ±20000 counts。CSP/ros2_control_node 运行时，脚本和 bridge 都会拒绝此命令，不会影响当前可用的三轴 CSP 流程。测试完成后若要恢复机械臂流程，请关闭 T1 后从 T1:4 → T2:6→7 完整重启。