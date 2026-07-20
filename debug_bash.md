操作步骤：
所有 T1/T2/T3 实机进程关闭后，在容器内运行一次：
bash ./rascl_debug.sh 1

按原流程复现即可：
T1: 4
T2: 6 → 7
T3: 8 → 14 → 9 → 10

如果再次发生 following error：立即停止发送、支撑机械臂；在 T2、T1 分别 Ctrl-C 结束旧会话。

在任意容器终端执行：
bash ./rascl_debug.sh 12

将生成的 /root/ws/ros_logs_时间.tar.gz 直接拖给我，无需复制终端文本。