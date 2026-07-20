printf 'GET_ALL\n' | nc 127.0.0.1 15001
ros2 topic echo --once /joint_states
sleep 20
printf 'GET_ALL\n' | nc 127.0.0.1 15001
ros2 topic echo --once /joint_states


rascl-container:~/ws$ printf 'GET_ALL\n' | nc 127.0.0.1 15001
ros2 topic echo --once /joint_states
sleep 20
printf 'GET_ALL\n' | nc 127.0.0.1 15001
ros2 topic echo --once /joint_states
bash: nc: command not found
WARNING: topic [/joint_states] does not appear to be published yet
Could not determine the type for the passed topic
bash: nc: command not found
WARNING: topic [/joint_states] does not appear to be published yet
Could not determine the type for the passed topic


现在先在 T3 执行：
cd /root/ws
bash ./rascl_debug.sh 8
这不会运动。若显示两个 controller 都是 active 且有 /joint_states，说明 T2 正常，刚才只是 ROS 域不一致。
然后在 T3 执行下面三行：
source /opt/ros/jazzy/setup.bash
source /root/ws/install/local_setup.bash
export ROS_DOMAIN_ID=88
读取原始 Drive counts 的正确命令是：
python3 -c "import socket; s=socket.create_connection(('127.0.0.1',15001),2); s.sendall(b'GET_ALL\n'); print(s.recv(4096).decode().strip()); s.close()"
再执行：
ros2 topic echo --once /joint_states