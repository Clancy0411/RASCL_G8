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


rascl-container:~/ws$ bash ./rascl_debug.sh 8
rascl_position_controller forward_command_controller/ForwardCommandController  active
joint_state_broadcaster   joint_state_broadcaster/JointStateBroadcaster        active
command interfaces
	lowerarm_joint/position [available] [claimed]
	shoulder_joint/position [available] [claimed]
	spur_gear_joint/position [available] [claimed]
	upperarm_joint/position [available] [claimed]
state interfaces
	lowerarm_joint/position
	lowerarm_joint/velocity
	shoulder_joint/position
	shoulder_joint/velocity
	spur_gear_joint/position
	spur_gear_joint/velocity
	upperarm_joint/position
	upperarm_joint/velocity
A message was lost!!!
	total count change:1
	total count: 1---
header:
  stamp:
    sec: 1784573064
    nanosec: 299499097
  frame_id: base_link
name:
- lowerarm_joint
- shoulder_joint
- spur_gear_joint
- upperarm_joint
position:
- 7.82643259125327e-06
- 0.6231581723889158
- -0.0072424789520916496
- 0.337874921396995
velocity:
- 0.0
- 0.0
- 0.0
- 0.0
effort:
- .nan
- .nan
- .nan
- .nan
---
测量 /joint_states 频率 10 秒……
average rate: 49.999
	min: 0.020s max: 0.020s std dev: 0.00012s window: 52
average rate: 49.997
	min: 0.020s max: 0.020s std dev: 0.00012s window: 103
average rate: 50.000
	min: 0.020s max: 0.020s std dev: 0.00012s window: 154
average rate: 50.001
	min: 0.020s max: 0.020s std dev: 0.00011s window: 205
average rate: 50.000
	min: 0.020s max: 0.020s std dev: 0.00011s window: 255
average rate: 50.000
	min: 0.020s max: 0.020s std dev: 0.00012s window: 306
average rate: 50.000
	min: 0.020s max: 0.020s std dev: 0.00012s window: 357
average rate: 50.001
	min: 0.019s max: 0.020s std dev: 0.00012s window: 408
average rate: 49.998
	min: 0.019s max: 0.021s std dev: 0.00014s window: 458
rascl-container:~/ws$ source /opt/ros/jazzy/setup.bash
source /root/ws/install/local_setup.bash
export ROS_DOMAIN_ID=88
rascl-container:~/ws$ python3 -c "import socket; s=socket.create_connection(('127.0.0.1',15001),2); s.sendall(b'GET_ALL\n'); print(s.recv(4096).decode().strip()); s.close()"
Traceback (most recent call last):
  File "<string>", line 1, in <module>
TimeoutError: timed out
rascl-container:~/ws$ ros2 topic echo --once /joint_states
header:
  stamp:
    sec: 1784573118
    nanosec: 439565040
  frame_id: base_link
name:
- lowerarm_joint
- shoulder_joint
- spur_gear_joint
- upperarm_joint
position:
- 7.82643259125327e-06
- 0.6231581723889158
- -0.0072424789520916496
- 0.33787296478884715
velocity:
- 0.0
- 0.0
- -0.0002383441561889667
- 0.0
effort:
- .nan
- .nan
- .nan
- .nan
---
