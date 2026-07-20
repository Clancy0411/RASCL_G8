printf '0.15\n-0.11\n0.253\n8\n' | bash ./rascl_debug.sh 14
bash ./rascl_debug.sh 9

rascl-container:~/ws$ printf '0.15\n-0.11\n0.253\n8\n' | bash ./rascl_debug.sh 14
bash ./rascl_debug.sh 9
目标已设置为 [0.15, -0.11, 0.253] m，时间 8 s。
下一步必须执行组 9，只规划成功后才能执行组 10。
[INFO] [1784573554.408174276] [_ros2cli_786]: waiting for service /controller_manager/list_controllers to become available...
rascl_position_controller forward_command_controller/ForwardCommandController  active
joint_state_broadcaster   joint_state_broadcaster/JointStateBroadcaster        active
[INFO] [1784573555.762436040] [wp3_tsk1]: WP3 Task 1 single-target minimum-jerk node started.
[INFO] [1784573555.762625523] [wp3_tsk1]: Target frame: base_link, unit: meter.
[INFO] [1784573555.762773790] [wp3_tsk1]: TCP definition: spur_gear_joint origin.
[INFO] [1784573555.762930677] [wp3_tsk1]: Calibration convention: URDF q=[0,0,0,0] remains the physical model-zero pose with nominal TCP (0.29756, -0.00177, 0.043001) m. The calibrated automatic-Home switch pose is nominally q=[0,+pi/2,+pi/2,0], not four zeros.
[INFO] [1784573555.763056618] [wp3_tsk1]: Waiting for /joint_states ...
[INFO] [1784573555.971188798] [wp3_tsk1]: Current joints [shoulder, upperarm, lowerarm, spur_gear] = [0.62316, 0.33787, 1e-05, -0.00724] rad
[INFO] [1784573555.971541659] [wp3_tsk1]: Current TCP in base_link = (0.2479, -0.1804, 0.1500) m
[INFO] [1784573555.971847164] [wp3_tsk1]: Requested target TCP in base_link = (0.1500, -0.1100, 0.2530) m
[INFO] [1784573555.977051343] [wp3_tsk1]: IK result: success=True, error=0.00157 m, q_arm=[0.62325, 0.3849, -1.11774], fk=(0.1496, -0.1097, 0.2515) m
[INFO] [1784573555.978653916] [wp3_tsk1]: Saved generated minimum-jerk trajectory to: /tmp/rascl_wp3_tsk1_last_trajectory.csv
[INFO] [1784573555.979250279] [wp3_tsk1]: Generated 401 trajectory samples, duration=8.00s, rate=50.00Hz.
[WARN] [1784573555.979412635] [wp3_tsk1]: execute=false: trajectory was generated but no command was published. Set execute:=true after checking the target and generated CSV.
time_from_start,shoulder_joint,upperarm_joint,lowerarm_joint,spur_gear_joint
0.000000,0.623158172,0.337874921,0.000007826,-0.007242479
0.020000,0.623158172,0.337874929,0.000007652,-0.007242479
0.040000,0.623158173,0.337874980,0.000006440,-0.007242479
0.060000,0.623158173,0.337875118,0.000003164,-0.007242479
7.920000,0.623254116,0.384899284,-1.117724499,-0.007242479
7.940000,0.623254116,0.384899551,-1.117730846,-0.007242479
7.960000,0.623254117,0.384899689,-1.117734122,-0.007242479
7.980000,0.623254117,0.384899740,-1.117735335,-0.007242479
8.000000,0.623254117,0.384899747,-1.117735509,-0.007242479
规划已通过；当前目标可由组 10 执行：[0.15, -0.11, 0.253] m
