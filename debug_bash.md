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
