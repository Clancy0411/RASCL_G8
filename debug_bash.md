printf 'GET_ALL\n' | nc 127.0.0.1 15001
ros2 topic echo --once /joint_states
sleep 20
printf 'GET_ALL\n' | nc 127.0.0.1 15001
ros2 topic echo --once /joint_states