rascl-container:~/ws$ bash ./rascl_debug.sh 7
保持 T1 的 Homing bridge 运行；ros2_control 将持续占用当前终端。
Drive 2 映射：direction=1，home_offset_counts=-802816
Drive 3 CSP 映射：direction=1，counts_per_revolution=1323008（不执行 Home）
进入 CSP 后，Home 的 lowerarm_joint 必须仍接近 +1.5708 rad；否则禁止发送目标。
[INFO] [launch]: All log files can be found below /root/.ros/log/2026-07-21-22-00-40-445262-irs-rascl06-304
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [robot_state_publisher-1]: process started with pid [308]
[robot_state_publisher-1] [INFO] [1784671240.552070717] [robot_state_publisher]: Robot initialized
[INFO] [ros2_control_node-2]: process started with pid [324]
[ros2_control_node-2] [INFO] [1784671242.626722614] [controller_manager]: Using Steady (Monotonic) clock for triggering controller manager cycles.
[ros2_control_node-2] [INFO] [1784671242.628846025] [controller_manager]: Subscribing to '/robot_description' topic for robot description.
[ros2_control_node-2] [INFO] [1784671242.630113154] [controller_manager]: update rate is 50 Hz
[ros2_control_node-2] [INFO] [1784671242.630127809] [controller_manager]: Overruns handling is : enabled
[ros2_control_node-2] [INFO] [1784671242.630131623] [controller_manager]: Spawning controller_manager RT thread with scheduler priority: 50
[ros2_control_node-2] [INFO] [1784671242.630249020] [controller_manager]: Successful set up FIFO RT scheduling policy with priority 50.
[ros2_control_node-2] [INFO] [1784671242.809064188] [controller_manager]: Received robot description from topic.
[ros2_control_node-2] [INFO] [1784671242.809134602] [controller_manager]: Enforcing command limits is disabled. Command limits from URDF will be ignored.
[ros2_control_node-2] [INFO] [1784671242.811965351] [controller_manager]: Loading hardware 'RasclBotHardware' 
[ros2_control_node-2] [INFO] [1784671242.812882820] [controller_manager]: Loaded hardware 'RasclBotHardware' from plugin 'rascl_hardware_interface/RASCLHardwareInterface'
[ros2_control_node-2] [INFO] [1784671242.812971979] [controller_manager]: Initialize hardware 'RasclBotHardware' 
[ros2_control_node-2] [INFO] [1784671242.818293327] [RASCLHardwareInterface]: Initialized 4 RASCL joints. fake_hardware=false control_mode=csp bridge=127.0.0.1:15001
[ros2_control_node-2] [INFO] [1784671242.818320025] [controller_manager]: Successful initialization of hardware 'RasclBotHardware'
[ros2_control_node-2] [INFO] [1784671242.818539717] [controller_manager]: Activating component 'RasclBotHardware'.
[ros2_control_node-2] [INFO] [1784671242.818550318] [resource_manager]: 'configure' hardware 'RasclBotHardware' 
[ros2_control_node-2] [INFO] [1784671242.818553539] [RASCLHardwareInterface]: Connecting to Faulhaber TCP bridge...
[ros2_control_node-2] [INFO] [1784671242.818671888] [RASCLHardwareInterface]: Connected to Faulhaber bridge.
[ros2_control_node-2] [INFO] [1784671242.819397415] [resource_manager]: Successful 'configure' of hardware 'RasclBotHardware'
[ros2_control_node-2] [INFO] [1784671242.819410992] [resource_manager]: 'activate' hardware 'RasclBotHardware' 
[ros2_control_node-2] [ERROR] [1784671242.929774326] [RASCLHardwareInterface]: ENTER_CSP_ALL failed. response='ERR SdoError: (1, 24690, 0, 100728834, 'Attempt to write to a read only object')'
[ros2_control_node-2] [ERROR] [1784671242.929871367] [resource_manager]: Failed to 'activate' hardware 'RasclBotHardware'
[ros2_control_node-2] terminate called after throwing an instance of 'std::runtime_error'
[ros2_control_node-2]   what():  Failed to set the initial state of the component : RasclBotHardware to active
[ros2_control_node-2] Stack trace (most recent call last) in thread 344:
[ros2_control_node-2] #17   Object "/usr/lib/x86_64-linux-gnu/ld-linux-x86-64.so.2", at 0xffffffffffffffff, in 
[ros2_control_node-2] #16   Object "/usr/lib/x86_64-linux-gnu/libc.so.6", at 0x7c1c3abf1c6b, in 
[ros2_control_node-2] #15   Object "/usr/lib/x86_64-linux-gnu/libc.so.6", at 0x7c1c3ab64aa3, in 
[ros2_control_node-2] #14   Object "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.33", at 0x7c1c3adf4db3, in 
[ros2_control_node-2] #13   Object "/opt/ros/jazzy/lib/librclcpp.so", at 0x7c1c3b0e64d6, in rclcpp::executors::MultiThreadedExecutor::run(unsigned long)
[ros2_control_node-2] #12   Object "/opt/ros/jazzy/lib/librclcpp.so", at 0x7c1c3b0bbc69, in rclcpp::Executor::execute_any_executable(rclcpp::AnyExecutable&)
[ros2_control_node-2] #11   Object "/opt/ros/jazzy/lib/librclcpp.so", at 0x7c1c3b0bb53a, in rclcpp::Executor::execute_subscription(std::shared_ptr<rclcpp::SubscriptionBase>)
[ros2_control_node-2] #10   Object "/opt/ros/jazzy/lib/libcontroller_manager.so", at 0x7c1c3b396474, in 
[ros2_control_node-2] #9    Object "/opt/ros/jazzy/lib/libcontroller_manager.so", at 0x7c1c3b2ffdfa, in controller_manager::ControllerManager::robot_description_callback(std_msgs::msg::String_<std::allocator<void> > const&)
[ros2_control_node-2] #8    Object "/opt/ros/jazzy/lib/libcontroller_manager.so", at 0x7c1c3b2fd05b, in controller_manager::ControllerManager::init_resource_manager(std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&)
[ros2_control_node-2] #7    Object "/opt/ros/jazzy/lib/libcontroller_manager.so", at 0x7c1c3b2c1020, in 
[ros2_control_node-2] #6    Object "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.33", at 0x7c1c3adc3390, in __cxa_throw
[ros2_control_node-2] #5    Object "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.33", at 0x7c1c3adada54, in std::terminate()
[ros2_control_node-2] #4    Object "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.33", at 0x7c1c3adc30d9, in 
[ros2_control_node-2] #3    Object "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.33", at 0x7c1c3adadff4, in 
[ros2_control_node-2] #2    Object "/usr/lib/x86_64-linux-gnu/libc.so.6", at 0x7c1c3aaf08fe, in abort
[ros2_control_node-2] #1    Object "/usr/lib/x86_64-linux-gnu/libc.so.6", at 0x7c1c3ab0d27d, in raise
[ros2_control_node-2] #0    Object "/usr/lib/x86_64-linux-gnu/libc.so.6", at 0x7c1c3ab66b2c, in pthread_kill
[ros2_control_node-2] Aborted (Signal sent by tkill() 324 0)
[ERROR] [ros2_control_node-2]: process has died [pid 324, exit code -6, cmd '/opt/ros/jazzy/lib/controller_manager/ros2_control_node --ros-args --params-file /tmp/launch_params_sfhwow7h --params-file /root/ws/install/rascl_description/share/rascl_description/config/controllers_csp.yaml'].
[INFO] [spawner-3]: process started with pid [351]
[spawner-3] [INFO] [1784671244.712027449] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...
[INFO] [spawner-4]: process started with pid [366]
[spawner-3] [WARN] [1784671254.723159187] [spawner_joint_state_broadcaster]: Could not contact service /controller_manager/list_controllers
[spawner-3] [INFO] [1784671254.723529987] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...
[spawner-3] [WARN] [1784671264.733076264] [spawner_joint_state_broadcaster]: Could not contact service /controller_manager/list_controllers
[spawner-3] [INFO] [1784671264.733417563] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...
[spawner-4] [WARN] [1784671265.652753885] [ros2_control_controller_spawner_rascl_position_controller]: Failed to acquire lock in 20 seconds. Attempt 1 of 5 failed. Retrying in 3 seconds...



