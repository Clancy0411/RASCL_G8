rascl-container:~/ws$ bash ./rascl_debug.sh 7
保持 T1 的 Homing bridge 运行；ros2_control 将持续占用当前终端。
Drive 2 映射：direction=1，home_offset_counts=-802816
Drive 3 CSP 映射：direction=-1，counts_per_revolution=1323008，Method 37 会话零位=0 counts
进入 CSP 后，Home 的 lowerarm_joint 必须仍接近 +1.5708 rad；否则禁止发送目标。
[INFO] [launch]: All log files can be found below /root/.ros/log/2026-07-24-12-40-49-662570-irs-rascl06-282
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [robot_state_publisher-1]: process started with pid [286]
[robot_state_publisher-1] [INFO] [1784896849.771630430] [robot_state_publisher]: Robot initialized
[INFO] [ros2_control_node-2]: process started with pid [301]
[ros2_control_node-2] [INFO] [1784896851.790475720] [controller_manager]: Using Steady (Monotonic) clock for triggering controller manager cycles.
[ros2_control_node-2] [INFO] [1784896851.792515209] [controller_manager]: Subscribing to '/robot_description' topic for robot description.
[ros2_control_node-2] [INFO] [1784896851.793730322] [controller_manager]: update rate is 50 Hz
[ros2_control_node-2] [INFO] [1784896851.793742461] [controller_manager]: Overruns handling is : enabled
[ros2_control_node-2] [INFO] [1784896851.793746079] [controller_manager]: Spawning controller_manager RT thread with scheduler priority: 50
[ros2_control_node-2] [INFO] [1784896851.793870280] [controller_manager]: Successful set up FIFO RT scheduling policy with priority 50.
[ros2_control_node-2] [INFO] [1784896851.975000117] [controller_manager]: Received robot description from topic.
[ros2_control_node-2] [INFO] [1784896851.975071540] [controller_manager]: Enforcing command limits is disabled. Command limits from URDF will be ignored.
[ros2_control_node-2] [INFO] [1784896851.978184952] [controller_manager]: Loading hardware 'RasclBotHardware' 
[ros2_control_node-2] [INFO] [1784896851.979135150] [controller_manager]: Loaded hardware 'RasclBotHardware' from plugin 'rascl_hardware_interface/RASCLHardwareInterface'
[ros2_control_node-2] [INFO] [1784896851.979239232] [controller_manager]: Initialize hardware 'RasclBotHardware' 
[ros2_control_node-2] [INFO] [1784896851.983565327] [RASCLHardwareInterface]: Initialized 4 RASCL joints. fake_hardware=false control_mode=csp bridge=127.0.0.1:15001
[ros2_control_node-2] [INFO] [1784896851.983590542] [controller_manager]: Successful initialization of hardware 'RasclBotHardware'
[ros2_control_node-2] [INFO] [1784896851.983799443] [controller_manager]: Activating component 'RasclBotHardware'.
[ros2_control_node-2] [INFO] [1784896851.983809248] [resource_manager]: 'configure' hardware 'RasclBotHardware' 
[ros2_control_node-2] [INFO] [1784896851.983812784] [RASCLHardwareInterface]: Connecting to Faulhaber TCP bridge...
[ros2_control_node-2] [INFO] [1784896851.983926490] [RASCLHardwareInterface]: Connected to Faulhaber bridge.
[ros2_control_node-2] [INFO] [1784896851.984518839] [resource_manager]: Successful 'configure' of hardware 'RasclBotHardware'
[ros2_control_node-2] [INFO] [1784896851.984532293] [resource_manager]: 'activate' hardware 'RasclBotHardware' 
[ros2_control_node-2] [ERROR] [1784896851.984620611] [RASCLHardwareInterface]: ENTER_CSP_ALL failed. response='ERR RuntimeError: CSP handoff rejected: Drive 3 reference is incomplete; run home_all or complete Drive 0-2 home_one so Drive 3 can move +50000 counts and set zero'
[ros2_control_node-2] [ERROR] [1784896851.984668500] [resource_manager]: Failed to 'activate' hardware 'RasclBotHardware'
[ros2_control_node-2] terminate called after throwing an instance of 'std::runtime_error'
[ros2_control_node-2]   what():  Failed to set the initial state of the component : RasclBotHardware to active
[ros2_control_node-2] Stack trace (most recent call last) in thread 327:
[ros2_control_node-2] #17   Object "/usr/lib/x86_64-linux-gnu/ld-linux-x86-64.so.2", at 0xffffffffffffffff, in 
[ros2_control_node-2] #16   Object "/usr/lib/x86_64-linux-gnu/libc.so.6", at 0x7e9491a74c6b, in 
[ros2_control_node-2] #15   Object "/usr/lib/x86_64-linux-gnu/libc.so.6", at 0x7e94919e7aa3, in 
[ros2_control_node-2] #14   Object "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.33", at 0x7e9491c77db3, in 
[ros2_control_node-2] #13   Object "/opt/ros/jazzy/lib/librclcpp.so", at 0x7e9491f694d6, in rclcpp::executors::MultiThreadedExecutor::run(unsigned long)
[ros2_control_node-2] #12   Object "/opt/ros/jazzy/lib/librclcpp.so", at 0x7e9491f3ec69, in rclcpp::Executor::execute_any_executable(rclcpp::AnyExecutable&)
[ros2_control_node-2] #11   Object "/opt/ros/jazzy/lib/librclcpp.so", at 0x7e9491f3e53a, in rclcpp::Executor::execute_subscription(std::shared_ptr<rclcpp::SubscriptionBase>)
[ros2_control_node-2] #10   Object "/opt/ros/jazzy/lib/libcontroller_manager.so", at 0x7e9492219474, in 
[ros2_control_node-2] #9    Object "/opt/ros/jazzy/lib/libcontroller_manager.so", at 0x7e9492182dfa, in controller_manager::ControllerManager::robot_description_callback(std_msgs::msg::String_<std::allocator<void> > const&)
[ros2_control_node-2] #8    Object "/opt/ros/jazzy/lib/libcontroller_manager.so", at 0x7e949218005b, in controller_manager::ControllerManager::init_resource_manager(std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&)
[ros2_control_node-2] #7    Object "/opt/ros/jazzy/lib/libcontroller_manager.so", at 0x7e9492144020, in 
[ros2_control_node-2] #6    Object "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.33", at 0x7e9491c46390, in __cxa_throw
[ros2_control_node-2] #5    Object "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.33", at 0x7e9491c30a54, in std::terminate()
[ros2_control_node-2] #4    Object "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.33", at 0x7e9491c460d9, in 
[ros2_control_node-2] #3    Object "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.33", at 0x7e9491c30ff4, in 
[ros2_control_node-2] #2    Object "/usr/lib/x86_64-linux-gnu/libc.so.6", at 0x7e94919738fe, in abort
[ros2_control_node-2] #1    Object "/usr/lib/x86_64-linux-gnu/libc.so.6", at 0x7e949199027d, in raise
[ros2_control_node-2] #0    Object "/usr/lib/x86_64-linux-gnu/libc.so.6", at 0x7e94919e9b2c, in pthread_kill
[ros2_control_node-2] Aborted (Signal sent by tkill() 301 0)
[ERROR] [ros2_control_node-2]: process has died [pid 301, exit code -6, cmd '/opt/ros/jazzy/lib/controller_manager/ros2_control_node --ros-args --params-file /tmp/launch_params_3hhl4w8r --params-file /root/ws/install/rascl_description/share/rascl_description/config/controllers_csp.yaml'].
[INFO] [spawner-3]: process started with pid [328]
[spawner-3] [INFO] [1784896853.930548034] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...
[INFO] [spawner-4]: process started with pid [343]
[spawner-3] [WARN] [1784896863.943648948] [spawner_joint_state_broadcaster]: Could not contact service /controller_manager/list_controllers
[spawner-3] [INFO] [1784896863.944041816] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...
[spawner-3] [WARN] [1784896873.956783472] [spawner_joint_state_broadcaster]: Could not contact service /controller_manager/list_controllers
[spawner-3] [INFO] [1784896873.957154919] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...
[spawner-4] [WARN] [1784896874.871106937] [ros2_control_controller_spawner_rascl_position_controller]: Failed to acquire lock in 20 seconds. Attempt 1 of 5 failed. Retrying in 3 seconds...
[spawner-3] [WARN] [1784896883.970226855] [spawner_joint_state_broadcaster]: Could not contact service /controller_manager/list_controllers
[spawner-3] [INFO] [1784896883.970598377] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...
[spawner-3] [WARN] [1784896893.983581881] [spawner_joint_state_broadcaster]: Could not contact service /controller_manager/list_controllers
[spawner-3] [INFO] [1784896893.983941339] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...
[spawner-4] [WARN] [1784896897.873849956] [ros2_control_controller_spawner_rascl_position_controller]: Failed to acquire lock in 20 seconds. Attempt 2 of 5 failed. Retrying in 3 seconds...
[spawner-3] [WARN] [1784896903.995032596] [spawner_joint_state_broadcaster]: Could not contact service /controller_manager/list_controllers
[spawner-3] [INFO] [1784896903.995364691] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...

