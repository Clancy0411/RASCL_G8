ascl-container:~/ws$ bash ./rascl_debug.sh 6
waiting for service to become available...
requester: making request: std_srvs.srv.Trigger_Request()

response:
std_srvs.srv.Trigger_Response(success=True, message='Drive 0: physical=0x07/00000111, logical=0x00/00000000, polarity=0x07 | Drive 1: physical=0x03/00000011, logical=0x04/00000100, polarity=0x07 | Drive 2: physical=0x06/00000110, logical=0x01/00000001, polarity=0x07 | Drive 3: physical=0x01/00000001, logical=0x00/00000000, polarity=0x01')

home_all 先 Homing Drive 0-2；成功后 Drive 3 自动相对运动 50000 counts，再把到达位置设为 0 counts。
waiting for service to become available...
requester: making request: std_srvs.srv.Trigger_Request()

response:
std_srvs.srv.Trigger_Response(success=False, message='Home failed: Drive 3: following error during relative motion; statusword=0x3027')
ERROR: home_all 或 Drive 3 参考运动/置零失败；禁止进入 CSP
rascl-container:~/ws$ bash ./rascl_debug.sh 7
保持 T1 的 Homing bridge 运行；ros2_control 将持续占用当前终端。
Drive 2 映射：direction=1，home_offset_counts=-802816
Drive 3 CSP 映射：direction=-1，counts_per_revolution=1323008，Method 37 会话零位=0 counts
进入 CSP 后，Home 的 lowerarm_joint 必须仍接近 +1.5708 rad；否则禁止发送目标。
[INFO] [launch]: All log files can be found below /root/.ros/log/2026-07-24-11-44-35-431526-irs-rascl06-844
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [robot_state_publisher-1]: process started with pid [848]
[robot_state_publisher-1] [INFO] [1784893475.526132205] [robot_state_publisher]: Robot initialized
[INFO] [ros2_control_node-2]: process started with pid [863]
[ros2_control_node-2] [INFO] [1784893477.546060485] [controller_manager]: Using Steady (Monotonic) clock for triggering controller manager cycles.
[ros2_control_node-2] [INFO] [1784893477.548107236] [controller_manager]: Subscribing to '/robot_description' topic for robot description.
[ros2_control_node-2] [INFO] [1784893477.549443588] [controller_manager]: update rate is 50 Hz
[ros2_control_node-2] [INFO] [1784893477.549458045] [controller_manager]: Overruns handling is : enabled
[ros2_control_node-2] [INFO] [1784893477.549462092] [controller_manager]: Spawning controller_manager RT thread with scheduler priority: 50
[ros2_control_node-2] [INFO] [1784893477.549613854] [controller_manager]: Successful set up FIFO RT scheduling policy with priority 50.
[ros2_control_node-2] [INFO] [1784893478.063608142] [controller_manager]: Received robot description from topic.
[ros2_control_node-2] [INFO] [1784893478.063670694] [controller_manager]: Enforcing command limits is disabled. Command limits from URDF will be ignored.
[ros2_control_node-2] [INFO] [1784893478.066692284] [controller_manager]: Loading hardware 'RasclBotHardware' 
[ros2_control_node-2] [INFO] [1784893478.067784818] [controller_manager]: Loaded hardware 'RasclBotHardware' from plugin 'rascl_hardware_interface/RASCLHardwareInterface'
[ros2_control_node-2] [INFO] [1784893478.067884213] [controller_manager]: Initialize hardware 'RasclBotHardware' 
[ros2_control_node-2] [INFO] [1784893478.072466234] [RASCLHardwareInterface]: Initialized 4 RASCL joints. fake_hardware=false control_mode=csp bridge=127.0.0.1:15001
[ros2_control_node-2] [INFO] [1784893478.072491112] [controller_manager]: Successful initialization of hardware 'RasclBotHardware'
[ros2_control_node-2] [INFO] [1784893478.072693668] [controller_manager]: Activating component 'RasclBotHardware'.
[ros2_control_node-2] [INFO] [1784893478.072703324] [resource_manager]: 'configure' hardware 'RasclBotHardware' 
[ros2_control_node-2] [INFO] [1784893478.072706950] [RASCLHardwareInterface]: Connecting to Faulhaber TCP bridge...
[ros2_control_node-2] [INFO] [1784893478.072830994] [RASCLHardwareInterface]: Connected to Faulhaber bridge.
[ros2_control_node-2] [INFO] [1784893478.073441883] [resource_manager]: Successful 'configure' of hardware 'RasclBotHardware'
[ros2_control_node-2] [INFO] [1784893478.073452506] [resource_manager]: 'activate' hardware 'RasclBotHardware' 
[ros2_control_node-2] [ERROR] [1784893478.073542912] [RASCLHardwareInterface]: ENTER_CSP_ALL failed. response='ERR RuntimeError: CSP handoff rejected: Drive 3 reference is incomplete; run home_all or complete Drive 0-2 home_one so Drive 3 can move +50000 counts and set zero'
[ros2_control_node-2] [ERROR] [1784893478.073599932] [resource_manager]: Failed to 'activate' hardware 'RasclBotHardware'
[ros2_control_node-2] terminate called after throwing an instance of 'std::runtime_error'
[ros2_control_node-2]   what():  Failed to set the initial state of the component : RasclBotHardware to active
[ros2_control_node-2] Stack trace (most recent call last) in thread 884:
[ros2_control_node-2] #17   Object "/usr/lib/x86_64-linux-gnu/ld-linux-x86-64.so.2", at 0xffffffffffffffff, in 
[ros2_control_node-2] #16   Object "/usr/lib/x86_64-linux-gnu/libc.so.6", at 0x7c7123649c6b, in 
[ros2_control_node-2] #15   Object "/usr/lib/x86_64-linux-gnu/libc.so.6", at 0x7c71235bcaa3, in 
[ros2_control_node-2] #14   Object "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.33", at 0x7c712384cdb3, in 
[ros2_control_node-2] #13   Object "/opt/ros/jazzy/lib/librclcpp.so", at 0x7c7123b3e4d6, in rclcpp::executors::MultiThreadedExecutor::run(unsigned long)
[ros2_control_node-2] #12   Object "/opt/ros/jazzy/lib/librclcpp.so", at 0x7c7123b13c69, in rclcpp::Executor::execute_any_executable(rclcpp::AnyExecutable&)
[ros2_control_node-2] #11   Object "/opt/ros/jazzy/lib/librclcpp.so", at 0x7c7123b1353a, in rclcpp::Executor::execute_subscription(std::shared_ptr<rclcpp::SubscriptionBase>)
[ros2_control_node-2] #10   Object "/opt/ros/jazzy/lib/libcontroller_manager.so", at 0x7c7123dee474, in 
[ros2_control_node-2] #9    Object "/opt/ros/jazzy/lib/libcontroller_manager.so", at 0x7c7123d57dfa, in controller_manager::ControllerManager::robot_description_callback(std_msgs::msg::String_<std::allocator<void> > const&)
[ros2_control_node-2] #8    Object "/opt/ros/jazzy/lib/libcontroller_manager.so", at 0x7c7123d5505b, in controller_manager::ControllerManager::init_resource_manager(std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&)
[ros2_control_node-2] #7    Object "/opt/ros/jazzy/lib/libcontroller_manager.so", at 0x7c7123d19020, in 
[ros2_control_node-2] #6    Object "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.33", at 0x7c712381b390, in __cxa_throw
[ros2_control_node-2] #5    Object "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.33", at 0x7c7123805a54, in std::terminate()
[ros2_control_node-2] #4    Object "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.33", at 0x7c712381b0d9, in 
[ros2_control_node-2] #3    Object "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.33", at 0x7c7123805ff4, in 
[ros2_control_node-2] #2    Object "/usr/lib/x86_64-linux-gnu/libc.so.6", at 0x7c71235488fe, in abort
[ros2_control_node-2] #1    Object "/usr/lib/x86_64-linux-gnu/libc.so.6", at 0x7c712356527d, in raise
[ros2_control_node-2] #0    Object "/usr/lib/x86_64-linux-gnu/libc.so.6", at 0x7c71235beb2c, in pthread_kill
[ros2_control_node-2] Aborted (Signal sent by tkill() 863 0)
[ERROR] [ros2_control_node-2]: process has died [pid 863, exit code -6, cmd '/opt/ros/jazzy/lib/controller_manager/ros2_control_node --ros-args --params-file /tmp/launch_params_ijqyg98h --params-file /root/ws/install/rascl_description/share/rascl_description/config/controllers_csp.yaml'].
[INFO] [spawner-3]: process started with pid [890]
[spawner-3] [INFO] [1784893479.644353716] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...
[INFO] [spawner-4]: process started with pid [905]
[spawner-3] [WARN] [1784893489.658031255] [spawner_joint_state_broadcaster]: Could not contact service /controller_manager/list_controllers
[spawner-3] [INFO] [1784893489.658404721] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...
[spawner-3] [WARN] [1784893499.670048841] [spawner_joint_state_broadcaster]: Could not contact service /controller_manager/list_controllers
[spawner-3] [INFO] [1784893499.670387618] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...
[spawner-4] [WARN] [1784893500.666693850] [ros2_control_controller_spawner_rascl_position_controller]: Failed to acquire lock in 20 seconds. Attempt 1 of 5 failed. Retrying in 3 seconds...
[spawner-3] [WARN] [1784893509.680543068] [spawner_joint_state_broadcaster]: Could not contact service /controller_manager/list_controllers
[spawner-3] [INFO] [1784893509.680863261] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...
[spawner-3] [WARN] [1784893519.691885592] [spawner_joint_state_broadcaster]: Could not contact service /controller_manager/list_controllers
[spawner-3] [INFO] [1784893519.692217516] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...
[spawner-4] [WARN] [1784893523.682091705] [ros2_control_controller_spawner_rascl_position_controller]: Failed to acquire lock in 20 seconds. Attempt 2 of 5 failed. Retrying in 3 seconds...
[spawner-3] [WARN] [1784893529.703089521] [spawner_joint_state_broadcaster]: Could not contact service /controller_manager/list_controllers
[spawner-3] [INFO] [1784893529.703375671] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...
[spawner-3] [WARN] [1784893539.712631519] [spawner_joint_state_broadcaster]: Could not contact service /controller_manager/list_controllers
[spawner-3] [INFO] [1784893539.712967957] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...
[spawner-4] [WARN] [1784893546.721525604] [ros2_control_controller_spawner_rascl_position_controller]: Failed to acquire lock in 20 seconds. Attempt 3 of 5 failed. Retrying in 3 seconds...
[spawner-3] [WARN] [1784893549.721597081] [spawner_joint_state_broadcaster]: Could not contact service /controller_manager/list_controllers
[spawner-3] [INFO] [1784893549.721923205] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...
[spawner-3] [WARN] [1784893559.729927657] [spawner_joint_state_broadcaster]: Could not contact service /controller_manager/list_controllers
[spawner-3] [INFO] [1784893559.730265044] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...
[spawner-3] [WARN] [1784893569.740620464] [spawner_joint_state_broadcaster]: Could not contact service /controller_manager/list_controllers
[spawner-3] [INFO] [1784893569.740955044] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...
[spawner-4] [WARN] [1784893569.751997235] [ros2_control_controller_spawner_rascl_position_controller]: Failed to acquire lock in 20 seconds. Attempt 4 of 5 failed. Retrying in 3 seconds...


