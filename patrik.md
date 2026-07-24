> 历史调试输出，仅用于保留当时报错。当前流程已变化：Drive 0–2 Homing 成功后，
> Drive 3 会相对运动 `-50000 counts` 并用 Method 37 置零；请以
> `WP3_Task1_MinJerk_Debug_Guide_CN.md` 和当前脚本输出为准。

rascl-container:~/ws$ bash ./rascl_debug.sh 6
waiting for service to become available...
requester: making request: std_srvs.srv.Trigger_Request()

response:
std_srvs.srv.Trigger_Response(success=True, message='Drive 0: physical=0x07/00000111, logical=0x00/00000000, polarity=0x07 | Drive 1: physical=0x03/00000011, logical=0x04/00000100, polarity=0x07 | Drive 2: physical=0x07/00000111, logical=0x00/00000000, polarity=0x07 | Drive 3: physical=0x00/00000000, logical=0x01/00000001, polarity=0x01')

home_all 只会运动 Drive 0-2；预装的 Drive 3 不执行 Home，随后仍会进入 CSP。
waiting for service to become available...
requester: making request: std_srvs.srv.Trigger_Request()

response:
std_srvs.srv.Trigger_Response(success=True, message='Homing completed for required drives; CSP handoff armed: drive0=-893 drive1=-738 drive2=761')

waiting for service to become available...
requester: making request: std_srvs.srv.Trigger_Request()

response:
std_srvs.srv.Trigger_Response(success=True, message='Drive 2 protection: 0x607B position_range=[-2147483648, 2147483647], 0x607D software_limit=[-802816, 802816], 0x6065 following_window=16384 counts, 0x6066 following_timeout=48 ms')

rascl-container:~/ws$ bash ./rascl_debug.sh 7
保持 T1 的 Homing bridge 运行；ros2_control 将持续占用当前终端。
Drive 2 映射：direction=1，home_offset_counts=-802816
Drive 3 CSP 映射：direction=1，counts_per_revolution=1323008（不执行 Home）
进入 CSP 后，Home 的 lowerarm_joint 必须仍接近 +1.5708 rad；否则禁止发送目标。
[INFO] [launch]: All log files can be found below /root/.ros/log/2026-07-22-16-53-02-754620-irs-rascl06-291
[INFO] [launch]: Default logging verbosity is set to INFO
[ERROR] [launch]: Caught exception in launch (see debug for traceback): Unable to parse the value of parameter robot_description as yaml. If the parameter is meant to be a string, try wrapping it in launch_ros.parameter_descriptions.ParameterValue(value, value_type=str)
