rascl-container:~/ws$ bash ./rascl_debug.sh 6
waiting for service to become available...
requester: making request: std_srvs.srv.Trigger_Request()

response:
std_srvs.srv.Trigger_Response(success=True, message='Drive 0: physical=0x07/00000111, logical=0x00/00000000, polarity=0x07 | Drive 1: physical=0x03/00000011, logical=0x04/00000100, polarity=0x07 | Drive 2: physical=0x07/00000111, logical=0x00/00000000, polarity=0x07 | Drive 3: physical=0x01/00000001, logical=0x00/00000000, polarity=0x01')

home_all 先 Homing Drive 0-2；成功后 Drive 3 自动相对运动 50000 counts，再把到达位置设为 0 counts。
waiting for service to become available...
requester: making request: std_srvs.srv.Trigger_Request()

response:
std_srvs.srv.Trigger_Response(success=False, message='Home failed: Drive 3: following error during relative motion; statusword=0x3027')
ERROR: home_all 或 Drive 3 参考运动/置零失败；禁止进入 CSP
rascl-container:~/ws$ bash ./rascl_debug.sh 17
waiting for service to become available...
requester: making request: std_srvs.srv.Trigger_Request()

response:
std_srvs.srv.Trigger_Response(success=False, message='Drive 3: absolute_counts=207421, statusword=0x1427, mode=1, source=SDO, reference_complete=false, reference_delta=50000, pre_zero_raw=None; zero reference is not complete')
ERROR: Drive 3 counts 已读取，但本次零位参考尚未成功；禁止进入 CSP

