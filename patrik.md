rascl-container:~/ws$ bash ./rascl_debug.sh 17
waiting for service to become available...
requester: making request: std_srvs.srv.Trigger_Request()

response:
std_srvs.srv.Trigger_Response(success=False, message='Drive 3: absolute_counts=157720, statusword=0x0040, mode=1, source=SDO, reference_complete=false, reference_delta=50000, pre_zero_raw=None; zero reference is not complete')
ERROR: Drive 3 counts 已读取，但本次零位参考尚未成功；禁止进入 CSP


