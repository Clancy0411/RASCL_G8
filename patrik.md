rascl-container:~/ws$ cd /root/ws
bash ./rascl_debug.sh 6
waiting for service to become available...
requester: making request: std_srvs.srv.Trigger_Request()

response:
std_srvs.srv.Trigger_Response(success=True, message='Drive 0: physical=0x07/00000111, logical=0x00/00000000, polarity=0x07; 0x2310 lower/upper/option/reference=0x01/0x04/1/2; 0x2324.01=0x0000000B [none] | Drive 1: physical=0x03/00000011, logical=0x04/00000100, polarity=0x07; 0x2310 lower/upper/option/reference=0x01/0x04/1/2; 0x2324.01=0x0400004B [positive_limit_switch] | Drive 2: physical=0x07/00000111, logical=0x00/00000000, polarity=0x07; 0x2310 lower/upper/option/reference=0x01/0x04/1/2; 0x2324.01=0x0000000B [none] | Drive 3: physical=0x01/00000001, logical=0x00/00000000, polarity=0x01; 0x2310 lower/upper/option/reference=0x00/0x00/1/1; 0x2324.01=0x00000003 [none]')

home_all 先让 Drive 0-2 穿过各自参考输入区间并回到中点置零；成功后 Drive 3 自动相对运动 50000 counts，再把到达位置设为 0 counts。
waiting for service to become available...
requester: making request: std_srvs.srv.Trigger_Request()

response:
std_srvs.srv.Trigger_Response(success=False, message='Home failed: Drive 1: reference input did not become inactive within 100000 counts after the first edge; internal_limit_seen=false')
ERROR: home_all 或 Drive 3 参考运动/置零失败；禁止进入 CSP
