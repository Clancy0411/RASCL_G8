rascl-container:~/ws$ bash ./rascl_debug.sh 18
读取 Drive 0-3 输入状态及 0x2310 映射（仅限 CSP 启动前）：
waiting for service to become available...
requester: making request: std_srvs.srv.Trigger_Request()

response:
std_srvs.srv.Trigger_Response(success=True, message='Drive 0: physical=0x05/00000101, logical=0x02/00000010, polarity=0x07; 0x2310 lower/upper/option/reference=0x01/0x04/1/2; 0x2324.01=0x0200101B [none] | Drive 1: physical=0x03/00000011, logical=0x04/00000100, polarity=0x07; 0x2310 lower/upper/option/reference=0x01/0x04/1/2; 0x2324.01=0x04001043 [positive_limit_switch] | Drive 2: physical=0x07/00000111, logical=0x00/00000000, polarity=0x07; 0x2310 lower/upper/option/reference=0x01/0x04/1/2; 0x2324.01=0x00001003 [none] | Drive 3: physical=0x01/00000001, logical=0x00/00000000, polarity=0x01; 0x2310 lower/upper/option/reference=0x00/0x00/1/1; 0x2324.01=0x0000100B [none]')

读取 Drive 2 的 0x607B/0x607D 与 following-error 参数：
waiting for service to become available...
requester: making request: std_srvs.srv.Trigger_Request()

response:
std_srvs.srv.Trigger_Response(success=True, message='Drive 2 protection: 0x607B position_range=[-2147483648, 2147483647], 0x607D software_limit=[-802816, 802816], 0x6065 following_window=16384 counts, 0x6066 following_timeout=48 ms')

