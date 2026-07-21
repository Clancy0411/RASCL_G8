rascl-container:~/ws$ bash ./rascl_debug.sh 6
waiting for service to become available...
requester: making request: std_srvs.srv.Trigger_Request()

response:
std_srvs.srv.Trigger_Response(success=True, message='Drive 0: physical=0x07/00000111, logical=0x00/00000000, polarity=0x07 | Drive 1: physical=0x03/00000011, logical=0x04/00000100, polarity=0x07 | Drive 2: physical=0x07/00000111, logical=0x00/00000000, polarity=0x07 | Drive 3: physical=0x00/00000000, logical=0x01/00000001, polarity=0x01')

home_all 只会运动 Drive 0-2；预装的 Drive 3 不执行 Home，随后仍会进入 CSP。
waiting for service to become available...
requester: making request: std_srvs.srv.Trigger_Request()

response:
std_srvs.srv.Trigger_Response(success=True, message='Homing completed for required drives; CSP handoff armed: drive0=-875 drive1=-769 drive2=935')

waiting for service to become available...
requester: making request: std_srvs.srv.Trigger_Request()

response:
std_srvs.srv.Trigger_Response(success=True, message='Drive 2 protection: 0x607B position_range=[-2147483648, 2147483647], 0x607D software_limit=[-802816, 802816], 0x6065 following_window=25000 counts, 0x6066 following_timeout=250 ms')
