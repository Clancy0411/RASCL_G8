已恢复 Drive 3 的完整四轴功能。
默认 home_all 现在会自动 Homing Drive 0–3；Drive 3 使用现有的 method=24 / reference input=1，到绿色传感器位置后才允许进入 CSP。
Drive 3 参与 CSP 准入、PDO 插值、OP 状态与 following-error 检查。
ignore_spur_gear_in_csp:=true 仅保留为硬件故障时的紧急三轴回退，默认已改为 false。
新增调试脚本组 15：在 CSP 中用绝对 raw counts 控制 Drive 3，并保持三个机械臂关节当前位置。
使用方法：
T1：1（本次代码更新后首次需要）
T1：4
T2：6 → HOME
T2：7
T3：8
T3：15 → 输入 500000 → SPUR
组 15 的 500000 是 Drive 3 的绝对 0x6064 counts；Home 名义值是 0，不是“增加 500000”。脚本会按当前 direction=1、home_offset_counts=0、1323008 counts/rev 换算为关节角度，经现有 controller + CSP/PDO 发送。URDF 当前 ±π 限制下，500000 合法。
同一 T3 可交替执行：
15 → 14 → 9 → 10 → 15
组 15 会阻止与正在运行的 wp3_tsk1 轨迹抢占，也会清除旧规划授权；每次在组 15 后执行 Task 1，都要重新 14 → 9 → 10。Task 1 会读取新的 spur gear 实时位置并在 Cartesian 轨迹中保持它。