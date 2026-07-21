Drive 3 已恢复为四轴 CSP 的正常关节：它参与自动 Homing、CSP 准入、PDO 状态检查和轨迹保持。

完整启动：T1 组 4；T2 组 6 输入 HOME；T2 组 7；T3 组 8。Drive 3 的绿色参考开关也必须在 Home 时成功。

控制夹爪：在组 7 持续运行、两个 controller 都是 active 且没有 wp3_tsk1 轨迹节点时，在 T3 执行：

```bash
bash ./rascl_debug.sh 15
```

输入的是 Drive 3 绝对 raw `0x6064` counts，Home 名义值为 0；例如 `500000` 不是增量。
脚本使用相同的 direction、home offset 和 counts-per-revolution 换算，经
`/rascl_position_controller/commands` 发送 CSP 指令，并保持三个机械臂关节当前位置。

组 15 会清除旧的 Task 1 规划授权；之后要运行 Task 1 必须重新执行 `14 → 9 → 10`。
Task 1 会读取该 spur gear 实时位置并保持它，因此可在同一 CSP 会话中交替执行：
`15 → 14 → 9 → 10 → 15`。
