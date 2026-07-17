# Car Mode

这里保留的是可复用的车辆行为逻辑，不是任务流程。`Idle`、`Straight`、
`Turning`、`Curve`、`Brake`、`Done` 六个模块互不切换；产品 Mode 组装输入、
选择当前 Mode，并处理下一状态。`Task` 不在本目录中，也不是这些 Mode 的依赖。

## 边界

- C 接口统一采用 `Context + Input -> Output`，状态只存在调用方持有的 Context 中。
- Mode 只依赖 `Algorithm_Control`，不包含 BSP、Device、FreeRTOS、Task 或 MCU 头文件。
- `now_ms` 和 `dt_seconds` 都由调用方明确传入；没有“调用一次等于固定毫秒”的假设。
- 输出只是左右轮控制意图。`motion_requested` 不是电机使能许可；产品 Mode 必须默认保持电机锁，
  完成离线保护、限幅、急停和状态仲裁之后，才能把意图交给 Device/BSP。
- 任一 `Update` 返回 `false` 时，输出会被置为安全零值，Context 不推进。
- 再次进入一个 Mode 时重新调用对应 `Init`，不要复用上次运行的积分或完成状态。

## 输入约定

角度单位是度，角度误差使用 `[-180, 180)` 的最短路径。Curve 的
`measured_heading_rate_deg_per_second` 必须采用统一物理方向：正值表示
`measured_heading_deg` 正在增加。如果某个 IMU 的原始角速度极性相反，转换应放在
Device/Task 适配处，不能把具体传感器极性带进 Mode。

左右轮输出单位由产品 Mode 决定，但同一次调用中的基础值、修正值、速度阈值必须使用
同一单位。Mode 不直接假定 PWM 的 0–10000 范围；最终范围由 Device 安全层限制。

## 默认行为

| Mode | 默认行为 |
|---|---|
| Idle | 持续输出零，不报告任务完成 |
| Straight | 航向 PID `1.5/0/0.2`，修正绝对上限 200，同时不超过基础值的 50% |
| Turning | PID `1.5/0.5/2.0`，最大 150、最小 50，45° 内减速，0.5° 到位后锁存完成并保持零输出 |
| Curve | 航向外环 `Kp=100`、最大 160°/s、约 20 ms 更新；角速度内环 `1.5/0.1`、最大差速 250；差速斜率 700/s |
| Brake | 双轮绝对速度均小于 10 并持续约 25 ms 后完成；500 ms 仍未停稳则报告超时；始终输出零目标 |
| Done | 永久零输出并报告终态 |

Straight/Turning 的公开 `Kp/Ki/Kd` 保留原工程按控制周期整定的数值，默认
`gain_reference_seconds=0.005`。初始化时会把 `Ki/Kd` 换算为按秒的内部系数，
所以在 5 ms 周期保持原有离散响应，在周期抖动或调整时仍按真实 `dt_seconds`
推进。`integral_output_limit` 表示最终 I 项的上限，初始化时会换算为
`Algorithm_Control_PID` 使用的原始积分状态上限；Curve 内环的 100 因此不会被
错误缩小为 10。

共享 PID 在第一次更新时不制造 derivative kick，这是相对旧实现保留的一项安全
优化；重新进入 Mode 仍必须重新 Init，避免继承旧积分和微分历史。

Brake 的毫秒超时使用无符号差值，支持 `uint32_t` 回绕。调用间隔必须小于
`2^31` ms，这是标准回绕比较的边界。

## 验证

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\run_gcc_tests.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\run_arm_compile.ps1
```

若 Windows Smart App Control 阻止在模板目录执行新生成的测试程序，可为第一个脚本传入
策略允许的临时目录，例如 `-OutputDirectory D:\Project\.mode-tests`。脚本结束后会删除
生成的可执行文件。
