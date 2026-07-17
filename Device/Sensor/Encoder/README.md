# Device/Sensor/Encoder

`Device_Encoder` 读取 BSP 的非破坏性累计 snapshot 和消费式 delta，使用显式 `dt` 计算转数、距离与速度。

- 每个 BSP encoder handle 只能有一个 delta 消费者。
- snapshot 会在破坏性 delta 读取之前验证，snapshot 失败不会丢失 delta。
- 方向反转在 `INT64_MIN` 处安全失败，不执行未定义的整数取负。
- 数值换算失败时 Device 快照保持不变。
- 机械方向、每圈计数与每圈距离由 Product 配置。

测试：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\Device\Sensor\Encoder\tests\run_gcc_tests.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File .\Device\Sensor\Encoder\tests\run_arm_compile.ps1
```

