# Device/Actuator/Motor

`Device_Motor` 把 APP 的有符号 `-10000..10000` 命令转换为 BSP 的方向和 PWM 状态。

- 初始化立即向 backend 提交零命令，并保持锁定。
- 解锁前再次向 backend 提交零命令；零命令失败时绝不解锁。
- 重新锁定先改变软件锁，再请求 backend 停止。
- 非零命令提交失败会重新锁定并尽力提交零命令。
- `positive_is_forward` 只适配产品机械方向；引脚和 PWM 极性仍属于 BSP。

测试：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\Device\Actuator\Motor\tests\run_gcc_tests.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File .\Device\Actuator\Motor\tests\run_arm_compile.ps1
```

