# Device/Sensor/IMU/JY61P

该模块解析 JY61P 的 11 字节 `0x51` 加速度、`0x52` 角速度和 `0x53` 欧拉角帧。

- 校验和错误不提交传感器快照，并使用饱和错误计数。
- 损坏帧末尾的 `0x55` 会保留为下一候选帧头，缩短恢复时间。
- 16 位有符号值不依赖实现定义转换。
- 第一帧角度自动建立 Yaw 零点，也支持显式 Tare。
- Yaw 输出归一化到 `[-180, 180)`。
- 离线判断使用无符号时间差，支持 `uint32_t` 毫秒计数回绕。

测试：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\Device\Sensor\IMU\JY61P\tests\run_gcc_tests.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File .\Device\Sensor\IMU\JY61P\tests\run_arm_compile.ps1
```

