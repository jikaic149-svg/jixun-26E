# APP/Algorithm/Attitude

`Algorithm_Attitude` 是无硬件、无 RTOS、无动态内存的 6 轴 Mahony 非线性互补滤波器。
调用方持有完整 Context，每次输入三轴加速度、三轴角速度和真实 `dt_seconds`。

## 坐标与单位

- 右手 body 坐标，陀螺单位 rad/s。
- 静止且水平时，加速度方向必须是 body `+Z`，幅值约 `9.80665 m/s²`。
- 四元数顺序为 `w,x,y,z`；Euler 使用 roll-X、pitch-Y、yaw-Z，单位度。
- 安装方向、轴交换和正负号不属于算法，应由 IMU Device 先统一。

加速度幅值偏离参考重力超过配置容差时，本次只积分陀螺，不使用错误的动态加速度
校正姿态。有效加速度通过重力方向约束 roll/pitch；滤波器没有磁力计，因此绝对 yaw
不可观测。`yaw_absolute_observable` 永远为 false，yaw 只会由陀螺积分，并随残余 Z 轴
bias 漂移。它不能冒充航向传感器。

默认 `Kp=2.0 1/s`、`Ki=0.05 1/s²`、最大 dt 50 ms。配置 bias 在积分前从陀螺减去；
`SetGyroBias()` 可由 Device 静止校准更新，并会清空 Mahony integral feedback。

所有更新会归一化四元数。参数、输入、计算结果非有限，dt 非正或超过上限时返回 false，
Context 与输出保持不变。

## Tests

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\run_gcc_tests.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\run_arm_compile.ps1
```
