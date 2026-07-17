# Device/Sensor/IMU

`Device_IMU` 把具体 IMU driver 产生的样本转换成统一的云台反馈快照。它不选择 SPI/I2C、
不访问 BSP、不创建任务，也不在 ISR 中运行滤波。驱动或 BSP 只负责取得寄存器样本，
main/task 串行调用 `Device_IMU_PushSample()`。

## 数据边界

输入是 float sensor units；两个比例参数分别换算到 m/s² 与 deg/s。若上游已经完成比例
换算，保持 scale=1 即可。三个 body 输出轴分别配置 source axis 和 `+1/-1`，并必须形成
X/Y/Z 的排列，从而把任意安装方向统一为右手 body 坐标：roll-X、pitch-Y、yaw-Z，
静止水平加速度为 `+Z`。完整 signed-permutation 映射矩阵的 determinant 必须为 `+1`；
即使三个 source 没有重复，determinant 为 `-1` 的镜像/左手映射也会令
`Device_IMU_Init()` 失败。

Snapshot 同时保留：

- 映射后的加速度、未扣 bias 和已扣 bias 的角速度；
- 四元数、Euler 和加速度校正状态；
- 直接供 Gimbal Product 使用的 `pitch_deg/yaw_deg` 与
  `pitch_rate_dps/yaw_rate_dps`；
- sample timestamp、online、feedback_valid；
- gyro bias 与校准进度。

组装 `Mode_Gimbal_FeedbackInput` 时可直接使用上述四个 Pitch/Yaw 字段，并把
`feedback_valid` 传入 `imu_feedback_valid`。绝对 yaw 仍不可观测；没有磁力计时只能积分，
snapshot 内的 Attitude 标志会明确保持 false。

## Gyro bias calibration

校准由 APP 显式 `StartGyroBiasCalibration()`，或配置启动时自动开始。仅当加速度幅值接近
重力且原始角速度模长低于阈值时才累计；运动会清空这批未完成的连续样本，避免混入动态
数据。达到样本数后取三轴均值，原子更新 Attitude bias，并清空 Mahony integral feedback。

已有 bias 在新校准完成前继续生效。也可用 `SetGyroBiasDps()` 装载持久化结果。
`gyro_bias_valid` 区分“明确校准/配置的零 bias”和仅仅默认值为零。

默认离线期限 100 ms，使用无符号时间差支持 `uint32_t` 回绕。GetSnapshot 不修改 Device，
只根据调用时刻计算返回副本的 online/feedback_valid。

`PushSample()` 在首个样本之后还会检查 timestamp 串行顺序。令
`delta = new_timestamp - previous_timestamp`（`uint32_t` 运算），只接受
`1..0x7fffffff`：因此正常的计数器回绕可以继续推进；重复 timestamp、倒序 timestamp
以及方向不确定的 `delta=0x80000000` 都会被拒绝，Device 状态保持不变。

## Tests

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\run_gcc_tests.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\run_arm_compile.ps1
```
