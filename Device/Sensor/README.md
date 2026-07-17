# Device/Sensor

传感器设备负责帧/计数解释、单位换算、快照与在线状态，不直接包含 MCU 外设实例。
# Sensor

- `Encoder/`：编码器累计值、增量、速度与里程。
- `IMU/`：通用 IMU、BMI088 与 JY61P。
- `LineTracking/`：数字/模拟多通道循迹的阈值、权重、位置、偏差、丢线与交叉口识别。
