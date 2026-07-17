# Spin

原地匀速旋转 Mode。`spin_drive` 使用调用方选择的轮速/驱动单位：正值输出
`left=-spin_drive, right=+spin_drive`，负值反向旋转，零值停止。

`maximum_drive` 对输入做对称限幅。该 Mode 不读取编码器或 IMU、不判断目标角度、
不做加减速，也不解锁电机；这些职责由 Task、控制器和 Device 安全层承担。
