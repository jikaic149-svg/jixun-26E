# LQR

从 `jixun3/Application/algorithm/lqr_controller.*` 迁移的硬件无关 LQR 算法，包含：

- 四状态平衡车状态向量构造和左右轮 yaw 混合；
- 固定增益 LQR；
- 最多四点的线性增益调度；
- 带输出和增量限幅的增量式 LQR。

本模块不读取 IMU、编码器或电机，不持有任务和全局控制状态。
