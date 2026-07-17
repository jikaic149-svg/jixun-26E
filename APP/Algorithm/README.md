# Algorithm

放置与 MCU、引脚和具体外设实例无关的可复用组件，例如 FIFO、协议编解码、
滤波和数学算法。每个组件应建立独立 CMake target，并由 Product 按需链接。

模板不包含或链接逐飞 GPL 源码。所需功能依据 TI/器件官方资料重新实现。
公共代码只使用融合后的接口名称，不提供 `zf_*` 兼容 wrapper。
相同功能的实现发生取舍时先由用户确认。

当前模块：

- `SPSC_Ring`：调用方提供存储的单生产者/单消费者字节 ring；通过
  `TEMPLATE_ENABLE_ALGORITHM_SPSC_RING=ON` 显式加入编译链。
- `Packet`：`@1|TYPE|SEQ|LEN|PAYLOAD|CRC16\n` 的独立编解码与增量解析；
  通过 `TEMPLATE_ENABLE_ALGORITHM_PACKET=ON` 显式加入编译链。
- `ReliableLink`：单个在途关键包的 ACK、30 ms 总期限、最多 3 次重发和
  接收端重复抑制；通过 `TEMPLATE_ENABLE_ALGORITHM_RELIABLE_LINK=ON` 显式
  加入编译链，并只自动带入它必需的 `Packet`。
- `Control`：两套 Mode 共用的角度、限幅、PID 和斜率限制基础；通过
  `TEMPLATE_ENABLE_ALGORITHM_CONTROL=ON` 显式加入编译链。
- `Filter`：一阶低通、滑动平均、三点中值、Biquad、角度 Kalman 和标量
  Kalman；通过
  `TEMPLATE_ENABLE_ALGORITHM_FILTER=ON` 显式加入编译链。
- `Attitude`：无硬件、无 RTOS 的 6 轴 Mahony 姿态解算；通过
  `TEMPLATE_ENABLE_ALGORITHM_ATTITUDE=ON` 显式加入编译链，选择通用
  `Device/Sensor/IMU` 时会自动带入。
- `LQR`：固定、增益调度和增量式 LQR；通过
  `TEMPLATE_ENABLE_ALGORITHM_LQR=ON` 显式加入编译链。
- `ADRC`：一阶 ESO 与 PI-ESO；通过
  `TEMPLATE_ENABLE_ALGORITHM_ADRC=ON` 显式加入编译链。
- `CRC`：可重入 CRC-16/IBM 与 Modbus 计算；通过
  `TEMPLATE_ENABLE_ALGORITHM_CRC=ON` 显式加入编译链。

旧工程全局 Mahony 实现没有复制，姿态统一复用 `Attitude`；算法层不包含
HAL、BSP 或 FreeRTOS。

对应关系：逐飞/旧工程的 PID 需求由 `Control` 独立实现，FIFO 由
`SPSC_Ring` 实现，CRC16 由 `Packet` 实现，通用滤波由 `Filter` 实现；不复制
或链接 GPL 源文件。
