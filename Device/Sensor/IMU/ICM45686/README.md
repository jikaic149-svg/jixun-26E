# ICM-45686

板载 ICM-45686 的 MCU 无关 Device。依据用户提供的 TDK/InvenSense
`DS-000489 Rev 1.1` 实现 4-wire SPI 寄存器初始化、WHO_AM_I 校验、14 字节
同步采样、物理量换算和 `Device_IMU`/`Algorithm_Attitude` 接入。

默认配置面向平衡车：Accel ±4 g、Gyro ±1000 dps、两者 800 Hz Low-Noise，
INT1 为 push-pull、active-high、pulse Data Ready。WHO_AM_I 为 `0xE9`；上电后
等待至少 1 ms，软复位后再次校验，启用陀螺后等待 35 ms。

数据寄存器从 `0x00` 开始依次为 Accel XYZ、Gyro XYZ、Temperature，每项
16 bit。器件复位默认是 little-endian，本驱动不修改间接的
`SREG_DATA_ENDIAN_SEL`，因此 Init 只接受 little-endian；独立解码函数保留
big-endian 测试能力。温度公式为 `raw / 128 + 25 °C`。SPI backend 必须按
MSB first、上升沿采样/下降沿改变、CS 低有效实现，读取首字节为
`0x80 | register`，频率不得超过 24 MHz。

G3519 原理图连接为 SPI1：PB16 SCLK、PB15 PICO/MOSI、PB14 POCI/MISO、
PB12 CS1、PB22 INT1、PB28 INT2。具体 SPI/GPIO/DMA/IRQ 实例属于 BSP 和
SysConfig，不进入本 Device；当前工程 `.syscfg` 未应用这些推荐项。
