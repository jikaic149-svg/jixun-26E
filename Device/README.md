# Device 结构

Device 负责“这是什么设备、协议怎样、状态怎样”，不负责 MCU 引脚。

```text
Device/
├─ Input/
│  └─ Key/                    短按、长按、双击
├─ Actuator/
│  ├─ Motor/                  普通方向 + PWM 电机
│  └─ QD4310/                 QD4310 CAN 电机协议
├─ Sensor/
│  ├─ Encoder/                累计值、增量、速度、里程
│  ├─ LineTracking/           循迹标定、差比和、丢线、交叉口
│  └─ IMU/
│     ├─ device_imu.*         通用 IMU 融合与安装方向接口
│     ├─ BMI088/              BMI088 SPI 寄存器驱动
│     ├─ ICM45686/            板载 ICM-45686 SPI 驱动与姿态接入
│     └─ JY61P/               JY61P UART 协议驱动
└─ Communication/
   ├─ Vision/                 视觉数据包通信
   └─ ChassisCommand/         Jixun3 串口命令的无副作用类型化解析
```

`IMU` 不是某个传感器型号。它是 APP 使用的统一接口；`BMI088` 和 `JY61P` 才是
具体设备。这样更换传感器时，姿态算法和云台 Mode 不需要跟着重写。

所有 Device 的实物型号、产品归属、连接参数、软件默认值和待确认项统一记录在
[`docs/DEVICE_PARAMETERS.md`](../docs/DEVICE_PARAMETERS.md)。型号或实物参数没有可靠来源时，
必须保留“待确认”，不得用常见模块参数代替。

模块通过 CMake 选项按需加入编译链，例如：

```text
TEMPLATE_ENABLE_DEVICE_BMI088
TEMPLATE_ENABLE_DEVICE_ICM45686
TEMPLATE_ENABLE_DEVICE_JY61P
TEMPLATE_ENABLE_DEVICE_QD4310
TEMPLATE_ENABLE_DEVICE_LINE_TRACKING
TEMPLATE_ENABLE_DEVICE_CHASSIS_COMMAND
```
