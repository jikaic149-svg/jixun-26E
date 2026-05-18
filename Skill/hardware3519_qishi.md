---
trigger: always_on
---

# Hardware Skill: MSPM0G3519 V2.0 启是科技开发板 引脚映射与硬件规范

作为底层开发助手，在进行 SysConfig 配置、引脚分配或修改 `bsp_` 层代码时，必须严格遵守以下基于 **MSPM0G3519 (Core SOM_V2.0 + EVM_V2.0)** 的硬件约束。

## 1. 绝对禁止系统保留引脚 (System Prohibited Pins)
**[FATAL ERROR] 绝对禁止修改、复用或外部强制拉高/拉低（涉及启动与调试）：**
* **PA19 (SWCLK):** 调试时钟线，复用会导致下载器彻底断开，无法烧录。
* **PA20 (SWDIO):** 调试数据线，复用会导致下载器彻底断开，无法烧录。
* **NRST (NRST):** 硬件全局复位脚，禁止外部接其他电平。
* **PA18 (BOOT):** 启动模式控制脚，上电瞬间若为高电平将进入 BSL 刷机模式，导致程序不运行。

## 2. 核心板自带资源 (Core Onboard Resources)
核心板自带的板载外设（即使不插在 EVM 底板上也能独立使用的资源）：
* **PA14:** 蓝色 LED1（普通推挽输出，高电平亮）
* **PA17:** 橙色 LED2（普通推挽输出，高电平亮）
* **PA29:** SK6812_RGB 智能灯珠控制引脚（若不用，务必设为推挽输出低电平，防闪烁）
* **PB31:** 侧边用户按键 (USER_KEY，按下为低电平，释放高电平)
* **PA10 / PA11:** UART0_TX / UART0_RX（连接板载 Type-C CH340，用于 Printf 调试/BSL）
* **PA30:** 预留 TFT 背光控制引脚

### 核心板六轴姿态传感器 (IMU) SPI 总线
* **PB12:** 姿态传感器片选 (IMU_CS)
* **PB14:** 姿态传感器 SPI_MISO
* **PB15:** 姿态传感器 SPI_MOSI
* **PB16:** 姿态传感器 SPI_CLK
* **PB21:** 姿态传感器中断 1 (IMU_INT1) *(注：与 ADC A1_8 复用)*
* **PB27:** 姿态传感器中断 2 (IMU_INT2) *(注：与矩阵键盘 V4 复用)*

---

## 3. EVM 扩展底板引脚映射全表
若插在 EVM 底板上使用，请严格按照以下分配进行代码初始化：

### EVM_A 区 (基础通信与流水灯)
* **UART1:** PB4(TX1) / PB5(RX1)
* **I2C:** PA1(SCL) / PA0(SDA)
* **DAC:** PA15 (DAC 输出)
* **蜂鸣器:** PB1 (高电平响)
* **8-bit LED:** PB0(Bit0), PB26(Bit3), PB28(Bit4), PB29(Bit5), PB30(Bit6), PC6(Bit7), PC7(Bit8)

### EVM_B 区 (拨码开关与辅助通信)
* **UART2:** PB10(TX2) / PB11(RX2)
* **扩展定时器:** PA12(TIM1) / PA13(TIM2)
* **SPI FLASH:** PB13 (片选)
* **8位拨码开关:** PA7(SW1), PA8(SW2), PA9(SW3), PA12(SW4/复用TIM1), PA13(SW5/复用TIM2), PA16(SW6/复用TIM3), PA22(SW7/复用TIM4), PB22(SW8)

### EVM_C 区 (PWM 控制与 ADC 采样)
* **普通定时器:** PA16(TIM3), PA22(TIM4)
* **高级定时器 TIMA0 (PWM 及其互补):** 
  * 通道0: PC2(CH0) / PC3(CH0N)
  * 通道1: PC4(CH1) / PC5(CH1N)
  * 通道2: PC0(CH2) / PC1(CH2N)
  * 通道3: PA28(CH3) / PA31(CH3N)
* **ADC0 采样通道:** PA27(A0_0), PA26(A0_1), PA25(A0_2), PA24(A0_3)
* **ADC1 采样通道:** PB17(A1_4), PB18(A1_5), PB19(A1_6), PB21(A1_8/复用IMU_INT1)

### EVM_D 区 (OLED 屏幕与矩阵键盘)
* **OLED SPI 驱动:** PB3(CLK), PB2(MOSI), PC8(DC 数据/命令切换), PC9(CS 片选), PB23(RES 硬件复位)
* **矩阵键盘 (4x4):**
  * **行 (Row):** PB6(H1), PB7(H2), PB8(H3), PB9(H4)
  * **列 (Col):** PB20(V1), PB24(V2), PB25(V3), PB27(V4/复用IMU_INT2)

---

## 4. 硬件开发与调试避坑指南 (Hardware Precautions)
1. **供电问题：** 仿真器 (J-Link) 的 VTref 引脚只用于检测参考电压，**不能**给开发板供电。板子必须连接 Type-C 独立供电。
2. **烧录协议：** 必须采用 SWD 协议，若提示找不到核心，检查引脚是否接反，或在软件中降低时钟频率 (如改至 1000kHz)。
3. **空工程表现：** 烧录 empty 空工程后，除电源灯外其他外设无反应属正常现象。若 RGB 亮瞎眼白光，见核心板资源 PA29 说明，务必配置为推挽低电平。
4. **编译环境要求：** **禁止**将官方 SDK 的例程单独复制到桌面编译，必须在 SDK 原相对路径下开发，否则 Sysconfig 与头文件路径必定报错。
5. **引脚驱动力：** 芯片引脚直接驱动能力极弱，若需驱动舵机或大功率电机，必须使用类似 TB6612 的外部驱动板，严禁将 PWM 引脚直连电机引脚。
6. **引脚复用冲突风险：** 底板拨码开关 SW4/SW5/SW6/SW7 与 TIM1/TIM2/TIM3/TIM4 复用；矩阵键盘 V4 与 IMU_INT2 复用。使用时务必注意分时或避开冲突。

---

## 5. 全量引脚元数据 (Hardware Metadata)
```json
{
  "device": "MSPM0G3519 (Core SOM_V2.0 + EVM_V2.0)",
  "pin_map": {
    "PA0":  {"func": "I2C_SDA", "peripheral": "EVM_A", "status": "usable_ready"},
    "PA1":  {"func": "I2C_SCL", "peripheral": "EVM_A", "status": "usable_ready"},
    "PA7":  {"func": "SW1", "peripheral": "EVM_B", "status": "usable_ready"},
    "PA8":  {"func": "SW2", "peripheral": "EVM_B", "status": "usable_ready"},
    "PA9":  {"func": "SW3", "peripheral": "EVM_B", "status": "usable_ready"},
    "PA10": {"func": "UART0_TX", "peripheral": "CORE", "status": "usable_ready"},
    "PA11": {"func": "UART0_RX", "peripheral": "CORE", "status": "usable_ready"},
    "PA12": {"func": "TIM1 / SW4", "peripheral": "EVM_B", "status": "conflict_warning"},
    "PA13": {"func": "TIM2 / SW5", "peripheral": "EVM_B", "status": "conflict_warning"},
    "PA14": {"func": "LED1 (Blue)", "peripheral": "CORE", "status": "usable_ready"},
    "PA15": {"func": "DAC_OUT", "peripheral": "EVM_A", "status": "usable_ready"},
    "PA16": {"func": "TIM3 / SW6", "peripheral": "EVM_B/C", "status": "conflict_warning"},
    "PA17": {"func": "LED2 (Orange)", "peripheral": "CORE", "status": "usable_ready"},
    "PA18": {"func": "BOOT", "status": "not_recommended", "warning": "Bootloader Conflict"},
    "PA19": {"func": "SWCLK", "status": "unusable_system_reserved"},
    "PA20": {"func": "SWDIO", "status": "unusable_system_reserved"},
    "PA22": {"func": "TIM4 / SW7", "peripheral": "EVM_B/C", "status": "conflict_warning"},
    "PA24": {"func": "ADC0_3", "peripheral": "EVM_C", "status": "usable_ready"},
    "PA25": {"func": "ADC0_2", "peripheral": "EVM_C", "status": "usable_ready"},
    "PA26": {"func": "ADC0_1", "peripheral": "EVM_C", "status": "usable_ready"},
    "PA27": {"func": "ADC0_0", "peripheral": "EVM_C", "status": "usable_ready"},
    "PA28": {"func": "TIMA0_CH3", "peripheral": "EVM_C", "status": "usable_ready"},
    "PA29": {"func": "RGB_DIN", "peripheral": "CORE", "status": "usable_ready", "warning": "Keep output LOW if unused"},
    "PA30": {"func": "TFT_BLK", "peripheral": "CORE", "status": "usable_ready"},
    "PA31": {"func": "TIMA0_CH3N", "peripheral": "EVM_C", "status": "usable_ready"},
    "PB0":  {"func": "LED0_Bit0", "peripheral": "EVM_A", "status": "usable_ready"},
    "PB1":  {"func": "BUZZER", "peripheral": "EVM_A", "status": "usable_ready"},
    "PB2":  {"func": "OLED_MOSI", "peripheral": "EVM_D", "status": "usable_ready"},
    "PB3":  {"func": "OLED_CLK", "peripheral": "EVM_D", "status": "usable_ready"},
    "PB4":  {"func": "UART1_TX", "peripheral": "EVM_A", "status": "usable_ready"},
    "PB5":  {"func": "UART1_RX", "peripheral": "EVM_A", "status": "usable_ready"},
    "PB6":  {"func": "KEY_H1", "peripheral": "EVM_D", "status": "usable_ready"},
    "PB7":  {"func": "KEY_H2", "peripheral": "EVM_D", "status": "usable_ready"},
    "PB8":  {"func": "KEY_H3", "peripheral": "EVM_D", "status": "usable_ready"},
    "PB9":  {"func": "KEY_H4", "peripheral": "EVM_D", "status": "usable_ready"},
    "PB10": {"func": "UART2_TX", "peripheral": "EVM_B", "status": "usable_ready"},
    "PB11": {"func": "UART2_RX", "peripheral": "EVM_B", "status": "usable_ready"},
    "PB12": {"func": "IMU_CS", "peripheral": "CORE", "status": "usable_ready"},
    "PB13": {"func": "FLASH_CS", "peripheral": "EVM_B", "status": "usable_ready"},
    "PB14": {"func": "IMU_MISO", "peripheral": "CORE", "status": "usable_ready"},
    "PB15": {"func": "IMU_MOSI", "peripheral": "CORE", "status": "usable_ready"},
    "PB16": {"func": "IMU_CLK", "peripheral": "CORE", "status": "usable_ready"},
    "PB17": {"func": "ADC1_4", "peripheral": "EVM_C", "status": "usable_ready"},
    "PB18": {"func": "ADC1_5", "peripheral": "EVM_C", "status": "usable_ready"},
    "PB19": {"func": "ADC1_6", "peripheral": "EVM_C", "status": "usable_ready"},
    "PB20": {"func": "KEY_V1", "peripheral": "EVM_D", "status": "usable_ready"},
    "PB21": {"func": "ADC1_8 / IMU_INT1", "peripheral": "CORE/EVM_C", "status": "conflict_warning"},
    "PB22": {"func": "SW8", "peripheral": "EVM_B", "status": "usable_ready"},
    "PB23": {"func": "OLED_RES", "peripheral": "EVM_D", "status": "usable_ready"},
    "PB24": {"func": "KEY_V2", "peripheral": "EVM_D", "status": "usable_ready"},
    "PB25": {"func": "KEY_V3", "peripheral": "EVM_D", "status": "usable_ready"},
    "PB26": {"func": "LED3_Bit3", "peripheral": "EVM_A", "status": "usable_ready"},
    "PB27": {"func": "KEY_V4 / IMU_INT2", "peripheral": "CORE/EVM_D", "status": "conflict_warning"},
    "PB28": {"func": "LED4_Bit4", "peripheral": "EVM_A", "status": "usable_ready"},
    "PB29": {"func": "LED5_Bit5", "peripheral": "EVM_A", "status": "usable_ready"},
    "PB30": {"func": "LED6_Bit6", "peripheral": "EVM_A", "status": "usable_ready"},
    "PB31": {"func": "USER_KEY", "peripheral": "CORE", "status": "usable_ready"},
    "PC0":  {"func": "TIMA0_CH2", "peripheral": "EVM_C", "status": "usable_ready"},
    "PC1":  {"func": "TIMA0_CH2N", "peripheral": "EVM_C", "status": "usable_ready"},
    "PC2":  {"func": "TIMA0_CH0", "peripheral": "EVM_C", "status": "usable_ready"},
    "PC3":  {"func": "TIMA0_CH0N", "peripheral": "EVM_C", "status": "usable_ready"},
    "PC4":  {"func": "TIMA0_CH1", "peripheral": "EVM_C", "status": "usable_ready"},
    "PC5":  {"func": "TIMA0_CH1N", "peripheral": "EVM_C", "status": "usable_ready"},
    "PC6":  {"func": "LED7_Bit7", "peripheral": "EVM_A", "status": "usable_ready"},
    "PC7":  {"func": "LED8_Bit8", "peripheral": "EVM_A", "status": "usable_ready"},
    "PC8":  {"func": "OLED_DC", "peripheral": "EVM_D", "status": "usable_ready"},
    "PC9":  {"func": "OLED_CS", "peripheral": "EVM_D", "status": "usable_ready"},
    "NRST": {"func": "NRST", "status": "unusable_system_reserved"}
  }
}
```

## 6. 机械与外设件规格 (Mechanical & Peripheral Specs)
### 6.1 MG310 电机与 GMR 编码器参数
* **型号**: MG310电机 (带塑料后盖保护的GMR编码器)
* **接口**: PH2.0 6p 端子线
* **减速比**: 标称 1:20，精确值为 **1:20.409**（建议在里程计等精确定位解算中使用此值减小累计误差）。
* **重量**: 70g

#### 6.1.1 驱动性能参数
* **额定电压**: 7.4V (建议 7~13V，通常2S航模电池或升压模块)。
* **空载状态**: 500±13% RPM, 电流 ≤220mA
* **额定运行**: 400±13% RPM, 电流 ≤500mA, 扭矩 0.4 Kgf.cm
* **极限(堵转)**: 电流 ≤2.0A, 扭矩 1.5± Kgf.cm
* **⚠️严禁堵转**: 不可长时间卡死。PID 或电流环需要做限幅与过载保护。

#### 6.1.2 GMR 编码器特性
* **传感器**: 巨磁阻效应 (GMR) 编码器，供电 3.3V-5V (可直连 MCU 3.3V)。
* **物理线数**: 500 ppr (转子转一圈 500 个脉冲)。
* **减速后输出线数**: 理论每输出轴一圈产生 $500 \times 20.409 \approx 10204.5$ 个脉冲边缘 (高精度)。
* **电气接口**: **自带硬件上拉电阻** (默认上拉至VCC)。即使 MCU 设置为浮空输入也能读到方波，但建议配置 `PULL_UP` 以增加抗干扰能力。
* **中断警告**: 因为高频脉冲在电机高速转动时频率极高（普通中断处理必然丢步），必须依赖硬件 QEI 外设（如 TIMG8/TIMG9 的 `DL_TIMER_QEI_MODE_2_INPUT`）处理，目前工程配置完全匹配该需求。
