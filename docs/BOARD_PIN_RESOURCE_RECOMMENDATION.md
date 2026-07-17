# MSPM0 板级引脚与资源推荐

最后核对日期：2026-07-16

本文同时记录当前配置和剩余推荐。表中状态严格区分：

- **板载占用**：由原理图、LaunchPad 引脚表确认，不能当作普通空闲 GPIO。
- **当前已配置**：当前工程 `.syscfg` 已实际生成的资源。
- **推荐未应用**：TI SysConfig PinMux 数据允许、且结合板载冲突选择的候选方案；
  必须由产品接线确认后再写入 `.syscfg`。

资料来源：

- `LP-MSPM0G3507引脚功能表.xlsx`
- `MCU_MSPM0G3519_V2_0_202601101011_Schematic.pdf`
- TDK/InvenSense `ICM-45686 DS-000489 Rev 1.1`
- TI SysConfig 1.27.1 的 `MSPM0G350X.json`、`MSPM0G351X.json`
- 当前两份 `.syscfg`

## 一、MSPM0G3519 板

### 1. 板载资源和固定连接

| 板载资源 | MCU 引脚/外设 | 占用结论 |
|---|---|---|
| 40 MHz HFXT | PA5 HFXIN、PA6 HFXOUT | 固定时钟，不复用 |
| 32.768 kHz LFXT | PA3 LFXIN、PA4 LFXOUT | 板载晶振占用；即使当前 SysConfig 未启用，也不推荐复用 |
| SWD | PA19 SWDIO、PA20 SWCLK | 调试固定保留 |
| CH340/DAP-Link UART | PA10 UART0_TX、PA11 UART0_RX | 调试串口优先保留 |
| ICM-45686 SPI | PB16 SCLK、PB15 PICO/MOSI、PB14 POCI/MISO | 板载 SPI1 固定连接 |
| ICM-45686 片选/中断 | PB12 CS、PB22 INT1、PB28 INT2 | 板载固定连接 |
| OLED/TFT SPI | PB3 SCLK、PB2 PICO/MOSI | 板载 SPI0 固定连接 |
| OLED/TFT 控制 | PC9 CS、PC8 DC、PB23 RESET、PA30 背光 | 使用显示时全部占用 |
| SK6812 RGB | PA29 / TIMG6_CCP0 | 当前工程已经配置 |
| LED1/LED2 | PA14、PA16 | 板载 LED 占用 |
| 用户按键 | PB31 | 板载按键占用 |
| BOOT 按键 | PA18 | 启动功能占用，不推荐复用 |
| 4×4 键盘行 | PB6、PB7、PB8、PB9 | 使用矩阵键盘时占用 |
| 4×4 键盘列 | PB20、PB21、PB24、PB27 | 使用矩阵键盘时占用 |

原理图没有板载 CAN 收发器。PA12/PA13 只能作为 MCU 侧 CAN_TX/CAN_RX，
不得直接连接 CANH/CANL；实际使用 QD4310 时还需要 3.3 V 兼容 CAN
收发器、正确供电、共地和总线终端电阻。

### 2. 当前 SysConfig 实际状态

| 模块 | 当前设置 | 状态 |
|---|---|---|
| SYSCTL | PA5/PA6 40 MHz HFXT，SYSPLL，CPUCLK 80 MHz，UDIV /2 | 已配置 |
| Board | PA19/PA20 SWD | 已配置 |
| PWM | TIMG6_CCP0 → PA29，计数 100，RGB 发送时启动 | 已配置 |
| 电机 PWM | TIMA0_CCP0 → PC2、CCP1 → PC4，20 kHz | 已配置 |
| 电机方向 | PA24/PA25（左）、PA26/PA27（右） | 已配置 |
| UART1 | PB4 TX、PB5 RX，115200 | 已配置 |
| UART0 | PA10 TX、PA11 RX，115200 | 已配置 |
| UART4 / JY61P | PB10 TX、PB11 RX，9600，RX 上拉 | 已配置 |
| 编码器 | PB29/PB30（左）、PB6/PB7（右），双边沿、上拉 | 已配置 |
| 灰度模块 | PB8 DAT（上拉）、PB9 CLK，8 路同步串行 | 已配置 |
| SPI1 / ICM-45686 | 未加入 `.syscfg` | 未配置 |
| SPI0 / 显示 | 未加入 `.syscfg` | 未配置 |
| CANFD0 | 未加入 `.syscfg` | 未配置 |
| DMA | 未加入 `.syscfg` | 未配置 |
| ICM INT1/INT2 GPIO | 未加入 `.syscfg` | 未配置 |

### 3. 推荐引脚分配（未应用）

| 产品角色 | 推荐外设 | 推荐引脚 | 选择理由与限制 |
|---|---|---|---|
| 板载 ICM-45686 | SPI1 | PB16 SCLK、PB15 MOSI、PB14 MISO | 原理图固定，不可换到其他 SPI |
| ICM 软件片选 | GPIO 输出 | PB12，低有效，上电先置高 | 原理图固定；建议软件 CS，便于一次 burst 保持完整片选周期 |
| ICM Data Ready | GPIO 上升沿 IRQ | PB22 / INT1 | 原理图固定；INT1 推荐作 800 Hz DRDY |
| ICM 辅助中断 | GPIO 输入 | PB28 / INT2 | 先保留，未定义用途前不启用 IRQ |
| QD4310 CAN | CANFD0 | PA12 TX、PA13 RX | 避开 ICM 和键盘；外接 CAN 收发器后才可使用 |
| Vision | UART4 | PB10 TX、PB11 RX | 已被当前 JY61P 占用，不可同时启用 |
| ChassisCommand | UART1 | PB4 TX、PB5 RX | 已被当前蓝牙/主调试口占用，复用时协议必须统一 |
| 8 路串口循迹 | UART3 | PC6 TX、PC7 RX | 模拟量版和数字量版都先在 Device 层解析各自串口协议 |
| 调试日志 | UART0 | PA10 TX、PA11 RX | 已按当前辅助调试口配置，复用板载 CH340/DAP-Link 通道 |
| 激光开关 | GPIO 输出 | PA7 | 只表达开关时使用 GPIO；必须由 Mode 安全门控制 |
| 激光调功（可选） | TIMG7_CCP1 | PA7 | 仅在确认激光驱动允许 PWM 后使用，不与 GPIO 模式同时配置 |
| 板载显示（可选） | SPI0 + GPIO | PB3/PB2、PC9/PC8/PB23/PA30 | 完全按原理图；不用显示时仍需先确认板上电路是否允许复用 |

不推荐 CANFD1 的 PB21/PB22 组合：PB22 已连接 ICM INT1，PB21 属于板载
矩阵键盘列。CANFD0 PA12/PA13 冲突更少。

### 4. 外设参数（当前与推荐）

| SysConfig 模块 | 推荐配置 | 说明 |
|---|---|---|
| SPI1 / ICM-45686 | Controller、8 bit、MSB first、Mode 0、10 MHz | 规格书允许 Mode 0/3，最大 24 MHz；10 MHz 留出板级裕量 |
| SPI1 帧/CS | SCLK+PICO+POCI，PB12 单独 GPIO CS | TI SysConfig 可使用 `MOTO3` 让 SPI 只管三根数据信号；加软件 CS 后，物理连接仍是完整 4-wire SPI |
| ICM INT1 | PB22 输入、上升沿、无内部上下拉 | Device 默认把 INT1 配成 push-pull、active-high、pulse DRDY |
| ICM INT2 | PB28 普通输入、IRQ 关闭 | 用途未确定前不抢占共享 GPIO 中断入口 |
| CANFD0 | PA12/PA13，仲裁段 1 Mbps | QD4310 手册规定 CAN 1 Mbps；具体采样点需结合总线长度和收发器确认 |
| UART0 / 辅助调试 | 115200、8N1 | 当前已应用 |
| UART1 / 蓝牙主调试 | 115200、8N1 | 当前已应用 |
| UART4 / JY61P | 9600、8N1、RX 上拉 | 当前按 JY61P 常用出厂波特率应用；模块改过波特率时必须同步修改 |
| TIMG6 | 保持当前 PA29 SK6812 | 已被 RGB 时序占用，不再分配给控制周期或电机 PWM |

ICM-45686 默认建议为 Accel ±4 g、Gyro ±1000 dps、两者 800 Hz Low-Noise。
FreeRTOS 控制 Task 可以保持 1 kHz，但只在新 DRDY 到达后读取并解算；不得把
同一份 800 Hz 样本重复当作新的 1 kHz 姿态样本。

### 5. 推荐 DMA 分配（未应用）

G3519 有 12 个 DMA 通道，其中 CH0..CH5 优先留给约束更少的 full channel
需求。下表只是统一编号建议，不代表当前已经配置：

| DMA 通道 | 推荐用途 | 优先级 |
|---|---|---|
| CH0 | SPI1 RX，ICM burst | 最高 |
| CH1 | SPI1 TX，ICM 地址/时钟数据 | 最高 |
| CH2 | UART4 RX，Vision | 高 |
| CH3 | UART4 TX，Vision | 高 |
| CH4 | UART3 RX，循迹 | 高 |
| CH5 | UART1 RX，ChassisCommand | 高 |
| CH6 | SPI0 TX，显示刷新 | 中 |
| CH7 | UART0 RX，调试 | 低 |
| CH8..CH11 | CAN 搬运扩展、其他串口或备用 | 预留 |

14 字节 ICM burst 在 10 MHz 下很短。首版 BSP 可以先做有超时的阻塞 SPI，
验证 WHO_AM_I、字节序、量程和安装轴后再切 DMA；没有完成 DMA 状态机时，不应
只在文档或宏名中声称“已启用 DMA”。

### 6. IRQ 和 Task 所有权建议

| 事件 | ISR 职责 | Task/Device 职责 |
|---|---|---|
| ICM INT1 DRDY | 清 PB22 中断、累计序号或通知 IMU Task | SPI burst、单位换算、轴映射和姿态解算 |
| SPI1 DMA 完成 | 只记录完成/错误并通知 Task | 检查完整帧后提交 `Device_ICM45686` |
| UART RX/DMA | 搬运字节、记录溢出 | 对应 Vision/LineTracking/ChassisCommand Device 解析 |
| CAN RX | 收帧入队 | QD4310 Device 解码，Mode 决定是否允许输出 |
| 1 kHz 控制周期 | 唤醒静态 FreeRTOS Task | Task 搬运 Device 快照并调用 Mode |

`GROUP1_IRQHandler` 是 GPIO 聚合入口。未来同时启用按键、ICM 和其他 GPIO
中断时，必须由 BSP 统一分发，不能让多个 `.c` 文件分别定义同名向量。

## 二、LP-MSPM0G3507

### 1. LaunchPad 板载占用与跳线资源

| 引脚/资源 | 板上连接 | 使用约束 |
|---|---|---|
| PA2 | ROSC | 不可作为普通 IO |
| PA3/PA4 | LFXIN/LFXOUT | 晶振占用，不复用 |
| PA5/PA6 | HFXIN/HFXOUT | 晶振占用，不复用 |
| PA19/PA20 | SWDIO/SWCLK | 调试保留 |
| PA21/PA23 | VREF-/VREF+ | 模拟参考保留 |
| PA18 | S1/BSL/SW2 复用 | 启动相关，不推荐作产品 IO |
| PA0 | 5 V open-drain / J4 LED1 | 只有确认电气模式和跳线后才能复用 |
| PA9 或 PB23 | SW1，J14 选择 | 由跳线决定实际占用 |
| PA10/PA11 | XDS UART，J21/J22 | 推荐保留为 UART0 调试 |
| PA16 或 PA18 | SW2，J15 选择 | 由跳线决定；PA18 仍不推荐复用 |
| PA22/PA26/PA27 | 光线传感器 | 断开对应跳线后才可复用 |
| PA29/PA30/PB14 | QEI 接口，J12 默认未焊 | 使用前确认 J12 和外接接口 |
| PB21 | S2 | 板载按键占用 |
| PB22/PB26/PB27 | RGB 蓝/红/绿，J5/J6/J7 | 断开跳线后才可复用 |
| PB24 | 温度传感器，J9 | 断开 J9 后才可复用 |
| PB25 | SMA ADC | 使用前确认 SMA 端接和输入范围 |

### 2. 当前 SysConfig 实际状态

| 模块 | 当前设置 | 状态 |
|---|---|---|
| SYSCTL | 默认时钟，生成 CPUCLK 32 MHz | 已配置 |
| Board | PA19/PA20 SWD | 已配置 |
| UART/SPI/CAN/Timer/DMA/GPIO 业务资源 | 无 | 未配置 |

### 3. 推荐引脚分配（未应用）

| 产品角色 | 推荐外设 | 推荐引脚 | 选择理由与限制 |
|---|---|---|---|
| QD4310 CAN | CANFD0 | PA12 TX、PA13 RX | SysConfig PinMux 允许；仍需外接 CAN 收发器 |
| 调试日志 | UART0 | PA10 TX、PA11 RX | 对应 LaunchPad XDS UART 跳线 |
| Vision | UART1 | PB4 TX、PB5 RX | 与主要板载资源无冲突 |
| 8 路串口循迹 | UART2 | PB15 TX、PB16 RX | 为两种串口循迹 Device 预留，协议参数待实物确认 |
| ChassisCommand | UART3 | PB12 TX、PB13 RX | 与推荐 CAN/SPI 无冲突 |
| 外接 BMI088 SPI | SPI0 | PB18 SCLK、PB17 MOSI、PB19 MISO | SysConfig PinMux 允许，三根总线共用 |
| BMI088 Accel CS | GPIO 输出 | PA8 | 软件 CS，低有效 |
| BMI088 Gyro CS | GPIO 输出 | PA7 | 软件 CS，低有效 |
| BMI088 DRDY | GPIO/IRQ | PB10、PB11 | 两个 die 的 DRDY 分开记录 |
| 第一组编码器/QEI | QEI/捕获 | PA29、PA30、PB14 | 对应 J12；先确认接口焊接和编码器电平 |
| 第二组编码器 | Timer 捕获 | PA26、PA27 | 使用前断开光线传感器跳线 |
| 双路电机 PWM 候选 | TIMG6 | PB26、PB27 | 使用前断开 RGB 红/绿跳线；方向脚必须等驱动器型号确认后再分配 |

Motor 的方向脚、PWM 极性、刹车/滑行逻辑不能只根据 MCU 空闲引脚决定。必须先
确认实际驱动器型号和接线，再完成 PWM+DIR 或双 PWM 的成套分配。

### 4. 推荐 DMA 分配（未应用）

G3507 有 7 个 DMA 通道，CH0..CH2 优先用于最关键的 SPI/高速 RX：

| DMA 通道 | 推荐用途 |
|---|---|
| CH0 | SPI0 RX，BMI088 |
| CH1 | SPI0 TX，BMI088 |
| CH2 | UART1 RX，Vision |
| CH3 | UART1 TX，Vision |
| CH4 | UART2 RX，循迹 |
| CH5 | UART3 RX，ChassisCommand |
| CH6 | UART0 RX，调试 |

### 5. 跳线冲突检查清单

在把推荐写入 G3507 SysConfig 之前，至少检查：

1. J21/J22 是否继续连接 XDS UART；若是，PA10/PA11 只给调试 UART0。
2. J5/J6/J7 是否断开；未断开时 PB22/PB26/PB27 不得外接其他推挽输出。
3. 光线传感器跳线是否断开；未断开时 PA22/PA26/PA27 不得作为编码器输入。
4. J12 的 QEI 接口是否焊接、信号电平是否为 3.3 V。
5. J9 温度传感器和 PB25 SMA 是否仍要使用。
6. CAN 收发器的型号、待机脚、终端电阻和供电是否已经确定。

## 三、推荐的 SysConfig 补充顺序

以下是将来确认接线后再执行的顺序，不是本次已执行操作：

1. 先锁定板载晶振、SWD、ICM、显示、RGB、按键等不可移动资源。
2. 再加入 CAN/UART/SPI 实例并固定到本文推荐引脚，立即处理 PinMux 冲突。
3. 为软件 CS、DRDY、激光和按键加入 GPIO；所有输出设置安全上电初值。
4. 最后加入 DMA 和 IRQ，先记录通道/向量所有权，再生成代码。
5. 生成后核对 `ti_msp_dl_config.h` 中的实例、引脚、CPUCLK 和 IRQ 宏。
6. BSP 只引用 `bsp_resource_map.h` 的私有映射；APP、Device、Mode、Task 不引用
   `ti_msp_dl_config.h`，也不直接出现 PA/PB/PC 引脚号。

## 四、当前未决参数

这些参数不能由原理图或 SysConfig 推断：

- ICM-45686 在车体上的 sensor X/Y/Z 到 body X/Y/Z 的轴交换和正负号。
- QD4310 的三台实际 CAN ID、左右/yaw 安装方向和主控侧 CAN 收发器。
- 两种 8 路循迹模块各自的波特率、帧格式、校验和更新率。
- Vision 串口协议参数。
- MG310 对应驱动器型号、PWM 方式、有效极性和方向脚。

上述项目未确认时，代码应保持 Device 可编译、BSP 未绑定、Mode 锁定和零输出。
