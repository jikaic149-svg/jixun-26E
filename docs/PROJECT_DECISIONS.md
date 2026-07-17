# Project decisions

本文记录模板的实质设计选择，避免后续重构重新引入已经否决的方案。

## 已确认

- 模板定位：通用 TI MSPM0 模板；Car25E、Jixun2 和 Jixun3 是可选产品 Mode。
- MCU：MSPM0G3507 与 MSPM0G3519。
- G3519 板：用户原理图对应的 LQFP-80(PN)；不迁入 `-25E`
  LQFP-100(PZ) 的引脚配置。
- G3519 当前基础资源：PA29/TIMG6 RGB、PA5/PA6 HFXT、
  PA19/PA20 SWD。
- BSP 粒度：目前只按 MCU 区分；同一 MCU 改引脚时修改其 SysConfig
  和 BSP。
- 底层：TI DriverLib + SysConfig；不使用 TI 高层 Drivers/DPL。
- 构建：VS Code + CMake + GCC；每个 feature 显式加入编译链，禁止
  全目录 GLOB。
- FreeRTOS：保留 V11.3.0 移植；所有产品 preset 创建静态控制 Task 并启动调度器，
  默认关闭 heap_4、Timer、EventGroup、StreamBuffer 和 trace。
- Task：只负责周期、优先级和 Device/Mode 数据搬运，不迁入 `-25E` 的旧业务 Task。
- 逐飞许可边界：不复制、不链接逐飞 GPL 源码或二进制；需要的功能依据
  TI/器件官方资料和公开需求独立实现。
- 公共 API：只公开融合后的 `BSP_*`、`Algorithm_*`、`Device_*`
  接口；不提供 `zf_*` 兼容 wrapper。
- API 命名：采用 N1 的模块前缀和 Pascal 风格动作名，例如
  `BSP_Init()`、`BSP_RGB_SetPixel()`、`BSP_Time_DelayMs()`、
  `Mode_Car_Update()`、`Task_Update()` 和 `Task_Init()`；Live Watch 全局量保留 `g_` 前缀并
  使用相同模块大小写风格。
- 引脚与外设后端：完全属于各 MCU BSP。RGB 公共接口只表达像素和颜色；
  当前 G3519 BSP 配置 1 颗灯并使用 PA29/TIMG6，未来换引脚、Timer、
  SPI 或 DMA 不改变 APP/Device。
- BSP 资源映射：PWM、UART、Encoder、Key 原始输入等硬件能力只公开逻辑
  handle。实际引脚在 SysConfig 中选择，生成的实例/IRQ/DMA 符号集中经各 MCU
  的私有 `bsp_resource_map.h` 使用。
- Key：原始按下状态由 BSP 提供；Device 层实现消抖、短按、长按和双击。
- PWM：BSP 公共占空比标度为整数 0–10000。
- Encoder：BSP 使用显式 handle，并同时提供累计 snapshot 与增量读取。
- UART：BSP 提供 blocking 和可选 DMA 字节传输；数据包/视觉协议不进入 BSP，
  上层以 `\n` 作为帧结束标志。
- 缓冲：使用与 RTOS 无关的 SPSC ring；FreeRTOS 只做可选通知适配。
- ISR：UART、PWM 等 MCU 外设 vector 和寄存器处理跟随其 BSP owner；
  Device 协议处理跟随对应 Device。控制环路定时 ISR 只通知 FreeRTOS Task，
  不在 ISR 内运行控制业务。
- Delay：`BSP_Time_DelayMs()` 保持忙等；调度器启动后任务代码显式调用
  `vTaskDelay()`。
- FreeRTOS 策略：控制 Task 使用静态 TCB/栈；Timer、EventGroup、StreamBuffer
  和 trace 默认关闭并按需加入编译链。
- Mode 分类包含可复用模式库 `Mode/Car`、`Mode/Gimbal` 和产品业务 Mode
  `Mode/Car/Control`、`Task`、`Mode/Balance`、`Task`。
- Device 按职责分类为 `Device/Input`、`Device/Actuator`、`Device/Sensor`
  与 `Device/Communication`，不按仓库或 Product 重复实现同一设备。
- `Car25E` 与 `Jixun2` 作为产品业务 Mode；Mode 负责输入契约、模式选择、安全锁、
  急停和最终逻辑限幅，Task 负责以 10 ms 周期调用它。板级资源尚未绑定时只传入
  无反馈快照并校验零输出，不伪造传感器反馈、不自动解锁或提交电机命令。
- 通用 IMU Device 负责轴映射、单位换算、陀螺校准和在线状态；硬件无关的
  Attitude Algorithm 负责 6 轴姿态解算。Mode 只消费抽象反馈，不依赖具体 IMU。
- `-25E` 与 `jixun-2` 的正式 Mode 和 Task 行为按需求独立重写，不复制源码。
- `jixun3/Application` 整体按职责迁移：纯算法进入 APP，协议进入 Device，
  平衡/云台/产品安全状态进入 Mode，1 kHz 执行与 Device 搬运进入静态
  FreeRTOS Task。旧 STM32 HAL、全局外设对象和重复 Mahony 实现不复制。
- Jixun3 的 CAN ID、安装极性、IMU 轴和激光 GPIO 不设猜测默认值；产品 Task
  只有收到完整显式绑定才初始化 QD4310，否则保持锁定、零输出。
- 视觉工具：不包含 Python 视觉工具，只保留 MCU 端视觉通信代码。
- 模板许可证：模板自有新代码采用根目录 MIT 许可证；TI、CMSIS、FreeRTOS
  以及其他第三方/生成文件继续使用各自许可证。
- 探针序列号：模板不保存真实序列号。DAPLink 与 XDS110 的序列号分别只存于
  模板同级 `TI ENV/local/openocd/daplink.cfg` 和 `xds110.cfg`；OpenOCD 与
  Doctor/Flash 共同读取这两个本机文件。
- 未稳定代码不进入正式源码树。
- 启动安全：电机默认锁定，具体项目显式解锁。
- 控制算法：保留既有参数和数值行为，只修明确错误。

## 尚未决定

无。G3507 在没有板级 LED 资料前只保留 Live Watch，不猜测引脚。
