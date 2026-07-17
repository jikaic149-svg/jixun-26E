# TI MSPM0 通用模板

这是一个以 DriverLib 和 SysConfig 为底层、按 MCU 选择 BSP 的通用工程模板。
目录风格参考 `-25E`，但不包含巡检车业务、逐飞 GPL 源码或未授权的硬件映射。

## 默认行为

- FreeRTOS V11.3.0 始终编译；Hex25E 由静态分配的 10 ms Task 执行。
- Task 直接读取 Device、调用 Mode，并把经过安全检查的输出提交给 Device。
- 当前题目的全部可调参数集中在 `config/hex25e_config.h`。
- 电机及其他可能产生运动的输出不在基础模板中初始化。

## 目录

```text
TI Template/
├─ APP/                       与硬件无关的算法组件
├─ BSP/
│  ├─ *.h                     与 MCU 无关的 BSP 接口
│  ├─ MSPM0G3507/             G3507 SysConfig、实现和 DriverLib 清单
│  └─ MSPM0G3519/             G3519 SysConfig、实现和 DriverLib 清单
├─ APP/Algorithm/                 可复用纯软件组件，默认不编译
│  ├─ Control/                PID、角度、限幅和斜率限制
│  ├─ Filter/                 低通、滑动平均和三点中值滤波
│  ├─ Attitude/               无硬件依赖的 6 轴 Mahony 姿态解算
│  ├─ Packet/                 LF/CRC16 数据包编解码
│  ├─ ReliableLink/           ACK、超时、重试和重复抑制
│  ├─ SPSC_Ring/              RTOS 无关的单生产者/消费者字节环
│  ├─ LQR/                    固定、调度与增量式 LQR
│  ├─ ADRC/                   一阶 ESO 与 PI-ESO
│  ├─ Balance/                平衡外环、启停辅助和零偏搜索
│  └─ CRC/                    CRC-16/IBM 与 Modbus
├─ Device/                    外部器件驱动，默认不编译
│  ├─ Input/
│  │  └─ Key/                 短按、长按和双击事件
│  ├─ Actuator/
│  │  └─ Motor/               默认锁定的有符号电机命令
│  ├─ Sensor/
│  │  ├─ Encoder/             累计值、增量、距离和速度
│  │  ├─ JY61P/               字节流解析、Yaw 清零和离线检测
│  │  └─ IMU/                 轴映射、零偏标定和姿态快照
│  └─ Communication/
│     ├─ Vision/              独占 UART 上层的可靠视觉通信
│     └─ ChassisCommand/      Jixun3 串口命令解析
├─ FreeRTOS/
│  ├─ Config/                 FreeRTOS 配置和移植支持
│  └─ Kernel/                 FreeRTOS Kernel V11.3.0
├─ Mode/Car/                  六个通用车辆 Mode
├─ Mode/Gimbal/               云台正式 Mode 与 Scheduler
├─ main.c                     初始化并启动 FreeRTOS
├─ Task/Hex25E/               当前题目的调度、路线与通信
├─ config/                    参与构建和调试的声明式配置
│  ├─ cmake/                 工具链和 DriverLib 模块映射
│  └─ openocd/               DAPLink/XDS110 OpenOCD 配置
├─ docs/                      决策、审计、工具锁定和验证记录
├─ tools/                     构建、烧录和 SysConfig 脚本
├─ LICENSE                    模板自有新代码的 MIT 许可证
└─ .vscode/                   VS Code 构建与调试配置
```

## 构建

VS Code 中按 `Ctrl+Shift+B` 默认构建 MSPM0G3519 Debug。也可以运行：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tools\mspm0.ps1 `
  -Action Build -Device MSPM0G3519 -Configuration Debug
```

四套 preset：

```text
g3507-debug
g3507-release
g3519-debug
g3519-release
```

可直接构建 Hex25E：

```text
g3507-hex25e-debug
g3519-hex25e-debug
```

Debug 使用 `-Og -g3`，适合断点和变量观察；Release 使用 `-Os`，
体积更小，但未被程序读取的变量可能被优化或链接回收。

运行 VS Code 任务 `Template: Check Architecture` 可以检查上层是否误用
DriverLib/固定外设、是否出现隐式源码通配符、是否混入逐飞源码，以及
`launch.json`/`tasks.json` 的任务引用是否断开或重名。

构建只编译所选 BSP 明确登记的 DriverLib 源。增加 UART、SPI、ADC 等功能时，
把对应模块加入该 BSP 或组件的 manifest；禁止用文件通配符编译全部 DriverLib。
成功构建会显示简洁的 text/data/bss 尺寸和主 Flash HEX 地址范围。
同时显示总编译单元以及 BSP、Algorithm、Device、DriverLib、FreeRTOS 数量；
增加 `-Verbose` 可以查看每个实际进入编译链的源文件。

`Clean` 只执行 Ninja 的目标清理并保留 CMake cache；`Purge` 只删除所选
MCU/配置的整个构建目录，用于从零重新配置。

Algorithm 与 Device 默认关闭，只有显式打开对应选项时才进入编译链：

- Algorithm：`TEMPLATE_ENABLE_ALGORITHM_CONTROL`、
  `TEMPLATE_ENABLE_ALGORITHM_FILTER`、`TEMPLATE_ENABLE_ALGORITHM_ATTITUDE`、
  `TEMPLATE_ENABLE_ALGORITHM_PACKET`、
  `TEMPLATE_ENABLE_ALGORITHM_RELIABLE_LINK`、
  `TEMPLATE_ENABLE_ALGORITHM_SPSC_RING`、`TEMPLATE_ENABLE_ALGORITHM_LQR`、
  `TEMPLATE_ENABLE_ALGORITHM_ADRC`、`TEMPLATE_ENABLE_ALGORITHM_CRC`。
- Device：`TEMPLATE_ENABLE_DEVICE_KEY`、`TEMPLATE_ENABLE_DEVICE_MOTOR`、
  `TEMPLATE_ENABLE_DEVICE_ENCODER`、`TEMPLATE_ENABLE_DEVICE_JY61P`、
  `TEMPLATE_ENABLE_DEVICE_IMU`、`TEMPLATE_ENABLE_DEVICE_VISION`、
  `TEMPLATE_ENABLE_DEVICE_QD4310`、
  `TEMPLATE_ENABLE_DEVICE_CHASSIS_COMMAND`。

设置时在选项名后加 `=ON`。CMake 只自动带入被选模块必需的传递依赖，例如
IMU 带入 Attitude，Vision 带入 ReliableLink/Packet；不会把整个 Algorithm 或
Device 目录全部编译。

各模块保留 host 测试源码和 ARM 编译脚本；当前电脑已经可以直接编译并运行
host 测试，同时继续用 Cortex-M0+ 严格编译检查目标端兼容性，不需要关闭或绕过
Smart App Control。

## 切换 MCU 或引脚

应用层不包含 MCU 寄存器和固定引脚。切换 preset 时，CMake 自动选择相应的：

- `BSP/MSPM0Gxxxx`
- SysConfig 配置
- DriverLib 模块清单
- 启动文件
- 链接脚本
- MCU 定义

同一 MCU 修改引脚时，只改它的 `.syscfg` 和 BSP 实现。
`MSPM0_SDK_ROOT` 与 `SYSCONFIG_ROOT` 默认从模板同级的 `TI ENV`
推导；其他安装位置必须用 CMake `-D...` 显式覆盖，避免旧系统环境变量
悄悄选中错误 SDK。

公开 C API 采用模块前缀和 Pascal 风格动作名，例如 `BSP_Init()`、
`Device_Motor_SetCommand()`、`Mode_Car_Tracking_Update()` 和
`Task_Hex25E_Update()`。

## FreeRTOS

所有 preset 都会编译 FreeRTOS 和静态控制 Task，并由 `main.c` 调用
`vTaskStartScheduler()`。默认关闭 `heap_4`、软件 Timer、EventGroup、
StreamBuffer 和 trace；需要时通过 `TEMPLATE_FREERTOS_USE_*` 选项显式启用。

## 调试与烧录

- MSPM0G3519 只使用 DAPLink 和定制 OpenOCD。
- MSPM0G3507 只使用 XDS110 和独立的标准 OpenOCD。
- `.vscode/launch.json` 为两颗 MCU 提供构建、烧录和调试入口，并保留
  Live Watch 和两个 SysConfig GUI 入口。
- 每个 Cortex-Debug 启动项都会先运行 Doctor，校验探针组合和工具路径。
  G3519 还会校验定制 OpenOCD、USB/HID runtime、MSPM0 target script 和工程
  cfg 的 SHA-256；G3507 校验 stock OpenOCD 可执行文件和工程 cfg，然后才构建
  和启动。
- 模板不保存真实探针序列号。DAPLink 与 XDS110 的本机选择分别位于同级
  `TI ENV/local/openocd/daplink.cfg` 和 `xds110.cfg`；工程 cfg、VS Code 直接
  启动 OpenOCD 以及 Doctor/Flash 都读取同一值。首次迁移请按
  [`config/openocd/README.md`](config/openocd/README.md) 从无序列号 example
  创建本机文件。
- Doctor/Flash 会先校验本机配置，再确认其中指定的 USB VID/PID 与序列号当前
  确实在线。配置缺失、格式错误或探针未连接时会明确报错，不会继续目标访问。

两个 MCU 的 Debug 固件都可以观察：

```text
g_TASK_Heartbeat
g_TASK_Fault
g_TASK_Ready
g_TASK_MotorLocked
g_TASK_OutputEnabled
g_TASK_ModeFault
g_TASK_UpdateCount
g_TASK_NowMs
```

G3519 RGB BSP 还可以观察：

```text
g_BSP_RGB_LastGRB
g_BSP_RGB_FrameCount
```

系统和 FreeRTOS 故障还可以观察：

```text
g_MAIN_Fault
g_FreeRTOS_Fault
```

模板自有新代码使用根目录 [MIT LICENSE](LICENSE)。该许可证不覆盖 TI、CMSIS、
FreeRTOS、SysConfig 生成文件或其他第三方材料；其版本、来源和许可证见
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
已确认的架构选择和待确认问题分别见
[`docs/PROJECT_DECISIONS.md`](docs/PROJECT_DECISIONS.md) 与
[`docs/PENDING_DECISIONS.md`](docs/PENDING_DECISIONS.md)。
已验证工具版本与 OpenOCD 哈希见
[`docs/TOOLCHAIN_LOCK.md`](docs/TOOLCHAIN_LOCK.md)。
`-25E` 的源码归属边界见
[`docs/CAR25E_SOURCE_AUDIT.md`](docs/CAR25E_SOURCE_AUDIT.md)。
项目文本由 `.editorconfig` 和 `.gitattributes` 固定为 UTF-8/LF；
vendored FreeRTOS Kernel 保留上游原始字节。
