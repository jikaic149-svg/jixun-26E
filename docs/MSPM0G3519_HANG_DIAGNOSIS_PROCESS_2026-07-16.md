# MSPM0G3519 程序“卡死”诊断 Process 报告

- 日期：2026-07-16
- 目标：MSPM0G3519，Car25E，Debug
- 探针：DAPLink / CMSIS-DAPv2
- 固件：`build/Current/MSPM0G3519/Debug/MSPM0G3519.elf`
- 方法：按 aixprobe 的 hang、HardFault、内存、外设和 ELF 符号诊断流程执行
- 安全条件：电机保持锁定，未执行烧录，未进行车轮带载测试

## 1. 结论摘要

1. 开始检查时没有发现应用 CPU 卡死。5 次 PC 采样均落在 FreeRTOS Idle
   正常循环，主程序、Task 和 FreeRTOS 故障变量均为 0。
2. 受控复位后持续观察约 60 s，`g_TASK_Heartbeat` 和
   `g_TASK_UpdateCount` 都稳定增长到 6010，符合 10 ms 控制周期；没有再次进入
   HardFault、NMI 或栈溢出。
3. 当前业务状态为 `TASK_HEX25E_STATE_STOPPED`，即程序在正常等待 USER KEY，
   不是停止调度。PB31 实测为高电平，按键状态机也显示“未按下”。
4. 两路硬件 QEI 静止时计数稳定为 0，没有编码器 GPIO 中断，也没有 QEI 中断
   风暴证据。
5. 诊断过程中执行 OpenOCD `verify_image` 时，成功复现了一次真正的永久卡死：
   CPU 进入 NMI 后落入 `Default_Handler`。NMI 来源被寄存器明确解码为 SRAM ECC
   双位错误（`SRAMDED`）。
6. 该 NMI 是当前 OpenOCD 工作区地址错误引起的调试器副作用：OpenOCD 的 MSPM0
   target 脚本默认使用 `0x20000000`，而 MSPM0G3519 的实际 SRAM 范围是
   `0x20200000-0x2021FFFF`。CRC 加速算法在错误地址运行后触发 SRAM DED。

因此，本次检查没有证明 QEI 或轮速闭环导致应用卡死；已经证明当前 Debug/Image
Verify 工具路径可以制造 NMI 卡死。平时看到“没有动作”时，还要先区分业务状态
`STOPPED` 与 CPU 真正停止运行。

## 2. 检查对象与版本确认

### 2.1 工具和探针

执行项目 Doctor：

- DAPLink USB 在线。
- 序列号：`000100011a5a5bb600000000000425a1a5a5a5a597969908`
- OpenOCD：`0.12.0+dev-02555-g1bf1ac444-dirty`
- 目标识别：Cortex-M0+ r0p1。
- SWD：2 MHz。

### 2.2 ELF 内存布局

从当前 ELF/map 文件确认：

| 区域 | 地址 | 大小 |
|---|---:|---:|
| Flash | `0x00000000` | 512 KiB |
| SRAM Bank 0 | `0x20200000` | 64 KiB |
| SRAM Bank 1 | `0x20210000` | 64 KiB |
| Control 静态栈 | `0x202005A8` | 4096 B |

当前固件主 Flash HEX 长度为 34940 B。`verify_image` 的目标端 CRC 加速阶段失败，
随后 OpenOCD 回退读取并报告 34940 B 校验通过。由于 CRC 加速本身会触发本报告
后述的错误，在修复工作区前不应再把 `verify_image` 用于健康检查。

## 3. Process 记录

### 3.1 第一次 PC 多点采样

按照 hang 流程进行 halt/read/resume，间隔约 100 ms：

| 采样 | PC | ELF 定位 |
|---:|---:|---|
| 1 | `0x000027A8` | `prvIdleTask` |
| 2 | `0x00002788` | `prvCheckTasksWaitingTermination` |
| 3 | `0x00002788` | `prvCheckTasksWaitingTermination` |
| 4 | `0x000027A8` | `prvIdleTask` |
| 5 | `0x000027A8` | `prvIdleTask` |

同时：

- `xPSR = 0x61000000`，异常号为 0，处于 Thread 模式。
- `SP/PSP = 0x20201D00`，位于合法 SRAM。
- PC 位于合法 Flash。

判定：PC 在 FreeRTOS Idle 的正常小循环内变化，不是非法地址、HardFault 或单点
死循环。Idle 出现说明当时没有更高优先级任务持续占用 CPU。

### 3.2 初始 Fault 与应用变量

| 变量/寄存器 | 值 | 解读 |
|---|---:|---|
| `g_MAIN_Fault` | 0 | 启动成功 |
| `g_TASK_Fault` | 0 | Task 无故障 |
| `g_TASK_ModeFault` | 0 | Hex25E 无业务故障 |
| `g_FreeRTOS_Fault` | 0 | 无 malloc/栈溢出故障 |
| CFSR/HFSR | 全 0 | 无 Cortex Fault |
| `g_TASK_Heartbeat` | 41 | 控制任务已运行 |
| `g_TASK_UpdateCount` | 41 | 控制更新未停止 |
| `g_TASK_Ready` | 1 | Task 已就绪 |
| `g_TASK_RouteState` | 0 | `STOPPED` |
| `g_TASK_OutputEnabled` | 0 | 电机锁定 |

### 3.3 控制栈检查

Control 栈已经从旧版的 512 words 增加到 1024 words，即 4096 B。本次读取栈底
前 256 B，仍全部保持 FreeRTOS 的 `0xA5A5A5A5` 填充值；
`g_FreeRTOS_Fault=0`。

判定：本次没有复现此前的 Control 栈溢出，当前“无动作”不能归因于已知栈问题。

### 3.4 `verify_image` 后的 NMI 现场

执行目标端镜像校验后，OpenOCD 报告：

```text
timed out while waiting for target halted
[mspm0x.cpu] error executing cortex_m crc algorithm
```

随后 CPU 现场：

| 项目 | 值 | 解读 |
|---|---:|---|
| PC | `0x00002108` | `Default_Handler` |
| xPSR | `0x01000002` | 异常号 2，NMI |
| LR | `0xFFFFFFFD` | NMI 打断使用 PSP 的 Thread |
| PRIMASK | 1 | 中断关闭 |
| `SYSCTL_NMIIIDX` | 6 | `SRAMDED` |
| `SYSCTL_NMIRIS` | `0x20` | SRAM DED raw flag 置位 |
| `SYSCTL_DEDERRADDR` | `0x20000030` | 错误地址位于错误的 OpenOCD 工作区别名 |
| NMI 前堆栈 PC | `0x20000000` | 不在 G3519 实际 SRAM/Flash |

从 `MSPM0G351X.svd` 解码：

- `NMIIIDX=6` 明确表示 SRAM Double Error Detect。
- `NMIRIS[5]=1` 明确表示 SRAM DED 已发生。
- 这不是 HFXT、PLL、HardFault 或 FreeRTOS 异常。

### 3.5 OpenOCD 工作区根因

当前 OpenOCD target 脚本：

`D:/OPENOCD/openocd-custom-20260615/share/openocd/scripts/target/ti/mspm0.cfg`

其中默认值为：

```tcl
set _WORKAREABASE 0x20000000
set _WORKAREASIZE 0x400
```

随后配置为 Cortex-M 算法工作区。项目 cfg 在 source 该 target 脚本前没有覆盖
`WORKAREABASE`。

G3519 实际 SRAM 从 `0x20200000` 开始。因此目标端 CRC 算法被安排到不存在的
`0x20000000` 区域，和 NMI 堆栈中的 `PC=0x20000000`、
`DEDERRADDR=0x20000030` 完全吻合。

判定：这是调试配置缺陷，不是轮速闭环/QEI 的运行时写越界。

### 3.6 受控复位后的持续观察

清除被调试器制造的 NMI 后执行一次 `reset run`，保持同一个 OpenOCD 会话，避免
反复连接干扰。约 60 s 后：

| 项目 | 值 | 解读 |
|---|---:|---|
| PC | `0x000041EA` | 数字循迹短延时函数 |
| xPSR 异常号 | 0 | Thread 模式 |
| `g_TASK_Heartbeat` | 6010 | 持续增长 |
| `g_TASK_UpdateCount` | 6010 | 持续增长 |
| `g_TASK_NowMs` | 60100 ms | 与 10 ms 周期一致 |
| `g_MAIN_Fault` | 0 | 正常 |
| `g_TASK_Fault` | 0 | 正常 |
| `g_FreeRTOS_Fault` | 0 | 正常 |
| 状态 | `STOPPED` | 等待启动命令/按键 |
| 左右电机命令 | 0 / 0 | 安全锁定 |
| 左右 QEI 累计 | 0 / 0 | 静止无假计数 |
| `SYSCTL_NMIIIDX` | 0 | 无 NMI |
| `SYSCTL_NMIRIS` | 0 | 无 NMI raw flag |

判定：当前固件在不执行错误的 target-side verify 时可以持续正常调度。

### 3.7 USER KEY 检查

PB31 GPIO 数据寄存器实测 bit31=1，符合内部上拉和“低电平按下”的配置。按键状态机：

- `initialized=true`
- `raw_pressed=false`
- `stable_pressed=false`
- debounce=30 ms

本次采样窗口没有捕获到实际按键按下，因此只证明初始化、电平和轮询任务正常，
未完成一次完整的 `1 -> 0 -> 1` 启停动作复测。

## 4. 排除项

本次证据可以排除：

- HardFault/BusFault/UsageFault。
- Control 任务再次栈溢出。
- QEI GPIO 边沿中断风暴；3519 两路编码器均为硬件 QEI，未启用 QEI IRQ。
- 轮速闭环在静止/STOPPED 状态把 PWM 拉满；状态和电机命令均为 0。
- QEI 静止噪声造成大量虚假计数。
- USER KEY 未初始化或 PB31 固定为按下状态。

尚未排除：

- 实际按键按下时 PB31 是否确实从 1 变 0。
- 车轮转动后 QEI 的计数方向、四倍频计数比例和速度换算。
- 电机电源噪声/地弹对板卡供电或编码器信号的影响。
- 用户看到“卡死”时是否刚好处于业务 `STOPPED`、2 s 启动延时或故障锁定状态。

## 5. 建议的后续动作（本次未实施）

1. 在项目 G3519 OpenOCD cfg 中，在 source `target/ti/mspm0.cfg` 前覆盖合法工作区：
   使用未被链接器占用的 SRAM Bank 1 地址，并同步限定 1 KiB 大小；不能简单使用
   `0x20200000`，因为该处已经存放应用 `.data/.bss`。
2. 修复前禁止使用会执行目标端算法的 `verify_image`/CRC 路径。烧录继续使用当前
   `dump_image + 主机 SHA-256` 回读方案，该流程不依赖错误的 Cortex-M 工作区。
3. 增加项目自有 `NMI_Handler`：第一时间锁电机，并保存
   `NMIIIDX/NMIRIS/DEDERRADDR/堆栈 PC`，使下次 NMI 不再只表现为无信息的
   `Default_Handler` 卡死。
4. 增加可观察量：按键原始电平、稳定电平、QEI 原始计数、最近速度、控制状态和
   NMI 快照，通过 Live Watch 和蓝牙状态一起输出。
5. 车轮架空后完成一次按键闭环复测：
   `PB31 1->0->1`、`STOPPED->START_DELAY->运动`、QEI 计数变化、再次按键后 PWM
   立即归零。

## 6. 最终判定

当前程序在本次受控观察中没有应用级卡死。可复现的永久卡死来自 OpenOCD 使用了
错误的 Cortex-M 工作区地址，触发 MSPM0G3519 SRAM ECC DED NMI，而启动代码把
NMI 弱符号映射到了 `Default_Handler`。应优先修复 Debug 工作区并补充 NMI 现场
记录，再进行按键和车轮架空动态验证。
