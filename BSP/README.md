# BSP 结构

BSP 只负责 MCU 外设和引脚。APP、Mode、Device、Algorithm 都不能直接包含
`ti_msp_dl_config.h`，也不能写 PA/PB 引脚、定时器实例或中断号。

```text
BSP/
├─ bsp_init.h                 BSP 总初始化
├─ bsp_time.h                 忙等延时
├─ bsp_rgb.h                  RGB 接口
├─ bsp_key.h                  原始按键电平
├─ bsp_pwm.h                  0..10000 PWM
├─ bsp_motor.h                方向 + PWM 的普通电机输出
├─ bsp_encoder.h              编码器累计值和增量
├─ bsp_uart.h                 UART blocking / 可选 DMA
├─ bsp_can.h                  Classic CAN / CAN FD 公共帧接口
├─ bsp_line_sensor.h          数字/模拟循迹采样接口
├─ bsp_not_configured.c       未分配外设时的安全空实现
├─ MSPM0G3507/
│  ├─ MSPM0G3507.syscfg       3507 的引脚和外设配置
│  ├─ bsp_resource_map.h      SysConfig 名称映射
│  ├─ bsp_manifest.cmake      只加入实际使用的 DriverLib 文件
│  └─ bsp_init/rgb/time.c     3507 的实际实现
└─ MSPM0G3519/
   └─ 与 3507 相同的四类文件
```

使用规则很简单：

1. 上层只调用 `bsp_*.h`。
2. 换引脚时只修改对应 MCU 的 `.syscfg` 和 `bsp_resource_map.h`。
3. 新外设的实际实现放在 `MSPM0G3507/` 或 `MSPM0G3519/`。
4. 没配置的外设由 `bsp_not_configured.c` 返回零资源/失败，不猜引脚。

G3507 有 CANFD0，G3519 有 CANFD0/CANFD1。公共 CAN API 已建立；当前两个
资源表的 `BSP_RESOURCE_HAS_CAN` 都是 `0`，因为还没有选择 CAN RX/TX 引脚和
位速率。配置 SysConfig 后再加入实际 MCAN backend 和 `dl_mcan.c`。

## MSPM0G3519 当前小车资源

| BSP 资源/顺序 | 外设与引脚 | 参数 |
|---|---|---|
| UART 0 | UART1：PB4 TX、PB5 RX | 115200，蓝牙/主调试 |
| UART 1 | UART0：PA10 TX、PA11 RX | 115200，辅助串口 |
| UART 2 | UART4：PB10 TX、PB11 RX | JY61P，115200，RX DMA_CH2，上拉 |
| PWM 0 / Motor 0 | TIMA0_CCP0 PC2；PA24/PA25 | 左电机，20 kHz |
| PWM 1 / Motor 1 | TIMA0_CCP1 PC4；PA26/PA27 | 右电机，20 kHz |
| Encoder 0 | TIMG9 QEI：PB29 A、PB30 B | 左编码器，硬件 x4 正交计数、上拉 |
| Encoder 1 | TIMG8 QEI：PB6 A、PB7 B | 右编码器，硬件 x4 正交计数、上拉 |
| LineSensor 0 | PB8 DAT、PB9 CLK | 8 路同步串行数字量，DAT 上拉 |
| Key 0 | PB31 USER KEY | 低电平按下，内部上拉 |

本项目连接的 JY61P 使用 115200 baud；SysConfig 和 Hex25E transport 检查必须
保持相同波特率。灰度模块后端每次读取 8 bit，
第一个时钟采样映射为通道 0，低电平返回 0（黑线极性由 Device 配置处理）。

## MSPM0G3507 当前编码器资源

| BSP 资源/顺序 | 外设与引脚 | 参数 |
|---|---|---|
| Encoder 0 | TIMG8 QEI：PA29 A、PA30 B | 左编码器，硬件 x4 正交计数、上拉 |
| Encoder 1 | TIMG7 Capture：PA26 A；GPIO PA27 B | 右编码器，A 上升沿中断判向，x1 结果按 4 折算为 x4 计数单位 |

G3507 的 PA29/PA30 对应 LaunchPad J12 QEI 接口；PA26/PA27 与板载光线传感器
跳线复用，接第二路编码器前必须按板卡说明确认 J12 焊接并断开冲突跳线。
