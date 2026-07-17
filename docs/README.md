# 项目文档

本目录只保存架构决策、来源审计、工具链锁定与验证记录，不参与构建。
可执行的构建和调试配置统一放在根目录 `config/`；MCU、引脚、时钟和中断配置
只放在 `BSP/MSPM0G3507` 或 `BSP/MSPM0G3519`。

- [`DEVICE_PARAMETERS.md`](DEVICE_PARAMETERS.md)：每个 Device 的型号、连接、代码默认值、
  实物参数、确认来源和未确认项台账。
- [`BOARD_PIN_RESOURCE_RECOMMENDATION.md`](BOARD_PIN_RESOURCE_RECOMMENDATION.md)：
  两颗 MCU 的板载占用、SysConfig 可选引脚、推荐分配、DMA/IRQ 规划；只作推荐，
  不代表已经修改 `.syscfg`。
