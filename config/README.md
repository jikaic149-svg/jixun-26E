# 工程配置

本目录只保存参与构建、烧录或调试的机器无关声明式配置：

- `cmake/`：GNU Arm 工具链定义与 DriverLib 模块映射；
- `openocd/`：DAPLink/XDS110 的工程配置与无序列号示例；
- `hex25e_config.h`：Hex25E 的周期、机械参数、PID、串口和阈值。

编译产物、缓存和日志只能进入根目录 `build/`。项目决策、审计和验证记录只能
进入根目录 `docs/`。真实探针序列号属于机器本地状态，只能放在模板同级的
`TI ENV/local/openocd/`，不得写入本目录。
