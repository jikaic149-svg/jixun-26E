# APP 算法层

`APP` 只保存与硬件、业务模式和调度器无关的可复用算法。算法采用显式输入输出，
不访问 BSP、Device、Mode、Task、FreeRTOS 或跨模块全局变量。

业务状态机、周期、优先级和 Device/Mode 数据搬运位于 `Task`。根目录
`main.c` 只负责初始化并启动调度器。

当前算法组件位于 `APP/Algorithm`，每个组件拥有独立 CMake target，并由实际
使用它的 Mode 或 Device 显式链接。
