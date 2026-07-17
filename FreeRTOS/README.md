# FreeRTOS

这里保存 FreeRTOS Kernel V11.3.0、GCC ARM_CM0 端口和模板配置。

所有产品 preset 都编译内核、创建静态分配的控制 Task，并由根目录 `main.c`
调用 `vTaskStartScheduler()`。模板不使用 `heap_4`，因此不会依赖动态内存。

软件 Timer、EventGroup、StreamBuffer 和 trace 默认关闭；需要时通过
`TEMPLATE_FREERTOS_USE_*` 显式启用。关闭的服务不会把对应源文件加入编译链。

`Task/` 负责 FreeRTOS 周期、优先级和数据搬运；`Mode/` 负责业务状态机与安全
策略；FreeRTOS 内核本身不承载产品逻辑。
