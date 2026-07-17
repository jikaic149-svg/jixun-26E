# 中断与执行入口归属

根目录 `main.c` 负责 SysConfig/BSP 初始化、创建 Task 并启动 FreeRTOS 调度器。

- UART、PWM、Encoder、GPIO 等 MCU vector、寄存器清标志和 DMA 完成处理跟随对应
  BSP 模块。
- Device 驱动不直接声明 `UARTx_IRQHandler`、`TIMGx_IRQHandler` 等 MCU 符号；
  BSP 只把字节、完成事件或规范化设备 IRQ 事件交给 Device。
- 控制算法在 Task 上下文执行，不在 ISR 内运行。需要硬件 Timer 时，ISR 只使用
  `...FromISR` API 通知 Task 并请求必要的上下文切换。
- Task 负责周期和优先级，Mode 不包含 FreeRTOS API，APP 算法也不依赖调度器。
