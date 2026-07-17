# Device/Communication

通信设备实现独立于 UART backend 的协议状态机。BSP/ISR 只搬运字节；完整解析在 main 或 task 上下文执行。

