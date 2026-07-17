# Task

当前工程只保留 `Hex25E` 任务及其根级 FreeRTOS 入口。

- `task_app.c/.h`：创建静态控制任务并调度 `Hex25E`。
- `Hex25E/`：保存当前题目的状态机、通信、IMU 与遥测逻辑。
- 通用硬件访问、器件抽象和可复用控制器分别放在 `BSP/`、`Device/` 和 `Mode/`，不在 Task 中重复实现。

其他题目的 Task 不在当前工程中保留。
