# Device/Communication/Vision

与具体 UART、MCU 和调度器无关的视觉通信状态机。Product 必须给视觉分配独占 UART，调试/遥测使用另一条 UART；ISR/DMA 只搬运字节，协议 API 在串行化的 main/task 上下文运行。

线协议由 `APP/Algorithm/Packet` 定义：

```text
@1|TYPE|SEQ|LEN|PAYLOAD|CRC16\n
```

支持双方主动发起 `START`、`END`、`REQUEST`，并处理 `ACK` 与只含颜色/形状的 `RESULT`。视觉结果不会直接产生转向、PWM 或电机命令，最终运动仲裁始终属于 Task。

关键约束：

- `local_session` 必须是每次本机启动都变化的非零值；默认配置故意返回零，不能直接初始化。
- START/END/REQUEST 使用独立的 8 项关键事件队列；队列满时不提交去重状态，也不 ACK。
- RESULT 使用 latest-wins 快照和递增 generation，不占关键事件队列。
- 关键 TX 遵循 `Peek -> UART accepted -> Commit -> blocking/DMA complete -> TxComplete`。
- 零拷贝 DMA 期间，调用方必须保持 TX/ACK 缓冲有效且不变。
- ACK、重试、重复抑制由 `APP/Algorithm/ReliableLink` 处理。

目录移动没有改变 `Device_Vision_*` API 或 `device_vision` CMake target。

测试：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\Device\Communication\Vision\tests\run_gcc_tests.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File .\Device\Communication\Vision\tests\run_arm_compile.ps1
```
