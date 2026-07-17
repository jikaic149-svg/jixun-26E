# ReliableLink component

`APP/Algorithm/ReliableLink` 是与 UART、MCU 和 RTOS 无关的 stop-and-wait 可靠发送状态机。
它使用 `APP/Algorithm/Packet` 编码并保留一帧，不分配动态内存。消息 TYPE、ACK payload、
session 生成和业务语义仍由 Device 层负责。

## 发送所有权

发送采用显式三阶段，UART 拒绝数据不会被误算成一次发送：

1. `Algorithm_ReliableLink_PeekDueFrame()` 返回当前待发 view，但不启动期限、
   不增加 retry，也不转移缓冲区所有权。UART busy 时可以反复 Peek，得到同一帧。
2. 只有 UART blocking 发送返回 `OK`，或 DMA Start 返回 `OK` 后，才调用
   `Algorithm_ReliableLink_CommitTxAccepted(now_ms)`。这一步启动总期限或消耗一次 retry，
   并标记 `tx_in_flight`。
3. blocking 发送完成后立即调用 `Algorithm_ReliableLink_TxComplete()`；DMA 必须在它的
   唯一完成回调中通知 main/task 上下文，再由串行化上下文调用 TxComplete。

DMA 的典型流程：

```text
Peek -> BSP_UART_StartWriteDMA
             BUSY/error: 不 Commit，稍后继续 Peek
             OK: CommitTxAccepted(now)
DMA callback -> 只记录完成
main/task    -> TxComplete
```

blocking 调用可在调用前保存开始时间；返回 `OK` 后用该时间 Commit，紧接着 Complete。
失败或部分发送不得 Commit，UART adapter 还应完成流重新同步。

view 指向组件内部帧。ACK 或总超时发生在 DMA in-flight 期间时，内部帧仍不会释放或覆盖；
只有 TxComplete 后才允许下一次 `Begin()` 复用它。及时 ACK 会先锁存，TxComplete 再完成状态。

## 重试与截止时间

默认策略是一帧初发、30 ms 总期限内最多重试 3 次。固定 retry offset 为：

```text
ceil(total_timeout_ms * k / (max_retries + 1))
```

因此默认是 8、15、23 ms，并在 30 ms 失败。最大 retry 配置限制为 8；所有 offset
只在 Init 中预计算，周期性 Peek 不运行迟到循环或软件除法。UART busy 不消耗 retry，
但总期限一旦初发 Commit 后不会暂停。

ACK 必须携带初发帧的 sequence，并由 Device 验证 TYPE、payload 和 session 后调用
`Algorithm_ReliableLink_Acknowledge(link, sequence, now_ms)`。到达或超过总期限的 ACK
被拒绝，即使在此之前没有再次 Peek。ACK 本身不得送入 ReliableLink，也不要求 ACK。

毫秒判断使用无符号差值，支持一次 `uint32_t` 回绕；调用方必须提供单调时钟，且一次
未决事务不能跨越完整的 `2^32` ms 周期。sequence 从 `UINT16_MAX` 自然回到 0。

## 接收去重

Receiver 按 `(nonzero session, sequence)` 识别立即重发。相同键返回
`DUPLICATE_REACK`，调用方重发 ACK 但不再次交付；新 session 即使 sequence 相同也会交付。
session 必须是对端每次启动都更新的 32 位 boot epoch。单独使用 sequence 不能抵抗单边重启。

Receiver 适用于有序、单对端 stop-and-wait 通道；不同物理/逻辑对端必须使用不同实例。

## Build and tests

选用它的 Product 先加入 `APP/Algorithm/Packet`，再加入本目录并链接
`algorithm_reliable_link`。

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\run_gcc_tests.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\run_arm_compile.ps1
```
