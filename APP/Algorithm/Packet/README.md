# 数据包组件

`APP/Algorithm/Packet` 是与硬件和 RTOS 无关的 ASCII 数据包编解码器。它不独占 UART、不分配内存、不调度重试，也不定义 ACK 策略。Device 或 Product 通信模块可以将它连接到任意 BSP UART。

## 线上格式

```text
@1|TYPE|SEQ|LEN|PAYLOAD|CRC16\n
```

- 标准发送帧包含 LF 在内最多为 128 字节。
- 编码器始终发送 LF（`0x0A`），从不发送 CR。
- 解析器也接受紧邻 LF 之前的一个 CR，前提是接收到的完整 CRLF 帧仍不超过 128 字节。
- `TYPE` 必须匹配 `[A-Z][A-Z0-9_]{0,15}`。
- `SEQ` 是规范十进制 `uint16_t`，范围为 `0`～`65535`，不得包含前导零。
- `LEN` 是规范十进制有效载荷字节数，不得包含前导零。
- `PAYLOAD` 只能包含 `0x20`～`0x7E` 的可打印 ASCII。它可以包含 `|`；准确边界由 `LEN` 决定，而不是通过扫描分隔符确定。
- 有效载荷的绝对上限为 111 字节。较长的 TYPE 或 SEQ 会压缩可用载荷，因为完整帧长度限制仍然生效。
- `CRC16` 必须正好是四个大写十六进制数字。

CRC-16/CCITT-FALSE 使用多项式 `0x1021`、初始值 `0xFFFF`，输入和输出均不反射，异或输出值为 `0x0000`。校验范围包含从起始 `@` 到四位 CRC 数字之前分隔符 `|` 的所有线上字节，首尾均包含；CRC 数字和行结束符不参与计算。标准测试向量 `123456789` 的结果为 `0x29B1`。

例如，下面帧中方括号内的文本是 CRC 覆盖范围：

```text
[@1|VISION|42|3|RED|]5202\n
```

## API 与所有权

`Algorithm_Packet_Encode()` 仅在输出缓冲区能够容纳完整帧时才写入。缓冲区过小时不会写入任何内容，并会报告所需大小。

每个 `Algorithm_Packet_Parser` 都有独立状态，并使用调用方提供的不小于 `ALGORITHM_PACKET_PARSER_STORAGE_SIZE` 字节的缓冲区。解析完成后的 `Algorithm_Packet_View` 指向该缓冲区，其有效期只持续到同一解析器的下一次 feed 调用；需要长期保存的数据必须复制。

字段格式错误、不可打印载荷、长度不一致、CRC 错误和超长输入都会被拒绝。如果在 LF 之前检测到故障，解析器只计数一次，并丢弃数据直到下一个 LF，之后才接受新帧。API 使用错误不计为线上错误。计数器在 `UINT32_MAX` 饱和。

解析器设计为由一个串行字节消费者使用。如果 ISR 接收 UART 字节，应先把字节放入 SPSC 环形缓冲区，再由一个前台或 FreeRTOS 上下文调用解析器；不得并发调用同一个解析器实例。

## 构建与验证

本组件有意不加入模板顶层默认构建。Product 必须通过 `add_subdirectory(APP/Algorithm/Packet)` 显式启用，并链接 `algorithm_packet`。

主机边界测试：

```powershell
pwsh -File .\tests\run_gcc_tests.ps1
```

如果 Windows Application Control 允许编译、但阻止执行新生成的程序，仍可使用下面的命令对同一严格测试翻译单元进行只编译检查：

```powershell
pwsh -File .\tests\run_gcc_tests.ps1 -CompileOnly
```

使用锁定版本 GNU Arm 工具链执行 Cortex-M0+ 只编译检查：

```powershell
pwsh -File .\tests\run_arm_compile.ps1
```
