# 单生产者单消费者环形缓冲区

`Algorithm_SPSC_Ring` 是与 MCU 和 RTOS 无关的字节环形缓冲区，存储空间由调用方提供。只能由一个生产者调用写入 API，并由一个消费者调用读取 API。批量写入要么整体原子提交，要么整体拒绝，不会发布不完整的数据帧。

需要使用下面的选项显式启用固件目标：

```text
TEMPLATE_ENABLE_ALGORITHM_SPSC_RING=ON
```

仓库保留了主机测试源码和 `tests/run_gcc_tests.ps1`。当前工作站的 Smart App Control 策略可能阻止运行新生成的未签名测试 EXE；不要关闭或绕过该策略。本地备用验证方式是使用锁定版本的 GNU Arm 工具链进行编译和符号审计。
