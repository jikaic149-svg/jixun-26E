# CRC

提供可重入的 reflected CRC-16/IBM（poly `0xA001`）和 Modbus 初值封装，用于迁移
`jixun3` 蓝牙协议。旧实现的 `crc_16()` 与 `crc_modbus()` 参数完全相同，本实现明确
保留其线协议结果，同时删除 512 字节全局查表和非线程安全的延迟初始化。
