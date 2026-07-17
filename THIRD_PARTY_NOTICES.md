# Third-party notices

根目录 `LICENSE` 只适用于模板贡献者新编写并拥有版权的代码和文档，不替代、
扩展或重新许可本文件列出的第三方材料。第三方文件自身的许可证声明与许可证
文件具有优先效力。

## Texas Instruments MSPM0 SDK 2.10.00.04

模板从本机 SDK 引用 TI Device headers、DriverLib、CMSIS headers、启动文件和
链接脚本，并通过 TI SysConfig 生成 BSP 配置：

```text
D:\TI MSPM0\TI ENV\mspm0_sdk_2_10_00_04
```

TI 文件及 SysConfig 生成文件保留各自的 BSD-3-Clause 许可证声明。
外部引用的 CMSIS Core headers 使用 Apache-2.0 许可证。
这些内容包括从 SDK 直接参与构建的 Device/CMSIS headers、DriverLib、启动文件、
链接脚本，以及构建目录内由 SysConfig 生成的文件；它们不受根 MIT 许可证覆盖。

本机安装是精简的 source-only SDK，不包含 `examples` 或 `docs` 目录，
因此本模板不声明代码来自某个本机示例工程。

## FreeRTOS Kernel V11.3.0

- 上游仓库：`https://github.com/FreeRTOS/FreeRTOS-Kernel.git`
- 版本：`V11.3.0`
- 来源记录：`9b777ae5c5b8e9e456065a00294d1e5f5f9facf5`
- 许可证：MIT
- 许可证文件：`FreeRTOS/Kernel/LICENSE.md`

除本地化翻译的 `FreeRTOS/Kernel/README.md` 外，`FreeRTOS/Kernel/**` 的源码、
头文件和许可证均保留上游原始字节。该第三方目录适用其目录内 MIT 许可证，
而不是用根许可证取代上游声明。

## 逐飞功能参考边界

本模板不包含、不链接逐飞 MSPM0G3507 Library 的 GPL 源码或二进制库。
后续需要的兼容能力只能依据 TI 官方资料、器件数据手册和公开功能需求独立实现，
并为每个实现记录资料来源和测试。

`jikaic149-svg/-25E` 只作为用户授权的目录和功能需求参考。其硬件配置不会在
封装、引脚和时钟未经确认时自动迁入 BSP。
