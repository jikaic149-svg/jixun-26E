# OpenOCD 调试探针配置

项目 cfg 文件包含固定的接口、传输方式、速度和目标芯片选择，但不保存探针序列号。它们会从模板同级目录读取与当前机器对应的本地配置文件：

```text
TI MSPM0/
├─ TI ENV/local/openocd/
│  ├─ daplink.cfg
│  └─ xds110.cfg
└─ TI Template/config/openocd/
```

每个本机配置文件必须只包含一条生效命令：

```tcl
adapter serial "<probe serial>"
```

可复制 `daplink.local.cfg.example` 或 `xds110.local.cfg.example` 作为起点。示例文件有意不包含真实序列号。VS Code 启动配置和 `tools/mspm0.ps1` 都以工作区根目录作为当前目录运行 OpenOCD，因此相对的 `TI ENV` 引用路径会得到一致解析。
