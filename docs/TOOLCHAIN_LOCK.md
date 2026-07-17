# Toolchain lock

Validated on 2026-07-14:

| Tool | Version / path |
|---|---|
| GNU Arm GCC | 10.3.1, `D:\GCC\ARMGCC\10 2021.10` |
| GDB | 10.2.90 |
| CMake | 3.30.2 |
| Ninja | 1.12.1 |
| MSPM0 SDK | 2.10.00.04, sibling `TI ENV` |
| SysConfig | 1.27.1+4634, sibling `TI ENV` |
| G3519 OpenOCD | 0.12.0+dev, custom 2026-06-15 |
| G3507 OpenOCD | xPack 0.12.0-7 |

The template derives SDK, SysConfig and SVD paths from its parent directory:

```text
TI MSPM0/
├─ TI ENV/
└─ TI Template/
```

## OpenOCD SHA-256

```text
custom openocd.exe
CB97B5EB254FF359772A4AF0438CF7987E05FDA038DBFDE73B767390A1797D2F

custom libusb-1.0.dll
EDDD25DF84B078158F477DFCE80B26500F91ECDB5AF850581D2F5D09D6F21C26

custom libhidapi-0.dll
0133795F06C3BA6C5A21EB1A1B24CFCC80B38F979719D5BD29A5786AE87ED744

custom target/ti/mspm0.cfg
49EFF22E64336FEB787314D26EDFCC6E0930EB858771345E5B4C8E3CDEC28590

stock xPack openocd.exe
10567F01D37C66A0975537960D66E61F7B4E2A17EB2C9637E2C2CBC569F70C79

config/openocd/mspm0_daplink_custom.cfg
B4BDCB031A086E154525B82C5E9133ED6E5EB8BCAEF505D0552933A2E605E4A6

config/openocd/mspm0_xds110.cfg
3BF909DF2BE66F59DFE2712AB59E1EB8D5CBF67C843E30CE978436F531AA0EED
```

Both project cfg hashes are stored in `tools/mspm0.ps1`, because a legitimate
cfg edit must update its expected value in the same change. The project cfgs
contain no serial numbers; they source the matching machine-local file under
sibling `TI ENV/local/openocd/`. Those local files intentionally are not hash
locked, so each machine can select its own probe without changing the template.
