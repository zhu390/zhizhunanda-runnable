# AixProbe RA6M5 调试配置

- 目标器件：`R7FA6M5BF2CBG`（RA6M5，Cortex-M33）
- 调试器：WCH CMSIS-DAP 2.0（USB `1A86:8011`）
- 传输：SWD，初始频率 500 kHz
- SVD：`R7FA6M5BH.svd`
- MCP：`http://10.176.44.5:8080/mcp`

`renesas_ra6m5.cfg` 只启用在线调试能力，不声明 Flash bank。固件烧录继续使用 Keil/Renesas 工具，禁止使用 AI-Link `flash_write` 的默认 `0x08000000` 地址。

连接顺序：

1. DAP 的 `SWDIO`、`SWCLK`、`GND`、`VTref/3.3V sense` 与 RA6M5 对应连接，建议同时连接 `nRESET`。
2. DAP 插到 AixProbe 支持供电的 USB 7/9 口，不使用 8 口。
3. AixProbe 切换到 USB Host，`lsusb` 必须能看到 DAP VID/PID。
4. 启动 AI-Link 后，通过 MCP 依次执行 `connect`、`halt`、寄存器/内存读取、`resume`。

WCH 探针的 USB 产品名是 `WCH-Link`，OpenOCD 不会按名称自动选中，配置中必须显式指定 `cmsis-dap vid_pid 0x1a86 0x8011`。当前实测 SW-DP IDCODE 为 `0x6BA02477`。
