# USART1 串口调试

## 连接关系

板上原用于 K230 的接口直接连接 STM32F103 的 USART1，也可以连接 USB 转 TTL
模块进行电脑联调，不需要安装 K230：

| USB 转 TTL | STM32F103 |
| :--- | :--- |
| TXD | PA10 / USART1_RX |
| RXD | PA9 / USART1_TX |
| GND | GND |

信号电平必须为 3.3 V。除非确认需要由 USB 转 TTL 给整板供电，否则不要连接模块的
VCC 或 5 V 引脚。

串口参数固定为 `115200-8-N-1`、无硬件流控。当前电脑识别出的物理串口为
`COM14`，拔插模块后应重新枚举确认。

## 协议

通信使用 ASCII 行协议。每条命令以 `LF` 结束，固件同时接受 `CRLF`；命令不区分
大小写并忽略首尾空格。命令内容最长 96 字节，空行直接忽略。

| 命令 | 响应 / 行为 |
| :--- | :--- |
| `PING` | `OK PONG` |
| `HELP` | `OK HELP PING STATUS STREAM OLED` |
| `STATUS` | 返回状态、运行时间、标定状态、循迹结果和左右占空比 |
| `STREAM <ms>` | 周期状态上报；`0` 关闭，其他值范围为 50～5000 ms |
| `OLED <text>` | 在 OLED 第 7 行显示 `UART:<text>`；限 1～16 个可打印 ASCII 字符 |

正常响应以 `OK` 开头，错误以 `ERR` 开头，板端主动上报以 `EVT` 开头。固件启动后
发送 `EVT BOOT state=WHITE`，周期状态使用 `EVT STATUS ...`。`OLED` 只修改调试
显示内容，不能启动电机、切换标定状态或修改 PID 参数。

常见错误：

| 错误 | 含义 |
| :--- | :--- |
| `ERR UNKNOWN_COMMAND` | 命令不存在 |
| `ERR INVALID_ARGUMENT` | 参数缺失、格式错误或超出范围 |
| `ERR LINE_TOO_LONG` | 命令内容超过 96 字节 |
| `ERR RX_OVERFLOW` | 接收环形缓冲区溢出，当前行已丢弃 |
| `ERR UART_RX` | USART 检测到接收错误并已恢复接收 |

## Python 工具

仓库内的 `tools/serial_debug.py` 使用 `pyserial`。安装依赖：

```powershell
python -m pip install -r tools/requirements.txt
```

常用命令：

```powershell
# 枚举端口及 USB 设备信息
python tools/serial_debug.py --list

# 发送命令并监听 3 秒
python tools/serial_debug.py --port COM14 --send PING --listen-seconds 3

# 一次发送多条命令，并同时查看原始字节
python tools/serial_debug.py --port COM14 --send PING --send STATUS `
  --listen-seconds 3 --hex

# 交互模式，输入 /quit 退出
python tools/serial_debug.py --port COM14 --interactive
```

`--listen-seconds 0` 表示持续监听。输出包含本机时间戳以及 `TX`、`RX` 方向；脚本
在正常结束、异常或 `Ctrl+C` 后关闭串口。

## LLCOM

[LLCOM](https://github.com/chenxuuu/llcom) 用于人工长时间调试。使用 GitHub Release
中的 x64 便携版，配置如下：

- 端口：`COM14`
- 波特率：`115200`
- 数据位：8
- 校验：None
- 停止位：1
- 流控：None
- 发送结尾：追加 `LF`（`0A`）
- 接收：文本模式；需要排查协议时同时开启 HEX 和自动日志

LLCOM 和 Python 工具不能同时打开同一个 COM 口。出现“拒绝访问”或“端口被
占用”时，先关闭另一个串口工具；如果程序异常退出，重新插拔 USB 转 TTL 后再次
执行 `-List`。LLCOM 二进制只作为电脑端独立工具使用，不提交到本仓库。

## 联调顺序

1. 断开电机电源，确认 TX/RX 交叉连接且共地。
2. 打开串口后复位 F103，确认收到 `EVT BOOT state=WHITE`。
3. 发送 `PING` 并确认收到 `OK PONG`。
4. 发送 `STATUS` 检查板端状态。
5. 使用 `STREAM 100` 观察持续数据，结束时发送 `STREAM 0`。
