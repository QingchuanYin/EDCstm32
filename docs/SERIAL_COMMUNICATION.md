# 串口通信使用指南

## 硬件配置

**串口**: USART1  
**波特率**: 115200  
**数据位**: 8  
**停止位**: 1  
**校验位**: 无  
**流控**: 无

## 引脚连接

根据你的STM32F103配置，USART1通常使用：
- **TX**: PA9
- **RX**: PA10

连接USB转TTL模块：
- STM32 PA9 (TX) → USB转TTL RX
- STM32 PA10 (RX) → USB转TTL TX
- GND → GND

## 支持的命令

串口控制台支持以下命令（不区分大小写）：

### 1. PING - 连接测试
```
PING
```
**响应**: `OK PONG`

**用途**: 测试串口通信是否正常

---

### 2. HELP - 查看命令列表
```
HELP
```
**响应**: `OK HELP PING STATUS STREAM OLED`

**用途**: 列出所有可用命令

---

### 3. STATUS - 获取系统状态
```
STATUS
```
**响应示例**:
```
OK STATUS state=READY uptime_ms=12345 calibrated=1 line_valid=1 position=128 confidence=150 left=50 right=50
```

**字段说明**:
- `state`: 当前状态 (WHITE/SWEEP/CAL/READY/RUN/LOST/FAIL/FAULT)
- `uptime_ms`: 系统运行时间（毫秒）
- `calibrated`: 灰度传感器是否已校准 (0=否, 1=是)
- `line_valid`: 是否检测到线 (0=否, 1=是)
- `position`: 线的位置 (-1000~1000)
- `confidence`: 检测置信度
- `left`: 左电机占空比百分比 (0-100)
- `right`: 右电机占空比百分比 (0-100)

---

### 4. STREAM - 自动状态上报
```
STREAM <period_ms>
```

**参数**:
- `period_ms`: 上报周期（毫秒）
  - 范围: 50 ~ 5000
  - 设置为 0 停止上报

**示例**:
```
STREAM 100      # 每100ms上报一次状态
STREAM 0        # 停止上报
```

**响应**: 
```
OK STREAM period_ms=100
```

之后每隔指定周期会自动发送：
```
EVT STATUS state=RUN uptime_ms=15000 calibrated=1 line_valid=1 position=-50 confidence=200 left=60 right=40
```

---

### 5. OLED - 显示串口文本
```
OLED 123
```

**响应**: `OK OLED text=123`

OLED 第 7 行显示 `UART:123`。文本限 1～16 个可打印 ASCII 字符，该命令不会改变
电机、标定或循迹状态。

---

## 启动消息

系统上电后会自动发送启动消息：
```
EVT BOOT state=WHITE
```

---

## 错误响应

- `ERR UNKNOWN_COMMAND` - 未知命令
- `ERR INVALID_ARGUMENT` - 参数无效
- `ERR LINE_TOO_LONG` - 命令行过长（超过96字符）
- `ERR RX_OVERFLOW` - 接收缓冲区溢出
- `ERR UART_RX` - UART接收错误

---

## 使用示例

### 使用串口终端工具

#### Windows - PuTTY
1. 下载并安装 [PuTTY](https://www.putty.org/)
2. 选择 Serial 连接类型
3. 设置串口号（如 COM3）
4. 速度设置为 115200
5. 点击 Open

#### Windows/Mac/Linux - Arduino IDE 串口监视器
1. 打开 Arduino IDE
2. 工具 → 串口监视器
3. 选择对应串口
4. 设置波特率为 115200
5. 选择"换行符"或"CR+LF"

#### Linux/Mac - minicom
```bash
minicom -D /dev/ttyUSB0 -b 115200
```

#### Python 脚本示例
```python
import serial
import time

# 打开串口
ser = serial.Serial('COM3', 115200, timeout=1)
time.sleep(2)  # 等待连接建立

# 测试连接
ser.write(b'PING\n')
print(ser.readline().decode())

# 获取状态
ser.write(b'STATUS\n')
print(ser.readline().decode())

# 启动自动上报
ser.write(b'STREAM 200\n')
print(ser.readline().decode())

# 持续接收数据
try:
    while True:
        if ser.in_waiting:
            line = ser.readline().decode().strip()
            print(line)
except KeyboardInterrupt:
    # 停止上报
    ser.write(b'STREAM 0\n')
    ser.close()
```

---

## 状态机说明

系统状态转换流程：

1. **WHITE** - 等待白色地面校准
   - 按K1键 → 采集白色样本 → SWEEP

2. **SWEEP** - 等待开始黑色扫描
   - 按K1键 → 开始5秒扫描 → CAL

3. **CAL** - 校准中（扫描黑色线）
   - 成功 → READY
   - 失败 → FAIL

4. **READY** - 就绪，等待开始跟踪
   - 按K1键 → RUN

5. **RUN** - 运行中（循迹模式）
   - 丢失线 → LOST
   - 按K1键 → READY

6. **LOST** - 丢失线（仍在运行，尝试恢复）

7. **FAIL** - 校准失败
   - 按K1键 → 重新开始 → WHITE

---

## 调试技巧

### 1. 验证通信
```
PING
```
如果收到 `OK PONG`，说明通信正常。

### 2. 监控实时数据
```
STREAM 100
```
每100ms获取一次系统状态，适合调试循迹算法。

### 3. 单次查询
```
STATUS
```
获取当前状态快照，不影响系统运行。

### 4. 检查校准
发送 `STATUS` 命令，检查 `calibrated` 字段：
- `calibrated=0` - 未校准，需要执行校准流程
- `calibrated=1` - 已校准

### 5. 监控循迹性能
观察 `STATUS` 输出：
- `position`: 线偏移量，0表示居中
- `confidence`: 越高表示检测越可靠
- `left/right`: 两个电机的输出占空比

---

## 注意事项

1. **命令格式**: 所有命令以换行符结尾（`\n` 或 `\r\n`）
2. **大小写**: 命令不区分大小写，`ping` 和 `PING` 等效
3. **缓冲区**: 命令行最长96字符
4. **上报周期**: STREAM周期建议不低于50ms，避免过载
5. **中断驱动**: 串口使用中断模式，不会阻塞主循环

---

## 故障排除

### 问题1: 无任何输出
- 检查串口线连接（TX、RX是否交叉）
- 检查波特率是否为115200
- 检查USB转TTL驱动是否安装
- 尝试按复位键，应该看到 `EVT BOOT` 消息

### 问题2: 乱码
- 检查波特率设置
- 检查数据位/停止位/校验位配置
- 检查GND是否连接

### 问题3: 命令无响应
- 确认发送了换行符
- 尝试发送 `PING\n`
- 检查是否有错误消息返回

### 问题4: 数据丢失
- 减小STREAM上报频率
- 检查串口终端的缓冲区设置
- 使用流控（如果硬件支持）

---

## 扩展开发

如需添加自定义命令，编辑 `serial_console.c` 中的 `SerialConsole_ProcessCommand` 函数。

参考现有命令格式：
```c
else if (strcmp(start, "YOUR_COMMAND") == 0)
{
    // 处理命令
    (void)SerialConsole_WriteLine("OK YOUR_RESPONSE");
}
```
