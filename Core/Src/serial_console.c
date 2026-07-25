#include "serial_console.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define SERIAL_CONSOLE_RX_BUFFER_SIZE       256u
#define SERIAL_CONSOLE_TX_BUFFER_SIZE       512u
#define SERIAL_CONSOLE_LINE_MAX              96u
#define SERIAL_CONSOLE_OUTPUT_MAX           192u
#define SERIAL_CONSOLE_RX_PROCESS_BUDGET     64u
#define SERIAL_CONSOLE_STREAM_MIN_MS         50u
#define SERIAL_CONSOLE_STREAM_MAX_MS       5000u

static UART_HandleTypeDef *console_uart;
static SerialConsole_StatusProviderTypeDef console_status_provider;
static SerialConsole_DisplayHandlerTypeDef console_display_handler;

static uint8_t rx_buffer[SERIAL_CONSOLE_RX_BUFFER_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static uint8_t rx_irq_byte;
static volatile bool rx_overflow_pending;
static volatile bool rx_error_pending;
static volatile bool rx_restart_pending;

static uint8_t tx_buffer[SERIAL_CONSOLE_TX_BUFFER_SIZE];
static volatile uint16_t tx_head;
static volatile uint16_t tx_tail;
static volatile uint16_t tx_inflight_length;
static volatile bool tx_active;

static char command_line[SERIAL_CONSOLE_LINE_MAX + 1u];
static uint16_t command_length;
static bool command_too_long;
static bool discard_until_newline;

static uint32_t stream_period_ms;
static uint32_t last_stream_ms;
static bool console_initialized;

static uint16_t Ring_Next(uint16_t index, uint16_t size)
{
    index++;
    return (index == size) ? 0u : index;
}

static void SerialConsole_StartTransmit(void)
{
    uint16_t length;
    uint32_t primask;

    if (!console_initialized || (console_uart == NULL))
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (tx_active || (tx_tail == tx_head))
    {
        __set_PRIMASK(primask);
        return;
    }

    if (tx_head > tx_tail)
    {
        length = tx_head - tx_tail;
    }
    else
    {
        length = SERIAL_CONSOLE_TX_BUFFER_SIZE - tx_tail;
    }

    tx_active = true;
    tx_inflight_length = length;
    if (HAL_UART_Transmit_IT(console_uart, &tx_buffer[tx_tail], length) != HAL_OK)
    {
        tx_active = false;
        tx_inflight_length = 0u;
    }
    __set_PRIMASK(primask);
}

static bool SerialConsole_QueueBytes(const uint8_t *data, uint16_t length)
{
    uint16_t free_space;
    uint16_t head;
    uint16_t tail;
    uint16_t used;
    uint16_t i;

    if (!console_initialized || (data == NULL) || (length == 0u))
    {
        return false;
    }

    head = tx_head;
    tail = tx_tail;
    used = (head >= tail) ? (head - tail) :
           (SERIAL_CONSOLE_TX_BUFFER_SIZE - tail + head);
    free_space = SERIAL_CONSOLE_TX_BUFFER_SIZE - used - 1u;
    if (length > free_space)
    {
        return false;
    }

    for (i = 0u; i < length; i++)
    {
        tx_buffer[head] = data[i];
        head = Ring_Next(head, SERIAL_CONSOLE_TX_BUFFER_SIZE);
    }
    tx_head = head;
    SerialConsole_StartTransmit();
    return true;
}

static bool SerialConsole_WriteLine(const char *format, ...)
{
    char output[SERIAL_CONSOLE_OUTPUT_MAX];
    int written;
    va_list args;

    va_start(args, format);
    written = vsnprintf(output, sizeof(output) - 1u, format, args);
    va_end(args);
    if (written < 0)
    {
        return false;
    }
    if ((size_t)written >= (sizeof(output) - 1u))
    {
        written = (int)sizeof(output) - 2;
    }
    output[written++] = '\n';
    return SerialConsole_QueueBytes((const uint8_t *)output,
                                    (uint16_t)written);
}

static void SerialConsole_GetStatus(SerialConsole_StatusTypeDef *status)
{
    memset(status, 0, sizeof(*status));
    status->state = "UNKNOWN";
    status->uptime_ms = HAL_GetTick();
    if (console_status_provider != NULL)
    {
        console_status_provider(status);
    }
    if (status->state == NULL)
    {
        status->state = "UNKNOWN";
    }
}

static void SerialConsole_WriteStatus(const char *prefix)
{
    SerialConsole_StatusTypeDef status;

    SerialConsole_GetStatus(&status);
    (void)SerialConsole_WriteLine(
        "%s state=%s uptime_ms=%lu calibrated=%u line_valid=%u "
        "position=%d confidence=%u left=%u right=%u",
        prefix,
        status.state,
        (unsigned long)status.uptime_ms,
        status.calibrated ? 1u : 0u,
        status.line_valid ? 1u : 0u,
        (int)status.position,
        (unsigned int)status.confidence,
        (unsigned int)status.left_duty_percent,
        (unsigned int)status.right_duty_percent);
}

static bool SerialConsole_ParsePeriod(const char *text, uint32_t *period_ms)
{
    uint32_t value = 0u;

    if ((text == NULL) || (period_ms == NULL) || (*text == '\0'))
    {
        return false;
    }

    while (*text != '\0')
    {
        if ((*text < '0') || (*text > '9'))
        {
            return false;
        }
        value = (value * 10u) + (uint32_t)(*text - '0');
        if (value > SERIAL_CONSOLE_STREAM_MAX_MS)
        {
            return false;
        }
        text++;
    }

    if ((value != 0u) && (value < SERIAL_CONSOLE_STREAM_MIN_MS))
    {
        return false;
    }
    *period_ms = value;
    return true;
}

static void SerialConsole_ProcessCommand(char *line)
{
    char *start = line;
    char *end;
    char *argument;
    uint32_t period_ms;

    while ((*start == ' ') || (*start == '\t'))
    {
        start++;
    }
    end = start + strlen(start);
    while ((end > start) && ((end[-1] == ' ') || (end[-1] == '\t')))
    {
        *--end = '\0';
    }
    if (*start == '\0')
    {
        return;
    }

    argument = start;
    while ((*argument != '\0') &&
           (*argument != ' ') && (*argument != '\t'))
    {
        if ((*argument >= 'a') && (*argument <= 'z'))
        {
            *argument = (char)(*argument - ('a' - 'A'));
        }
        argument++;
    }
    if (*argument != '\0')
    {
        *argument++ = '\0';
        while ((*argument == ' ') || (*argument == '\t'))
        {
            argument++;
        }
    }

    if (strcmp(start, "PING") == 0)
    {
        if (*argument == '\0')
        {
            (void)SerialConsole_WriteLine("OK PONG");
        }
        else
        {
            (void)SerialConsole_WriteLine("ERR INVALID_ARGUMENT");
        }
    }
    else if (strcmp(start, "HELP") == 0)
    {
        if (*argument == '\0')
        {
            (void)SerialConsole_WriteLine(
                "OK HELP PING STATUS STREAM OLED");
        }
        else
        {
            (void)SerialConsole_WriteLine("ERR INVALID_ARGUMENT");
        }
    }
    else if (strcmp(start, "STATUS") == 0)
    {
        if (*argument == '\0')
        {
            SerialConsole_WriteStatus("OK STATUS");
        }
        else
        {
            (void)SerialConsole_WriteLine("ERR INVALID_ARGUMENT");
        }
    }
    else if (strcmp(start, "STREAM") == 0)
    {
        if (!SerialConsole_ParsePeriod(argument, &period_ms))
        {
            (void)SerialConsole_WriteLine("ERR INVALID_ARGUMENT");
            return;
        }
        stream_period_ms = period_ms;
        last_stream_ms = HAL_GetTick();
        (void)SerialConsole_WriteLine("OK STREAM period_ms=%lu",
                                      (unsigned long)period_ms);
    }
    else if (strcmp(start, "OLED") == 0)
    {
        if ((*argument == '\0') || (console_display_handler == NULL) ||
            !console_display_handler(argument))
        {
            (void)SerialConsole_WriteLine("ERR INVALID_ARGUMENT");
            return;
        }
        (void)SerialConsole_WriteLine("OK OLED text=%s", argument);
    }
    else
    {
        (void)SerialConsole_WriteLine("ERR UNKNOWN_COMMAND");
    }
}

static void SerialConsole_AcceptByte(uint8_t byte)
{
    if (byte == '\r')
    {
        return;
    }
    if (byte == '\n')
    {
        if (command_too_long)
        {
            (void)SerialConsole_WriteLine("ERR LINE_TOO_LONG");
        }
        else if (!discard_until_newline)
        {
            command_line[command_length] = '\0';
            SerialConsole_ProcessCommand(command_line);
        }
        command_length = 0u;
        command_too_long = false;
        discard_until_newline = false;
        return;
    }

    if (discard_until_newline || command_too_long)
    {
        return;
    }
    if (command_length >= SERIAL_CONSOLE_LINE_MAX)
    {
        command_too_long = true;
        command_length = 0u;
        return;
    }
    command_line[command_length++] = (char)byte;
}

HAL_StatusTypeDef SerialConsole_Init(
    UART_HandleTypeDef *uart,
    SerialConsole_StatusProviderTypeDef status_provider,
    SerialConsole_DisplayHandlerTypeDef display_handler)
{
    SerialConsole_StatusTypeDef status;

    if ((uart == NULL) || (status_provider == NULL) ||
        (display_handler == NULL))
    {
        return HAL_ERROR;
    }

    console_uart = uart;
    console_status_provider = status_provider;
    console_display_handler = display_handler;
    rx_head = 0u;
    rx_tail = 0u;
    tx_head = 0u;
    tx_tail = 0u;
    tx_inflight_length = 0u;
    command_length = 0u;
    stream_period_ms = 0u;
    rx_overflow_pending = false;
    rx_error_pending = false;
    rx_restart_pending = false;
    tx_active = false;
    command_too_long = false;
    discard_until_newline = false;
    console_initialized = true;

    if (HAL_UART_Receive_IT(console_uart, &rx_irq_byte, 1u) != HAL_OK)
    {
        console_initialized = false;
        return HAL_ERROR;
    }

    SerialConsole_GetStatus(&status);
    (void)SerialConsole_WriteLine("EVT BOOT state=%s", status.state);
    return HAL_OK;
}

void SerialConsole_Process(uint32_t now_ms)
{
    uint8_t byte;
    uint16_t processed = 0u;

    if (!console_initialized)
    {
        return;
    }

    if (rx_restart_pending &&
        (console_uart->RxState == HAL_UART_STATE_READY) &&
        (HAL_UART_Receive_IT(console_uart, &rx_irq_byte, 1u) == HAL_OK))
    {
        rx_restart_pending = false;
    }

    if (rx_error_pending)
    {
        rx_error_pending = false;
        command_length = 0u;
        command_too_long = false;
        discard_until_newline = true;
        (void)SerialConsole_WriteLine("ERR UART_RX");
    }
    if (rx_overflow_pending)
    {
        rx_overflow_pending = false;
        command_length = 0u;
        command_too_long = false;
        discard_until_newline = true;
        (void)SerialConsole_WriteLine("ERR RX_OVERFLOW");
    }

    while ((rx_tail != rx_head) &&
           (processed < SERIAL_CONSOLE_RX_PROCESS_BUDGET))
    {
        byte = rx_buffer[rx_tail];
        rx_tail = Ring_Next(rx_tail, SERIAL_CONSOLE_RX_BUFFER_SIZE);
        SerialConsole_AcceptByte(byte);
        processed++;
    }

    if ((stream_period_ms != 0u) &&
        ((now_ms - last_stream_ms) >= stream_period_ms))
    {
        last_stream_ms = now_ms;
        SerialConsole_WriteStatus("EVT STATUS");
    }
    SerialConsole_StartTransmit();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint16_t next;

    if (!console_initialized || (huart != console_uart))
    {
        return;
    }

    next = Ring_Next(rx_head, SERIAL_CONSOLE_RX_BUFFER_SIZE);
    if (next == rx_tail)
    {
        rx_overflow_pending = true;
    }
    else
    {
        rx_buffer[rx_head] = rx_irq_byte;
        rx_head = next;
    }

    if (HAL_UART_Receive_IT(console_uart, &rx_irq_byte, 1u) != HAL_OK)
    {
        rx_error_pending = true;
        rx_restart_pending = true;
    }
    else
    {
        rx_restart_pending = false;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (!console_initialized || (huart != console_uart))
    {
        return;
    }

    tx_tail = (uint16_t)((tx_tail + tx_inflight_length) %
                         SERIAL_CONSOLE_TX_BUFFER_SIZE);
    tx_inflight_length = 0u;
    tx_active = false;
    SerialConsole_StartTransmit();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (!console_initialized || (huart != console_uart))
    {
        return;
    }

    rx_error_pending = true;
    if (huart->RxState == HAL_UART_STATE_READY)
    {
        rx_restart_pending =
            HAL_UART_Receive_IT(console_uart, &rx_irq_byte, 1u) != HAL_OK;
    }
}
