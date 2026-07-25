#ifndef SERIAL_CONSOLE_H
#define SERIAL_CONSOLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

typedef struct
{
    const char *state;
    uint32_t uptime_ms;
    bool calibrated;
    bool line_valid;
    int16_t position;
    uint16_t confidence;
    uint8_t left_duty_percent;
    uint8_t right_duty_percent;
} SerialConsole_StatusTypeDef;

typedef void (*SerialConsole_StatusProviderTypeDef)(
    SerialConsole_StatusTypeDef *status);
typedef bool (*SerialConsole_DisplayHandlerTypeDef)(const char *text);

HAL_StatusTypeDef SerialConsole_Init(
    UART_HandleTypeDef *uart,
    SerialConsole_StatusProviderTypeDef status_provider,
    SerialConsole_DisplayHandlerTypeDef display_handler);
void SerialConsole_Process(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_CONSOLE_H */
