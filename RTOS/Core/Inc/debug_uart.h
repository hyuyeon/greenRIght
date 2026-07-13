#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#include <stdint.h>

#ifndef UART3_LOG_LEVEL
#define UART3_LOG_LEVEL 3U
#endif

#define UART3_LOG_ERROR_LEVEL 1U
#define UART3_LOG_INFO_LEVEL  2U
#define UART3_LOG_DEBUG_LEVEL 3U
#define UART3_LOG_TRACE_LEVEL 4U

#define UART3_LOG_ENABLED(level) ((uint32_t)(level) <= (uint32_t)UART3_LOG_LEVEL)

#define LOG_ERROR(...) Uart3_Logf(UART3_LOG_ERROR_LEVEL, __VA_ARGS__)
#define LOG_INFO(...)  Uart3_Logf(UART3_LOG_INFO_LEVEL,  __VA_ARGS__)
#define LOG_DEBUG(...) Uart3_Logf(UART3_LOG_DEBUG_LEVEL, __VA_ARGS__)
#define LOG_TRACE(...) Uart3_Logf(UART3_LOG_TRACE_LEVEL, __VA_ARGS__)

void Uart3_Logf(uint8_t level, const char *fmt, ...);
void Uart3_Printf(const char *fmt, ...);
void UART3_Send_String(const char *str);
void UART3_Send_Byte(char ch);

#endif
