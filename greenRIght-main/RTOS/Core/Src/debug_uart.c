#include "debug_uart.h"

#include "stm32f4xx.h"

#include <stdarg.h>
#include <stdio.h>

static void Uart3_Vprintf(const char *fmt, va_list ap)
{
    if (fmt == 0) {
        return;
    }

    char string[256];
    (void)vsnprintf(string, sizeof(string), fmt, ap);
    UART3_Send_String(string);
}

void UART3_Send_Byte(char ch)
{
    if (ch == '\n') {
        while ((USART3->SR & USART_SR_TXE) == 0U) { }
        USART3->DR = (uint16_t)'\r';
    }

    while ((USART3->SR & USART_SR_TXE) == 0U) { }
    USART3->DR = (uint16_t)(uint8_t)ch;
}

void UART3_Send_String(const char *str)
{
    if (str == 0) {
        return;
    }

    while (*str != '\0') {
        UART3_Send_Byte(*str++);
    }

    while ((USART3->SR & USART_SR_TC) == 0U) { }
}

void Uart3_Logf(uint8_t level, const char *fmt, ...)
{
    if (UART3_LOG_ENABLED(level) == 0) {
        return;
    }

    va_list ap;
    va_start(ap, fmt);
    Uart3_Vprintf(fmt, ap);
    va_end(ap);
}

void Uart3_Printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    Uart3_Vprintf(fmt, ap);
    va_end(ap);
}
