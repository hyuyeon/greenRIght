/*
 * uart_debug.c
 *
 *  Created on: 2026. 7. 3.
 *      Author: 한국전파진흥협회
 */


#include "main.h"
#include "uart_debug.h"
#include <stdio.h>
#include <stdarg.h>

void _send_char(char ch)
{
    if (ch == '\n')
    {
        USART3->DR = 0x0d;
        while (((USART3->SR >> 7) & 0x1) == 0);
    }

    USART3->DR = ch;
    while (((USART3->SR >> 7) & 0x1) == 0);
}

void _send_string(char *p)
{
    while (*p)
    {
        _send_char(*p++);
    }
}

void _uart_printf(char *fmt, ...)
{
    va_list ap;
    char string[256];

    va_start(ap, fmt);
    vsprintf(string, fmt, ap);
    va_end(ap);

    _send_string(string);
}
