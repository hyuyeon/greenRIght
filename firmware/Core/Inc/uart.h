#ifndef __UART_H
#define __UART_H

#include "main.h"
#include <stdarg.h>

extern UART_HandleTypeDef huart3;

void MX_USART3_UART_Init(void);

void UART3_Send_Byte(char ch);
void UART3_Send_String(char *p);
void Uart3_Printf(char *fmt, ...);

#endif