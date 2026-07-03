#include "uart.h"
#include <stdio.h>

UART_HandleTypeDef huart3;

void UART3_Send_Byte(char ch)
{
    if (ch == '\n')
    {
        USART3->DR = 0x0D;
        while (((USART3->SR >> 7) & 0x1) == 0);
    }

    USART3->DR = ch;
    while (((USART3->SR >> 7) & 0x1) == 0);
}

void UART3_Send_String(char *p)
{
    while (*p)
    {
        UART3_Send_Byte(*p++);
    }
}

void Uart3_Printf(char *fmt, ...)
{
    va_list ap;
    char string[256];

    va_start(ap, fmt);
    vsprintf(string, fmt, ap);
    va_end(ap);

    UART3_Send_String(string);
}

void MX_USART3_UART_Init(void)
{
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler();
    }
}