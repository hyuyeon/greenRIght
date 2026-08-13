/*
 * led.c
 *
 *  Created on: 2026. 7. 6.
 *      Author: 한국전파진흥협회
 */


#include "led.h"

void LED_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // PA5, PA6 출력 모드
    GPIOA->MODER &= ~((3 << (5 * 2)) | (3 << (6 * 2)));
    GPIOA->MODER |=  ((1 << (5 * 2)) | (1 << (6 * 2)));

    // Push-Pull
    GPIOA->OTYPER &= ~((1 << 5) | (1 << 6));

    // High Speed
    GPIOA->OSPEEDR |= ((3 << (5 * 2)) | (3 << (6 * 2)));

    // 초기 상태 OFF
    GPIOA->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BR6;
}
