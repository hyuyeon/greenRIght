#include "exti_button.h"

void EXTI_Button_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // PC11, PC12 입력 모드
    GPIOC->MODER &= ~((3 << (11 * 2)) | (3 << (12 * 2)));
    GPIOC->PUPDR &= ~((3 << (11 * 2)) | (3 << (12 * 2)));
    GPIOC->PUPDR |=  ((1 << (11 * 2)) | (1 << (12 * 2)));   // Pull-down

    // EXTI11 ← PC11
    SYSCFG->EXTICR[2] &= ~(0xF << 12);
    SYSCFG->EXTICR[2] |=  (0x2 << 12);

    // EXTI12 ← PC12
    SYSCFG->EXTICR[3] &= ~(0xF << 0);
    SYSCFG->EXTICR[3] |=  (0x2 << 0);

    // Rising Edge Trigger
    EXTI->IMR  |= (EXTI_IMR_IM11 | EXTI_IMR_IM12);
    EXTI->FTSR &= ~(EXTI_FTSR_TR11 | EXTI_FTSR_TR12);
    EXTI->RTSR |=  (EXTI_RTSR_TR11 | EXTI_RTSR_TR12);

    NVIC_EnableIRQ(EXTI15_10_IRQn);
}