
//stm32f4xx_it.c

#include "main.h" // 여기서 State_t와 extern state를 가져옵니다.
#include "stm32f4xx_it.h"

/* --- 외부 함수 선언 (main.c에 선언됨) --- */
extern void Uart3_Printf(char *fmt, ...);

/******************************************************************************/
/* Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
void NMI_Handler(void) { while (1) {} }
void HardFault_Handler(void) { while (1) {} }
void MemManage_Handler(void) { while (1) {} }
void BusFault_Handler(void) { while (1) {} }
void UsageFault_Handler(void) { while (1) {} }
void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/******************************************************************************/

void EXTI15_10_IRQHandler(void)
{
    static uint32_t last_press = 0;

    if (EXTI->PR & EXTI_PR_PR12)
    {
        EXTI->PR = EXTI_PR_PR12;

        if (HAL_GetTick() - last_press > 200)   // 200ms 이내 재트리거 무시
        {
            last_press = HAL_GetTick();
            if (state == IDLE)
                state = WAIT_STEER_RIGHT;
            else
                state = IDLE;
        }
    }
    if (EXTI->PR & EXTI_PR_PR11)
        {
            EXTI->PR = EXTI_PR_PR11;

            if (HAL_GetTick() - last_press > 200)   // 200ms 이내 재트리거 무시
            {
                last_press = HAL_GetTick();
                if (state == IDLE)
                    state = WAIT_STEER_LEFT;
                else
                    state = IDLE;
            }
        }

}
