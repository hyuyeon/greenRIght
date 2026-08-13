#ifndef BUZZER_H_
#define BUZZER_H_

#include <stdint.h>
#include "main.h"
/*
 * PC0  -> Buzzer
 * TIM7 -> Buzzer frequency timer
 *
 * 수동 부저 기준
 */

extern volatile uint8_t buzzer_busy;
extern volatile uint8_t melody_playing;

/* 초기화 */
void Buzzer_GPIO_Init(void);
void TIM7_Buzzer_Init(void);

/* 단일 음 출력 */
void Buzzer_Play_ms(uint32_t freq_hz, uint32_t duration_ms);
void Buzzer_Stop(void);

/* 종료음 */
void Buzzer_StartExitMelody(void);

/* 시스템 종료 시 호출 */
void System_Exit(void);

/* 완전 종료 */
void Buzzer_DeInit(void);

/* TIM7 IRQ Handler에서 호출되는 함수 */
void TIM7_Buzzer_IRQHandler(void);

#endif /* BUZZER_H_ */
