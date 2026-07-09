/*
 * speed_sensor.c
 *
 *  Created on: 2026. 7. 3.
 *      Author: 한국전파진흥협회
 */


#include "main.h"
#include "speed_sensor.h"

#define PI_F                     3.1415926535f

#define WHEEL_DIAMETER_CM        7.0f
#define PULSES_PER_ROTATION      2.0f
#define SPEED_SAMPLE_PERIOD_MS   1000U
#define MARK_LOCKOUT_MS          10U

static volatile uint8_t stable_state = 0;
static volatile uint32_t speed_sample_elapsed_ms = 0;
static volatile uint32_t pulse_count = 0;
static volatile uint32_t last_edge_tick = 0;
static volatile uint32_t rejected_cnt = 0;
static volatile float speed_mps = 0.0f;
volatile uint32_t msTick = 0;

void SpeedSensor_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

    GPIOB->MODER &= ~(3U << (7U * 2U));
    GPIOB->MODER |=  (1U << (7U * 2U));

    GPIOE->MODER &= ~(3U << (2U * 2U));
    GPIOE->PUPDR &= ~(3U << (2U * 2U));

    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    TIM3->PSC = 8400U - 1U;
    TIM3->ARR = 10U - 1U;
    TIM3->CNT = 0;
    TIM3->DIER |= TIM_DIER_UIE;

    NVIC_SetPriority(TIM3_IRQn, 6);
    NVIC_EnableIRQ(TIM3_IRQn);

    TIM3->CR1 |= TIM_CR1_CEN;
}

static void Sensor_Sample_1ms(void)
{
    static uint8_t last_raw = 0;
    static uint8_t same_cnt = 0;

    uint8_t raw = (GPIOE->IDR & (1U << 2)) ? 1U : 0U;

    if (raw == last_raw)
    {
        if (same_cnt < 3U)
            same_cnt++;
    }
    else
    {
        same_cnt = 0;
        last_raw = raw;
    }

    if (same_cnt >= 3U && raw != stable_state)
    {
        stable_state = raw;

        if (stable_state == 1U)
        {
            pulse_count++;
            GPIOB->ODR ^= (1U << 7);
        }
        else
        {
            rejected_cnt++;
        }
    }
}

float Speed_GetMps(void)
{
    return speed_mps;
}

void TIM3_IRQHandler(void)
{
    if (TIM3->SR & TIM_SR_UIF)
    {
        TIM3->SR &= ~TIM_SR_UIF;

        Sensor_Sample_1ms();

        speed_sample_elapsed_ms++;

        if (speed_sample_elapsed_ms >= SPEED_SAMPLE_PERIOD_MS)
        {
            speed_sample_elapsed_ms = 0;

            uint32_t captured_pulses;

            __disable_irq();
            captured_pulses = pulse_count;
            pulse_count = 0;
            __enable_irq();

            float rotations = (float)captured_pulses / PULSES_PER_ROTATION;
            float wheel_circumference_cm = WHEEL_DIAMETER_CM * PI_F;
            float distance_m = (rotations * wheel_circumference_cm) / 100.0f;

            speed_mps = distance_m / (SPEED_SAMPLE_PERIOD_MS / 1000.0f);

            if (captured_pulses == 0)
            {
                speed_mps = 0.0f;
            }
        }
    }
}

