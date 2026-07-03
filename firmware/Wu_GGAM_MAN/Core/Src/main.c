//main.c
#include "main.h"
#include "string.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <math.h>
#include "uart.h"
#include "i2c2.h"
#include "adxl345.h"
#include "led.h"
#include "exti_button.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

volatile State_t state = IDLE;

/* 함수 선언 -----------------------------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* MAIN ----------------------------------------------------------------------*/
int main(void)
{
  int16_t ax, ay, az;
  float roll, pitch;

  uint32_t blink_tick = 0;      // ← 추가
  uint8_t blink_on = 0;         // ← 추가 (현재 켜짐/꺼짐 상태)

  HAL_Init();
  SystemClock_Config();
  MX_USART3_UART_Init();
  I2C2_Init();
  LED_Init();
  EXTI_Button_Init();

  HAL_Delay(100);
  Uart3_Printf("ADXL345 Initialization...\r\n");

  /* --- 통신 디버깅 코드 시작 --- */
  I2C2_Start();
  I2C2_Address(ADXL345_ADDR_W);
  I2C2_Write(0x00); // DEVID 레지스터 주소

  I2C2_Start(); // Repeated Start
  I2C2_Address(ADXL345_ADDR_R);
  uint8_t dev_id = I2C2_Read_Nack();
  I2C2_Stop();

  Uart3_Printf("Device ID (Expected: 229) : %d\r\n", dev_id);

  if(dev_id != 229) {
      Uart3_Printf("ERROR: I2C Communication Failed! Check Wiring & I2C Address.\r\n");
      while(1) { HAL_Delay(1000); }
  }
  /* --- 통신 디버깅 코드 끝 --- */

  ADXL345_Init();
  Uart3_Printf("ADXL345 Ready!\r\n");

  while(1)
    {
        // 딜레이 없이 센서 데이터 지속적 업데이트
        ADXL345_ReadXYZ(&ax, &ay, &az);
        RollPitch_Calc(ax, ay, az, &roll, &pitch);

        // --- 상태 제어 로직 ---
        switch(state)
        {
        case IDLE:
            GPIOA->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BR6;   // 둘 다 끔
            blink_on = 0;                                   // 다음 점등을 켜짐부터 시작
            break;

        case WAIT_STEER_RIGHT:
            if(roll >= 40.0f) state = WAIT_RETURN_RIGHT;
            break;                                          // LED는 아래 공통 처리

        case WAIT_RETURN_RIGHT:
            if(fabsf(roll) <= 5.0f) state = IDLE;
            break;

        case WAIT_STEER_LEFT:
            if(roll <= -40.0f) state = WAIT_RETURN_LEFT;
            break;

        case WAIT_RETURN_LEFT:
            if(fabsf(roll) <= 5.0f) state = IDLE;
            break;
        }

        // --- UART 출력 로직 (1초 주기) ---
        if(state != IDLE)
        {
            if(HAL_GetTick() - blink_tick >= 500)   // 500ms마다 토글
            {
                blink_tick = HAL_GetTick();
                blink_on = !blink_on;
            }

            // 현재 어느 방향인지에 따라 해당 LED만 깜빡
            if(state == WAIT_STEER_RIGHT || state == WAIT_RETURN_RIGHT)
            {
                if(blink_on) GPIOA->BSRR = GPIO_BSRR_BS5;
                else         GPIOA->BSRR = GPIO_BSRR_BR5;
            }
            else  // LEFT 계열
            {
                if(blink_on) GPIOA->BSRR = GPIO_BSRR_BS6;
                else         GPIOA->BSRR = GPIO_BSRR_BR6;
            }
        }
    }
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) { Error_Handler(); }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
