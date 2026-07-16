/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
#include "uart_debug.h"
#include "i2c1_bus.h"
#include "bno055.h"
#include "speed_sensor.h"
#include "position.h"
#include "can.h"
#include "led.h"
#include "exti_button.h"
#include "adxl345.h"
#include "i2c2.h"
#include <math.h>


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

ETH_TxPacketConfig TxConfig;
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

ETH_HandleTypeDef heth;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
volatile State_t state = IDLE;
static uint32_t last_imu_time = 0;
static uint32_t last_can_time = 0;
static uint32_t last_print_time = 0;
static uint32_t last_adxl_time = 0;
static uint32_t blink_tick = 0;

#define IMU_UPDATE_PERIOD_MS      20U
#define ADXL_UPDATE_PERIOD_MS     20U
#define CAN_TX_PERIOD_MS          50U
#define DEBUG_PRINT_PERIOD_MS    200U

static int32_t used_heading_x100 = 0;

static uint8_t blink_on = 0;
static uint8_t adxl_available = 0;
/* 마지막으로 CAN 송신에 성공한 값 */
static uint8_t previous_speed_mpm = 0U;
static uint16_t previous_x = 0U;
static uint16_t previous_y = 0U;
static uint16_t previous_heading = 0U;
static uint8_t previous_turn_signal = CAN_TURN_OFF;

/* 최초 프레임은 모든 필드를 갱신 */
static uint8_t can_first_tx = 1U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();        // 기본 GPIO, PE2 클럭 포함
  GPIO_Init();           // PB8/PB9 I2C1 설정
  // MX_ETH_Init();      // IMU/속도센서 테스트 중에는 비활성 권장
  MX_USART3_UART_Init();
  // MX_USB_OTG_FS_PCD_Init();
  I2C1_Init();
  I2C2_Init();
  SpeedSensor_Init();
  LED_Init();
  EXTI_Button_Init();
  if (!CAN1_Init_500kbps())
  {
      _uart_printf("CAN Init Failed\n");
  }
  else
  {
      _uart_printf("CAN Init OK\n");
  }
  /* USER CODE BEGIN 2 */
  if (!BNO055_Init())
  {
      _uart_printf("BNO055 Init Failed\n");
      HAL_Delay(1000);
  }
  else
  {
      _uart_printf("BNO055 Init OK\n");


      HAL_Delay(1000);
      BNO055_ReadHeading();
      Position_Reset();

      _uart_printf("Start heading: %ld deg\n",
                   (long)(IMU_GetHeadingX100() / 100));
  }

  _uart_printf("ADXL345 Initialization...\r\n");
  if (ADXL345_Init())
  {
      adxl_available = 1U;
      _uart_printf("ADXL345 Ready!\r\n");
  }
  else
  {
      adxl_available = 0U;
      _uart_printf("ADXL345 disabled. Continue without accel.\r\n");
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      uint32_t now = HAL_GetTick();

      /*
       * 1. IMU Heading 20ms마다 읽기
       * 2. BNO055 heading 값을 로컬 heading으로 사용
       * 3. 현재 속도와 heading으로 x,y 누적
       */
      if ((uint32_t)(now - last_imu_time) >= IMU_UPDATE_PERIOD_MS)
      {
          last_imu_time = now;

          if (BNO055_ReadHeading() && IMU_IsValid())
          {
              int32_t heading_x100 = IMU_GetHeadingX100();
              used_heading_x100 = ApplyHeadingDeadband_X100(heading_x100);

              float current_speed_mps = Speed_GetMps();

              Position_Update(current_speed_mps, used_heading_x100);
          }
      }
      /*
          * 2. ADXL345 Roll/Pitch Update + Turn State Logic
          */
         if (adxl_available &&
             ((uint32_t)(now - last_adxl_time) >= ADXL_UPDATE_PERIOD_MS))
         {
             last_adxl_time = now;

             int16_t ax, ay, az;
             float roll, pitch;

             if (!ADXL345_ReadXYZ(&ax, &ay, &az))
             {
                 adxl_available = 0U;
                 state = IDLE;
                 GPIOA->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BR6;
                 _uart_printf("ADXL345 read failed. Accel disabled.\r\n");
             }
             else
             {
                 RollPitch_Calc(ax, ay, az, &roll, &pitch);

                 switch (state)
                 {
                     case IDLE:
                         GPIOA->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BR6;
                         blink_on = 0;
                         break;

                     case WAIT_STEER_RIGHT:
                         if (roll >= 40.0f)
                         {
                             state = WAIT_RETURN_RIGHT;
                         }
                         break;

                     case WAIT_RETURN_RIGHT:
                         if (fabsf(roll) <= 5.0f)
                         {
                             state = IDLE;
                         }
                         break;

                     case WAIT_STEER_LEFT:
                         if (roll <= -40.0f)
                         {
                             state = WAIT_RETURN_LEFT;
                         }
                         break;

                     case WAIT_RETURN_LEFT:
                         if (fabsf(roll) <= 5.0f)
                         {
                             state = IDLE;
                         }
                         break;

                     default:
                         state = IDLE;
                         break;
                 }
             }
         }

         /*
          * 3. LED Blink Control
          */
         if (state != IDLE)
         {
             if ((uint32_t)(now - blink_tick) >= 500U)
             {
                 blink_tick = now;
                 blink_on = !blink_on;
             }

             if (state == WAIT_STEER_RIGHT || state == WAIT_RETURN_RIGHT)
             {
                 GPIOA->BSRR = blink_on ? GPIO_BSRR_BS5 : GPIO_BSRR_BR5;
                 GPIOA->BSRR = GPIO_BSRR_BR6;
             }
             else if (state == WAIT_STEER_LEFT || state == WAIT_RETURN_LEFT)
             {
                 GPIOA->BSRR = blink_on ? GPIO_BSRR_BS6 : GPIO_BSRR_BR6;
                 GPIOA->BSRR = GPIO_BSRR_BR5;
             }
         }
         else
         {
             GPIOA->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BR6;
         }

         if ((uint32_t)(now - last_can_time) >= CAN_TX_PERIOD_MS)
         {
             float speed_mps;
             uint8_t speed_mpm;
             uint16_t x;
             uint16_t y;
             uint16_t heading;
             uint8_t turn_signal;
             uint16_t timestamp;
             uint8_t update_mask;
             uint8_t tx_success;

             last_can_time = now;

             speed_mps = Speed_GetMps();
             speed_mpm = CAN_SpeedMpsToMpm8(speed_mps);

             /*
              * 현재 Position_GetXcm(), Position_GetYcm()는 cm 단위입니다.
              * 프로토콜 범위에 맞게 제한합니다.
              */
             x = CAN_ClampU16(Position_GetXcm(), 0U, 1023U);
             y = CAN_ClampU16(Position_GetYcm(), 0U, 2047U);

             heading = CAN_HeadingX100(used_heading_x100);

             if ((state == WAIT_STEER_RIGHT) ||
                 (state == WAIT_RETURN_RIGHT))
             {
                 turn_signal = CAN_TURN_RIGHT;
             }
             else if ((state == WAIT_STEER_LEFT) ||
                      (state == WAIT_RETURN_LEFT))
             {
                 turn_signal = CAN_TURN_LEFT;
             }
             else
             {
                 turn_signal = CAN_TURN_OFF;
             }

             /*
              * 최초 송신은 Linux에 기준 상태가 없으므로 전체 갱신합니다.
              */
             if (can_first_tx)
             {
                 update_mask = CAN_UPDATE_ALL;
             }
             else
             {
                 update_mask = 0U;

                 if (speed_mpm != previous_speed_mpm)
                 {
                     update_mask |= CAN_UPDATE_SPEED;
                 }

                 if (x != previous_x)
                 {
                     update_mask |= CAN_UPDATE_X;
                 }

                 if (y != previous_y)
                 {
                     update_mask |= CAN_UPDATE_Y;
                 }

                 if (heading != previous_heading)
                 {
                     update_mask |= CAN_UPDATE_HEADING;
                 }

                 if (turn_signal != previous_turn_signal)
                 {
                     update_mask |= CAN_UPDATE_TURN_SIGNAL;
                 }
             }

             timestamp = CAN_GetTimestamp12();

             /*
              * update_mask가 0이어도 송신합니다.
              * Linux는 상태를 덮어쓰지 않지만 프레임 수신 시각을 통해
              * 통신 freshness를 유지할 수 있습니다.
              */
             tx_success = CAN_SendEgoStatus0000(timestamp,
                                               update_mask,
                                               speed_mpm,
                                               x,
                                               y,
                                               heading,
                                               turn_signal);

             /*
              * 실제 CAN 메일박스 등록에 성공했을 때만 이전 값을 갱신합니다.
              * 송신 실패 시 다음 주기에 동일 변경사항을 다시 전송합니다.
              */
             if (tx_success)
             {
                 previous_speed_mpm = speed_mpm;
                 previous_x = x;
                 previous_y = y;
                 previous_heading = heading;
                 previous_turn_signal = turn_signal;
                 can_first_tx = 0U;
             }
         }

      /*
       * UART 출력은 느리므로 200ms마다만 출력합니다.
       */
      if ((uint32_t)(now - last_print_time) >= DEBUG_PRINT_PERIOD_MS)
      {
          last_print_time = now;

          int32_t heading_x100 = IMU_GetHeadingX100();
          float current_speed_mps = Speed_GetMps();

          _uart_printf("raw:%ld deg used:%ld deg speed:%ld m/min x:%ld cm y:%ld cm total:%ld cm\n",
                       (long)(heading_x100 / 100),
                       (long)(used_heading_x100 / 100),
                       (long)(current_speed_mps * 60.0f),
                       (long)Position_GetXcm(),
                       (long)Position_GetYcm(),
					   (long)Position_GetTotalDistanceCm());
      }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
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
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


//void EXTI2_IRQHandler(void)
//{
//    if (EXTI->PR & (1U << 2))
//    {
//        EXTI->PR |= (1U << 2);
//
//        if ((uint32_t)(msTick - last_edge_tick) >= MARK_LOCKOUT_MS)
//        {
//            pulse_count++;
//            last_edge_tick = msTick;
//
//            if (GPIOE->IDR & (1U << 2))
//                GPIOB->ODR |= (1U << 7);
//            else
//                GPIOB->ODR &= ~(1U << 7);
//        }
//        else
//        {
//            rejected_cnt++;
//        }
//    }
//}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
