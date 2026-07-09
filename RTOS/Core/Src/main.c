#include "main.h"
#include "string.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

UART_HandleTypeDef huart3;
//for RX task to interrupt
volatile uint8_t canRxFlag = 0;
volatile uint16_t rx_id;
CAN_Header_t rx_header;
CAN_Payload_t rx_payload;

osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
/* USER CODE BEGIN PV */

void UART3_Send_Byte(char ch);
void UART3_Send_String(char* p);
void Uart3_Printf(char *fmt,...);

void SystemClock_Config(void);
static void MX_USART3_UART_Init(void);
void CAN_Config(void);
void CAN_Tx(uint16_t can_id,CAN_Header_t *header,CAN_Payload_t *payload);
uint8_t CAN_Rx(uint16_t *can_id, CAN_Header_t *header, CAN_Payload_t *payload);
void StartDefaultTask(void const * argument);
void vTask_CAN_Tx(void *argument);
void vTask_CAN_Rx(void *argument);

/* USER CODE BEGIN PFP */

int main(void)
{
  //시스템 클럭 설정 및 RTOS 설정
  HAL_Init();
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  //UART 초기화
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  //CAN configuration
  CAN_Config();
  BaseType_t ret;
  ret = xTaskCreate(
      vTask_CAN_Tx,
      "CAN_TX",
      256*4,
      NULL,
      3,
      NULL);
  Uart3_Printf("ret=%d\r\n", ret);

  xTaskCreate(
      vTask_CAN_Rx,
      "CAN_RX",
      256*4,
      NULL,
      3,
      NULL);

  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  Uart3_Printf("Before Scheduler\r\n");
  osKernelStart();


  Uart3_Printf("After Scheduler\r\n");
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void CAN_Config(void)
{
    // 1. CAN1 Clock
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;

    // 2. GPIOD Clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

    // 2-1. PD0(RX), PD1(TX) Alternate Function
    GPIOD->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1);
    GPIOD->MODER |= GPIO_MODER_MODE0_1 | GPIO_MODER_MODE1_1;

    // 2-2. set AFmode AF9(CAN)
    GPIOD->AFR[0] &= ~((0xF<<0) | (0xF<<4));
    GPIOD->AFR[0] |= (9<<0) | (9<<4);

    // Init mode
    CAN1->MCR = CAN_MCR_ABOM; // Automatic Bus-Off Recovery <- 씹새
    CAN1->MCR |= CAN_MCR_INRQ;
    while(!(CAN1->MSR & CAN_MSR_INAK)); //wait until ACK comes

    // Bit Timing
    CAN1->BTR |= (1 << 30);//loopback
//    CAN1->BTR =
//        ((1-1) << 24) |   // SJW = 1
//        ((2-1) << 20) |   // TS2 = 2
//        ((11-1) << 16) |  // TS1 = 11
//        ((6-1) << 0);     // BRP = 6

    // Filter Init
    CAN1->FMR |= CAN_FMR_FINIT;

    // Filter 0 : 모든 메시지 허용
    CAN1->FA1R &= ~1;
    CAN1->FM1R &= ~1;
    CAN1->FS1R |= 1;

    CAN1->sFilterRegister[0].FR1 = 0;
    CAN1->sFilterRegister[0].FR2 = 0;

    CAN1->FFA1R &= ~1;
    CAN1->FA1R |= 1;

    CAN1->FMR &= ~CAN_FMR_FINIT;

    // Normal mode
    CAN1->MCR &= ~CAN_MCR_INRQ;
    while(CAN1->MSR & CAN_MSR_INAK);
    CAN1->IER |= CAN_IER_FMPIE0;

    // NVIC 설정
    NVIC_SetPriority(CAN1_RX0_IRQn, 5);
    NVIC_EnableIRQ(CAN1_RX0_IRQn);
    Uart3_Printf("Normalmode ON\r\n");
}

void CAN_Tx(uint16_t can_id,
            CAN_Header_t *header,
            CAN_Payload_t *payload)
{
    uint64_t frame = 0;

    //================ Header (24bit) ================//
    frame |= ((uint64_t)(header->msg_id     & 0x0F)) << 60;//4
    frame |= ((uint64_t)(header->timestamp	& 0xFF)) << 48;//8
    frame |= ((uint64_t)(header->updateMask	& 0xFF)) << 40;//8

    //================ Payload (40bit) ================//
    frame |= ((uint64_t)(payload->speed      & 0xFF))  << 32;//8
    frame |= ((uint64_t)(payload->x          & 0x3FF)) << 22;//10
    frame |= ((uint64_t)(payload->y          & 0x7FF)) << 11;//11
    frame |= ((uint64_t)(payload->heading    & 0x1FF)) << 2;//9
    frame |= ((uint64_t)(payload->turnSignal & 0x03));//2

    uint8_t data[8];

    for(int i=0;i<8;i++)
    {
        data[7-i] = frame & 0xFF;
        frame >>= 8;
    }

    while(!(CAN1->TSR & CAN_TSR_TME0));

    CAN1->sTxMailBox[0].TIR = (can_id << 21);
    CAN1->sTxMailBox[0].TDTR = 8;

    CAN1->sTxMailBox[0].TDLR =
          data[0]
        | (data[1] << 8)
        | (data[2] << 16)
        | (data[3] << 24);

    CAN1->sTxMailBox[0].TDHR =
          data[4]
        | (data[5] << 8)
        | (data[6] << 16)
        | (data[7] << 24);

    CAN1->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;

    while(!(CAN1->TSR & CAN_TSR_RQCP0));

    CAN1->TSR = CAN_TSR_RQCP0;
}

uint8_t CAN_Rx(uint16_t *can_id,
               CAN_Header_t *header,
               CAN_Payload_t *payload)
{
    if((CAN1->RF0R & CAN_RF0R_FMP0) == 0)
        return 0;

    *can_id = (CAN1->sFIFOMailBox[0].RIR >> 21) & 0x7FF;

    uint32_t low  = CAN1->sFIFOMailBox[0].RDLR;
    uint32_t high = CAN1->sFIFOMailBox[0].RDHR;
    CAN1->RF0R |= CAN_RF0R_RFOM0;

    uint64_t frame =
        ((uint64_t)high << 32) | low;

    //================ Header =================//
    header->msg_id     = (frame >> 60) & 0x0F;
    header->timestamp  = (frame >> 48) & 0xFFF;
    header->updateMask = (frame >> 40) & 0xFF;

    //================ Payload ================//
    payload->speed      = (frame >> 32) & 0xFF;
    payload->x          = (frame >> 22) & 0x3FF;
    payload->y          = (frame >> 11) & 0x7FF;
    payload->heading    = (frame >> 2)  & 0x1FF;
    payload->turnSignal = frame & 0x03;

    return 1;
}

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

/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

void vTask_CAN_Tx(void *argument)
{
	CAN_Header_t header =
	{
	    .msg_id = rand()%0xF,
	    .timestamp = rand()%0xFFF,
	    .updateMask = rand()%0x1F
	};

	CAN_Payload_t payload =
	{
	    .speed = rand()%0xFF,
	    .x = rand()%0x3FF,
	    .y = rand()%0x7FF,
	    .heading = rand()%0x1FF,
	    .turnSignal = 1
	};
    while(1)
    {
        CAN_Tx(0x200,&header,&payload);

        Uart3_Printf("CAN TX\r\n");

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void vTask_CAN_Rx(void *argument)
{
    while(1){ulTaskNotifyTake(pdTRUE, portMAX_DELAY);}
}
void UART3_Send_Byte(char ch){
  if (ch=='\n'){                        // new line인 경우  '\n'을 추가전송
      USART3->DR = 0x0d;
      while (((USART3->SR >> 7)&0x1)==0);
  }
  USART3->DR = ch;
  while (((USART3->SR >> 7)&0x1)==0);    // wait TXE
}

void UART3_Send_String(char* p){
    while (*p){
        UART3_Send_Byte(*p++);            // 널문자 전까지 출력
    }
}

void Uart3_Printf(char *fmt,...)
{
  va_list ap;
  char string[256];

  va_start(ap,fmt);
  vsprintf(string,fmt,ap);
  va_end(ap);
  UART3_Send_String(string);
}

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
