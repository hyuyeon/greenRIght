#include "main.h"
#include "string.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

UART_HandleTypeDef huart3;

// ========================================================
// 전역 변수 정의
// ========================================================
volatile uint8_t canRxFlag = 0;
volatile uint16_t rx_id;
CAN_Header_t rx_header;

// 어플리케이션 계층 전역 상태 변수 (CAN_Rx/Tx에서 다이렉트로 접근)
EgoVehicle ego;
CandidateVehicle candidateVehicle;
TrafficLight tl;

osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
void UART3_Send_Byte(char ch);
void UART3_Send_String(char* p);
void Uart3_Printf(char *fmt,...);

void SystemClock_Config(void);
static void MX_USART3_UART_Init(void);
void CAN_Config(void);

// payload 매개변수가 제거된 송수신 함수
void CAN_Tx(uint16_t can_id, CAN_Header_t *header);
uint8_t CAN_Rx(uint16_t *can_id, CAN_Header_t *header);

void StartDefaultTask(void const * argument);
void vTask_CAN_Tx(void *argument);
void vTask_CAN_Rx(void *argument);
/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

int main(void)
{
  // 시스템 클럭 설정 및 RTOS 설정
  HAL_Init();
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  // UART 초기화
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
  // CAN configuration
  CAN_Config();

//  BaseType_t ret;
//  ret = xTaskCreate(
//      vTask_CAN_Tx,
//      "CAN_TX",
//      256*4,
//      NULL,
//      3,
//      NULL);
//  Uart3_Printf("ret=%d\r\n", ret);

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
    CAN1->MCR = CAN_MCR_ABOM; // Automatic Bus-Off Recovery
    CAN1->MCR |= CAN_MCR_INRQ;
    while(!(CAN1->MSR & CAN_MSR_INAK)); // wait until ACK comes

    // Bit Timing
    CAN1->BTR =
    		((1-1) << 24) |   // SJW = 1
            ((2-1) << 20) |   // TS2 = 2
            ((11-1) << 16) |  // TS1 = 11
            ((6-1) << 0);     // BRP = 6

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

    // RX 인터럽트 활성화
    CAN1->IER |= CAN_IER_FMPIE0;

    // NVIC 설정
    NVIC_SetPriority(CAN1_RX0_IRQn, 5);
    NVIC_EnableIRQ(CAN1_RX0_IRQn);
    Uart3_Printf("Normalmode ON\r\n");
}

void CAN_Tx(uint16_t can_id, CAN_Header_t *header)
{
    uint64_t frame = 0;

    //================ Header 패킹 (24bit) ================//
    frame |= ((uint64_t)(header->msg_id     & 0x0F)) << 60; // 4bit
    frame |= ((uint64_t)(header->timestamp  & 0xFFF)) << 48; // 12bit
    frame |= ((uint64_t)(header->updateMask & 0xFF)) << 40; // 8bit

    //================ Payload 패킹 (40bit) ================//
    // msg_id 기반 분기하여 전역 상태 변수에서 다이렉트로 값을 읽어옴
    switch(header->msg_id)
    {
    case 0x0:     // Vehicle Status (Ego)
        frame |= ((uint64_t)(ego.speed      & 0xFF))  << 32;
        frame |= ((uint64_t)(ego.x          & 0x3FF)) << 22;
        frame |= ((uint64_t)(ego.y          & 0x7FF)) << 11;
        frame |= ((uint64_t)(ego.heading    & 0x1FF)) << 2;
        break;

    case 0x4:     // Crosswalk Zone (candidateVehicle)
        frame |= ((uint64_t)(candidateVehicle.type & 0xFF))  << 32;
        frame |= ((uint64_t)(candidateVehicle.cz_x & 0x3FF)) << 22;
        frame |= ((uint64_t)(candidateVehicle.cz_y & 0x7FF)) << 11;
        break;

    case 0x5:     // Opposite Vehicle (candidateVehicle)
        frame |= ((uint64_t)(candidateVehicle.type  & 0xFF))  << 32;
        frame |= ((uint64_t)(candidateVehicle.speed & 0xFF))  << 24;
        frame |= ((uint64_t)(candidateVehicle.x     & 0x3FF)) << 14;
        frame |= ((uint64_t)(candidateVehicle.y     & 0x7FF)) << 3;
        break;

    case 0x6:     // Traffic Light
        frame |= ((uint64_t)(tl.color     & 0x03)) << 30;
        frame |= ((uint64_t)(tl.time_left & 0x0F)) << 26;
        frame |= ((uint64_t)(tl.cz_x      & 0x3FF)) << 16;
        frame |= ((uint64_t)(tl.cz_y      & 0x7FF)) << 5;
        break;
    }

    Uart3_Printf("TX FRAME=%08lX %08lX\r\n", (uint32_t)(frame >> 32), (uint32_t)frame);

    // 하드웨어 레지스터 제어
    CAN1->sTxMailBox[0].TDTR = 8; // DLC 8바이트
    CAN1->sTxMailBox[0].TDLR = (uint32_t)frame;
    CAN1->sTxMailBox[0].TDHR = (uint32_t)(frame >> 32);
    CAN1->sTxMailBox[0].TIR  = (can_id << 21) | CAN_TI0R_TXRQ;

    while(!(CAN1->TSR & CAN_TSR_RQCP0));
    CAN1->TSR = CAN_TSR_RQCP0;
}

uint8_t CAN_Rx(uint16_t *can_id, CAN_Header_t *header)
{
    if((CAN1->RF0R & CAN_RF0R_FMP0) == 0)
    {
        return 0;
    }

    *can_id = (CAN1->sFIFOMailBox[0].RIR >> 21) & 0x7FF;
    //뉴 방식
    uint32_t rdlr = CAN1->sFIFOMailBox[0].RDLR;
    uint32_t rdhr = CAN1->sFIFOMailBox[0].RDHR;

    uint8_t b0 = (uint8_t)((rdlr >> 0)  & 0xFF);
    uint8_t b1 = (uint8_t)((rdlr >> 8)  & 0xFF);
    uint8_t b2 = (uint8_t)((rdlr >> 16) & 0xFF);
    uint8_t b3 = (uint8_t)((rdlr >> 24) & 0xFF);

    uint8_t b4 = (uint8_t)((rdhr >> 0)  & 0xFF);
    uint8_t b5 = (uint8_t)((rdhr >> 8)  & 0xFF);
    uint8_t b6 = (uint8_t)((rdhr >> 16) & 0xFF);
    uint8_t b7 = (uint8_t)((rdhr >> 24) & 0xFF);

    CAN1->RF0R |= CAN_RF0R_RFOM0;

    uint64_t frame = 0;

    frame |= ((uint64_t)b0 << 56);
    frame |= ((uint64_t)b1 << 48);
    frame |= ((uint64_t)b2 << 40);
    frame |= ((uint64_t)b3 << 32);
    frame |= ((uint64_t)b4 << 24);
    frame |= ((uint64_t)b5 << 16);
    frame |= ((uint64_t)b6 << 8);
    frame |= ((uint64_t)b7 << 0);

// 기존 방식
//    uint32_t low  = CAN1->sFIFOMailBox[0].RDLR;
//    uint32_t high = CAN1->sFIFOMailBox[0].RDHR;
//
//    CAN1->RF0R |= CAN_RF0R_RFOM0;
//
//    uint64_t frame = ((uint64_t)high << 32) | low;

    //================ Header 복원 =================//
    header->msg_id     = (frame >> 60) & 0x0F;
    header->timestamp  = (frame >> 48) & 0x0FFF;
    header->updateMask = (frame >> 40) & 0xFF;

    //================ Payload 복원 및 전역 상태 갱신 =================//
    switch(header->msg_id)
    {
    case 0x0:     // Vehicle Status (Ego)
        ego.speed     = (frame >> 32) & 0xFF;
        ego.x         = (frame >> 22) & 0x3FF;
        ego.y         = (frame >> 11) & 0x7FF;
        ego.heading   = (frame >> 2)  & 0x1FF;
        ego.timestamp = header->timestamp;
        break;

    case 0x4:     // Crosswalk Zone (candidateVehicle)
        candidateVehicle.type = (frame >> 32) & 0xFF;
        candidateVehicle.cz_x = (frame >> 22) & 0x3FF;
        candidateVehicle.cz_y = (frame >> 11) & 0x7FF;
        candidateVehicle.timestamp_ms = header->timestamp;
        break;

    case 0x5:     // Opposite Vehicle (candidateVehicle)
        candidateVehicle.type  = (frame >> 32) & 0xFF;
        candidateVehicle.speed = (frame >> 24) & 0xFF;
        candidateVehicle.x     = (frame >> 14) & 0x3FF;
        candidateVehicle.y     = (frame >> 3)  & 0x7FF;
        candidateVehicle.timestamp_ms = header->timestamp;
        break;

    case 0x6:     // Traffic Light
        {
            // 1. 수신된 프레임에서 새로운 값 추출
            uint8_t new_color     = (frame >> 30) & 0x03;
            uint8_t new_time_left = (frame >> 26) & 0x0F;

            // 2. 기존 값과 비교하여 변경 플래그 만들깅
            uint8_t is_changed = 0;
            if ((tl.color != new_color) || (tl.time_left != new_time_left))
            {
                is_changed = 1;
            }

            // 3. 전역 상태 갱신
            tl.color     = new_color;
            tl.time_left = new_time_left;
            tl.cz_x      = (frame >> 16) & 0x3FF;
            tl.cz_y      = (frame >> 5)  & 0x7FF;

            // 4. 값이 변경되었다면 세마포어 전달
            if (is_changed)
            {
            	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            	xSemaphoreGiveFromISR(tlDisplaySem, &xHigherPriorityTaskWoken);
            	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
            break;
        }

    default:
        return 0;
    }
    return 1;
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators */
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

  /** Initializes the CPU, AHB and APB buses clocks */
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

void StartDefaultTask(void const * argument)
{
  for(;;)
  {
    osDelay(1);
  }
}

void vTask_CAN_Tx(void *argument)
{
    // 테스트용 헤더 설정 (Opposite Vehicle)
    CAN_Header_t header =
    {
        .msg_id = 0x5,
        .timestamp = 0,
        .updateMask = 0
    };

    // 송신할 데이터를 전역 변수에 다이렉트로 할당
    candidateVehicle.type = 0x01;
    candidateVehicle.speed = 60;
    candidateVehicle.x = 120;
    candidateVehicle.y = 350;

    while(1)
    {
        // 전역 변수에서 데이터를 읽어오므로 payload 포인터 생략
        CAN_Tx(0x200, &header);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void vTask_CAN_Rx(void *argument)
{
    while(1){ulTaskNotifyTake(pdTRUE, portMAX_DELAY);}
}

void UART3_Send_Byte(char ch){
  if (ch=='\n'){
      USART3->DR = 0x0d;
      while (((USART3->SR >> 7)&0x1)==0);
  }
  USART3->DR = ch;
  while (((USART3->SR >> 7)&0x1)==0);
}

void UART3_Send_String(char* p){
    while (*p){
        UART3_Send_Byte(*p++);
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
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
