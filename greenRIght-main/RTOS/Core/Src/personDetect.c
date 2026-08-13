/*
 * ped_task_isr.c  (PedTaskISR)
 *
 * UART4 RX interrupt receives the AI camera(YOLO) pedestrian flag.
 *
 * ?占쎌젣: ?占쎌쫰踰좊━?占쎌씠??"?占쏀깭媛 諛뷂옙??占쎈쭔" 1諛붿씠?占쏙옙? 蹂대궦??(edge-triggered).
 *
 * pedFlag 占?
 *   0 (00) = 蹂댄뻾???占쎌쓬
 *   1 (01) = 蹂댄뻾???占쎌쓬
 *   2 (10) = 移대찓???占쎌떇 ?占쎈윭
 *
 * ?占쎌옉:
 *   1. ISR?占쎌꽌 ?占쎌떊 諛붿씠?占쎈줈 ?占쎌뿭 pedFlag 媛깆떊
 *   2. ?占쎌뿭 maneuver媛 ?占쏀쉶?占쎌씠占?turnJudgeSem??give?占쎌꽌 TurnJudgeTask占?源⑨옙?
 *   3. 占???maneuver占??占쎈Т寃껊룄 ????(?占쎈룄???占쎌옉)
 *
 * AI camera UART: UART4 = PC10(TX)/PC11(RX), AF8
 * USART3 PD8/PD9 remains available for ST-LINK VCP debug logs.
 */

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "personDetect.h"

/* ===================== ?占쎌젙占?===================== */
#define UART4_BAUDRATE      115200U
#define APB1_CLK_HZ         42000000U   /* SystemClock_Config 湲곤옙? ?占쎌륫: HSE8MHz,PLLM4,PLLN168,PLLP2,APB1DIV4 -> 42MHz */
#define IRQ_PRIORITY_UART4  6U

#define PED_DEBUG   1   /* ?占쎌텧 ??0?占쎈줈 諛붽씀占?LED/移댁슫???占쎈쾭占?肄붾뱶媛 ?占쎌㎏占?鍮좎쭚 */

#if PED_DEBUG
volatile uint32_t uartIrqCount         = 0U;
volatile uint32_t uartRxByteCount      = 0U;
volatile uint32_t uartValidByteCount   = 0U;
volatile uint8_t  uartLastRawByte      = 0U;
volatile uint8_t  uartLastDecodedFlag  = 0U;
volatile uint32_t uartOverrunCount     = 0U;
volatile uint32_t uartFramingCount     = 0U;
volatile uint32_t uartNoiseCount       = 0U;
volatile uint32_t uartInvalidByteCount = 0U;
volatile uint32_t semGiveCount         = 0U;

#define LED_GPIO_PORT   GPIOB
#define LED_PIN         0U   /* Nucleo-F429ZI 洹몃┛ LED(LD1) = PB0 */

static void LED_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    LED_GPIO_PORT->MODER &= ~(0x3U << (LED_PIN * 2U));
    LED_GPIO_PORT->MODER |=  (0x1U << (LED_PIN * 2U));
}

static inline void LED_Set(uint8_t on)
{
    if (on) LED_GPIO_PORT->BSRR = (1U << LED_PIN);
    else    LED_GPIO_PORT->BSRR = (1U << (LED_PIN + 16U));
}
#endif /* PED_DEBUG */

/* ===================== UART4 init (CMSIS register access) ===================== */
void UART4_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_UART4EN;

    GPIOC->MODER &= ~((0x3U << (10U * 2U)) | (0x3U << (11U * 2U)));
    GPIOC->MODER |=  ((0x2U << (10U * 2U)) | (0x2U << (11U * 2U)));

    /* PC10,PC11?占??占?8~15 援ш컙 -> AFR[1] ?占쎌슜 */
    GPIOC->AFR[1] &= ~((0xFU << ((10U - 8U) * 4U)) | (0xFU << ((11U - 8U) * 4U)));
    GPIOC->AFR[1] |=  ((0x8U << ((10U - 8U) * 4U)) | (0x8U << ((11U - 8U) * 4U)));

    GPIOC->OSPEEDR |= (0x2U << (10U * 2U)) | (0x2U << (11U * 2U));
    GPIOC->PUPDR &= ~((0x3U << (10U * 2U)) | (0x3U << (11U * 2U)));
    GPIOC->PUPDR |=  (0x1U << (11U * 2U));   /* RX(PC11) ?占??*/

    UART4->CR1 = 0;
    uint32_t usartdiv = (APB1_CLK_HZ + (UART4_BAUDRATE / 2U)) / UART4_BAUDRATE;
    UART4->BRR = usartdiv;

    UART4->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_UE;

    NVIC_SetPriority(UART4_IRQn, IRQ_PRIORITY_UART4);
    NVIC_EnableIRQ(UART4_IRQn);
}

/* ===================== PedTaskISR (UART4 RX interrupt) ===================== */
void UART4_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t sr = UART4->SR;

#if PED_DEBUG
    uartIrqCount++;
    if (sr & USART_SR_ORE) uartOverrunCount++;
    if (sr & USART_SR_FE)  uartFramingCount++;
    if (sr & USART_SR_NE)  uartNoiseCount++;
#endif

    if (sr & USART_SR_RXNE)
    {
        uint8_t received = (uint8_t)(UART4->DR & 0xFFU);
        uint8_t decoded;
        uint8_t isValid = 0U;

#if PED_DEBUG
        uartRxByteCount++;
        uartLastRawByte = received;
#endif

        if (received <= 2U) {
            decoded = received;
            isValid = 1U;
        } else if ((received >= (uint8_t)'0') && (received <= (uint8_t)'2')) {
            decoded = (uint8_t)(received - (uint8_t)'0');
            isValid = 1U;
        } else if (received == 0xFFU) {
            decoded = 2U;
            isValid = 1U;
        } else {
            decoded = 0U;
        }

        if (isValid != 0U)
        {
            static uint8_t hasPedSample = 0U;
            uint8_t prevPedFlag = pedFlag;
            uint8_t shouldJudge = (hasPedSample == 0U) || (decoded != prevPedFlag);

            pedFlag = decoded;
            hasPedSample = 1U;

#if PED_DEBUG
            uartValidByteCount++;
            uartLastDecodedFlag = decoded;
#endif

            if ((shouldJudge != 0U) && (maneuver == MANEUVER_RIGHT_TURN))
            {
                BaseType_t ok = xSemaphoreGiveFromISR(turnJudgeSem, &xHigherPriorityTaskWoken);
#if PED_DEBUG
                if (ok == pdTRUE) semGiveCount++;
#else
                (void)ok;
#endif
            }
            /* Not a right-turn situation: update pedFlag only. */
        }
#if PED_DEBUG
        else
        {
            uartInvalidByteCount++;
        }
#endif
    }
#if PED_DEBUG
    else if (sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE))
    {
        (void)UART4->DR;
    }
#endif

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


/* ===================== 珥덇린??吏꾩엯??===================== */
void PersonFlag_Init(void)
{
    UART4_Init();
    LED_Init();
}
void PersonFlag_DebugPrint(const char *msg)
{
    if (msg == 0) {
        return;
    }

    while (*msg != '\0') {
        if (*msg == '\n') {
            while ((USART3->SR & USART_SR_TXE) == 0U) { }
            USART3->DR = (uint16_t)'\r';
        }

        while ((USART3->SR & USART_SR_TXE) == 0U) { }
        USART3->DR = (uint16_t)(uint8_t)(*msg);
        msg++;
    }

    while ((USART3->SR & USART_SR_TC) == 0U) { }
}

void PersonFlag_GetDebugSnapshot(PersonFlagDebugSnapshot *snapshot)
{
    if (snapshot == 0) {
        return;
    }

#if PED_DEBUG
    snapshot->irqCount = uartIrqCount;
    snapshot->rxByteCount = uartRxByteCount;
    snapshot->validByteCount = uartValidByteCount;
    snapshot->invalidByteCount = uartInvalidByteCount;
    snapshot->semGiveCount = semGiveCount;
    snapshot->overrunCount = uartOverrunCount;
    snapshot->framingCount = uartFramingCount;
    snapshot->noiseCount = uartNoiseCount;
    snapshot->lastRawByte = uartLastRawByte;
    snapshot->lastDecodedFlag = uartLastDecodedFlag;
#else
    snapshot->irqCount = 0U;
    snapshot->rxByteCount = 0U;
    snapshot->validByteCount = 0U;
    snapshot->invalidByteCount = 0U;
    snapshot->semGiveCount = 0U;
    snapshot->overrunCount = 0U;
    snapshot->framingCount = 0U;
    snapshot->noiseCount = 0U;
    snapshot->lastRawByte = 0U;
    snapshot->lastDecodedFlag = 0U;
#endif
}