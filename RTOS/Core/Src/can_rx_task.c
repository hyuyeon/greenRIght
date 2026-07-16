#include "can_rx_task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "debug_uart.h"
#include <string.h>
#include <stdio.h>

static const osThreadAttr_t CanRxTask_attributes = {
    .name = "CAN_RX",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

volatile uint8_t canRxFlag = 0U;
static volatile uint16_t rx_id = 0U;
static CAN_Header_t rx_header;

extern SemaphoreHandle_t turnJudgeSem;
extern SemaphoreHandle_t tlDisplaySem;

/*
 * updateMask is interpreted per message payload. A cleared bit means that
 * the receiver must retain the previously accepted value for that field.
 */
#define EGO_UPD_SPEED          (1U << 0)
#define EGO_UPD_X              (1U << 1)
#define EGO_UPD_Y              (1U << 2)
#define EGO_UPD_HEADING        (1U << 3)

#define CAND_INTRO_UPD_TYPE    (1U << 0)
#define CAND_INTRO_UPD_CZ_X    (1U << 1)
#define CAND_INTRO_UPD_CZ_Y    (1U << 2)

#define CAND_STATUS_UPD_TYPE   (1U << 0)
#define CAND_STATUS_UPD_SPEED  (1U << 1)
#define CAND_STATUS_UPD_X      (1U << 2)
#define CAND_STATUS_UPD_Y      (1U << 3)

#define TL_UPD_TYPE            (1U << 0)
#define TL_UPD_COLOR           (1U << 1)
#define TL_UPD_TIME_LEFT       (1U << 2)
#define TL_UPD_CZ_X            (1U << 3)
#define TL_UPD_CZ_Y            (1U << 4)
#define TL_UPD_MANEUVER        (1U << 5)

static uint8_t CAN_Rx(uint16_t *can_id, CAN_Header_t *header, uint64_t *frame_out);
static void CanRxTask(void *argument);
static void CanRx_HandleFrame(uint64_t frame, const CAN_Header_t *header);

void CAN_Config(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

    GPIOD->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1);
    GPIOD->MODER |= GPIO_MODER_MODE0_1 | GPIO_MODER_MODE1_1;

    GPIOD->AFR[0] &= ~((0xFU << 0) | (0xFU << 4));
    GPIOD->AFR[0] |= (9U << 0) | (9U << 4);

    CAN1->MCR = CAN_MCR_ABOM;
    CAN1->MCR |= CAN_MCR_INRQ;
    while ((CAN1->MSR & CAN_MSR_INAK) == 0U) { }

    CAN1->BTR =
        ((1U - 1U) << 24) |
        ((2U - 1U) << 20) |
        ((11U - 1U) << 16) |
        ((6U - 1U) << 0);

    CAN1->FMR |= CAN_FMR_FINIT;

    CAN1->FA1R &= ~1U;
    CAN1->FM1R &= ~1U;
    CAN1->FS1R |= 1U;

    CAN1->sFilterRegister[0].FR1 = 0U;
    CAN1->sFilterRegister[0].FR2 = 0U;

    CAN1->FFA1R &= ~1U;
    CAN1->FA1R |= 1U;

    CAN1->FMR &= ~CAN_FMR_FINIT;

    CAN1->MCR &= ~CAN_MCR_INRQ;
    while ((CAN1->MSR & CAN_MSR_INAK) != 0U) { }

    NVIC_SetPriority(CAN1_RX0_IRQn, 5U);
    NVIC_EnableIRQ(CAN1_RX0_IRQn);

    CAN1->IER |= CAN_IER_FMPIE0;
}

void CAN_Tx(uint16_t can_id, CAN_Header_t *header)
{
    uint64_t frame = 0U;

    frame |= ((uint64_t)(header->msg_id & 0x0FU)) << 60;
    frame |= ((uint64_t)(header->timestamp & 0x0FFFU)) << 48;
    frame |= ((uint64_t)(header->updateMask & 0xFFU)) << 40;

    switch (header->msg_id)
    {
    case 0x0:
        frame |= ((uint64_t)(ego.speed & 0xFFU)) << 32;
        frame |= ((uint64_t)(ego.x & 0x03FFU)) << 22;
        frame |= ((uint64_t)(ego.y & 0x07FFU)) << 11;
        frame |= ((uint64_t)(ego.heading & 0x01FFU)) << 2;
        break;

    case 0x4:
        frame |= ((uint64_t)(candidateVehicle.type & 0xFFU)) << 32;
        frame |= ((uint64_t)(candidateVehicle.cz_x & 0x03FFU)) << 22;
        frame |= ((uint64_t)(candidateVehicle.cz_y & 0x07FFU)) << 11;
        break;

    case 0x5:
        frame |= ((uint64_t)(candidateVehicle.type & 0xFFU)) << 32;
        frame |= ((uint64_t)(candidateVehicle.speed & 0xFFU)) << 24;
        frame |= ((uint64_t)(candidateVehicle.x & 0x03FFU)) << 14;
        frame |= ((uint64_t)(candidateVehicle.y & 0x07FFU)) << 3;
        break;

    case 0x6:
        frame |= ((uint64_t)(tl.color & 0x03U)) << 30;
        frame |= ((uint64_t)(tl.time_left & 0x0FU)) << 26;
        frame |= ((uint64_t)(tl.cz_x & 0x03FFU)) << 16;
        frame |= ((uint64_t)(tl.cz_y & 0x07FFU)) << 5;
        frame |= ((uint64_t)(maneuver & 0x03U)) << 3;
        break;

    default:
        break;
    }

    CAN1->sTxMailBox[0].TDTR = 8U;
    CAN1->sTxMailBox[0].TDLR = (uint32_t)frame;
    CAN1->sTxMailBox[0].TDHR = (uint32_t)(frame >> 32);
    CAN1->sTxMailBox[0].TIR = ((uint32_t)can_id << 21) | CAN_TI0R_TXRQ;

    while ((CAN1->TSR & CAN_TSR_RQCP0) == 0U) { }
    CAN1->TSR = CAN_TSR_RQCP0;
}

static uint8_t CAN_Rx(uint16_t *can_id, CAN_Header_t *header, uint64_t *frame_out)
{
    if ((CAN1->RF0R & CAN_RF0R_FMP0) == 0U)
    {
        return 0U;
    }

    *can_id = (uint16_t)((CAN1->sFIFOMailBox[0].RIR >> 21) & 0x07FFU);

    uint32_t rdlr = CAN1->sFIFOMailBox[0].RDLR;
    uint32_t rdhr = CAN1->sFIFOMailBox[0].RDHR;

    uint8_t b0 = (uint8_t)((rdlr >> 0) & 0xFFU);
    uint8_t b1 = (uint8_t)((rdlr >> 8) & 0xFFU);
    uint8_t b2 = (uint8_t)((rdlr >> 16) & 0xFFU);
    uint8_t b3 = (uint8_t)((rdlr >> 24) & 0xFFU);
    uint8_t b4 = (uint8_t)((rdhr >> 0) & 0xFFU);
    uint8_t b5 = (uint8_t)((rdhr >> 8) & 0xFFU);
    uint8_t b6 = (uint8_t)((rdhr >> 16) & 0xFFU);
    uint8_t b7 = (uint8_t)((rdhr >> 24) & 0xFFU);

    CAN1->RF0R |= CAN_RF0R_RFOM0;

    uint64_t frame = 0U;
    frame |= ((uint64_t)b0 << 56);
    frame |= ((uint64_t)b1 << 48);
    frame |= ((uint64_t)b2 << 40);
    frame |= ((uint64_t)b3 << 32);
    frame |= ((uint64_t)b4 << 24);
    frame |= ((uint64_t)b5 << 16);
    frame |= ((uint64_t)b6 << 8);
    frame |= ((uint64_t)b7 << 0);

    header->msg_id = (uint8_t)((frame >> 60) & 0x0FU);
    header->timestamp = (uint16_t)((frame >> 48) & 0x0FFFU);
    header->updateMask = (uint8_t)((frame >> 40) & 0xFFU);
    *frame_out = frame;

    return 1U;
}

static void CanRx_DebugPrintMsg4(uint16_t can_id, uint64_t frame, const CAN_Header_t *header, const CandidateVehicle *candSnap)
{
    char line[192];

    (void)snprintf(line, sizeof(line),
        "[CAN4] can=0x%03x ts=%u mask=0x%02x raw=%08lx%08lx type=0x%02x cz=(%u,%u)\n",
        (unsigned)can_id,
        (unsigned)header->timestamp,
        (unsigned)header->updateMask,
        (unsigned long)(uint32_t)(frame >> 32),
        (unsigned long)(uint32_t)frame,
        (unsigned)candSnap->type,
        (unsigned)candSnap->cz_x,
        (unsigned)candSnap->cz_y);

    LOG_DEBUG("%s", line);
}

static void CanRx_HandleFrame(uint64_t frame, const CAN_Header_t *header)
{
    uint8_t giveJudge = 0U;
    uint8_t giveTlDisplay = 0U;

    switch (header->msg_id)
    {
    case 0x0:
    {
        uint8_t mask = header->updateMask;

        taskENTER_CRITICAL();
        if ((mask & EGO_UPD_SPEED) != 0U)
        {
            ego.speed = (uint8_t)((frame >> 32) & 0xFFU);
        }
        if ((mask & EGO_UPD_X) != 0U)
        {
            ego.x = (uint16_t)((frame >> 22) & 0x03FFU);
        }
        if ((mask & EGO_UPD_Y) != 0U)
        {
            ego.y = (uint16_t)((frame >> 11) & 0x07FFU);
        }
        if ((mask & EGO_UPD_HEADING) != 0U)
        {
            ego.heading = (uint16_t)((frame >> 2) & 0x01FFU);
        }

        /* A heartbeat still advances the source freshness timestamp. */
        ego.timestamp = header->timestamp;
        taskEXIT_CRITICAL();
        break;
    }

    case 0x4:
    {
        CandidateVehicle candSnap;
        uint8_t mask = header->updateMask;

        taskENTER_CRITICAL();
        if ((mask & CAND_INTRO_UPD_TYPE) != 0U)
        {
            candidateVehicle.type = (uint8_t)((frame >> 32) & 0xFFU);
        }
        if ((mask & CAND_INTRO_UPD_CZ_X) != 0U)
        {
            candidateVehicle.cz_x = (uint16_t)((frame >> 22) & 0x03FFU);
        }
        if ((mask & CAND_INTRO_UPD_CZ_Y) != 0U)
        {
            candidateVehicle.cz_y = (uint16_t)((frame >> 11) & 0x07FFU);
        }
        candSnap = candidateVehicle;
        taskEXIT_CRITICAL();

        CanRx_DebugPrintMsg4(rx_id, frame, header, &candSnap);
        break;
    }

    case 0x5:
    {
        uint8_t mask = header->updateMask;
        uint8_t type = (uint8_t)((frame >> 32) & 0xFFU);
        uint32_t receivedTick = HAL_GetTick();

        taskENTER_CRITICAL();
        if (((mask & CAND_STATUS_UPD_TYPE) != 0U) &&
            (type == CAND_NONE))
        {
            memset(&candidateVehicle, 0, sizeof(candidateVehicle));
            candidateVehicle.timestamp_ms = header->timestamp;
            candidateVehicle.received_timestamp = receivedTick;
        }
        else if (((mask & CAND_STATUS_UPD_TYPE) != 0U) &&
                 (type == CAND_COMM_ERROR))
        {
            memset(&candidateVehicle, 0, sizeof(candidateVehicle));
            candidateVehicle.type = CAND_COMM_ERROR;
            candidateVehicle.timestamp_ms = header->timestamp;
            candidateVehicle.received_timestamp = receivedTick;
        }
        else
        {
            if ((mask & CAND_STATUS_UPD_TYPE) != 0U)
            {
                candidateVehicle.type = type;
            }
            if ((mask & CAND_STATUS_UPD_SPEED) != 0U)
            {
                candidateVehicle.speed = (uint8_t)((frame >> 24) & 0xFFU);
            }
            if ((mask & CAND_STATUS_UPD_X) != 0U)
            {
                candidateVehicle.x = (uint16_t)((frame >> 14) & 0x03FFU);
            }
            if ((mask & CAND_STATUS_UPD_Y) != 0U)
            {
                candidateVehicle.y = (uint16_t)((frame >> 3) & 0x07FFU);
            }

            /* A valid status heartbeat refreshes communication freshness. */
            candidateVehicle.timestamp_ms = header->timestamp;
            candidateVehicle.received_timestamp = receivedTick;
        }
        taskEXIT_CRITICAL();
        giveJudge = 1U;
        break;
    }

    case 0x6:
    {
        uint8_t mask = header->updateMask;
        uint8_t tl_type_mask = (uint8_t)((frame >> 32) & 0xFFU);
        uint8_t raw_color = (uint8_t)((frame >> 30) & 0x03U);
        uint8_t new_time_left = (uint8_t)((frame >> 26) & 0x0FU);
        uint16_t new_cz_x = (uint16_t)((frame >> 16) & 0x03FFU);
        uint16_t new_cz_y = (uint16_t)((frame >> 5) & 0x07FFU);
        uint8_t new_maneuver = (uint8_t)((frame >> 3) & 0x03U);
        uint8_t new_color = raw_color;

        if (((mask & TL_UPD_TYPE) != 0U) &&
            (tl_type_mask == TL_NONE))
        {
            new_color = 255U;
        }

        taskENTER_CRITICAL();
        uint8_t prev_maneuver = (uint8_t)maneuver;

        if ((((mask & TL_UPD_TYPE) != 0U) && (tl.type != tl_type_mask)) ||
            (((mask & TL_UPD_TYPE) != 0U) &&
             (tl_type_mask == TL_NONE) &&
             ((tl.color != 255U) || (tl.time_left != 0U))) ||
            (((mask & TL_UPD_COLOR) != 0U) && (tl.color != new_color)) ||
            (((mask & TL_UPD_TIME_LEFT) != 0U) &&
             (tl.time_left != new_time_left)))
        {
            giveTlDisplay = 1U;
        }

        if ((mask & TL_UPD_TYPE) != 0U)
        {
            tl.type = tl_type_mask;

            /* TL_NONE semantically invalidates the previous color. */
            if (tl_type_mask == TL_NONE)
            {
                tl.color = 255U;
                tl.time_left = 0U;
            }
        }
        if ((mask & TL_UPD_COLOR) != 0U)
        {
            tl.color = new_color;
        }
        if ((mask & TL_UPD_TIME_LEFT) != 0U)
        {
            tl.time_left = new_time_left;
        }
        if ((mask & TL_UPD_CZ_X) != 0U)
        {
            tl.cz_x = new_cz_x;
        }
        if ((mask & TL_UPD_CZ_Y) != 0U)
        {
            tl.cz_y = new_cz_y;
        }
        if ((mask & TL_UPD_MANEUVER) != 0U)
        {
            maneuver = (int8_t)new_maneuver;
        }
        taskEXIT_CRITICAL();

        if (((mask & TL_UPD_MANEUVER) != 0U) &&
            (new_maneuver == MANEUVER_STRAIGHT) &&
            ((prev_maneuver == MANEUVER_RIGHT_TURN) ||
             (prev_maneuver == MANEUVER_LEFT_TURN_UNPROT)))
        {
            giveJudge = 1U;
        }
        break;
    }

    default:
        break;
    }

    if ((giveTlDisplay != 0U) && (tlDisplaySem != NULL))
    {
        (void)xSemaphoreGive(tlDisplaySem);
    }

    if ((giveJudge != 0U) && (turnJudgeSem != NULL))
    {
        (void)xSemaphoreGive(turnJudgeSem);
    }
}

static void CanRxTask(void *argument)
{
    (void)argument;

    CAN_Config();

    for (;;)
    {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uint64_t frame;
        while (CAN_Rx((uint16_t *)&rx_id, &rx_header, &frame) != 0U)
        {
            canRxFlag = 1U;
            CanRx_HandleFrame(frame, &rx_header);
        }

        CAN1->IER |= CAN_IER_FMPIE0;
    }
}

void CanRxTask_Init(void)
{
    canRxTaskHandle = osThreadNew(CanRxTask, NULL, &CanRxTask_attributes);
}

void CAN1_RX0_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    CAN1->IER &= ~CAN_IER_FMPIE0;

    if (canRxTaskHandle != NULL)
    {
        vTaskNotifyGiveFromISR((TaskHandle_t)canRxTaskHandle, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
