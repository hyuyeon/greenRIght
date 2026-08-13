#include "can_rx_task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "debug_uart.h"
#include <string.h>
#include <stdio.h>
#include "time_sync.h"

#define CAN_MSG_EGO_STATUS        0x0U
#define CAN_MSG_TIME_SYNC         0x2U
#define CAN_MSG_CANDIDATE_INTRO   0x4U
#define CAN_MSG_CANDIDATE_STATUS  0x5U
#define CAN_MSG_TRAFFIC_LIGHT     0x6U

#define CAN_TIMESTAMP_MASK        0x0FFFU


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

//void CAN_Tx(uint16_t can_id, CAN_Header_t *header)
//{
//    uint64_t frame = 0U;
//
//    frame |= ((uint64_t)(header->msg_id & 0x0FU)) << 60;
//    frame |= ((uint64_t)(header->timestamp & 0x0FFFU)) << 48;
//    frame |= ((uint64_t)(header->updateMask & 0xFFU)) << 40;
//
//    switch (header->msg_id)
//    {
//    case 0x0:
//        frame |= ((uint64_t)(ego.speed & 0xFFU)) << 32;
//        frame |= ((uint64_t)(ego.x & 0x03FFU)) << 22;
//        frame |= ((uint64_t)(ego.y & 0x07FFU)) << 11;
//        frame |= ((uint64_t)(ego.heading & 0x01FFU)) << 2;
//        break;
//
//    case 0x4:
//        frame |= ((uint64_t)(candidateVehicle.type & 0xFFU)) << 32;
//        frame |= ((uint64_t)(candidateVehicle.cz_x & 0x03FFU)) << 22;
//        frame |= ((uint64_t)(candidateVehicle.cz_y & 0x07FFU)) << 11;
//        break;
//
//    case 0x5:
//        frame |= ((uint64_t)(candidateVehicle.type & 0xFFU)) << 32;
//        frame |= ((uint64_t)(candidateVehicle.speed & 0xFFU)) << 24;
//        frame |= ((uint64_t)(candidateVehicle.x & 0x03FFU)) << 14;
//        frame |= ((uint64_t)(candidateVehicle.y & 0x07FFU)) << 3;
//        break;
//
//    case 0x6:
//        frame |= ((uint64_t)(tl.color & 0x03U)) << 30;
//        frame |= ((uint64_t)(tl.time_left & 0x0FU)) << 26;
//        frame |= ((uint64_t)(tl.cz_x & 0x03FFU)) << 16;
//        frame |= ((uint64_t)(tl.cz_y & 0x07FFU)) << 5;
//        frame |= ((uint64_t)(maneuver & 0x03U)) << 3;
//        break;
//
//    default:
//        break;
//    }
//
//    CAN1->sTxMailBox[0].TDTR = 8U;
//    CAN1->sTxMailBox[0].TDLR = (uint32_t)frame;
//    CAN1->sTxMailBox[0].TDHR = (uint32_t)(frame >> 32);
//    CAN1->sTxMailBox[0].TIR = ((uint32_t)can_id << 21) | CAN_TI0R_TXRQ;
//
//    while ((CAN1->TSR & CAN_TSR_RQCP0) == 0U) { }
//    CAN1->TSR = CAN_TSR_RQCP0;
//}

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

    header->msg_id =
        (uint8_t)((frame >> 60) & 0x0FU);

    header->timestamp =
        (uint16_t)((frame >> 48) & CAN_TIMESTAMP_MASK);
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
        (unsigned long)(uint32_t)(frame >> 32),
        (unsigned long)(uint32_t)frame,
        (unsigned)candSnap->type,
        (unsigned)candSnap->cz_x,
        (unsigned)candSnap->cz_y
		);

    LOG_DEBUG("%s", line);
}

static void CanRx_HandleFrame(uint64_t frame,
                              const CAN_Header_t *header)
{
    uint8_t giveJudge = 0U;
    uint8_t giveTlDisplay = 0U;

    switch (header->msg_id)
    {
        /* =================================================
         * 0000 Ego Status
         * ================================================= */
        case CAN_MSG_EGO_STATUS:
        {
            taskENTER_CRITICAL();

            ego.speed =
                (uint8_t)((frame >> 32) & 0xFFU);

            ego.x =
                (uint16_t)((frame >> 22) & 0x03FFU);

            ego.y =
                (uint16_t)((frame >> 11) & 0x07FFU);

            ego.heading =
                (uint16_t)((frame >> 2) & 0x01FFU);

            ego.turn_signal =
                (uint8_t)(frame & 0x03U);

            ego.timestamp =
                header->timestamp;

            taskEXIT_CRITICAL();

            break;
        }

        /* =================================================
         * 0010 Time Sync
         *
         * bit63~60 : msg_id
         * bit59~48 : timestamp
         * bit47~8  : sync_epoch_ms (40 bit)
         * bit7~0   : sync_status
         * ================================================= */
        case CAN_MSG_TIME_SYNC:
        {
            uint64_t syncEpochMs;
            uint8_t syncStatus;

            syncEpochMs =
                (frame >> 8U) &
                0xFFFFFFFFFFULL;

            syncStatus =
                (uint8_t)(frame & 0xFFU);

            /*
             * Header timestamp와
             * syncEpochMs 하위 12bit가 같은지 검증
             */
            if (header->timestamp ==
                (uint16_t)(syncEpochMs &
                           CAN_TIMESTAMP_MASK))
            {
                if (TimeSync_OnSyncFrame(
                        syncEpochMs,
                        syncStatus) != 0U)
                {
                    LOG_INFO(
                        "[TIME SYNC] OK epoch=%llu status=%u\n",
                        (unsigned long long)syncEpochMs,
                        (unsigned)syncStatus
                    );
                }
            }
            else
            {
                LOG_DEBUG(
                    "[TIME SYNC] INVALID header=%u epochLow=%u\n",
                    (unsigned)header->timestamp,
                    (unsigned)(syncEpochMs &
                               CAN_TIMESTAMP_MASK)
                );
            }

            break;
        }

        /* =================================================
         * 0100 Candidate Intro
         * ================================================= */
        case CAN_MSG_CANDIDATE_INTRO:
        {
            CandidateVehicle candSnap;

            taskENTER_CRITICAL();

            candidateVehicle.type =
                (uint8_t)((frame >> 32) & 0xFFU);

            candidateVehicle.cz_x =
                (uint16_t)((frame >> 22) & 0x03FFU);

            candidateVehicle.cz_y =
                (uint16_t)((frame >> 11) & 0x07FFU);

            candSnap =
                candidateVehicle;

            taskEXIT_CRITICAL();

            CanRx_DebugPrintMsg4(
                rx_id,
                frame,
                header,
                &candSnap
            );

            break;
        }

        /* =================================================
         * 0101 Candidate Status
         *
         * bit63~60 : msg_id
         * bit59~48 : source timestamp
         * bit47~40 : candidate type
         * bit39~32 : speed
         * bit31~22 : x
         * bit21~11 : y
         * bit10~2  : heading
         * ================================================= */
        case CAN_MSG_CANDIDATE_STATUS:
        {
            CandidateVehicle newCandidate;
            uint8_t type;

            memset(
                &newCandidate,
                0,
                sizeof(newCandidate)
            );

            type =
                (uint8_t)((frame >> 40) & 0xFFU);

            newCandidate.type =
                type;

            /*
             * 중요:
             * Linux가 여기 넣어주는 값은
             * 뉴비 원본 timestamp의 하위 12bit여야 함.
             */
            newCandidate.timestamp_ms =
                (uint64_t)header->timestamp;

            /*
             * 까비 RTOS가 실제 CAN을 받은 시각
             */
            newCandidate.received_timestamp =
                (uint64_t)HAL_GetTick();

            if ((type != CAND_NONE) &&
                (type != CAND_COMM_ERROR))
            {
                /*
                 * Conflict Zone은 0100에서 받았던 값 유지
                 */
                taskENTER_CRITICAL();

                newCandidate.cz_x =
                    candidateVehicle.cz_x;

                newCandidate.cz_y =
                    candidateVehicle.cz_y;

                taskEXIT_CRITICAL();

                newCandidate.speed =
                    (uint8_t)((frame >> 32) & 0xFFU);

                newCandidate.x =
                    (uint16_t)((frame >> 22) & 0x03FFU);

                newCandidate.y =
                    (uint16_t)((frame >> 11) & 0x07FFU);

                newCandidate.heading =
                    (uint16_t)((frame >> 2) & 0x01FFU);
            }

            taskENTER_CRITICAL();

            candidateVehicle =
                newCandidate;

            taskEXIT_CRITICAL();

            giveJudge = 1U;

            break;
        }

        /* =================================================
         * 0110 Traffic Light
         * ================================================= */
        case CAN_MSG_TRAFFIC_LIGHT:
        {
            TrafficLight newTl;

            uint8_t newManeuver;
            uint8_t prevManeuver;

            memset(
                &newTl,
                0,
                sizeof(newTl)
            );

            newTl.type =
                (uint8_t)((frame >> 32) & 0xFFU);

            newTl.color =
                (uint8_t)((frame >> 30) & 0x03U);

            newTl.time_left =
                (uint8_t)((frame >> 26) & 0x0FU);

            newTl.cz_x =
                (uint16_t)((frame >> 16) & 0x03FFU);

            newTl.cz_y =
                (uint16_t)((frame >> 5) & 0x07FFU);

            newTl.timestamp =
                header->timestamp;

            newTl.received_timestamp =
                (uint64_t)HAL_GetTick();

            newManeuver =
                (uint8_t)((frame >> 3) & 0x03U);

            taskENTER_CRITICAL();

            prevManeuver =
                (uint8_t)maneuver;

            if ((tl.type != newTl.type) ||
                (tl.color != newTl.color) ||
                (tl.time_left != newTl.time_left))
            {
                giveTlDisplay = 1U;
            }

            tl =
                newTl;

            maneuver =
                (int8_t)newManeuver;

            taskEXIT_CRITICAL();

            /*
             * 회전 상황이 끝난 경우에도
             * 판단 Task를 깨워서 화면/판단 상태 갱신
             */
            if ((newManeuver ==
                 MANEUVER_STRAIGHT) &&
                ((prevManeuver ==
                  MANEUVER_RIGHT_TURN) ||
                 (prevManeuver ==
                  MANEUVER_LEFT_TURN_UNPROT)))
            {
                giveJudge = 1U;
            }

            break;
        }

        default:
        {
            break;
        }
    }

    if ((giveTlDisplay != 0U) &&
        (tlDisplaySem != NULL))
    {
        (void)xSemaphoreGive(
            tlDisplaySem
        );
    }

    if ((giveJudge != 0U) &&
        (turnJudgeSem != NULL))
    {
        (void)xSemaphoreGive(
            turnJudgeSem
        );
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
            Uart3_Printf(
                    "[CAN RAW] id=0x%03X msg=0x%X ts=%u\r\n",
                    (unsigned)rx_id,
                    (unsigned)rx_header.msg_id,
                    (unsigned)rx_header.timestamp);
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

