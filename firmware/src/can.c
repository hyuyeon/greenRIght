/*
 * can.c
 *
 *  Created on: 2026. 7. 6.
 *      Author: 한국전파진흥협회
 */

#include "main.h"
#include "can.h"

#define CAN_INIT_TIMEOUT_MS       100U

/*
 * 500kbps @ PCLK1 42MHz
 *
 * tq count = 1 sync + BS1 + BS2 = 1 + 11 + 2 = 14
 * prescaler = 6
 *
 * bitrate = 42MHz / (6 * 14) = 500kbps
 */
#define CAN_BTR_500KBPS_42MHZ     ((0U << 24) |  /* SJW  = 1TQ  -> 0 */ \
                                   (1U << 20) |  /* TS2  = 2TQ  -> 1 */ \
                                   (10U << 16) | /* TS1  = 11TQ -> 10 */ \
                                   (5U << 0))    /* BRP  = 6    -> 5 */

static uint8_t CAN1_WaitInitAckSet(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while ((CAN1->MSR & CAN_MSR_INAK) == 0U)
    {
        if ((uint32_t)(HAL_GetTick() - start) > timeout_ms)
            return 0U;
    }

    return 1U;
}

static uint8_t CAN1_WaitInitAckClear(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while ((CAN1->MSR & CAN_MSR_INAK) != 0U)
    {
        if ((uint32_t)(HAL_GetTick() - start) > timeout_ms)
            return 0U;
    }

    return 1U;
}

static void CAN1_GPIO_Init_PD0_PD1(void)
{
    /*
     * PA11 = CAN1_RX
     * PA12 = CAN1_TX
     * Alternate Function AF9
     *
     * 주의:
     * PA11/PA12는 USB FS DM/DP와 겹칩니다.
     * 현재 main.c에서 USB 초기화를 주석 처리하고 있으므로 사용 가능합니다.
     */

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

    /*
     * MODER: Alternate Function mode
     */
    GPIOD->MODER &= ~((3U << (0U * 2U)) | (3U << (1U * 2U)));
    GPIOD->MODER |=  ((2U << (0U * 2U)) | (2U << (1U * 2U)));

    /*
     * OTYPER:
     * CAN_TX는 push-pull.
     * CAN_RX도 AF 입력으로 동작하지만 OTYPER bit는 0으로 둡니다.
     */
    GPIOD->OTYPER &= ~((1U << 0U) | (1U << 1U));


    /*
     * OSPEEDR: Very high speed
     */
    GPIOD->OSPEEDR |= ((3U << (0U * 2U)) | (3U << (1U * 2U)));

    /*
     * PUPDR:
     * CAN transceiver가 RX를 구동하므로 보통 No pull.
     */
    GPIOD->PUPDR &= ~((3U << (0U * 2U)) | (3U << (1U * 2U)));


    /*
     * AFRH:
     * PA11, PA12 -> AF9
     */
    GPIOD->AFR[0] &= ~((0xFU << (0U * 4U)) |
                           (0xFU << (1U * 4U)));

        GPIOD->AFR[0] |=  ((9U << (0U * 4U)) |
                           (9U << (1U * 4U)));
}

uint8_t CAN1_Init_500kbps(void)
{
	CAN1_GPIO_Init_PD0_PD1();

    /*
     * CAN1 peripheral clock enable
     */
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;

    /*
     * CAN1 reset
     */
    RCC->APB1RSTR |= RCC_APB1RSTR_CAN1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_CAN1RST;

    /*
     * Exit sleep mode
     */
    CAN1->MCR &= ~CAN_MCR_SLEEP;

    /*
     * Request initialization mode
     */
    CAN1->MCR |= CAN_MCR_INRQ;

    if (!CAN1_WaitInitAckSet(CAN_INIT_TIMEOUT_MS))
        return 0U;

    /*
     * MCR 설정
     *
     * ABOM = Automatic bus-off management enable
     * NART = 0, automatic retransmission enable
     * TXFP = 0, identifier priority
     */
    CAN1->MCR = CAN_MCR_INRQ | CAN_MCR_ABOM;

    /*
     * Bit timing 설정
     * Normal mode.
     * Silent / loopback 안 씀.
     */
    CAN1->BTR = CAN_BTR_500KBPS_42MHZ;

    /*
     * Leave initialization mode
     */
    CAN1->MCR &= ~CAN_MCR_INRQ;

    if (!CAN1_WaitInitAckClear(CAN_INIT_TIMEOUT_MS))
        return 0U;

    return 1U;
}

uint8_t CAN1_IsTxReady(void)
{
    if ((CAN1->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) != 0U)
        return 1U;

    return 0U;
}

uint8_t CAN1_SendData(uint16_t std_id, uint8_t *data, uint8_t dlc)
{
    uint8_t mailbox;

    if (data == 0)
        return 0U;

    if (std_id > 0x7FFU)
        return 0U;

    if (dlc > 8U)
        return 0U;

    /*
     * 빈 TX mailbox 선택
     */
    if (CAN1->TSR & CAN_TSR_TME0)
    {
        mailbox = 0U;
    }
    else if (CAN1->TSR & CAN_TSR_TME1)
    {
        mailbox = 1U;
    }
    else if (CAN1->TSR & CAN_TSR_TME2)
    {
        mailbox = 2U;
    }
    else
    {
        /*
         * mailbox가 전부 차 있음.
         * 보통 수신 노드 ACK가 없거나 bus-off/배선 문제일 때도 여기로 올 수 있음.
         */
        return 0U;
    }

    /*
     * Standard ID, Data frame
     *
     * TIR:
     * STID bit31~21
     * IDE = 0
     * RTR = 0
     * TXRQ는 마지막에 set
     */
    CAN1->sTxMailBox[mailbox].TIR = ((uint32_t)(std_id & 0x7FFU) << 21);

    /*
     * DLC
     */
    CAN1->sTxMailBox[mailbox].TDTR = (uint32_t)(dlc & 0x0FU);

    /*
     * DATA0~DATA3
     */
    CAN1->sTxMailBox[mailbox].TDLR =
        ((uint32_t)data[0] << 0)  |
        ((uint32_t)data[1] << 8)  |
        ((uint32_t)data[2] << 16) |
        ((uint32_t)data[3] << 24);

    /*
     * DATA4~DATA7
     */
    CAN1->sTxMailBox[mailbox].TDHR =
        ((uint32_t)data[4] << 0)  |
        ((uint32_t)data[5] << 8)  |
        ((uint32_t)data[6] << 16) |
        ((uint32_t)data[7] << 24);

    /*
     * Transmission request
     */
    CAN1->sTxMailBox[mailbox].TIR |= 1U;

    return 1U;
}

void CAN_PackEgoStatus0000(uint8_t data[8],
                           uint16_t timestamp,
                           uint8_t update_mask,
                           uint8_t speed_mpm,
                           uint16_t x,
                           uint16_t y,
                           uint16_t heading_9bit,
                           uint8_t turn_signal)
{
    uint64_t frame = 0ULL;

    /*
     * 64bit CAN data = Header 24bit + Payload 40bit
     *
     * Header:
     * bit63~60 : message_id
     * bit59~48 : timestamp
     * bit47~40 : update_mask
     *
     * Payload:
     * bit39~32 : speed 8bit
     * bit31~22 : x 10bit
     * bit21~11 : y 11bit
     * bit10~2  : heading 9bit
     * bit1~0   : turn_signal 2bit
     */

    frame |= ((uint64_t)(CAN_MSG_ID_EGO_STATUS_0000 & 0x0FU)) << 60;
    frame |= ((uint64_t)(timestamp & 0x0FFFU))                << 48;
    frame |= ((uint64_t)(update_mask & 0xFFU))                << 40;

    frame |= ((uint64_t)(speed_mpm & 0xFFU))                  << 32;
    frame |= ((uint64_t)(x & 0x03FFU))                        << 22;
    frame |= ((uint64_t)(y & 0x07FFU))                        << 11;
    frame |= ((uint64_t)(heading_9bit & 0x01FFU))             << 2;
    frame |= ((uint64_t)(turn_signal & 0x03U));

    /*
     * data[0]이 bit63~56입니다.
     * Linux/RTOS decode도 같은 endian 약속을 써야 합니다.
     */
    data[0] = (uint8_t)((frame >> 56) & 0xFFU);
    data[1] = (uint8_t)((frame >> 48) & 0xFFU);
    data[2] = (uint8_t)((frame >> 40) & 0xFFU);
    data[3] = (uint8_t)((frame >> 32) & 0xFFU);
    data[4] = (uint8_t)((frame >> 24) & 0xFFU);
    data[5] = (uint8_t)((frame >> 16) & 0xFFU);
    data[6] = (uint8_t)((frame >> 8)  & 0xFFU);
    data[7] = (uint8_t)(frame & 0xFFU);
}

uint8_t CAN_SendEgoStatus0000(uint16_t timestamp,
                              uint8_t update_mask,
                              uint8_t speed_mpm,
                              uint16_t x,
                              uint16_t y,
                              uint16_t heading_9bit,
                              uint8_t turn_signal)
{
    uint8_t data[8];

    CAN_PackEgoStatus0000(data,
                          timestamp,
                          update_mask,
                          speed_mpm,
                          x,
                          y,
                          heading_9bit,
                          turn_signal);

    return CAN1_SendData(CAN_EGO_STATUS_STD_ID, data, 8U);
}

uint16_t CAN_GetTimestamp12(void)
{
    /*
     * 1ms tick 기준 0~4095ms에서 wrap.
     * freshness 용도로만 사용.
     */
    return (uint16_t)(HAL_GetTick() & 0x0FFFU);
}

uint8_t CAN_SpeedMpsToMpm8(float speed_mps)
{
    int32_t speed_mpm;

    if (speed_mps < 0.0f)
        speed_mps = 0.0f;

    /*
     * m/s -> m/min
     */
    speed_mpm = (int32_t)(speed_mps * 60.0f + 0.5f);

    /*
     * 프로토콜 요약 기준 0~150
     */
    if (speed_mpm < 0)
        speed_mpm = 0;

    if (speed_mpm > 150)
        speed_mpm = 150;

    return (uint8_t)speed_mpm;
}

uint16_t CAN_HeadingX100(int32_t heading_x100)
{
    /*
     * 현재 FW heading:
     *   9000 = 90.00 deg
     * protocol heading:
     *   0~511
     */

    while (heading_x100 < 0)
        heading_x100 += 36000;

    while (heading_x100 >= 36000)
        heading_x100 -= 36000;

    uint16_t heading_deg = (uint16_t)((heading_x100 + 50) / 100);
    if (heading_deg >= 360){
        heading_deg = 0;
    }
    return heading_deg;
}

uint16_t CAN_ClampU16(int32_t value, uint16_t min, uint16_t max)
{
    if (value < (int32_t)min)
        return min;

    if (value > (int32_t)max)
        return max;

    return (uint16_t)value;
}
