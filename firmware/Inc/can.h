/*
 * can.h
 *
 *  Created on: 2026. 7. 6.
 *      Author: 한국전파진흥협회
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include <stdint.h>

/*
 * 실제 CAN Standard ID.
 * 팀에서 RTOS/Linux 필터를 0x000으로 맞추면 그대로 사용.
 * 만약 CAN StdId를 0x100 등으로 따로 쓰기로 하면 여기만 바꾸면 됩니다.
 */
#define CAN_EGO_STATUS_STD_ID        0x200U

/*
 * payload header 안에 들어가는 message_id
 * bit63~60 : message_id = 0x0
 */
#define CAN_MSG_ID_EGO_STATUS_0000   0x0U

/*
 * update_mask
 * bit0 speed
 * bit1 x
 * bit2 y
 * bit3 heading
 * bit4 turn_signal
 */
#define CAN_UPDATE_SPEED             (1U << 0)
#define CAN_UPDATE_X                 (1U << 1)
#define CAN_UPDATE_Y                 (1U << 2)
#define CAN_UPDATE_HEADING           (1U << 3)
#define CAN_UPDATE_TURN_SIGNAL       (1U << 4)
#define CAN_UPDATE_ALL               0x1FU

/*
 * turn_signal protocol
 */
#define CAN_TURN_OFF                 0U
#define CAN_TURN_RIGHT               1U
#define CAN_TURN_LEFT                2U
#define CAN_TURN_RESERVED            3U

/*
 * CAN init / send
 */
uint8_t CAN1_Init_500kbps(void);
uint8_t CAN1_SendData(uint16_t std_id, uint8_t *data, uint8_t dlc);
uint8_t CAN1_IsTxReady(void);

/*
 * Protocol pack / send
 */
void CAN_PackEgoStatus0000(uint8_t data[8],
                           uint16_t timestamp,
                           uint8_t update_mask,
                           uint8_t speed_mpm,
                           uint16_t x,
                           uint16_t y,
                           uint16_t heading_9bit,
                           uint8_t turn_signal);

uint8_t CAN_SendEgoStatus0000(uint16_t timestamp,
                              uint8_t update_mask,
                              uint8_t speed_mpm,
                              uint16_t x,
                              uint16_t y,
                              uint16_t heading_9bit,
                              uint8_t turn_signal);

/*
 * Conversion helpers
 */
uint16_t CAN_GetTimestamp12(void);
uint8_t CAN_SpeedMpsToMpm8(float speed_mps);
uint16_t CAN_HeadingX100(int32_t heading_x100);
uint16_t CAN_ClampU16(int32_t value, uint16_t min, uint16_t max);

#endif /* INC_CAN_H_ */
