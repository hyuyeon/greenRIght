/*
 * time_sync.h
 *
 *  Created on: 2026. 8. 13.
 *      Author: 한국전파진흥협회
 */

#ifndef INC_TIME_SYNC_H_
#define INC_TIME_SYNC_H_

#include <stdint.h>

/*
 * Time Sync CAN Message ID
 *
 * 0010b = 0x2
 */
#define TIME_SYNC_MSG_ID             (0x02U)

/*
 * Linux NTP synchronization status
 */
#define TIME_SYNC_STATUS_OK          (0x00U)
#define TIME_SYNC_STATUS_RTC_ONLY    (0x01U)
#define TIME_SYNC_STATUS_NONE        (0x02U)

/*
 * Time Sync Task 생성
 */
void TimeSyncTask_Init(void);

/*
 * Linux -> RTOS Time Sync CAN Frame 수신 처리
 */
uint8_t TimeSync_OnSyncFrame(uint64_t sync_epoch_ms,
                             uint8_t sync_status);

/*
 * 현재 RTOS 시계가 유효하게 동기화되어 있는지
 */
uint8_t TimeSync_IsSynced(void);

/*
 * 현재 NTP 기준 동기화 시간(ms)
 */
uint8_t TimeSync_GetCurrentMs(uint64_t *current_ms);

/*
 * CAN Header에서 사용하는 하위 12bit timestamp
 */
uint8_t TimeSync_GetTimestamp12(uint16_t *timestamp12);

/*
 * Linux 기준시간과 RTOS 논리시간의 최근 차이
 */
int64_t TimeSync_GetLastDiffMs(void);


#endif /* INC_TIME_SYNC_H_ */
