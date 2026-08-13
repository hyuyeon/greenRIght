/*
 * time_sync.c
 *
 *  Created on: 2026. 8. 13.
 *      Author: 한국전파진흥협회
 */


#include "time_sync.h"

#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stddef.h>
#include <string.h>

/*
 * TimeSync Task 실행 주기
 */
#define TIME_SYNC_TASK_PERIOD_MS           100U

/*
 * 500ms 동안 정상 Sync Frame이 없으면
 * 동기화 상태를 invalid 처리
 */
#define TIME_SYNC_WATCHDOG_TIMEOUT_MS      500U

/*
 * 오차 50ms 초과 -> STEP
 * 오차 50ms 이하 -> SLEW
 */
#define TIME_SYNC_SLEW_THRESHOLD_MS        50LL

/*
 * 100ms마다 최대 1ms 보정
 */
#define TIME_SYNC_MAX_SLEW_PER_PERIOD_MS   1LL

typedef struct
{
    /*
     * logical_time
     * = HAL_GetTick() + currentOffsetMs
     */
    int64_t currentOffsetMs;

    /*
     * slew가 최종적으로 도달할 offset
     */
    int64_t targetOffsetMs;

    /*
     * 가장 최근 시각 오차
     */
    int64_t lastDiffMs;

    /*
     * 마지막 정상 Time Sync frame 수신 tick
     */
    uint32_t lastValidSyncTickMs;

    /*
     * 0 : 동기화 안 됨
     * 1 : 동기화 됨
     */
    uint8_t timeSynced;

} TimeSyncState_t;

static TimeSyncState_t gTimeSync;

/* =========================================================
 * Time Sync Task
 * ========================================================= */

static const osThreadAttr_t TimeSyncTask_attributes = {
    .name = "TIME_SYNC",
    .stack_size = 256U * 4U,
    .priority = (osPriority_t)osPriorityNormal,
};

static uint64_t TimeSync_Abs64(int64_t value)
{
    if (value < 0LL)
    {
        return (uint64_t)(-value);
    }

    return (uint64_t)value;
}

static void TimeSync_Reset(void)
{
    taskENTER_CRITICAL();

    memset(&gTimeSync, 0, sizeof(gTimeSync));

    taskEXIT_CRITICAL();
}

static void TimeSync_Process(void)
{
    uint32_t nowTick;
    int64_t offsetDiff;

    nowTick = HAL_GetTick();

    taskENTER_CRITICAL();

    if (gTimeSync.timeSynced == 0U)
    {
        taskEXIT_CRITICAL();
        return;
    }

    /*
     * Linux의 Time Sync frame이 끊겼는지 확인
     */
    if ((uint32_t)(nowTick - gTimeSync.lastValidSyncTickMs) >=
        TIME_SYNC_WATCHDOG_TIMEOUT_MS)
    {
        gTimeSync.timeSynced = 0U;

        taskEXIT_CRITICAL();
        return;
    }

    offsetDiff =
        gTimeSync.targetOffsetMs -
        gTimeSync.currentOffsetMs;

    /*
     * SLEW
     */
    if (offsetDiff > TIME_SYNC_MAX_SLEW_PER_PERIOD_MS)
    {
        gTimeSync.currentOffsetMs +=
            TIME_SYNC_MAX_SLEW_PER_PERIOD_MS;
    }
    else if (offsetDiff < -TIME_SYNC_MAX_SLEW_PER_PERIOD_MS)
    {
        gTimeSync.currentOffsetMs -=
            TIME_SYNC_MAX_SLEW_PER_PERIOD_MS;
    }
    else
    {
        gTimeSync.currentOffsetMs =
            gTimeSync.targetOffsetMs;
    }

    taskEXIT_CRITICAL();
}

static void TimeSyncTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        TimeSync_Process();

        osDelay(TIME_SYNC_TASK_PERIOD_MS);
    }
}

/* =========================================================
 * Public API
 * ========================================================= */

uint8_t TimeSync_OnSyncFrame(uint64_t sync_epoch_ms,
                             uint8_t sync_status)
{
    uint64_t monotonicMs;

    int64_t desiredOffsetMs;
    int64_t currentLogicalMs;
    int64_t diffMs;

    /*
     * Linux가 NTP 동기화되지 않은 상태면
     * 기준시각으로 사용하지 않음
     */
    if (sync_status != TIME_SYNC_STATUS_OK)
    {
        taskENTER_CRITICAL();

        gTimeSync.timeSynced = 0U;

        taskEXIT_CRITICAL();

        return 0U;
    }

    /*
     * STM32 monotonic clock
     */
    monotonicMs = (uint64_t)HAL_GetTick();

    /*
     * logical_time
     * = monotonic + offset
     *
     * offset
     * = sync_time - monotonic
     */
    desiredOffsetMs =
        (int64_t)sync_epoch_ms -
        (int64_t)monotonicMs;

    taskENTER_CRITICAL();

    gTimeSync.lastValidSyncTickMs =
        (uint32_t)monotonicMs;

    /*
     * 최초 동기화
     */
    if (gTimeSync.timeSynced == 0U)
    {
        gTimeSync.currentOffsetMs =
            desiredOffsetMs;

        gTimeSync.targetOffsetMs =
            desiredOffsetMs;

        gTimeSync.lastDiffMs =
            0LL;

        gTimeSync.timeSynced =
            1U;

        taskEXIT_CRITICAL();

        return 1U;
    }

    /*
     * 현재 RTOS 논리 시간
     */
    currentLogicalMs =
        (int64_t)monotonicMs +
        gTimeSync.currentOffsetMs;

    /*
     * Linux와 RTOS의 시간차
     */
    diffMs =
        (int64_t)sync_epoch_ms -
        currentLogicalMs;

    gTimeSync.lastDiffMs =
        diffMs;

    /*
     * 큰 오차 -> STEP
     */
    if (TimeSync_Abs64(diffMs) >
        (uint64_t)TIME_SYNC_SLEW_THRESHOLD_MS)
    {
        gTimeSync.currentOffsetMs =
            desiredOffsetMs;

        gTimeSync.targetOffsetMs =
            desiredOffsetMs;
    }
    /*
     * 작은 오차 -> SLEW
     */
    else
    {
        gTimeSync.targetOffsetMs =
            desiredOffsetMs;
    }

    taskEXIT_CRITICAL();

    return 1U;
}

uint8_t TimeSync_IsSynced(void)
{
    uint8_t synced;

    taskENTER_CRITICAL();

    synced = gTimeSync.timeSynced;

    taskEXIT_CRITICAL();

    return synced;
}

uint8_t TimeSync_GetCurrentMs(uint64_t *current_ms)
{
    uint64_t monotonicMs;

    int64_t offsetMs;
    int64_t logicalMs;

    uint8_t synced;

    if (current_ms == NULL)
    {
        return 0U;
    }

    monotonicMs =
        (uint64_t)HAL_GetTick();

    taskENTER_CRITICAL();

    synced =
        gTimeSync.timeSynced;

    offsetMs =
        gTimeSync.currentOffsetMs;

    taskEXIT_CRITICAL();

    if (synced == 0U)
    {
        return 0U;
    }

    logicalMs =
        (int64_t)monotonicMs +
        offsetMs;

    if (logicalMs < 0LL)
    {
        return 0U;
    }

    *current_ms =
        (uint64_t)logicalMs;

    return 1U;
}

uint8_t TimeSync_GetTimestamp12(uint16_t *timestamp12)
{
    uint64_t currentMs;

    if (timestamp12 == NULL)
    {
        return 0U;
    }

    if (TimeSync_GetCurrentMs(&currentMs) == 0U)
    {
        return 0U;
    }

    *timestamp12 =
        (uint16_t)(currentMs & 0x0FFFULL);

    return 1U;
}

int64_t TimeSync_GetLastDiffMs(void)
{
    int64_t diffMs;

    taskENTER_CRITICAL();

    diffMs =
        gTimeSync.lastDiffMs;

    taskEXIT_CRITICAL();

    return diffMs;
}

void TimeSyncTask_Init(void)
{
    TimeSync_Reset();

    (void)osThreadNew(
        TimeSyncTask,
        NULL,
        &TimeSyncTask_attributes
    );
}
