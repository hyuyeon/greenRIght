#include "temporal_qos.h"

#include <math.h>
#include <stddef.h>

#include "debug_uart.h"
#include "time_sync.h"

#define TEMPORAL_QOS_PI                 (3.14159265358979323846)
#define TEMPORAL_QOS_MS_PER_SECOND      (1000ULL)
#define TEMPORAL_QOS_SECONDS_PER_MINUTE (60ULL)
#define TEMPORAL_QOS_MINUTES_PER_HOUR   (60ULL)
#define TEMPORAL_QOS_HOURS_PER_DAY      (24ULL)
#define TEMPORAL_QOS_MS_PER_DAY         \
    (TEMPORAL_QOS_MS_PER_SECOND *       \
     TEMPORAL_QOS_SECONDS_PER_MINUTE *  \
     TEMPORAL_QOS_MINUTES_PER_HOUR *    \
     TEMPORAL_QOS_HOURS_PER_DAY)

uint16_t TemporalQos_CalculateAgeMs(
    uint16_t currentTimestamp,
    uint16_t sourceTimestamp
)
{
    return (uint16_t)(
        (currentTimestamp - sourceTimestamp) &
        TEMPORAL_QOS_TIMESTAMP_MASK
    );
}

void TemporalQos_CompensateLatency(
    const CandidateVehicle *candiOrigin,
    CandidateVehicle *compensatedCandi,
    uint16_t latencyMs
)
{
    double headingRadian;
    double distanceCm;
    double predictedX;
    double predictedY;

    if ((candiOrigin == NULL) || (compensatedCandi == NULL))
    {
        return;
    }

    *compensatedCandi = *candiOrigin;

    headingRadian =
        ((double)candiOrigin->heading * TEMPORAL_QOS_PI) / 180.0;

    distanceCm =
        ((double)candiOrigin->speed * (double)latencyMs) / 1000.0;

    predictedX =
        (double)candiOrigin->x +
        (distanceCm * sin(headingRadian));

    predictedY =
        (double)candiOrigin->y +
        (distanceCm * cos(headingRadian));

    if (predictedX < 0.0)
    {
        predictedX = 0.0;
    }
    else if (predictedX > 2047.0)
    {
        predictedX = 2047.0;
    }

    if (predictedY < 0.0)
    {
        predictedY = 0.0;
    }
    else if (predictedY > 2047.0)
    {
        predictedY = 2047.0;
    }

    compensatedCandi->x = (uint16_t)(predictedX + 0.5);
    compensatedCandi->y = (uint16_t)(predictedY + 0.5);
}

void TemporalQos_TraceStage(
    uint16_t logId,
    uint16_t srcTimestamp12
)
{
    uint64_t currentMs;
    uint64_t timeOfDayMs;
    uint32_t hour;
    uint32_t minute;
    uint32_t second;
    uint32_t millisecond;

    if (TimeSync_GetCurrentMs(&currentMs) == 0U)
    {
        Uart3_Printf(
            "[T%u] src_ts12=%u current=INVALID\r\n",
            (unsigned)logId,
            (unsigned)(srcTimestamp12 & TEMPORAL_QOS_TIMESTAMP_MASK)
        );
        return;
    }

    timeOfDayMs = currentMs % TEMPORAL_QOS_MS_PER_DAY;
    hour = (uint32_t)(timeOfDayMs / 3600000ULL);
    minute = (uint32_t)((timeOfDayMs / 60000ULL) % 60ULL);
    second = (uint32_t)((timeOfDayMs / 1000ULL) % 60ULL);
    millisecond = (uint32_t)(timeOfDayMs % 1000ULL);

    Uart3_Printf(
        "[T%u] src_ts12=%u [TIME SYNC] %02lu:%02lu:%02lu.%03lu UTC\r\n",
        (unsigned)logId,
        (unsigned)(srcTimestamp12 & TEMPORAL_QOS_TIMESTAMP_MASK),
        (unsigned long)hour,
        (unsigned long)minute,
        (unsigned long)second,
        (unsigned long)millisecond
    );
}
