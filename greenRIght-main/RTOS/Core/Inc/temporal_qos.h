#ifndef INC_TEMPORAL_QOS_H_
#define INC_TEMPORAL_QOS_H_

#include <stdint.h>

#include "common.h"

#define TEMPORAL_QOS_TIMESTAMP_MASK       (0x0FFFU)
#define TEMPORAL_QOS_FRESHNESS_LIMIT_MS   (300U)
#define TEMPORAL_QOS_TRACE_STAGE_ENABLE   (1U)

uint16_t TemporalQos_CalculateAgeMs(
    uint16_t currentTimestamp,
    uint16_t sourceTimestamp
);

void TemporalQos_CompensateLatency(
    const CandidateVehicle *candiOrigin,
    CandidateVehicle *compensatedCandi,
    uint16_t latencyMs
);

void TemporalQos_TraceStage(
    uint16_t logId,
    uint16_t srcTimestamp12
);

#endif /* INC_TEMPORAL_QOS_H_ */
