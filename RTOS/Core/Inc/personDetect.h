#ifndef PERSON_DETECT_H
#define PERSON_DETECT_H

#include "common.h"   /* pedFlag, maneuver, turnJudgeSem, MANEUVER_* 留ㅽ겕濡??꾨? ?ш린????*/

extern SemaphoreHandle_t  turnJudgeSem;
extern volatile uint8_t pedFlag;
extern volatile int8_t maneuver;

typedef struct {
    uint32_t irqCount;
    uint32_t rxByteCount;
    uint32_t validByteCount;
    uint32_t invalidByteCount;
    uint32_t semGiveCount;
    uint32_t overrunCount;
    uint32_t framingCount;
    uint32_t noiseCount;
    uint8_t lastRawByte;
    uint8_t lastDecodedFlag;
} PersonFlagDebugSnapshot;

void PersonFlag_Init(void);
void PersonFlag_DebugPrint(const char *msg);
void PersonFlag_GetDebugSnapshot(PersonFlagDebugSnapshot *snapshot);

#endif
