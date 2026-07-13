#ifndef RIGHT_TURN_JUDGE_TASK_H
#define RIGHT_TURN_JUDGE_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"      /* ?�스???�성(osThreadNew)?� 기존 main.c?� ?�일 */
#include "FreeRTOS.h"
#include "semphr.h"        /* SemaphoreHandle_t */
#include "queue.h"         /* QueueHandle_t */
#include <stdint.h>
#include "common.h"
#include "ttc.h"

extern volatile int8_t maneuver;
extern volatile uint8_t pedFlag;
extern EgoVehicle ego;
extern CandidateVehicle candidateVehicle;
extern TrafficLight tl;
extern SemaphoreHandle_t turnJudgeSem;
extern QueueHandle_t dicisionQueue;
extern osThreadId_t JudgeTaskHandle;

#define YELLOW_DURATION_SEC 3.0

void TurnJudgeTask_Init(void);

#endif
