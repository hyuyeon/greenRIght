#ifndef CAN_RX_TASK_H
#define CAN_RX_TASK_H

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "common.h"

extern EgoVehicle ego;
extern CandidateVehicle candidateVehicle;
extern TrafficLight tl;
extern volatile int8_t maneuver;
extern osThreadId_t canRxTaskHandle;

void CAN_Config(void);
void CanRxTask_Init(void);
#endif
