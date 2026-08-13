#include "buzzerTask.h"
#include "buzzer.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "semphr.h"

extern SemaphoreHandle_t buzzerSem;

static osThreadId_t BuzzerTaskHandle = NULL;

static const osThreadAttr_t BuzzerTask_attributes = {
    .name = "BuzzerTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal,
};

static void BuzzerTask(void *argument)
{
    (void)argument;

    for (;;) {
        if ((buzzerSem != NULL) && (xSemaphoreTake(buzzerSem, portMAX_DELAY) == pdTRUE)) {
            Buzzer_Play_ms(2500U, 180U);
        }
    }
}

void BuzzerTask_Init(void)
{
    BuzzerTaskHandle = osThreadNew(BuzzerTask, NULL, &BuzzerTask_attributes);
}