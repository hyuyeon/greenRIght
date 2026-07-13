#ifndef DISPLAY_TASKS_H
#define DISPLAY_TASKS_H

#include "cmsis_os.h"      /* ?œìŠ¤???ì„±(osThreadNew)?€ ê¸°ì¡´ main.c?€ ?µì¼ */
#include "FreeRTOS.h"
#include "semphr.h"        /* SemaphoreHandle_t */
#include "queue.h"         /* QueueHandle_t */
#include "common.h"
#include <stdint.h>

/* ?„ì—­ ?íƒœ / ?™ê¸°??ê°ì²´ (?¼ë‹¨ ?„ì—­?¼ë¡œ ?ê³  ?•ì¸?? */
extern TrafficLight       tl;
extern SemaphoreHandle_t  tlDisplaySem;
extern SemaphoreHandle_t  lcdMutex;
extern QueueHandle_t      dicisionQueue;
extern osThreadId_t       TlDisplayTaskHandle;
extern osThreadId_t       DicisionDisplayTaskHandle;


/* main()?ì„œ LCD_Init() ?¤ìŒ, osKernelStart() ?„ì— ??ë²??¸ì¶œ.
 * ?¸ë§ˆ?¬ì–´/?????œìŠ¤???ì„± + ì´ˆê¸° ?”ë©´(ì§ì§„, ê²½ê³ ?†ìŒ) 1??ê·¸ë¦¬ê¸?*/
void DisplayTasks_Init(void);

void TlDisplayTask(void *argument);
void DicisionDisplayTask(void *argument);

/* CAN RX ?±ì—??? í˜¸??ê°’ì´ ê°±ì‹ ?????¸ì¶œ.
 * color/time_leftê°€ ?¤ì œë¡?ë³€??ê²½ìš°?ë§Œ ?¸ë§ˆ?¬ì–´ë¥?give?? */
void TrafficLight_Update(uint8_t color, uint8_t time_left, uint16_t cz_x, uint16_t cz_y);

///* ?ë‹¨ ë¡œì§?ì„œ ê²°ê³¼ê°€ ?˜ì˜¬ ?Œë§ˆ???¸ì¶œ. ?ì— ?£ê¸°ë§??˜ê³  ë°”ë¡œ ë¦¬í„´. */
//void Dicision_Post(const Dicision *d);

#endif /* DISPLAY_TASKS_H */
