#include "display_tasks.h"
#include "debug_uart.h"
#include "lcd.h"
#include "personDetect.h"
#include <string.h>


static const osThreadAttr_t TlDisplayTask_attributes = {
    .name = "TlDisplayTask", .stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal,
};
static const osThreadAttr_t DicisionDisplayTask_attributes = {
    .name = "DicisionDisplayTask", .stack_size = 512 * 4, .priority = (osPriority_t) osPriorityAboveNormal,
};

static void Display_LockLcd(void)
{
    if (lcdMutex != NULL) {
        (void)xSemaphoreTake(lcdMutex, portMAX_DELAY);
    }
}

static void Display_UnlockLcd(void)
{
    if (lcdMutex != NULL) {
        (void)xSemaphoreGive(lcdMutex);
    }
}
static TurnDirection Dicision_ToTurnDirection(uint8_t turnState)
{
    switch (turnState) {
        case 1: return DIR_RIGHT;
        case 2:
        case 3: return DIR_LEFT;
        case 255: return DIR_ERROR;
        case 0:
        default: return DIR_STRAIGHT;
    }
}

static uint8_t Dicision_ToWarningMask(const Dicision *d)
{
    uint8_t mask = WARN_NONE;

    /* 蹂댄뻾???뺣낫??turnState? 臾닿???蹂꾨룄 梨꾨꼸(UART)?대씪 ??긽 ?낅┰?곸쑝濡??됯?.
     * 0=?놁쓬(?쒖떆 ?덊븿), 1=?덉쓬, 2=?먮윭(AI ?몄떇遺덇?) -> 0???꾨땲硫??쒖떆 */
    if (d->pedestrianFlag != 0) mask |= WARN_PEDESTRIAN;

    switch (d->turnState) {
        case 1: /* ?고쉶??*/
            if (d->LStraightFlag) mask |= WARN_STRAIGHT_VEHICLE;
            if (d->OppLeftFlag)   mask |= WARN_OPPOSITE_TURN;
            break;

        case 2: /* 鍮꾨낫??醫뚰쉶??*/
            if (d->OppStraightFlag) mask |= WARN_STRAIGHT_VEHICLE;
            if (d->OppRightFlag)    mask |= WARN_OPPOSITE_TURN;
            if (d->tlWarningFlag)   mask |= WARN_TL;
            break;

        case 3: /* 蹂댄샇 醫뚰쉶??*/
            if (d->OppStraightFlag) mask |= WARN_STRAIGHT_VEHICLE;
            if (d->OppRightFlag)    mask |= WARN_OPPOSITE_TURN;
            break;

        case 255: /* ?듭떊 ?ㅻ쪟: 諛⑺뼢/OPP瑜??먮떒 遺덇? -> 蹂댄뻾?????꾨? 臾댁떆 */
        	mask |= WARN_COMM_ERROR;
        	break;
        default:
            break;
    }
    return mask;
}

/* tlDisplaySem(binary semaphore)??xSemaphoreTake濡?湲곕떎?몃떎媛
 * ?좏샇??3??+ 移댁슫?몃떎?대쭔 遺遺?媛깆떊 */
void TlDisplayTask(void *argument)
{
    (void)argument;
    for (;;) {
        xSemaphoreTake(tlDisplaySem, portMAX_DELAY);

        TrafficLight tlSnap;
        taskENTER_CRITICAL();
        tlSnap = tl;
        taskEXIT_CRITICAL();

        SignalColor sc = (SignalColor)tlSnap.color;  /* enum이 00/01/10과 동일해서 바로 캐스팅 */
        Display_LockLcd();
        Dashboard_DrawSignalDots(sc);
        Dashboard_UpdateCountdown(tlSnap.time_left);
        Display_UnlockLcd();
    }
}

/* dicisionQueue?먯꽌 xQueueReceive濡?硫붿떆吏瑜?湲곕떎?몃떎媛
 * 諛⑺뼢/寃쎄퀬 怨꾩궛 ???꾩껜 redraw (遺遺꾧갚??API媛 ?놁뼱 DrawStatic ?ъ슜) */
void DicisionDisplayTask(void *argument)
{
    (void)argument;
    Dicision d;
    uint32_t waitCount = 0U;

    for (;;) {
        if (xQueueReceive(dicisionQueue, &d, pdMS_TO_TICKS(1000U)) == pdTRUE) {
            TurnDirection dir = Dicision_ToTurnDirection(d.turnState);
            uint8_t warningMask = Dicision_ToWarningMask(&d);

            Display_LockLcd();
            Dashboard_DrawDirection(dir);
            Dashboard_DrawWarnings(dir, warningMask, d.pedestrianFlag);
            Display_UnlockLcd();
        } else {
            waitCount++;
            if ((waitCount % 5U) == 0U) {
                PersonFlagDebugSnapshot pedDbg;
                PersonFlag_GetDebugSnapshot(&pedDbg);
                LOG_TRACE("[PED] irq=%lu rx=%lu valid=%lu invalid=%lu sem=%lu last=0x%02x dec=%u err(o%lu,f%lu,n%lu)\n",
                    (unsigned long)pedDbg.irqCount,
                    (unsigned long)pedDbg.rxByteCount,
                    (unsigned long)pedDbg.validByteCount,
                    (unsigned long)pedDbg.invalidByteCount,
                    (unsigned long)pedDbg.semGiveCount,
                    (unsigned)pedDbg.lastRawByte,
                    (unsigned)pedDbg.lastDecodedFlag,
                    (unsigned long)pedDbg.overrunCount,
                    (unsigned long)pedDbg.framingCount,
                    (unsigned long)pedDbg.noiseCount);
            }
        }
    }
}
void TrafficLight_Update(uint8_t color, uint8_t time_left, uint16_t cz_x, uint16_t cz_y)
{
    uint8_t changed = (tl.color != color) || (tl.time_left != time_left);

    tl.type      = (color == 255U) ? TL_NONE : tl.type;
    tl.color     = color;
    tl.time_left = time_left;
    tl.cz_x      = cz_x;
    tl.cz_y      = cz_y;

    if (changed) {
        xSemaphoreGive(tlDisplaySem);
    }
}


void DisplayTasks_Init(void)
{
    /* 珥덇린 ?붾㈃: 吏곸쭊, 寃쎄퀬 ?놁쓬, 鍮④컙遺?0珥?*/
    Dashboard_DrawStatic(DIR_STRAIGHT, WARN_NONE, SIG_RED, 0, 0);

    TlDisplayTaskHandle       = osThreadNew(TlDisplayTask,       NULL, &TlDisplayTask_attributes);
    DicisionDisplayTaskHandle = osThreadNew(DicisionDisplayTask, NULL, &DicisionDisplayTask_attributes);

    LOG_INFO("[INIT] Display tasks TL=%s DEC=%s\n",
        (TlDisplayTaskHandle != NULL) ? "ok" : "fail",
        (DicisionDisplayTaskHandle != NULL) ? "ok" : "fail");
}

