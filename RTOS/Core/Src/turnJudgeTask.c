#include "turnJudgeTask.h"
#include "debug_uart.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

extern SemaphoreHandle_t buzzerSem;

//unprotected left turn duration in real-time vehicle system(5.5s) -> experimental environment(8.73s)
//you can find this on 16p of our handout
#define CRITICAL_GAP_SEC 8.73

//snapshot is a copy of the latest data received from the CAN bus
//egoSnap: ego vehicle snapshot
//candSnap: candidate vehicle snapshot
//tlSnap: traffic light snapshot
static uint8_t JudgeLeftTurnOppStraight(
    const EgoVehicle *egoSnap,
    const CandidateVehicle *candSnap,
    const TrafficLight *tlSnap
);
static uint8_t JudgeLeftTurnOppRight(
    const EgoVehicle *egoSnap,
    const CandidateVehicle *candSnap,
    const TrafficLight *tlSnap
);
static uint32_t TtcToCentisec(double ttc);


static const osThreadAttr_t judgeTask_attributes = {
    .name = "turnJudgeTask", .stack_size = 512 * 4, .priority = (osPriority_t) osPriorityHigh,
};

//The reason "Why Judge~~ function return type is int" is very simple.
//This Function returns 0 when the ego vehicle can go, and returns 1 when the ego vehicle should stop.
//simply, 0 means zero error, and 1 means there is a warning. So, we can use int type to return the result.
static uint8_t JudgeRightTurnLeftStraight(
    const EgoVehicle *egoSnap,
    const CandidateVehicle *candSnap,
    const TrafficLight *tlSnap
)
{
	enum
	{
		RT_LS_REASON_NO_CZ = 0U,
		RT_LS_REASON_CAND_MOVING,
		RT_LS_REASON_NO_TL,
		RT_LS_REASON_SIGNAL_TIME,
		RT_LS_REASON_SIGNAL_OR_TIME
	};

	static uint8_t hasPreviousLog = 0U;
	static uint8_t previousJudge = 0U;
	static uint8_t previousReason = 0U;
	static TickType_t lastLogTick = 0U;
	const TickType_t nowTick = xTaskGetTickCount();

	if ((tlSnap->cz_x == 0U) && (tlSnap->cz_y == 0U)) //if the traffic light's conflict zone is (0,0)
	{
	    //then the ego vehicle can go because there is no traffic light in front of the ego vehicle.
		if ((hasPreviousLog == 0U) ||
		    (previousJudge != 0U) ||
		    (previousReason != RT_LS_REASON_NO_CZ) ||
		    ((TickType_t)(nowTick - lastLogTick) >= pdMS_TO_TICKS(500U)))
		{
			LOG_DEBUG(
			    "[RT-LS] safe reason=no_cz type=0x%02x ego=(%u,%u,%u) cand=(%u,%u,%u) tl=(c%u,t%u,cz%u,%u) egoTtc=N/A pass=N/A judge=0\n",
			    (unsigned)candSnap->type,
			    (unsigned)egoSnap->x,
			    (unsigned)egoSnap->y,
			    (unsigned)egoSnap->speed,
			    (unsigned)candSnap->x,
			    (unsigned)candSnap->y,
			    (unsigned)candSnap->speed,
			    (unsigned)tlSnap->color,
			    (unsigned)tlSnap->time_left,
			    (unsigned)tlSnap->cz_x,
			    (unsigned)tlSnap->cz_y
			);
			hasPreviousLog = 1U;
			previousJudge = 0U;
			previousReason = RT_LS_REASON_NO_CZ;
			lastLogTick = nowTick;
		}
		return 0U;
	}

    if (candSnap->speed > 0U) //if the candidate vehicle is moving
    {
        //then the ego vehicle should stop because the candidate vehicle has priority.
        if ((hasPreviousLog == 0U) ||
            (previousJudge != 1U) ||
            (previousReason != RT_LS_REASON_CAND_MOVING) ||
            ((TickType_t)(nowTick - lastLogTick) >= pdMS_TO_TICKS(500U)))
        {
            LOG_DEBUG(
                "[RT-LS] warn reason=cand_moving type=0x%02x ego=(%u,%u,%u) cand=(%u,%u,%u) tl=(c%u,t%u,cz%u,%u) judge=1\n",
                (unsigned)candSnap->type,
                (unsigned)egoSnap->x,
                (unsigned)egoSnap->y,
                (unsigned)egoSnap->speed,
                (unsigned)candSnap->x,
                (unsigned)candSnap->y,
                (unsigned)candSnap->speed,
                (unsigned)tlSnap->color,
                (unsigned)tlSnap->time_left,
                (unsigned)tlSnap->cz_x,
                (unsigned)tlSnap->cz_y
            );
            hasPreviousLog = 1U;
            previousJudge = 1U;
            previousReason = RT_LS_REASON_CAND_MOVING;
            lastLogTick = nowTick;
        }
        return 1U;
    }

    if (tlSnap->color == 255U) //if the traffic light's color is unknown
    {
        //then the ego vehicle skip judgement because the traffic light's color is unknown.
        if ((hasPreviousLog == 0U) ||
            (previousJudge != 0U) ||
            (previousReason != RT_LS_REASON_NO_TL) ||
            ((TickType_t)(nowTick - lastLogTick) >= pdMS_TO_TICKS(500U)))
        {
            LOG_DEBUG(
                "[RT-LS] safe reason=no_tl type=0x%02x ego=(%u,%u,%u) cand=(%u,%u,%u) tl=(c%u,t%u,cz%u,%u) egoTtc=N/A pass=N/A judge=0\n",
                (unsigned)candSnap->type,
                (unsigned)egoSnap->x,
                (unsigned)egoSnap->y,
                (unsigned)egoSnap->speed,
                (unsigned)candSnap->x,
                (unsigned)candSnap->y,
                (unsigned)candSnap->speed,
                (unsigned)tlSnap->color,
                (unsigned)tlSnap->time_left,
                (unsigned)tlSnap->cz_x,
                (unsigned)tlSnap->cz_y
            );
            hasPreviousLog = 1U;
            previousJudge = 0U;
            previousReason = RT_LS_REASON_NO_TL;
            lastLogTick = nowTick;
        }
        return 0U;
    }

    double egoTtc = calculate_Ego_TTC(*egoSnap, tlSnap->cz_x, tlSnap->cz_y, 0U);
    double passableTimeSec = (double)tlSnap->time_left;

    if (tlSnap->color == SIG_GREEN)
    {
        passableTimeSec += YELLOW_DURATION_SEC;
    }

    uint32_t egoTtcCs = TtcToCentisec(egoTtc);
    uint32_t passCs = TtcToCentisec(passableTimeSec);

    if (((tlSnap->color == SIG_GREEN) || (tlSnap->color == SIG_YELLOW)) &&
        (passableTimeSec > egoTtc))
    {
        if ((hasPreviousLog == 0U) ||
            (previousJudge != 0U) ||
            (previousReason != RT_LS_REASON_SIGNAL_TIME) ||
            ((TickType_t)(nowTick - lastLogTick) >= pdMS_TO_TICKS(500U)))
        {
            LOG_DEBUG(
                "[RT-LS] safe reason=signal_time type=0x%02x ego=(%u,%u,%u) cand=(%u,%u,%u) tl=(c%u,t%u,cz%u,%u) egoTtc=%lu.%02lus pass=%lu.%02lus judge=0\n",
                (unsigned)candSnap->type,
                (unsigned)egoSnap->x,
                (unsigned)egoSnap->y,
                (unsigned)egoSnap->speed,
                (unsigned)candSnap->x,
                (unsigned)candSnap->y,
                (unsigned)candSnap->speed,
                (unsigned)tlSnap->color,
                (unsigned)tlSnap->time_left,
                (unsigned)tlSnap->cz_x,
                (unsigned)tlSnap->cz_y,
                (unsigned long)(egoTtcCs / 100U),
                (unsigned long)(egoTtcCs % 100U),
                (unsigned long)(passCs / 100U),
                (unsigned long)(passCs % 100U)
            );
            hasPreviousLog = 1U;
            previousJudge = 0U;
            previousReason = RT_LS_REASON_SIGNAL_TIME;
            lastLogTick = nowTick;
        }
        return 0U;
    }

    if ((hasPreviousLog == 0U) ||
        (previousJudge != 1U) ||
        (previousReason != RT_LS_REASON_SIGNAL_OR_TIME) ||
        ((TickType_t)(nowTick - lastLogTick) >= pdMS_TO_TICKS(500U)))
    {
        LOG_DEBUG(
            "[RT-LS] warn reason=signal_or_time type=0x%02x ego=(%u,%u,%u) cand=(%u,%u,%u) tl=(c%u,t%u,cz%u,%u) egoTtc=%lu.%02lus pass=%lu.%02lus judge=1\n",
            (unsigned)candSnap->type,
            (unsigned)egoSnap->x,
            (unsigned)egoSnap->y,
            (unsigned)egoSnap->speed,
            (unsigned)candSnap->x,
            (unsigned)candSnap->y,
            (unsigned)candSnap->speed,
            (unsigned)tlSnap->color,
            (unsigned)tlSnap->time_left,
            (unsigned)tlSnap->cz_x,
            (unsigned)tlSnap->cz_y,
            (unsigned long)(egoTtcCs / 100U),
            (unsigned long)(egoTtcCs % 100U),
            (unsigned long)(passCs / 100U),
            (unsigned long)(passCs % 100U)
        );
        hasPreviousLog = 1U;
        previousJudge = 1U;
        previousReason = RT_LS_REASON_SIGNAL_OR_TIME;
        lastLogTick = nowTick;
    }

    return 1U;
}


static uint8_t JudgeRightTurnOppLeft(
    const EgoVehicle *egoSnap,
    const CandidateVehicle *candSnap
)
{
    (void)egoSnap;
    (void)candSnap;

    return 1U;
}

static void BuildRightTurnDecision(
    Dicision *decision,
    uint8_t pedFlagSnap,
    const EgoVehicle *egoSnap,
    const CandidateVehicle *candSnap,
    const TrafficLight *tlSnap
)
{
    memset(decision, 0, sizeof(Dicision));

    decision->turnState = MANEUVER_RIGHT_TURN;

    /*
     * 1. 蹂댄뻾???먮떒
     */
    if (pedFlagSnap != 0U)
    {
        decision->pedestrianFlag = pedFlagSnap;
    }

    /*
     * 2. 醫뚯륫 吏곸쭊 李⑤웾 ?먮떒
     */
    if ((candSnap->type & CAND_RT_LEFT_STRAIGHT) != 0U) //異⑸룎 媛?ν븳 李⑤웾???덇퀬 CAND_RT_LEFT_STRAIGHT 李⑤웾?대씪硫?

    {
        uint8_t leftStraightJudge = JudgeRightTurnLeftStraight(egoSnap, candSnap, tlSnap);

        if (leftStraightJudge != 0U)
        {
            decision->LStraightFlag = 1U;
        }
    }

    /*
     * 3. ???蹂댄샇(?쇰컲) 醫뚰쉶???먮떒
     */
    if ((candSnap->type & CAND_RT_OPP_LEFT) != 0U) //異⑸룎 媛?ν븳 李⑤웾???덇퀬 CAND_RT_OPP_LEFT 李⑤웾?대씪硫?
    {
        if (JudgeRightTurnOppLeft(egoSnap, candSnap) != 0U)
        {
            decision->OppLeftFlag = 1U;
        }
    }
}


static uint8_t JudgeLeftTurnTlTime(
    const EgoVehicle *egoSnap,
    const TrafficLight *tlSnap
)
{
    if ((tlSnap->cz_x == 0U) && (tlSnap->cz_y == 0U))
    {
        LOG_DEBUG(
            "[LT-TL] skip no_cz ego=(%u,%u,%u) tl=(c%u,t%u,cz%u,%u) egoTtc=N/A pass=N/A judge=0\n",
            (unsigned)egoSnap->x,
            (unsigned)egoSnap->y,
            (unsigned)egoSnap->speed,
            (unsigned)tlSnap->color,
            (unsigned)tlSnap->time_left,
            (unsigned)tlSnap->cz_x,
            (unsigned)tlSnap->cz_y
        );
        return 0U;
    }

    double egoTtc = calculate_Ego_TTC(*egoSnap, tlSnap->cz_x, tlSnap->cz_y, 1U);
    uint32_t egoTtcCs = TtcToCentisec(egoTtc);

    if (tlSnap->color == 255U)
    {
        LOG_DEBUG(
            "[LT-TL] skip no_tl ego=(%u,%u,%u) tl=(c%u,t%u,cz%u,%u) egoTtc=%lu.%02lus pass=N/A judge=0\n",
            (unsigned)egoSnap->x,
            (unsigned)egoSnap->y,
            (unsigned)egoSnap->speed,
            (unsigned)tlSnap->color,
            (unsigned)tlSnap->time_left,
            (unsigned)tlSnap->cz_x,
            (unsigned)tlSnap->cz_y,
            (unsigned long)(egoTtcCs / 100U),
            (unsigned long)(egoTtcCs % 100U)
        );
        return 0U;
    }

    if (tlSnap->color != SIG_GREEN)
    {
        LOG_DEBUG(
            "[LT-TL] skip not_green ego=(%u,%u,%u) tl=(c%u,t%u,cz%u,%u) egoTtc=%lu.%02lus pass=N/A judge=0\n",
            (unsigned)egoSnap->x,
            (unsigned)egoSnap->y,
            (unsigned)egoSnap->speed,
            (unsigned)tlSnap->color,
            (unsigned)tlSnap->time_left,
            (unsigned)tlSnap->cz_x,
            (unsigned)tlSnap->cz_y,
            (unsigned long)(egoTtcCs / 100U),
            (unsigned long)(egoTtcCs % 100U)
        );
        return 0U;
    }

    double passableTimeSec = (double)tlSnap->time_left + YELLOW_DURATION_SEC;
    uint32_t passCs = TtcToCentisec(passableTimeSec);

    if (egoTtc < passableTimeSec)
    {
        LOG_DEBUG(
            "[LT-TL] safe ego=(%u,%u,%u) tl=(c%u,t%u,cz%u,%u) egoTtc=%lu.%02lus pass=%lu.%02lus judge=0\n",
            (unsigned)egoSnap->x,
            (unsigned)egoSnap->y,
            (unsigned)egoSnap->speed,
            (unsigned)tlSnap->color,
            (unsigned)tlSnap->time_left,
            (unsigned)tlSnap->cz_x,
            (unsigned)tlSnap->cz_y,
            (unsigned long)(egoTtcCs / 100U),
            (unsigned long)(egoTtcCs % 100U),
            (unsigned long)(passCs / 100U),
            (unsigned long)(passCs % 100U)
        );
        return 0U;
    }

    LOG_DEBUG(
        "[LT-TL] warn ego=(%u,%u,%u) tl=(c%u,t%u,cz%u,%u) egoTtc=%lu.%02lus pass=%lu.%02lus judge=1\n",
        (unsigned)egoSnap->x,
        (unsigned)egoSnap->y,
        (unsigned)egoSnap->speed,
        (unsigned)tlSnap->color,
        (unsigned)tlSnap->time_left,
        (unsigned)tlSnap->cz_x,
        (unsigned)tlSnap->cz_y,
        (unsigned long)(egoTtcCs / 100U),
        (unsigned long)(egoTtcCs % 100U),
        (unsigned long)(passCs / 100U),
        (unsigned long)(passCs % 100U)
    );

    return 1U;
}
static void BuildLeftTurnDecision(
    Dicision *decision,
    const EgoVehicle *egoSnap,
    const CandidateVehicle *candSnap,
    const TrafficLight *tlSnap
)
{
    memset(decision, 0, sizeof(Dicision));

    decision->turnState = MANEUVER_LEFT_TURN_UNPROT;

    /*
     * 1. ?占쏀샇???占쎄컙 遺占??占쎈떒
     */
    uint8_t tlJudge = JudgeLeftTurnTlTime(egoSnap, tlSnap);
    if (tlJudge != 0U)
    {
        decision->tlWarningFlag = 1U;
    }

    /*
     * 2. ?占??吏곸쭊 李⑤웾 ?占쎈떒
     */

    if ((candSnap->type & CAND_LT_OPP_STRAIGHT) != 0U)
    {
        uint8_t oppStraightJudge = JudgeLeftTurnOppStraight(egoSnap, candSnap, tlSnap);

        if (oppStraightJudge != 0U)
        {
            decision->OppStraightFlag = 1U;
        }
    }

    if ((candSnap->type & CAND_LT_OPP_RIGHT) != 0U)
    {
        uint8_t oppRightJudge = JudgeLeftTurnOppRight(egoSnap, candSnap, tlSnap);

        if (oppRightJudge != 0U)
        {
            decision->OppRightFlag = 1U;
        }
    }
}


static uint8_t JudgeLeftTurnOppStraight(
    const EgoVehicle *egoSnap,
    const CandidateVehicle *candSnap,
    const TrafficLight *tlSnap
)
{

    double egoTtc = calculate_Ego_TTC(*egoSnap, candSnap->cz_x, candSnap->cz_y, 1U);
    double candTtc = calculate_Cand_TTC(*candSnap);
    double ttcGap = fabs(egoTtc - candTtc);
    uint32_t egoTtcCs = TtcToCentisec(egoTtc);
    uint32_t candTtcCs = TtcToCentisec(candTtc);
    uint32_t gapCs = TtcToCentisec(ttcGap);

    if (ttcGap >= CRITICAL_GAP_SEC)
    {
        LOG_DEBUG(
            "[LT-OS] safe type=0x%02x ego=(%u,%u,%u) cand=(%u,%u,%u,cz%u,%u) tl=c%u egoTtc=%lu.%02lus candTtc=%lu.%02lus gap=%lu.%02lus critical=%lu.%02lus judge=0\n",
            (unsigned)candSnap->type,
            (unsigned)egoSnap->x,
            (unsigned)egoSnap->y,
            (unsigned)egoSnap->speed,
            (unsigned)candSnap->x,
            (unsigned)candSnap->y,
            (unsigned)candSnap->speed,
            (unsigned)candSnap->cz_x,
            (unsigned)candSnap->cz_y,
            (unsigned)tlSnap->color,
            (unsigned long)(egoTtcCs / 100U),
            (unsigned long)(egoTtcCs % 100U),
            (unsigned long)(candTtcCs / 100U),
            (unsigned long)(candTtcCs % 100U),
            (unsigned long)(gapCs / 100U),
            (unsigned long)(gapCs % 100U),
            (unsigned long)((uint32_t)(CRITICAL_GAP_SEC * 100.0) / 100U),
            (unsigned long)((uint32_t)(CRITICAL_GAP_SEC * 100.0) % 100U)
        );
        return 0U;
    }

    LOG_DEBUG(
        "[LT-OS] warn type=0x%02x ego=(%u,%u,%u) cand=(%u,%u,%u,cz%u,%u) tl=c%u egoTtc=%lu.%02lus candTtc=%lu.%02lus gap=%lu.%02lus critical=%lu.%02lus judge=1\n",
        (unsigned)candSnap->type,
        (unsigned)egoSnap->x,
        (unsigned)egoSnap->y,
        (unsigned)egoSnap->speed,
        (unsigned)candSnap->x,
        (unsigned)candSnap->y,
        (unsigned)candSnap->speed,
        (unsigned)candSnap->cz_x,
        (unsigned)candSnap->cz_y,
        (unsigned)tlSnap->color,
        (unsigned long)(egoTtcCs / 100U),
        (unsigned long)(egoTtcCs % 100U),
        (unsigned long)(candTtcCs / 100U),
        (unsigned long)(candTtcCs % 100U),
        (unsigned long)(gapCs / 100U),
        (unsigned long)(gapCs % 100U),
        (unsigned long)((uint32_t)(CRITICAL_GAP_SEC * 100.0) / 100U),
        (unsigned long)((uint32_t)(CRITICAL_GAP_SEC * 100.0) % 100U)
    );

    return 1U;
}

static uint8_t JudgeLeftTurnOppRight(
    const EgoVehicle *egoSnap,
    const CandidateVehicle *candSnap,
    const TrafficLight *tlSnap
)
{
    double egoTtc = calculate_Ego_TTC(*egoSnap, candSnap->cz_x, candSnap->cz_y, 1U);
    double candTtc = calculate_Cand_TTC(*candSnap);
    double ttcGap = fabs(egoTtc - candTtc);
    uint32_t egoTtcCs = TtcToCentisec(egoTtc);
    uint32_t candTtcCs = TtcToCentisec(candTtc);
    uint32_t gapCs = TtcToCentisec(ttcGap);

    if (ttcGap >= CRITICAL_GAP_SEC)
    {
        LOG_DEBUG(
            "[LT-OR] safe type=0x%02x ego=(%u,%u,%u) cand=(%u,%u,%u,cz%u,%u) tl=c%u egoTtc=%lu.%02lus candTtc=%lu.%02lus gap=%lu.%02lus critical=%lu.%02lus judge=0\n",
            (unsigned)candSnap->type,
            (unsigned)egoSnap->x,
            (unsigned)egoSnap->y,
            (unsigned)egoSnap->speed,
            (unsigned)candSnap->x,
            (unsigned)candSnap->y,
            (unsigned)candSnap->speed,
            (unsigned)candSnap->cz_x,
            (unsigned)candSnap->cz_y,
            (unsigned)tlSnap->color,
            (unsigned long)(egoTtcCs / 100U),
            (unsigned long)(egoTtcCs % 100U),
            (unsigned long)(candTtcCs / 100U),
            (unsigned long)(candTtcCs % 100U),
            (unsigned long)(gapCs / 100U),
            (unsigned long)(gapCs % 100U),
            (unsigned long)((uint32_t)(CRITICAL_GAP_SEC * 100.0) / 100U),
            (unsigned long)((uint32_t)(CRITICAL_GAP_SEC * 100.0) % 100U)
        );
        return 0U;
    }

    LOG_DEBUG(
        "[LT-OR] warn type=0x%02x ego=(%u,%u,%u) cand=(%u,%u,%u,cz%u,%u) tl=c%u egoTtc=%lu.%02lus candTtc=%lu.%02lus gap=%lu.%02lus critical=%lu.%02lus judge=1\n",
        (unsigned)candSnap->type,
        (unsigned)egoSnap->x,
        (unsigned)egoSnap->y,
        (unsigned)egoSnap->speed,
        (unsigned)candSnap->x,
        (unsigned)candSnap->y,
        (unsigned)candSnap->speed,
        (unsigned)candSnap->cz_x,
        (unsigned)candSnap->cz_y,
        (unsigned)tlSnap->color,
        (unsigned long)(egoTtcCs / 100U),
        (unsigned long)(egoTtcCs % 100U),
        (unsigned long)(candTtcCs / 100U),
        (unsigned long)(candTtcCs % 100U),
        (unsigned long)(gapCs / 100U),
        (unsigned long)(gapCs % 100U),
        (unsigned long)((uint32_t)(CRITICAL_GAP_SEC * 100.0) / 100U),
        (unsigned long)((uint32_t)(CRITICAL_GAP_SEC * 100.0) % 100U)
    );

    return 1U;
}
static uint32_t TtcToCentisec(double ttc)
{
    if (ttc < 0.0) {
        return 0U;
    }

    if (ttc > 9999.99) {
        return 999999U;
    }

    return (uint32_t)((ttc * 100.0) + 0.5);
}

static uint8_t DecisionWarningMask(const Dicision *decision)
{
    uint8_t mask = 0U;

    if (decision->pedestrianFlag != 0U) mask |= 0x01U;
    if (decision->LStraightFlag != 0U)  mask |= 0x02U;
    if (decision->OppLeftFlag != 0U)    mask |= 0x04U;
    if (decision->tlWarningFlag != 0U)  mask |= 0x08U;
    if (decision->OppStraightFlag != 0U) mask |= 0x10U;
    if (decision->OppRightFlag != 0U)   mask |= 0x20U;

    return mask;
}
static uint8_t DecisionBuzzerMask(const Dicision *decision)
{
    uint8_t mask = 0U;

    if (decision->pedestrianFlag == 1U) mask |= 0x01U;
    if (decision->LStraightFlag != 0U)  mask |= 0x02U;
    if (decision->OppLeftFlag != 0U)    mask |= 0x04U;
    if (decision->tlWarningFlag != 0U)  mask |= 0x08U;
    if (decision->OppStraightFlag != 0U) mask |= 0x10U;
    if (decision->OppRightFlag != 0U)   mask |= 0x20U;

    return mask;
}
static void AppendDecisionReason(char *dst, size_t dstSize, const char *reason)
{
    size_t len;

    if ((dst == NULL) || (dstSize == 0U) || (reason == NULL)) {
        return;
    }

    len = strlen(dst);
    if (len >= (dstSize - 1U)) {
        return;
    }

    if (len != 0U) {
        (void)snprintf(&dst[len], dstSize - len, "|%s", reason);
    } else {
        (void)snprintf(dst, dstSize, "%s", reason);
    }
}

static void BuildDecisionReason(
    uint8_t maneuverSnap,
    uint8_t pedFlagSnap,
    const EgoVehicle *egoSnap,
    const CandidateVehicle *candSnap,
    const TrafficLight *tlSnap,
    const Dicision *decision,
    char *reason,
    size_t reasonSize
)
{
    double egoTtc;

    if ((reason == NULL) || (reasonSize == 0U)) {
        return;
    }

    reason[0] = '\0';

    if (decision->turnState == 255U) {
        AppendDecisionReason(reason, reasonSize, "COMM");
    }

    if (pedFlagSnap == 1U) {
        AppendDecisionReason(reason, reasonSize, "PED");
    } else if (pedFlagSnap == 2U) {
        AppendDecisionReason(reason, reasonSize, "AI_ERR");
    }

    if (decision->LStraightFlag != 0U) {
        if (candSnap->speed > 0U) {
            AppendDecisionReason(reason, reasonSize, "RT_LS_SPEED");
        } else {
            AppendDecisionReason(reason, reasonSize, "RT_LS_BLOCK");
        }
    } else if ((maneuverSnap == MANEUVER_RIGHT_TURN) &&
               ((candSnap->type & CAND_RT_LEFT_STRAIGHT) != 0U)) {
        egoTtc = calculate_Ego_TTC(*egoSnap, tlSnap->cz_x, tlSnap->cz_y, 0U);

        if ((tlSnap->cz_x == 0U) && (tlSnap->cz_y == 0U)) {
            AppendDecisionReason(reason, reasonSize, "RT_CZ_PASSED");
        } else if (tlSnap->color == 255U) {
            AppendDecisionReason(reason, reasonSize, "RT_NO_TL");
        } else {
            double passableTimeSec = (double)tlSnap->time_left;
            if (tlSnap->color == SIG_GREEN) {
                passableTimeSec += YELLOW_DURATION_SEC;
            }
            if (((tlSnap->color == SIG_GREEN) || (tlSnap->color == SIG_YELLOW)) &&
                (passableTimeSec > egoTtc)) {
                AppendDecisionReason(reason, reasonSize, "RT_LS_SAFE");
            }
        }
    }

    if (decision->OppLeftFlag != 0U) {
        AppendDecisionReason(reason, reasonSize, "RT_OL");
    }

    if (decision->tlWarningFlag != 0U) {
        if (tlSnap->color == SIG_RED) {
            AppendDecisionReason(reason, reasonSize, "LT_TL_RED");
        } else if (tlSnap->color == SIG_YELLOW) {
            AppendDecisionReason(reason, reasonSize, "LT_TL_YELLOW");
        } else {
            AppendDecisionReason(reason, reasonSize, "LT_TL_TIME");
        }
    }

    if (decision->OppStraightFlag != 0U) {
        AppendDecisionReason(reason, reasonSize, "LT_OS_GAP");
    }

    if (decision->OppRightFlag != 0U) {
        AppendDecisionReason(reason, reasonSize, "LT_OR_GAP");
    }

    if (reason[0] == '\0') {
        AppendDecisionReason(reason, reasonSize, "CLEAR");
    }
}

static void DecisionChange_InfoPrint(
    uint8_t maneuverSnap,
    uint8_t pedFlagSnap,
    const EgoVehicle *egoSnap,
    const CandidateVehicle *candSnap,
    const TrafficLight *tlSnap,
    const Dicision *decision,
    BaseType_t queueResult
)
{
    char reason[80];
    double egoTtc = -1.0;
    double candTtc = -1.0;
    double gap = -1.0;
    uint32_t egoTtcCs;
    uint32_t candTtcCs;
    uint32_t gapCs;

    BuildDecisionReason(maneuverSnap, pedFlagSnap, egoSnap, candSnap, tlSnap, decision, reason, sizeof(reason));

    if (maneuverSnap == MANEUVER_RIGHT_TURN) {
        egoTtc = calculate_Ego_TTC(*egoSnap, tlSnap->cz_x, tlSnap->cz_y, 0U);
    } else if (maneuverSnap == MANEUVER_LEFT_TURN_UNPROT) {
        egoTtc = calculate_Ego_TTC(*egoSnap, tlSnap->cz_x, tlSnap->cz_y, 1U);

        if ((candSnap->type & (CAND_LT_OPP_STRAIGHT | CAND_LT_OPP_RIGHT)) != 0U) {
            candTtc = calculate_Cand_TTC(*candSnap);
            gap = fabs(calculate_Ego_TTC(*egoSnap, candSnap->cz_x, candSnap->cz_y, 1U) - candTtc);
        }
    }

    egoTtcCs = TtcToCentisec(egoTtc);
    candTtcCs = TtcToCentisec(candTtc);
    gapCs = TtcToCentisec(gap);

    LOG_INFO(
        "[DEC] q=%ld man=%u reason=%s mask=0x%02x ped=%u cand=0x%02x/s%u tl=0x%02x,c%u,t%u,cz%u,%u ego=%lu.%02lus cand=%lu.%02lus gap=%lu.%02lus\n",
        (long)queueResult,
        (unsigned)maneuverSnap,
        reason,
        (unsigned)DecisionWarningMask(decision),
        (unsigned)decision->pedestrianFlag,
        (unsigned)candSnap->type,
        (unsigned)candSnap->speed,
        (unsigned)tlSnap->type,
        (unsigned)tlSnap->color,
        (unsigned)tlSnap->time_left,
        (unsigned)tlSnap->cz_x,
        (unsigned)tlSnap->cz_y,
        (unsigned long)(egoTtcCs / 100U),
        (unsigned long)(egoTtcCs % 100U),
        (unsigned long)(candTtcCs / 100U),
        (unsigned long)(candTtcCs % 100U),
        (unsigned long)(gapCs / 100U),
        (unsigned long)(gapCs % 100U));
}

static uint8_t IsSameDicision(const Dicision *a, const Dicision *b)
{
    if (a->turnState != b->turnState) return 0U;
    if (a->pedestrianFlag != b->pedestrianFlag) return 0U;
    if (a->LStraightFlag != b->LStraightFlag) return 0U;
    if (a->OppLeftFlag != b->OppLeftFlag) return 0U;
    if (a->tlWarningFlag != b->tlWarningFlag) return 0U;
    if (a->OppStraightFlag != b->OppStraightFlag) return 0U;
    if (a->OppRightFlag != b->OppRightFlag) return 0U;

    return 1U;
}

static void TurnJudgeTask(void *argument)
{
    (void)argument;

    Dicision prevDecision;
    uint8_t hasPrevDecision = 0U;


    memset(&prevDecision, 0, sizeof(prevDecision));

    while (1)
    {
        if (xSemaphoreTake(turnJudgeSem, portMAX_DELAY) == pdTRUE)
        {
            uint8_t maneuverSnap;
            uint8_t pedFlagSnap;

            EgoVehicle egoSnap;
            CandidateVehicle candSnap;
            TrafficLight tlSnap;

            Dicision decision;
            uint8_t shouldSendDecision = 1U;
            uint8_t decisionQueued = 0U;

            taskENTER_CRITICAL();
            {
                maneuverSnap = maneuver;
                pedFlagSnap = pedFlag;
                egoSnap = ego;
                candSnap = candidateVehicle;
                tlSnap = tl;
            }
            taskEXIT_CRITICAL();

            memset(&decision, 0, sizeof(decision));

            if ((candSnap.type == CAND_COMM_ERROR) || (tlSnap.type == TL_COMM_ERROR))
            {
                decision.turnState = 255U;
                decision.pedestrianFlag = pedFlagSnap;
            }
            else
            {
            /*
             * maneuver 媛믪뿉 ?占쎈씪 ?占쎈떒 ?占쎌닔 遺꾧린
             */
            switch (maneuverSnap)
            {
                case MANEUVER_STRAIGHT:
                    /*
                     * 吏곸쭊 ?占쏀깭 ?占쎌떆??decision.
                     * turnState占?吏곸쭊, ?占쎈㉧吏 flag???占쏙옙? 0.
                     */
                    decision.turnState = MANEUVER_STRAIGHT;
                    break;

                case MANEUVER_RIGHT_TURN:
                    BuildRightTurnDecision(
                        &decision,
                        pedFlagSnap,
                        &egoSnap,
                        &candSnap,
                        &tlSnap
                    );
                    break;

                case MANEUVER_LEFT_TURN_UNPROT:
                    BuildLeftTurnDecision(
                        &decision,
                        &egoSnap,
                        &candSnap,
                        &tlSnap
                    );
                    break;

                default:
                    /*
                     * ?占쎈컲 醫뚰쉶?? ?占쎌긽 占??占쏙옙? ?占쎈떒 ?占쎌떆 ?占쎈떒 ?占???占쎈떂.
                     * ?占쎌슂?占쎈㈃ ?占쎄린??decision.turnState = maneuverSnap?占쎈줈 蹂대궡????
                     */
                    shouldSendDecision = 0U;
                    break;
            }
            }

            if (shouldSendDecision == 0U)
            {
                hasPrevDecision = 0U;
                continue;
            }

            /*
             * ?占쎌쟾 ?占쎈떒 寃곌낵?占??占쎈씪議뚯쓣 ?占쎈쭔 DisplayTask占??占쎌넚
             *
             * ??
             * ?占쏀쉶??寃쎄퀬 ?占쏀깭 -> 吏곸쭊 ?占쏀깭
             * ??寃쎌슦 decision.turnState媛 ?占쎈씪吏誘占????占쎌넚??
             */
            if ((hasPrevDecision == 0U) ||
                (IsSameDicision(&prevDecision, &decision) == 0U))
            {
                if (dicisionQueue != NULL)
                {
                    BaseType_t queueResult = xQueueOverwrite(dicisionQueue, &decision);
                    decisionQueued = (queueResult == pdPASS) ? 1U : 0U;

                    DecisionChange_InfoPrint(
                        maneuverSnap,
                        pedFlagSnap,
                        &egoSnap,
                        &candSnap,
                        &tlSnap,
                        &decision,
                        queueResult
                    );
                }

                prevDecision = decision;
                hasPrevDecision = 1U;
            }

            if ((decisionQueued != 0U) &&
                (DecisionBuzzerMask(&decision) != 0U) &&
                (buzzerSem != NULL))
            {
                (void)xSemaphoreGive(buzzerSem);
            }


        }
    }
}

void TurnJudgeTask_Init(void)
{
	JudgeTaskHandle = osThreadNew(TurnJudgeTask,   NULL, &judgeTask_attributes);
}







