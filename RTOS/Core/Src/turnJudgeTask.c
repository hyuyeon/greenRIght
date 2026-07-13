#include "turnJudgeTask.h"
#include "debug_uart.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

extern SemaphoreHandle_t buzzerSem;


#define CRITICAL_GAP_SEC 8.73

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
static void LeftTurn_DebugPrintTlTime(
    const EgoVehicle *egoSnap,
    const TrafficLight *tlSnap,
    uint8_t judgeResult
);
static void LeftTurn_DebugPrintCandidate(
    const char *tag,
    const EgoVehicle *egoSnap,
    const CandidateVehicle *candSnap,
    const TrafficLight *tlSnap,
    uint8_t judgeResult
);
static const osThreadAttr_t judgeTask_attributes = {
    .name = "turnJudgeTask", .stack_size = 512 * 4, .priority = (osPriority_t) osPriorityHigh,
};


static uint8_t JudgeRightTurnLeftStraight(
    const EgoVehicle *egoSnap,
    const CandidateVehicle *candSnap,
    const TrafficLight *tlSnap
)
{
	if ((tlSnap->cz_x == 0U) && (tlSnap->cz_y == 0U)) //?대? ?고쉶?꾪븷??conflict zone 吏?щ떎????
	{
	    //?쒖뒪??援ъ“ ???대윺 ???녾릿 ?섏?留?洹몃깷 ?덉쇅 泥섎━
		return 0U;
	}

    if (candSnap->speed > 0U)
    {
        return 1U;
    }

    if (tlSnap->color == 255U) //?쒖뒪??援ъ“ ???대윺 ???녾릿 ?섏?留?洹몃깷 ?덉쇅 泥섎━
    {
          return 0U;
    }

    double egoTtc = calculate_Ego_TTC(*egoSnap, tlSnap->cz_x, tlSnap->cz_y, 0U);

    if ((tlSnap->color == SIG_GREEN) && ((double)tlSnap->time_left > egoTtc))
    {
        return 0U;
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

static void RightTurn_DebugPrintLeftStraight(
    const EgoVehicle *egoSnap,
    const CandidateVehicle *candSnap,
    const TrafficLight *tlSnap,
    uint8_t judgeResult
)
{
    char line[192];
    double egoTtc = calculate_Ego_TTC(*egoSnap, tlSnap->cz_x, tlSnap->cz_y, 0U);
    uint32_t egoTtcCs;
    uint8_t greenOk = (tlSnap->color == SIG_GREEN) ? 1U : 0U;
    uint8_t timeOk = ((double)tlSnap->time_left > egoTtc) ? 1U : 0U;

    if (egoTtc < 0.0) {
        egoTtcCs = 0U;
    } else if (egoTtc > 9999.99) {
        egoTtcCs = 999999U;
    } else {
        egoTtcCs = (uint32_t)((egoTtc * 100.0) + 0.5);
    }

    (void)snprintf(line, sizeof(line),
        "[RT-LS] type=0x%02x speed=%u ego=(%u,%u,%u) tl=(c%u,t%u,cz%u,%u) egoTtc=%lu.%02lus greenOk=%u timeOk=%u judge=%u\n",
        (unsigned)candSnap->type,
        (unsigned)candSnap->speed,
        (unsigned)egoSnap->x,
        (unsigned)egoSnap->y,
        (unsigned)egoSnap->speed,
        (unsigned)tlSnap->color,
        (unsigned)tlSnap->time_left,
        (unsigned)tlSnap->cz_x,
        (unsigned)tlSnap->cz_y,
        (unsigned long)(egoTtcCs / 100U),
        (unsigned long)(egoTtcCs % 100U),
        (unsigned)greenOk,
        (unsigned)timeOk,
        (unsigned)judgeResult);

    LOG_DEBUG("%s", line);
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
        RightTurn_DebugPrintLeftStraight(egoSnap, candSnap, tlSnap, leftStraightJudge);

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
    if (tlSnap->color == 255U)
    {
        return 0U;
    }

    if (tlSnap->color != SIG_GREEN)
    {
        return 0U;
    }

    double egoTtc = calculate_Ego_TTC(*egoSnap, tlSnap->cz_x, tlSnap->cz_y, 1U);

    if (egoTtc < ((double)tlSnap->time_left))
    {
        return 0U;
    }

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
    LeftTurn_DebugPrintTlTime(egoSnap, tlSnap, tlJudge);
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
        LeftTurn_DebugPrintCandidate("OS", egoSnap, candSnap, tlSnap, oppStraightJudge);
        if (oppStraightJudge != 0U)
        {
            decision->OppStraightFlag = 1U;
        }
    }

    if ((candSnap->type & CAND_LT_OPP_RIGHT) != 0U)
    {
        uint8_t oppRightJudge = JudgeLeftTurnOppRight(egoSnap, candSnap, tlSnap);
        LeftTurn_DebugPrintCandidate("OR", egoSnap, candSnap, tlSnap, oppRightJudge);
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

    if (ttcGap >= CRITICAL_GAP_SEC)
    {
        return 0U;
    }

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

    if (ttcGap >= CRITICAL_GAP_SEC)
    {
        return 0U;
    }

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

static void LeftTurn_DebugPrintTlTime(
    const EgoVehicle *egoSnap,
    const TrafficLight *tlSnap,
    uint8_t judgeResult
)
{
    char line[192];
    double egoTtc = calculate_Ego_TTC(*egoSnap, tlSnap->cz_x, tlSnap->cz_y, 1U);
    uint32_t egoTtcCs = TtcToCentisec(egoTtc);
    uint8_t redYellow = ((tlSnap->color == SIG_RED) || (tlSnap->color == SIG_YELLOW)) ? 1U : 0U;
    uint8_t timeOk = (egoTtc < ((double)tlSnap->time_left)) ? 1U : 0U;

    (void)snprintf(line, sizeof(line),
        "[LT-TL] ego=(%u,%u,%u) tl=(c%u,t%u,cz%u,%u) egoTtc=%lu.%02lus redYellow=%u timeOk=%u judge=%u\n",
        (unsigned)egoSnap->x,
        (unsigned)egoSnap->y,
        (unsigned)egoSnap->speed,
        (unsigned)tlSnap->color,
        (unsigned)tlSnap->time_left,
        (unsigned)tlSnap->cz_x,
        (unsigned)tlSnap->cz_y,
        (unsigned long)(egoTtcCs / 100U),
        (unsigned long)(egoTtcCs % 100U),
        (unsigned)redYellow,
        (unsigned)timeOk,
        (unsigned)judgeResult);

    LOG_DEBUG("%s", line);
}

static void LeftTurn_DebugPrintCandidate(
    const char *tag,
    const EgoVehicle *egoSnap,
    const CandidateVehicle *candSnap,
    const TrafficLight *tlSnap,
    uint8_t judgeResult
)
{
    char line[224];
    double egoTtc = calculate_Ego_TTC(*egoSnap, candSnap->cz_x, candSnap->cz_y, 1U);
    double candTtc = calculate_Cand_TTC(*candSnap);
    double gap = fabs(egoTtc - candTtc);
    uint32_t egoTtcCs = TtcToCentisec(egoTtc);
    uint32_t candTtcCs = TtcToCentisec(candTtc);
    uint32_t gapCs = TtcToCentisec(gap);
    uint8_t redYellow = ((tlSnap->color == SIG_RED) || (tlSnap->color == SIG_YELLOW)) ? 1U : 0U;
    uint8_t gapOk = (gap >= CRITICAL_GAP_SEC) ? 1U : 0U;

    (void)snprintf(line, sizeof(line),
        "[LT-%s] type=0x%02x speed=%u ego=(%u,%u,%u) cand=(%u,%u,cz%u,%u) tl=c%u egoTtc=%lu.%02lus candTtc=%lu.%02lus gap=%lu.%02lus redYellow=%u gapOk=%u judge=%u\n",
        tag,
        (unsigned)candSnap->type,
        (unsigned)candSnap->speed,
        (unsigned)egoSnap->x,
        (unsigned)egoSnap->y,
        (unsigned)egoSnap->speed,
        (unsigned)candSnap->x,
        (unsigned)candSnap->y,
        (unsigned)candSnap->cz_x,
        (unsigned)candSnap->cz_y,
        (unsigned)tlSnap->color,
        (unsigned long)(egoTtcCs / 100U),
        (unsigned long)(egoTtcCs % 100U),
        (unsigned long)(candTtcCs / 100U),
        (unsigned long)(candTtcCs % 100U),
        (unsigned long)(gapCs / 100U),
        (unsigned long)(gapCs % 100U),
        (unsigned)redYellow,
        (unsigned)gapOk,
        (unsigned)judgeResult);

    LOG_DEBUG("%s", line);
}
static void DebugPrintTtcValue(const char *name, double ttc)
{
    uint32_t centisec = TtcToCentisec(ttc);

    if (centisec >= 999999U) {
        LOG_DEBUG(" %s=SAFE", name);
    } else {
        LOG_DEBUG(" %s=%lu.%02lus", name,
            (unsigned long)(centisec / 100U),
            (unsigned long)(centisec % 100U));
    }
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
        } else if ((tlSnap->color == SIG_GREEN) && ((double)tlSnap->time_left > egoTtc)) {
            AppendDecisionReason(reason, reasonSize, "RT_LS_SAFE");
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
static void TurnJudge_DebugPrint(
    uint8_t maneuverSnap,
    uint8_t pedFlagSnap,
    const EgoVehicle *egoSnap,
    const CandidateVehicle *candSnap,
    const TrafficLight *tlSnap,
    const Dicision *decision,
    uint8_t queued
)
{
    LOG_DEBUG(
        "[TJ] man=%u ped=%u cand=0x%02x ego=(%u,%u,%u) cand=(%u,%u,%u,%u) tl=(c%u,t%u,cz%u,%u)",
        (unsigned)maneuverSnap,
        (unsigned)pedFlagSnap,
        (unsigned)candSnap->type,
        (unsigned)egoSnap->x,
        (unsigned)egoSnap->y,
        (unsigned)egoSnap->speed,
        (unsigned)candSnap->x,
        (unsigned)candSnap->y,
        (unsigned)candSnap->speed,
        (unsigned)candSnap->type,
        (unsigned)tlSnap->color,
        (unsigned)tlSnap->time_left,
        (unsigned)tlSnap->cz_x,
        (unsigned)tlSnap->cz_y);

    if (maneuverSnap == MANEUVER_RIGHT_TURN) {
        if ((candSnap->type & CAND_RT_LEFT_STRAIGHT) != 0U) {
            DebugPrintTtcValue("egoTlTtc", calculate_Ego_TTC(*egoSnap, tlSnap->cz_x, tlSnap->cz_y, 0U));
        }
    } else if (maneuverSnap == MANEUVER_LEFT_TURN_UNPROT) {
        DebugPrintTtcValue("egoTlTtc", calculate_Ego_TTC(*egoSnap, tlSnap->cz_x, tlSnap->cz_y, 1U));

        if ((candSnap->type & (CAND_LT_OPP_STRAIGHT | CAND_LT_OPP_RIGHT)) != 0U) {
            double egoTtc = calculate_Ego_TTC(*egoSnap, candSnap->cz_x, candSnap->cz_y, 1U);
            double candTtc = calculate_Cand_TTC(*candSnap);
            double gap = fabs(egoTtc - candTtc);

            DebugPrintTtcValue("egoCandTtc", egoTtc);
            DebugPrintTtcValue("candTtc", candTtc);
            DebugPrintTtcValue("gap", gap);
        }
    }

    LOG_DEBUG(
        " -> dec(turn=%u,ped=%u,LS=%u,OL=%u,TL=%u,OS=%u,OR=%u,mask=0x%02x,queued=%u)\n",
        (unsigned)decision->turnState,
        (unsigned)decision->pedestrianFlag,
        (unsigned)decision->LStraightFlag,
        (unsigned)decision->OppLeftFlag,
        (unsigned)decision->tlWarningFlag,
        (unsigned)decision->OppStraightFlag,
        (unsigned)decision->OppRightFlag,
        (unsigned)DecisionWarningMask(decision),
        (unsigned)queued);
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

            TurnJudge_DebugPrint(
                maneuverSnap,
                pedFlagSnap,
                &egoSnap,
                &candSnap,
                &tlSnap,
                &decision,
                decisionQueued
            );

        }
    }
}

void TurnJudgeTask_Init(void)
{
	JudgeTaskHandle = osThreadNew(TurnJudgeTask,   NULL, &judgeTask_attributes);
}







