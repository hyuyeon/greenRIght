#include <math.h>
#include "ttc.h"

/* 거리 ?�위 변?? cm -> m */
#define CM_TO_M              0.01
/* ?�도 ?�위 변?? m/min -> m/s */
#define MIN_TO_SEC           60.0
/* ??값보???�도가 ?�으�?"?�실???��?"�?간주 (부?�소?�점 0 비교 ?�??epsilon ?�용) */
#define SPEED_EPS_M_PER_SEC  1e-6

/* CZ(충돌구역)???��? ?�착?�다�?�??�여거리(cm).
 * ?�닝 ?�라미터가 ?�니???�치 ?�차�??�수?�기 ?�한 ?�용?�차(epsilon)?�다. */
#define CZ_ARRIVED_DIST_CM        5.0

/* 벡터 길이/?�이가 ??값보???�으�?"?�실??0"?�로 보고 ?�외 케?�스�?처리 */
#define DEGENERATE_LEN_EPS        1e-3

#define PI_CONST 3.14159265358979323846

/* ---------------------------------------------------------
 * 공용 ?�수
 * ------------------------------------------------------- */

double get_distance(double x1, double y1, double x2, double y2) {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

/* ---------------------------------------------------------
 * ?��? ?�퍼 ?�수 (???�일 ?�에?�만 ?�용)
 * ------------------------------------------------------- */

/* speed(m/min) -> m/s 변??*/
static double speed_m_per_min_to_m_per_sec(uint8_t speed_m_per_min) {
    uint8_t effective_speed_m_per_min = speed_m_per_min;

    if (effective_speed_m_per_min <= TTC_CREEP_SPEED_M_PER_MIN) {
        effective_speed_m_per_min = TTC_CREEP_SPEED_M_PER_MIN;
    }

    return (double)effective_speed_m_per_min / MIN_TO_SEC;
}

/* 거리(cm) -> m 변??*/
static double dist_cm_to_m(double dist_cm) {
    return dist_cm * CM_TO_M;
}

/*
 * [Ego ?�용] heading(진행방향) 기반 ?�호 거리 계산
 *
 * ?�이?�어:
 *   ?�전 중인 차량???�제 ?�동경로??직선???�니??곡선(?�호)?�다.
 *   "?�재 ?�치 + ?�재 진행방향(?�선) + ?�착지??CZ)" ?????�보가 ?�으�?
 *   �?조건??만족?�는 ?��? ?�학?�으�????�나�?결정?�다.
 *   (2?�만?�로???�이 무한??많아 ?�정?????��?�? ?�선 방향까�? ?�면
 *    ???�나�??�정??- ?�세???�도???�??기록 참고)
 *
 * 좌표�?규약:
 *   x: ?�른쪽으�?갈수�?증�?, y: ?�로 갈수�?증�? (?��? ?�학 좌표�?
 *   heading: 0??= ?�쪽(+y), ?�계방향?�로 증�??�는 ?�침반식 각도
 *   ?�라??진행방향 벡터??(sin, cos) ?�서�?계산?�야 ?�다.
 *   (?�반?�인 ?�학 각도?�면 (cos, sin)?��?�? ?�침반식?�라 축이 바뀜에 주의!)
 *
 * 매개변??
 *   pos         : ?�재 ?�치 (cm)
 *   heading_deg : 진행방향 각도 (0~360?? ?�침반식)
 *   turn_left   : 1?�면 좌회?? 0?�면 ?�회??
 *   cz          : 충돌?�험구역 좌표 (cm)
 *
 * 반환�? pos?�서 cz까�? ?�호�??�라 ?�동?�는 거리 (cm)
 */
static double compute_arc_distance_by_heading(Point pos, double heading_deg, int turn_left, Point cz) {
    double heading_rad = heading_deg * PI_CONST / 180.0;

    /* 진행방향 ?�위벡터. heading???�침반식(0=+y, ?�계방향)?��?�?sin/cos??swap?�서 매핑 */
    double dir_x = sin(heading_rad);
    double dir_y = cos(heading_rad);

    /* ?�전 중심???�치??쪽을 가리키??법선벡터.
     * 좌회??= 진행방향??반시�?90??= (-dir_y, dir_x)
     * ?�회??= 진행방향???�계 90??  = (dir_y, -dir_x)
     * (?�건 ?�침반식?�든 ?�니??x,y ?�면 기하?�이????�� ?�립) */
    double nx, ny;
    if (turn_left) {
        nx = -dir_y;
        ny = dir_x;
    } else {
        nx = dir_y;
        ny = -dir_x;
    }

    double dx = cz.x - pos.x;
    double dy = cz.y - pos.y;
    double d_len2 = dx * dx + dy * dy;
    double d_len = sqrt(d_len2);

    /* ?��? CZ ?�치???�음 */
    if (d_len < DEGENERATE_LEN_EPS) {
        return 0.0;
    }

    /* d(=pos->cz 벡터)�?법선벡터???�영??�? ?�의 반�?�?R??구하?????�용??
     * ??값이 0 ?�하?�면 지?�한 ?�전방향?�로???�이 ?�립?��? ?�는 ?�외?�황
     * (?? heading???��? cz�?거의 ?�면?�로 ?�하�??�어 곡선 보정??무의미한 경우)
     * -> ?�런 경우??그냥 직선거리�?근사(fallback)?�다. */
    double dot_dn = dx * nx + dy * ny;
    if (dot_dn < DEGENERATE_LEN_EPS) {
        return d_len;
    }

    /* ?�의 반�?�?R ?�도 (?�선 + ?�과??조건?�로부?? */
    double R = d_len2 / (2.0 * dot_dn);

    /* pos~cz 직선거리(chord)???�?�하???�의 중심�?theta.
     * ?�의 길이 = R * theta */
    double ratio = d_len / (2.0 * R);
    if (ratio > 1.0) {
        ratio = 1.0; /* 부?�소?�점 ?�차�?1???�짝 ?�는 경우 방�? */
    }
    double theta = 2.0 * asin(ratio);

    return R * theta;
}

/* ---------------------------------------------------------
 * ?��? 공개 ?�수
 * ------------------------------------------------------- */

double calculate_Ego_TTC(EgoVehicle ego, uint16_t cz_x, uint16_t cz_y, uint8_t turn_left) {
    double speed_m_s = speed_m_per_min_to_m_per_sec(ego.speed);
    if (speed_m_s < SPEED_EPS_M_PER_SEC) {
        return TTC_SAFE; /* ?��? ?�태 - 충돌?�험구역???�원???�달 ????*/
    }

    Point pos = { (double)ego.x, (double)ego.y };
    Point cz  = { (double)cz_x, (double)cz_y };

    /* heading ?�드??9bit ?�기?��?�??�?�된 �??�체가 ?��? 0~360??각도?��?�?
     * 별도 ?��??�링(?? /512*360) ?�이 그�?�?각도�??�용?�다. */
    double heading_deg = (double)ego.heading;

    double dist_cm = compute_arc_distance_by_heading(pos, heading_deg, turn_left, cz);

    if (dist_cm <= CZ_ARRIVED_DIST_CM) {
        return 0.0; /* ?��? CZ???�달??*/
    }

    double dist_m = dist_cm_to_m(dist_cm);
    return dist_m / speed_m_s; /* �??�위 TTC */
}

double calculate_Cand_TTC(CandidateVehicle cand) {
    double speed_m_s = speed_m_per_min_to_m_per_sec(cand.speed);
    if (speed_m_s < SPEED_EPS_M_PER_SEC) {
        return TTC_SAFE; /* ?��? ?�태 */
    }

    Point pos = { (double)cand.x, (double)cand.y };
    Point cz  = { (double)cand.cz_x, (double)cand.cz_y };

    /* ?�재 ?�나리오 ?�코?? ?��?차량?� ??�� 직진?��?�?
     * ?�전 경로 근사 ?�이 CZ까�? 직선거리�?그�?�??�용?�다.
     * (CAND_RT_OPP_LEFT, CAND_LT_OPP_RIGHT처럼 ?��?차량???�전?�는 ?�?��?
     *  ?�재 ?�코?�에???�어?��? ?�는?�는 ?�제) */
    double dist_cm = get_distance(pos.x, pos.y, cz.x, cz.y);

    if (dist_cm <= CZ_ARRIVED_DIST_CM) {
        return 0.0; /* ?��? CZ???�달??*/
    }

    double dist_m = dist_cm_to_m(dist_cm);
    return dist_m / speed_m_s; /* �??�위 TTC */
}
