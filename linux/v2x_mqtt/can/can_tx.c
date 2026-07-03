#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>

#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "types.h"
#include "mqtt_topics.h"   /* VEHICLE_ID_NONE */
#include "can_client.h"

/*
 * ---- CAN TX 프레임 포맷 (자차 -> 필터링된 특정 차량/신호등 정보 브로드캐스트) ----
 *
 * 헤더 24bit는 RX와 동일한 규칙 사용:
 *   message_id(4bit) | timestamp(12bit) | update_mask(8bit)
 *
 * message_id 0100 (필터링된 차량 "소개" - 조건에 따라 1회 또는 매 주기)
 *   type_mask : 8bit  (bit39~32)  아래 compute_type_mask() 참고. 직진/후진 시엔 0으로 고정
 *   cz_x      : 10bit (bit31~22)
 *   cz_y      : 11bit (bit21~11)
 *   (미사용 11bit, bit10~0)
 *
 * message_id 0101 (필터링된 차량 "상태" - 회전 중일 때만 매 주기 반복 전송)
 *   type_mask : 8bit  (bit39~32)  compute_type_mask() 결과 (1/2/4/8)
 *   speed     : 8bit  (bit31~24)
 *   x         : 10bit (bit23~14)
 *   y         : 11bit (bit13~3)
 *   (미사용 3bit, bit2~0)
 *
 * message_id 0110 (연동 신호등 - 매 주기 반복 전송)
 *   tl_type_mask : 8bit  (bit39~32)  0xFF = 자차 신호등
 *   color        : 2bit  (bit31~30)
 *   time_left    : 4bit  (bit29~26)
 *   cz_x         : 10bit (bit25~16)  publish 중인 VehicleInfo의 CZ 좌표
 *   cz_y         : 11bit (bit15~5)
 *   (미사용 5bit, bit4~0)
 *
 * ---- type_mask (1/2/4/8) 계산 규칙 ----
 * VehicleInfo.direction 2bit: 00=직진, 01=우회전, 10=좌회전, 11=후진
 *
 *   자차 direction == 00 또는 11  : msg 0100/0101 둘 다 보내지 않음 (0110만 계속 나감)
 *   자차 direction == 01(우회전)  : 상대 direction 00->1, 10->2, 그 외는 정의 안 됨
 *   자차 direction == 10(좌회전)  : 상대 direction 00->4, 01->8, 그 외는 정의 안 됨
 *
 * "정의 안 됨" 조합(또는 filtered_vehicle 없음, 또는 직진/후진 상태)은 보낼 값이 없으므로
 * 이번 주기는 0100/0101 전송을 건너뛴다.
 */

#define CAN_MSG_ID_FILTERED_INTRO   0x4  /* 0100 */
#define CAN_MSG_ID_FILTERED_STATUS  0x5  /* 0101 */
#define CAN_MSG_ID_TRAFFIC_LIGHT    0x6  /* 0110 */

#define DIR_STRAIGHT  0x0  /* 00 */
#define DIR_RIGHT     0x1  /* 01 */
#define DIR_LEFT      0x2  /* 10 */
#define DIR_REVERSE   0x3  /* 11 */

/* TODO: 실제 CAN 버스 ID 정책에 맞춰 조정. RX측은 can_id를 보지 않고 data[8]만
 * 해석하므로 현재는 임의 고정값. 다른 노드와 공유하는 버스라면 충돌 여부 확인 필요 */
#define CAN_TX_ARBITRATION_ID  0x321

/* msg 0100(소개)을 마지막으로 보낸 filtered_vehicle. 회전 중(01/10) 상태에서
 * filtered_vehicle이 바뀌었는지 감지하는 용도. */
static uint8_t g_last_intro_sent_for = VEHICLE_ID_NONE;

/* ----------------------------------------------------------------------- */

static uint16_t next_tx_timestamp(void)
{
    /* RX 쪽 12bit wrap-around 규칙과 동일하게 맞춤 */
    static uint16_t t = 0;
    t = (uint16_t)((t + 1) & 0x0FFF);
    return t;
}

/* raw 64bit(header24 | payload40)를 MSB first로 CAN data[8]에 실어 write() */
static int send_frame(uint8_t message_id, uint8_t update_mask, uint64_t payload40)
{
    int sock = can_handler_get_socket();
    if (sock < 0) {
        fprintf(stderr, "[CAN TX] 소켓이 초기화되지 않은 상태에서 송신 시도\n");
        return -1;
    }

    uint64_t raw = 0;
    raw |= ((uint64_t)(message_id & 0xF)) << 60;
    raw |= ((uint64_t)(next_tx_timestamp() & 0xFFF)) << 48;
    raw |= ((uint64_t)(update_mask & 0xFF)) << 40;
    raw |= (payload40 & 0xFFFFFFFFFFULL); /* 하위 40bit */

    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id  = CAN_TX_ARBITRATION_ID;
    frame.can_dlc = 8;

    for (int i = 0; i < 8; i++) {
        frame.data[i] = (uint8_t)((raw >> (8 * (7 - i))) & 0xFF);
    }

    ssize_t nbytes = write(sock, &frame, sizeof(frame));
    if (nbytes != (ssize_t)sizeof(frame)) {
        perror("[CAN TX] write 실패");
        return -1;
    }
    return 0;
}

/* ----------------------------------------------------------------------- */

/* 자차 direction(own_dir) + 상대(filtered) 차량 direction(other_dir) 조합으로
 * type_mask(1/2/4/8)를 계산한다. 정의되지 않은 조합이면 false 반환. */
static bool compute_type_mask(uint8_t own_dir, uint8_t other_dir, uint8_t* out_type)
{
    if (own_dir == DIR_RIGHT) {
        if (other_dir == DIR_STRAIGHT) { *out_type = 1; return true; }
        if (other_dir == DIR_LEFT)     { *out_type = 2; return true; }
        return false;
    }
    if (own_dir == DIR_LEFT) {
        if (other_dir == DIR_STRAIGHT) { *out_type = 4; return true; }
        if (other_dir == DIR_RIGHT)    { *out_type = 8; return true; }
        return false;
    }
    return false;
}

/* msg 0100 : 필터링된 차량 "소개" (type_mask + CZ 좌표) */
static int tx_filtered_intro(uint8_t type_mask, uint16_t cz_x, uint16_t cz_y)
{
    uint64_t payload = 0;
    payload |= ((uint64_t)(type_mask & 0xFF)) << 32;
    payload |= ((uint64_t)(cz_x & 0x3FF)) << 22;
    payload |= ((uint64_t)(cz_y & 0x7FF)) << 11;

    uint8_t update_mask = 0x07; /* bit0=type, bit1=cz_x, bit2=cz_y */
    return send_frame(CAN_MSG_ID_FILTERED_INTRO, update_mask, payload);
}

/* msg 0101 : 필터링된 차량 "상태" (회전 중일 때만 호출됨) */
static int tx_filtered_status(uint8_t type_mask, uint8_t speed, uint16_t x, uint16_t y)
{
    uint64_t payload = 0;
    payload |= ((uint64_t)(type_mask & 0xFF)) << 32;
    payload |= ((uint64_t)(speed & 0xFF)) << 24;
    payload |= ((uint64_t)(x & 0x3FF)) << 14;
    payload |= ((uint64_t)(y & 0x7FF)) << 3;

    uint8_t update_mask = 0x0F; /* bit0=type,1=speed,2=x,3=y */
    return send_frame(CAN_MSG_ID_FILTERED_STATUS, update_mask, payload);
}

/* msg 0110 : 연동 신호등 (매 주기 반복) */
static int tx_traffic_light(const TrafficLight* tl, uint16_t cz_x, uint16_t cz_y)
{
    if (!tl) return -1;

    uint64_t payload = 0;
    payload |= ((uint64_t)(tl->type_mask & 0xFF)) << 32;
    payload |= ((uint64_t)(tl->color & 0x3)) << 30;
    payload |= ((uint64_t)(tl->time_left & 0xF)) << 26;
    payload |= ((uint64_t)(cz_x & 0x3FF)) << 16;
    payload |= ((uint64_t)(cz_y & 0x7FF)) << 5;

    uint8_t update_mask = 0x1F; /* bit0=type,1=color,2=time_left,3=cz_x,4=cz_y */
    return send_frame(CAN_MSG_ID_TRAFFIC_LIGHT, update_mask, payload);
}

/* ----------------------------------------------------------------------- */
/* 외부 공개 API */

/*
 * main 루프에서 매 주기 한 번씩 호출.
 *
 *  own_direction   : 자차의 direction (VehicleInfo.direction, 00/01/10/11)
 *  filtered_vehicle: 현재 필터링된 타차 id (0이면 없음)
 *  v               : filtered_vehicle에 해당하는 VehicleInfo (mqtt_get_other_vehicle()로 조회).
 *                     filtered_vehicle==0이거나 조회 실패면 NULL 전달.
 *
 * - own_direction == 00/11 : msg 0100/0101 모두 전송하지 않음 (0110은 별도 함수로 계속 나감)
 * - own_direction == 01/10 : v->direction과 조합해 type_mask 계산.
 *     계산 성공 시 filtered_vehicle이 바뀐 시점에 0100(소개) 1회 + 매 주기 0101(상태)
 *     계산 불가(정의 안 된 조합, 또는 필터 대상 없음) 시 이번 주기는 아무것도 보내지 않음
 */
int can_tx_update_filtered_vehicle(uint8_t own_direction, uint8_t filtered_vehicle, const VehicleInfo* v)
{
    if (own_direction == DIR_STRAIGHT || own_direction == DIR_REVERSE) {
        /* 직진/후진 상태: 0100/0101 아무것도 보내지 않는다 (0110만 계속 나감).
         * 다음에 회전 상태로 바뀌었을 때 filtered_vehicle이 그대로라도 0100이
         * 다시 나가도록, 여기서 "마지막으로 소개한 대상" 기록을 초기화해둔다. */
        g_last_intro_sent_for = VEHICLE_ID_NONE;
        return 0;
    }

    /* 우회전/좌회전: type_mask 계산 시도 */
    uint8_t type_mask = 0;
    bool classified = false;

    if (filtered_vehicle != VEHICLE_ID_NONE && v != NULL) {
        classified = compute_type_mask(own_direction, v->direction, &type_mask);
    }

    if (!classified) {
        return 0; /* 정의되지 않은 조합 - 보낼 데이터 없음 */
    }

    if (filtered_vehicle != g_last_intro_sent_for) {
        if (tx_filtered_intro(type_mask, v->cz_x, v->cz_y) != 0) return -1;
        g_last_intro_sent_for = filtered_vehicle;
    }

    return tx_filtered_status(type_mask, v->speed, v->x, v->y);
}

int can_tx_send_traffic_light(const TrafficLight* tl, uint16_t cz_x, uint16_t cz_y)
{
    return tx_traffic_light(tl, cz_x, cz_y);
}