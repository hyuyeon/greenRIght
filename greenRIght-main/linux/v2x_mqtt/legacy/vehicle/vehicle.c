#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#include "types.h"
#include "mqtt_topics.h"
#include "vehicle_codec.h"
#include "mqtt_client.h"
#include "can.h"

static uint8_t g_vehicle_id = VEHICLE_ID_NONE;

static volatile bool g_running = true;

/* ----------------------------------------------------------------------- */

static void sleep_ms(int ms)
{
    struct timespec ts = { .tv_sec = ms / 1000, .tv_nsec = (ms % 1000) * 1000000L };
    nanosleep(&ts, NULL);
}

/* ----------------------------------------------------------------------- */
/* EgoVehicle(CAN 수신 스레드가 누적해둔 자차 상태) -> VehicleInfo(MQTT payload) 변환 */

static uint64_t monotonic_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000ULL);
}

static VehicleInfo ego_to_vehicle_info(const EgoVehicle* ego, uint8_t vehicle_id,
    const char* lanelet_id, const char* conflict_zone_id, const char* linked_tl_id)
{
    VehicleInfo v;
    memset(&v, 0, sizeof(v));

    v.vehicle_id = vehicle_id;
    v.x = ego->x & 0x03FF;
    v.y = ego->y & 0x07FF;
    v.speed = ego->speed;
    v.heading = ego->heading & 0x01FF;
    snprintf(v.lanelet_id, sizeof(v.lanelet_id), "%s", lanelet_id ? lanelet_id : "");
    snprintf(v.turn_state, sizeof(v.turn_state), "%s", ego->turn_signal ? "turn_signal_on" : "none");
    if (conflict_zone_id && conflict_zone_id[0] != '\0') {
        snprintf(v.conflict_zone_ids[0], sizeof(v.conflict_zone_ids[0]), "%s", conflict_zone_id);
        v.conflict_zone_count = 1;
    }
    snprintf(v.linked_tl_id, sizeof(v.linked_tl_id), "%s", linked_tl_id ? linked_tl_id : "");
    v.timestamp_ms = monotonic_ms();

    return v;
}

/* ----------------------------------------------------------------------- */

int main(int argc, char** argv)
{
    /* 실행 시 인자: [IP] [PORT] [VEHICLE_ID] [CAN_IFACE] */
    const char* host      = (argc > 1) ? argv[1] : "127.0.0.1";
    int         port      = (argc > 2) ? atoi(argv[2]) : 1883;
    const char* can_iface = (argc > 4) ? argv[4] : "can0";

    setvbuf(stdout, NULL, _IOLBF, 0); /* 파일로 리다이렉트해도 즉시 로그 확인 가능하도록 라인버퍼링 */

    if (argc > 3) {
        int input_id = atoi(argv[3]);
        if (input_id >= VEHICLE_ID_MIN && input_id <= VEHICLE_ID_MAX) {
            g_vehicle_id = (uint8_t)input_id;
        }
        else {
            fprintf(stderr, "ID는 %d 에서 %d 사이여야 합니다. (입력값: %d)\n", VEHICLE_ID_MIN, VEHICLE_ID_MAX, input_id);
            return 1;
        }
    }
    else {
        g_vehicle_id = 1;
        printf("[INFO] ID가 지정되지 않아 기본값 ID 1을 사용합니다.\n");
        printf("사용법: ./vehicle_app [IP] [PORT] [VEHICLE_ID(%d~%d)] [CAN_IFACE]\n",
            VEHICLE_ID_MIN, VEHICLE_ID_MAX);
    }

    printf("[ID] 현재 차량에 고유 ID %u 할당 완료\n", g_vehicle_id);

    /* ---- CAN 수신 스레드 기동 (자차 상태를 RTOS로부터 실시간 수신) ---- */
    if (can_handler_init(can_iface) != 0) {
        fprintf(stderr, "[FATAL] CAN 초기화 실패 (인터페이스: %s)\n", can_iface);
        return 1;
    }

    /* ---- MQTT 연결 (다른 차량 상태 publish/subscribe) ---- */
    if (mqtt_handler_init(host, port, g_vehicle_id) != 0) {
        fprintf(stderr, "[FATAL] MQTT 초기화 실패\n");
        can_handler_cleanup();
        return 1;
    }

    printf("=== V2X Vehicle Client 시작 (broker %s:%d, CAN %s) ===\n", host, port, can_iface);

    const char* lanelet_id = "L4";
    const char* conflict_zone_id = "cz1";
    const char* linked_tl_id = "TL1";
    uint8_t linked_tl_slot = 1;

    while (g_running) {
        EgoVehicle ego;

        /* CAN으로부터 아직 한 번도 수신 못했으면(초기 구간) publish를 건너뛴다.
         * -> 0으로 채워진 가짜 위치를 브로드캐스트하지 않기 위함 */
        if (!can_handler_get_ego(&ego)) {
            printf("[WAIT] CAN(%s)으로부터 자차 정보 수신 대기 중...\n", can_iface);
            sleep_ms(VEHICLE_PUBLISH_PERIOD_MS);
            continue;
        }

        VehicleInfo v = ego_to_vehicle_info(&ego, g_vehicle_id, lanelet_id, conflict_zone_id, linked_tl_id);

        mqtt_publish_vehicle_info(&v);

        /* [안정성] 매 루프마다 유령 차량 정리 (500ms 이상 미갱신 슬롯 무효화) */
        mqtt_cleanup_stale_vehicles();

        int others_n = mqtt_get_others_count();

        TrafficLight tl;
        bool tl_valid = mqtt_get_traffic_light(linked_tl_slot, &tl);


        printf("[TX] id=%u x=%u y=%u speed=%u heading=%u turn=%u | 주변 차량 %d대",
            v.vehicle_id, v.x, v.y, v.speed, v.heading, ego.turn_signal, others_n);

        if (tl_valid) {
            printf(" | 연동 신호등[%s] color=%u time_left=%u", v.linked_tl_id, tl.color, tl.time_left);
        }        printf("\n");

        sleep_ms(VEHICLE_PUBLISH_PERIOD_MS);
    }

    mqtt_handler_cleanup();
    can_handler_cleanup();
    return 0;
}