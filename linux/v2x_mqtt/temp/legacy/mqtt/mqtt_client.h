#ifndef _MQTT_HANDLER_H_
#define _MQTT_HANDLER_H_

#include "types.h"

/* vehicle_id를 배열 인덱스로 그대로 쓰기 때문에, VEHICLE_ID_MAX(63)보다
 * 커야 한다. mqtt_topics.h 쪽 값과 맞춰서 관리한다. */
#define MAX_OTHERS 64

/*
 * MQTT 모듈 초기화
 *  - mosquitto 라이브러리 초기화
 *  - 클라이언트 생성 (client_id는 vehicle_id로부터 생성, 충돌 방지)
 *  - LWT(Last Will) 설정: 비정상 종료 시 status 토픽에 빈 payload(retain) 발행
 *  - 브로커 연결 및 백그라운드 수신 루프(mosquitto_loop_start) 시작
 *  - v2x/vehicle/+/status, v2x/trafficlight/+ 구독은 on_connect 콜백에서 수행
 *
 * 반환값: 성공 0, 실패 -1
 */
int mqtt_handler_init(const char* host, int port, uint8_t vehicle_id);

/* MQTT 모듈 종료: 루프 정지 -> 클라이언트 파괴 -> 라이브러리 정리 */
void mqtt_handler_cleanup(void);

/* 자차 VehicleInfo를 v2x/vehicle/<vehicle_id>/status 토픽으로 publish (retain=true) */
void mqtt_publish_vehicle_info(const VehicleInfo* v);

/* [안정성] VEHICLE_TIMEOUT_MS 이상 갱신되지 않은 차량(유령 차량) 슬롯을 무효화한다.
 * main 루프에서 주기적으로 호출해줘야 한다. */
void mqtt_cleanup_stale_vehicles(void);

/* 현재 유효한(살아있는) 주변 차량 수 */
int mqtt_get_others_count(void);

/* 특정 vehicle_id 슬롯의 차량 정보를 조회한다.
 * 반환값이 false면 out은 채워지지 않는다 (해당 슬롯이 비어있거나 범위 초과). */
bool mqtt_get_other_vehicle(uint8_t vehicle_id, VehicleInfo* out);

/* 신호등(tl_id: 0~7) 정보 조회. 반환값이 false면 out은 채워지지 않는다. */
bool mqtt_get_traffic_light(uint8_t tl_id, TrafficLight* out);

#endif /* _MQTT_HANDLER_H_ */