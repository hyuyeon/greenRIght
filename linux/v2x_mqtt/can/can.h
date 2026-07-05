#ifndef _CAN_HANDLER_H_
#define _CAN_HANDLER_H_

#include "types.h"

/*
 * CAN 수신 모듈 초기화
 *  - SocketCAN raw socket 생성, ifname(예: "can0", "vcan0") 인터페이스에 bind
 *  - 백그라운드 수신 스레드(pthread) 기동
 *
 * 수신 스레드는 read()에서 블로킹된 채로 "잠들어" 있다가, 실제로 CAN 프레임이
 * 소켓 버퍼에 들어오는 순간에만 커널에 의해 깨어나 파싱을 수행한다.
 * (별도의 polling/sleep 루프 없음)
 *
 * 반환값: 성공 0, 실패 -1
 */
int can_handler_init(const char* ifname);

/* CAN 수신 스레드 종료 및 소켓 정리 */
void can_handler_cleanup(void);

/* message ID 0000(자차 상태)을 파싱해 누적된 최신 EgoVehicle 값을 조회한다.
 * update_mask로 갱신된 필드만 덮어써지고, 갱신 안 된 필드는 이전 값을 유지한다.
 * 반환값: 한 번이라도 수신된 적 있으면 true */
bool can_handler_get_ego(EgoVehicle* out);

/* message ID 0010(주변 인지 정보)을 파싱해 누적된 최신 값을 조회한다. */
bool can_handler_get_perception(PerceptionInfo* out);

/* can_handler_init()으로 연 CAN raw 소켓 fd를 그대로 반환한다.
 * CAN 송신 모듈(can_tx.c)이 RX와 동일한 소켓으로 write()하기 위해 사용.
 * 초기화 전(또는 실패 상태)이면 -1 */
int can_handler_get_socket(void);

/*
 * ---- CAN 송신 모듈 ----
 * 필터링된 타차(filtered_vehicle) 관련 정보를 msg 0100/0101로 송신.
 * main 루프에서 매 주기 한 번씩 호출.
 *
 *  own_direction   : 자차의 direction (VehicleInfo.direction, 00=직진/01=우회전/10=좌회전/11=후진)
 *  filtered_vehicle: 현재 필터링된 타차 id (0이면 없음, VEHICLE_ID_NONE)
 *  v               : filtered_vehicle에 해당하는 VehicleInfo (mqtt_get_other_vehicle()로 조회).
 *                     filtered_vehicle==0이거나 조회 실패 시 NULL 전달.
 *
 * 자차 direction이 직진/후진이면 아무것도 보내지 않고, 회전 중일 때만
 * 상대 차량 direction과 조합해 0100/0101을 보낸다. 세부 규칙은 can_tx.c 상단 주석 참고.
 *
 * 반환값: 성공 0, 소켓/송신 오류 -1 (전송할 데이터가 없어 스킵한 경우도 0)
 */
int can_tx_update_filtered_vehicle(uint8_t own_direction, uint8_t filtered_vehicle, const VehicleInfo* v);

/*
 * 연동 신호등 정보를 msg 0110으로 송신. direction 상태와 무관하게 매 주기 호출.
 * cz_x, cz_y는 현재 publish 중인 자차 VehicleInfo의 CZ 좌표를 그대로 넘기면 된다.
 *
 * 반환값: 성공 0, 실패 -1
 */
int can_tx_send_traffic_light(const TrafficLight* tl, uint16_t cz_x, uint16_t cz_y);

#endif /* _CAN_HANDLER_H_ */