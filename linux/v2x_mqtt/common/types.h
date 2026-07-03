#ifndef _TYPES_H_
#define _TYPES_H_

#include <stdint.h>
#include <stdbool.h>

// 공통 차량 정보 구조체 (자차 및 타차 모두 사용, MQTT)
typedef struct {
    uint8_t vehicle_id;
    uint16_t x;              // 10 bit (0~1023)
    uint16_t y;              // 11 bit (0~2047)
    uint8_t speed;           // 8 bit
    uint16_t heading;        // 9 bit (0~511)

    // 차선 및 방향 관련 (합쳐서 4bit 사용 가능)
    uint8_t lane;            // 2 bit (차선)
    uint8_t direction;       // 2 bit (진행 방향 또는 방향지시등 상태)

    uint16_t cz_x;           // 10 bit
    uint16_t cz_y;           // 11 bit

    uint8_t linked_tl;       // 2 bit (차량과 묶여있는 신호등)
    uint16_t timestamp;      // 12 bit

} VehicleInfo;

// CAN message ID 0000 자차 정보 구조체 (RTOS로부터 수신)
typedef struct {
    uint16_t x;              // 10 bit (0~1023)
    uint16_t y;              // 11 bit (0~2047)
    uint8_t speed;           // 8 bit
    uint16_t heading;        // 9 bit (0~511)
    uint8_t turn_signal;     // 2bit
    uint16_t timestamp;      // 12bit
} EgoVehicle;

// CAN message ID 0010 (주변 인지 정보: 차종/보행자/신호등 경고)
typedef struct {
    uint8_t vehicle_type_mask; // 8 bit, 주변 차량 종류 마스크
    uint8_t pedestrian_info;   // 2 bit, 보행자 정보
    uint8_t tl_warning;        // 2 bit, 신호등 경고
    uint16_t timestamp;        // 12 bit, CAN 헤더의 timestamp
} PerceptionInfo;

typedef struct {
    uint8_t type_mask;       // 신호등 종류 (8bit)
    uint8_t color;           // 색상 (2bit)
    uint8_t time_left;       // 남은 시간 (4bit)
} TrafficLight;



// MSB 프로토콜 변환용 64비트 프레임 (가독성을 위한 공용체/구조체 조합)
typedef union {
    uint64_t raw_data;
    uint8_t bytes[8];
    // 실제 구현 시 비트 시프트 연산(Bit-shifting)을 권장합니다.
    // (컴파일러의 엔디안/비트필드 패킹 이슈 방지)
} MSB_Frame;

#endif // _TYPES_H_