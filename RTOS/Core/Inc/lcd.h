#ifndef LCD_DRIVER_H
#define LCD_DRIVER_H

#include "stm32f4xx.h"
#include <stdint.h>
#include "common.h"

/* MADCTL ?�전 ?�용 ?? ?�플리�??�션?�서 ?�는 ?�리 ?�상??(가�?160 x ?�로 128).
 * ?�른 ?�일?�서 ?�면 범위 체크가 ?�요??????값을 참조?�면 ?�니?? */
#define LCD_W  160
#define LCD_H  128

/* 현재 주행(회전) 상태. 좌측 화살표 아이콘과 상단 라벨을 결정합니다. */
typedef enum {
    DIR_STRAIGHT = 0,
    DIR_LEFT,
    DIR_RIGHT,
	DIR_ERROR
} TurnDirection;

/* -------------------------------------------------------------------------
 * 우측 경고 목록 선택용 비트마스크.
 * 상황(직진/좌회전/우회전 + 실제 주변 차량/보행자 유무)에 따라 필요한 경고만
 * OR로 조합해서 Dashboard_DrawStatic()에 넘기면 그 항목만 화면에 표시됩니다.
 * 예) 우회전 중, 직진 차량은 없고 보행자만 있는 경우:
 *     Dashboard_DrawStatic(DIR_RIGHT, WARN_PEDESTRIAN, SIG_GREEN, 15);
 * ------------------------------------------------------------------------- */
#define WARN_NONE              0x00
#define WARN_STRAIGHT_VEHICLE  0x01u  /* 직진 차량 주의 */
#define WARN_OPPOSITE_TURN     0x02u  /* 대향 좌/우회전 차량 주의 */
#define WARN_PEDESTRIAN        0x04u  /* 보행자 (AI 인식 불가) */
#define WARN_TL                0x08u  /* 비보호좌회전 중 신호 임박 (곧 빨간불) */
#define WARN_COMM_ERROR        0x10u  /* turnState==255, 통신 오류 */

/* LCD 초기화 (SPI/GPIO 설정 + ST7735 컨트롤러 초기화, 랜드스케이프 회전 포함).
 * main()에서 HAL_Init() 다음에 한 번만 호출하면 됩니다. */
void LCD_Init(void);

/* 화면 전체를 그림: 신호등 3점(가로 배치) + 잔여시간 + 주행상태 화살표/라벨
 * (직진/좌회전/우회전 중 dir로 선택) + 우측 경고 목록(warning_mask로 선택,
 * WARN_* 비트마스크 OR 조합. 켜진 항목만 남은 공간에 균등 배치되어 표시됨).
 * 최초 1회, 또는 주행상태/경고 조합이 바뀔 때 호출하면 됩니다. */
void Dashboard_DrawStatic(TurnDirection dir, uint8_t warning_mask,
                           SignalColor active, uint8_t sec, uint8_t ped_flag);
void Dashboard_DrawDirection(TurnDirection dir);
void Dashboard_DrawWarnings(TurnDirection dir, uint8_t warning_mask, uint8_t ped_flag);
/* 잔여시간 숫자 영역만 지우고 다시 그림 (부분 리프레시). 매초 호출용. */
void Dashboard_UpdateCountdown(uint8_t sec);

/* 신호등 3점 영역만 지우고 다시 그림. 신호 색이 바뀔 때만 호출하면 됨. */
void Dashboard_DrawSignalDots(SignalColor active);

/* 해당 신호색의 지속시간(초)을 반환. main()에서 카운트다운 초기값/리셋값으로 사용. */
uint8_t Dashboard_SignalDuration(SignalColor color);

/* 다음 신호색을 반환 (RED->GREEN->YELLOW->RED 순환). */
SignalColor Dashboard_NextSignal(SignalColor color);

#endif /* LCD_DRIVER_H */
