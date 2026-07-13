#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>

/* ?占쏀샇???占쏀깭. RED -> GREEN -> YELLOW -> RED ?占쎌쑝占??占쏀솚?占쎈땲?? */
typedef enum {
    SIG_RED = 0,
    SIG_YELLOW,
    SIG_GREEN,
    SIG_COUNT
} SignalColor;

typedef struct {
    uint16_t x;              // 10 bit (0~1023)
    uint16_t y;              // 11 bit (0~2047)
    uint8_t speed;           // 8 bit
    uint16_t heading;        // 9 bit (0~511)
    uint16_t timestamp;      // 12bit
} EgoVehicle;

typedef struct {
	uint8_t type; //0?占쎈㈃ ?占쎈뒗 嫄곗엫 (candidate ?占쎌쓣 ?占쎈룄 ?占쎌쓬)
	//1: ?占쎌감 ?占쏀쉶??vs ?占쏙옙? 吏곸쭊
	//2: ?占쎌감 ?占쏀쉶??vs ?占쏙옙? 蹂댄샇 醫뚰쉶??
	//4: ?占쎌감 鍮꾨낫??醫뚰쉶??vs ?占쏙옙? 吏곸쭊
	//8: ?占쎌감 鍮꾨낫??醫뚰쉶??vs ?占쏙옙? ?占쏀쉶??
	uint16_t cz_x; //candidate 李⑤웾怨쇱쓽 conflict zone??醫뚰몴
  uint16_t cz_y; //candidate 李⑤웾怨쇱쓽 conflict zone??醫뚰몴
  uint16_t x; //candidate 李⑤웾???占쎌옱 醫뚰몴
  uint16_t y; //candidate 李⑤웾???占쎌옱 醫뚰몴
  uint8_t speed; //candidate 李⑤웾???占쎌옱 spped
	uint64_t timestamp_ms; //?占쎈젅?占쎌뿉 ?占쎄꺼???占쎈뒗 timstamp_ms 洹몌옙?占??占쎄린
	uint64_t received_timestamp; //RTOS媛 諛쏆븯?????占?占쎌뒪?占쏀봽.. ?占쎈떒 ?占쎌뼱??
} CandidateVehicle;


/* color: 00 red / 01 yellow / 10 green ??lcd.h??SignalColor?占?媛믪씠 ?占쎌씪?占쏙옙?占?
 * (SignalColor)tl.color 占?諛붾줈 罹먯뒪?占쏀빐???占쎌슜 媛??*/
typedef struct {
    uint8_t  type; // msgId 0110의 tl_type_mask 원본값: 0=신호등 없음, 0x80=MQTT 통신 에러
    uint8_t  color; //?먯감???좏샇?깆씠 ?놁쓣??怨?msgId 0110?먯꽌 tl_type_mask媛 0 ?대㈃ 255媛 ?ㅼ뼱媛?
    uint8_t  time_left; //?먯감???좏샇?깆씠 ?놁쓣??怨?msgId 0110?먯꽌 tl_type_mask媛 0 ?대㈃ 0???ㅼ뼱媛?
    uint16_t cz_x;  //?곷?李⑤웾?놁뼱???먯감媛 吏꾩엯?좊븣 蹂대뒗 conflict zone(??msgId 0110?먯꽌 tl_type_mask媛 0 ?대㈃ 0???ㅼ뼱媛?
    uint16_t cz_y;  //?곷?李⑤웾?놁뼱???먯감媛 吏꾩엯?좊븣 蹂대뒗 conflict zone (??msgId 0110?먯꽌 tl_type_mask媛 0 ?대㈃ 0???ㅼ뼱媛?
} TrafficLight;


typedef struct{
  uint8_t turnState; //?듭떊 ?먮윭??255

	uint8_t pedestrianFlag; /* 00 ?占쎌쓬 / 11 ?占쎌쓬 / 01 AI ?占쎌떇遺덌옙? (0???占쎈땲占?寃쎄퀬) */
	uint8_t LStraightFlag; //?占쏀쉶????醫뚯륫 吏곸쭊李⑤웾
	uint8_t OppLeftFlag;   //?占쏀쉶?????占???占쎈컲 醫뚰쉶??
	uint8_t tlWarningFlag; //醫뚰쉶?占쎌떆 ?占쏀샇???占쎄컙 遺占?
	uint8_t OppStraightFlag; //醫뚰쉶?????占??吏곸쭊
  uint8_t OppRightFlag;    //醫뚰쉶?????占???占쏀쉶??
} Dicision;

typedef struct
{
    uint8_t msg_id;      // 4bit
    uint16_t timestamp;	 // 12bit
    uint8_t updateMask;  // 8bit
} CAN_Header_t;



/* maneuver 占?*/
#define MANEUVER_STRAIGHT          0U
#define MANEUVER_RIGHT_TURN        1U
#define MANEUVER_LEFT_TURN_UNPROT  2U
#define MANEUVER_LEFT_TURN_PROT    3U

/* candidateVehicle.type */
#define CAND_NONE                  0x00U  /* no candidate vehicle */
#define CAND_COMM_ERROR            0x80U  /* MQTT communication error: candidate unavailable */

#define CAND_RT_LEFT_STRAIGHT      1U  /* right turn vs left-side straight vehicle */
#define CAND_RT_OPP_LEFT           2U  /* right turn vs opposite protected/general left-turn vehicle */

#define CAND_LT_OPP_STRAIGHT       4U  /* unprotected left turn vs opposite straight vehicle */
#define CAND_LT_OPP_RIGHT          8U  /* unprotected left turn vs opposite right-turn vehicle */

#define TL_NONE                    0U     /* no referenced traffic light */
#define TL_COMM_ERROR              0x80U  /* MQTT communication error: traffic light unavailable */

/* dicision queue??湲몄씠 overwrite 諛⑹떇?대씪 1*/
#define DICISION_QUEUE_LEN 1

#endif /* COMMON_H */
