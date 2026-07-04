#ifndef TEMP_TYPES_H
#define TEMP_TYPES_H

#include <stdbool.h>
#include <stdint.h>
#include "types.h"

#define TEMP_MAX_CONFLICT_ZONES VEHICLE_INFO_MAX_CONFLICT_ZONES
#define TEMP_MAX_OTHER_VEHICLES 64
#define TEMP_MAX_TRAFFIC_LIGHTS 8
#define TEMP_INVALID_ID 0xFF

typedef enum {
    TEMP_DIRECTION_UNKNOWN = 0,
    TEMP_DIRECTION_STRAIGHT = 1,
    TEMP_DIRECTION_RIGHT = 2,
    TEMP_DIRECTION_LEFT = 3,
    TEMP_DIRECTION_UNPROTECTED_LEFT = 4
} TempDirection;

typedef enum {
    TEMP_TURN_NONE = 0,
    TEMP_TURN_RIGHT = 1,
    TEMP_TURN_LEFT = 2
} TempTurnSignal;

typedef enum {
    TEMP_TURN_STATE_STRAIGHT = 0,
    TEMP_TURN_STATE_RIGHT_TURN = 1,
    TEMP_TURN_STATE_LEFT_TURN = 2,
    TEMP_TURN_STATE_UNPROTECTED_LEFT = 3
} TempTurnState;

typedef struct {
    bool found;
    bool in_intersection_center;
    char lanelet_id[VEHICLE_INFO_ID_LEN];
    TempDirection direction;
    bool unprotected_left;
    char traffic_light_id[VEHICLE_INFO_ID_LEN];
    uint8_t conflict_zone_count;
    char conflict_zone_ids[TEMP_MAX_CONFLICT_ZONES][VEHICLE_INFO_ID_LEN];
} MapContext;

static inline bool temp_is_right_signal(uint8_t turn_signal)
{
    return (turn_signal & TEMP_TURN_RIGHT) != 0;
}

static inline bool temp_is_left_signal(uint8_t turn_signal)
{
    return (turn_signal & TEMP_TURN_LEFT) != 0;
}

#endif
