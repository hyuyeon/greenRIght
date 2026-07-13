/*
 * position.c
 *
 *  Created on: 2026. 7. 3.
 *      Author: 한국전파진흥협회
 */


#include "main.h"
#include "position.h"
#include <math.h>

#define PI_F                     3.1415926535f
#define SPEED_STOP_THRESHOLD_MPS 0.02f

static float total_distance_m = 0.0f;
static float pos_x_m = 0.0f;
static float pos_y_m = 0.0f;
static uint32_t last_pos_time = 0;

void Position_Reset(void)
{
    pos_x_m = 0.0f;
    pos_y_m = 0.0f;
    total_distance_m = 0.0f;
    last_pos_time = HAL_GetTick();
}

void Position_Update(float current_speed_mps, int32_t heading_x100)
{
    uint32_t now = HAL_GetTick();

    if (last_pos_time == 0)
    {
        last_pos_time = now;
        return;
    }

    float dt = (now - last_pos_time) / 1000.0f;
    last_pos_time = now;

    if (current_speed_mps < SPEED_STOP_THRESHOLD_MPS)
    {
        return;
    }

    float heading_deg = heading_x100 / 100.0f;
    float heading_rad = heading_deg * PI_F / 180.0f;

    float distance_m = ( current_speed_mps * dt) ;

    total_distance_m += distance_m;
    pos_x_m += distance_m * sinf(heading_rad);
    pos_y_m += distance_m * cosf(heading_rad);
}

int32_t Position_GetXcm(void)
{
    return (int32_t)(pos_x_m * 100.0f);
}

int32_t Position_GetYcm(void)
{
    return (int32_t)(pos_y_m * 100.0f);
}

int32_t Position_GetTotalDistanceCm(void)
{
    return (int32_t)(total_distance_m * 100.0f);
}
