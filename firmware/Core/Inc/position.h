/*
 * position.h
 *
 *  Created on: 2026. 7. 3.
 *      Author: 한국전파진흥협회
 */

#ifndef INC_POSITION_H_
#define INC_POSITION_H_

#include <stdint.h>

void Position_Reset(void);
void Position_Update(float current_speed_mps, int32_t heading_x100);
int32_t Position_GetXcm(void);
int32_t Position_GetYcm(void);
int32_t Position_GetTotalDistanceCm(void);

#endif /* INC_POSITION_H_ */
