/*
 * bno055.h
 *
 *  Created on: 2026. 7. 3.
 *      Author: 한국전파진흥협회
 */

#ifndef INC_BNO055_H_
#define INC_BNO055_H_

#include <stdint.h>

uint8_t BNO055_Init(void);
uint8_t BNO055_ReadHeading(void);
int32_t IMU_GetHeadingX100(void);
uint8_t IMU_IsValid(void);
int32_t ApplyHeadingDeadband_X100(int32_t heading_x100);

#endif /* INC_BNO055_H_ */
