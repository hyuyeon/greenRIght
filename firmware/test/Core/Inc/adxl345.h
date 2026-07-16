/*
 * adxl345.h
 *
 *  Created on: 2026. 7. 6.
 *      Author: 한국전파진흥협회
 */

#ifndef INC_ADXL345_H_
#define INC_ADXL345_H_



#include "main.h"
#define ADXL345_DEVID         0x00
#define ADXL345_ADDR_W  (0x53 << 1)
#define ADXL345_ADDR_R  ((0x53 << 1) | 1)
#define ADXL345_DEVID   0x00
uint8_t ADXL345_Init(void);
uint8_t ADXL345_ReadXYZ(int16_t *ax, int16_t *ay, int16_t *az);
void RollPitch_Calc(int16_t ax, int16_t ay, int16_t az, float *roll, float *pitch);




#endif /* INC_ADXL345_H_ */
