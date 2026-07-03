#ifndef __ADXL345_H
#define __ADXL345_H

#include "main.h"
#define ADXL345_DEVID         0x00
#define ADXL345_ADDR_W  (0x53 << 1)
#define ADXL345_ADDR_R  ((0x53 << 1) | 1)
#define ADXL345_DEVID   0x00
void ADXL345_Init(void);
void ADXL345_ReadXYZ(int16_t *ax, int16_t *ay, int16_t *az);
void RollPitch_Calc(int16_t ax, int16_t ay, int16_t az, float *roll, float *pitch);

#endif
