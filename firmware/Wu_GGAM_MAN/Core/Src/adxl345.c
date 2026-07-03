#include "adxl345.h"
#include "i2c2.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ADXL345 I2C 주소
#define ADXL345_ADDR_W  (0x53 << 1)
#define ADXL345_ADDR_R  ((0x53 << 1) | 1)

// 레지스터
#define ADXL345_POWER_CTL     0x2D
#define ADXL345_DATA_FORMAT   0x31
#define ADXL345_DATAX0        0x32

static void ADXL345_WriteReg(uint8_t reg, uint8_t value);

static void ADXL345_WriteReg(uint8_t reg, uint8_t value)
{
    I2C2_Start();
    I2C2_Address(ADXL345_ADDR_W);
    I2C2_Write(reg);
    I2C2_Write(value);

    while (!(I2C2->SR1 & I2C_SR1_BTF));

    I2C2_Stop();
}

void ADXL345_Init(void)
{
    ADXL345_WriteReg(ADXL345_POWER_CTL, 0x08);
    ADXL345_WriteReg(ADXL345_DATA_FORMAT, 0x0B);
}

void ADXL345_ReadXYZ(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buffer[6];

    I2C2_Start();
    I2C2_Address(ADXL345_ADDR_W);
    I2C2_Write(ADXL345_DATAX0);

    I2C2_Start();
    I2C2_Address(ADXL345_ADDR_R);

    buffer[0] = I2C2_Read_Ack();
    buffer[1] = I2C2_Read_Ack();
    buffer[2] = I2C2_Read_Ack();
    buffer[3] = I2C2_Read_Ack();
    buffer[4] = I2C2_Read_Ack();
    buffer[5] = I2C2_Read_Nack();

    I2C2_Stop();

    *ax = (int16_t)((buffer[1] << 8) | buffer[0]);
    *ay = (int16_t)((buffer[3] << 8) | buffer[2]);
    *az = (int16_t)((buffer[5] << 8) | buffer[4]);
}

void RollPitch_Calc(int16_t ax, int16_t ay, int16_t az, float *roll, float *pitch)
{
    *roll  = atan2f((float)ay, (float)az) * (180.0f / M_PI);
    *pitch = atan2f(-(float)ax,
                    sqrtf((float)ay * ay + (float)az * az))
                    * (180.0f / M_PI);
}
