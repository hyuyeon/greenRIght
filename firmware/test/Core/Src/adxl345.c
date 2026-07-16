/*
 * adxl345.c
 *
 *  Created on: 2026. 7. 6.
 *      Author: 한국전파진흥협회
 */

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

#define ADXL345_EXPECTED_DEVID  229U

static uint8_t ADXL345_WriteReg(uint8_t reg, uint8_t value);
static uint8_t ADXL345_ReadReg(uint8_t reg, uint8_t *value);

static uint8_t ADXL345_WriteReg(uint8_t reg, uint8_t value)
{
    if (!I2C2_Start()) return 0U;
    if (!I2C2_Address(ADXL345_ADDR_W)) return 0U;
    if (!I2C2_Write(reg)) return 0U;
    if (!I2C2_Write(value)) return 0U;

    uint32_t start = HAL_GetTick();
    while (!(I2C2->SR1 & I2C_SR1_BTF))
    {
        if ((uint32_t)(HAL_GetTick() - start) >= 10U)
        {
            I2C2_Stop();
            return 0U;
        }
    }

    I2C2_Stop();
    return 1U;
}

static uint8_t ADXL345_ReadReg(uint8_t reg, uint8_t *value)
{
    if (!I2C2_Start()) return 0U;
    if (!I2C2_Address(ADXL345_ADDR_W)) return 0U;
    if (!I2C2_Write(reg)) return 0U;
    if (!I2C2_Start()) return 0U;
    if (!I2C2_Address(ADXL345_ADDR_R)) return 0U;
    if (!I2C2_Read_Nack(value)) return 0U;

    I2C2_Stop();
    return 1U;
}

uint8_t ADXL345_Init(void)
{
    uint8_t dev_id = 0U;

    if (!ADXL345_ReadReg(ADXL345_DEVID, &dev_id))
    {
        return 0U;
    }

    if (dev_id != ADXL345_EXPECTED_DEVID)
    {
        return 0U;
    }

    if (!ADXL345_WriteReg(ADXL345_POWER_CTL, 0x08)) return 0U;
    if (!ADXL345_WriteReg(ADXL345_DATA_FORMAT, 0x0B)) return 0U;

    return 1U;
}

uint8_t ADXL345_ReadXYZ(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buffer[6];

    if (!I2C2_Start()) return 0U;
    if (!I2C2_Address(ADXL345_ADDR_W)) return 0U;
    if (!I2C2_Write(ADXL345_DATAX0)) return 0U;

    if (!I2C2_Start()) return 0U;
    if (!I2C2_Address(ADXL345_ADDR_R)) return 0U;

    if (!I2C2_Read_Ack(&buffer[0])) return 0U;
    if (!I2C2_Read_Ack(&buffer[1])) return 0U;
    if (!I2C2_Read_Ack(&buffer[2])) return 0U;
    if (!I2C2_Read_Ack(&buffer[3])) return 0U;
    if (!I2C2_Read_Ack(&buffer[4])) return 0U;
    if (!I2C2_Read_Nack(&buffer[5])) return 0U;

    I2C2_Stop();

    *ax = (int16_t)((buffer[1] << 8) | buffer[0]);
    *ay = (int16_t)((buffer[3] << 8) | buffer[2]);
    *az = (int16_t)((buffer[5] << 8) | buffer[4]);
    return 1U;
}

void RollPitch_Calc(int16_t ax, int16_t ay, int16_t az, float *roll, float *pitch)
{
    *roll  = atan2f((float)ay, (float)az) * (180.0f / M_PI);
    *pitch = atan2f(-(float)ax,
                    sqrtf((float)ay * ay + (float)az * az))
                    * (180.0f / M_PI);
}

