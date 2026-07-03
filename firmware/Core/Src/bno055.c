/*
 * bno055.c
 *
 *  Created on: 2026. 7. 3.
 *      Author: 한국전파진흥협회
 */


#include "main.h"
#include "bno055.h"
#include "i2c1_bus.h"

#define BNO055_ADDR_7BIT       0x28
#define BNO055_CHIP_ID_REG     0x00
#define BNO055_PAGE_ID_REG     0x07
#define BNO055_OPR_MODE_REG    0x3D
#define BNO055_PWR_MODE_REG    0x3E
#define BNO055_EULER_H_LSB     0x1A

#define BNO055_CHIP_ID         0xA0
#define OPERATION_MODE_CONFIG  0x00
#define OPERATION_MODE_NDOF    0x0C

static int32_t imu_heading_x100 = 0;
static uint8_t imu_valid = 0;

static uint8_t BNO055_Write8(uint8_t reg, uint8_t value)
{
    return I2C1_WriteReg(BNO055_ADDR_7BIT, reg, value);
}

static uint8_t BNO055_Read8(uint8_t reg, uint8_t *value)
{
    return I2C1_ReadReg(BNO055_ADDR_7BIT, reg, value);
}

uint8_t BNO055_ReadHeading(void)
{
    uint8_t lsb;
    uint8_t msb;
    int16_t heading_raw;

    if (!BNO055_Read8(BNO055_EULER_H_LSB, &lsb))
    {
        imu_valid = 0;
        return 0;
    }

    if (!BNO055_Read8(BNO055_EULER_H_LSB + 1, &msb))
    {
        imu_valid = 0;
        return 0;
    }

    heading_raw = (int16_t)((msb << 8) | lsb);

    imu_heading_x100 = ((int32_t)heading_raw * 100) / 16;

    imu_valid = 1;
    return 1;
}

int32_t IMU_GetHeadingX100(void)
{
    return imu_heading_x100;
}

uint8_t IMU_IsValid(void)
{
    return imu_valid;
}

int32_t ApplyHeadingDeadband_X100(int32_t heading_x100)
{
    while (heading_x100 >= 36000)
        heading_x100 -= 36000;

    while (heading_x100 < 0)
        heading_x100 += 36000;

    if (heading_x100 <= 500 || heading_x100 >= 35500)
        return 0;

    if (heading_x100 >= 8500 && heading_x100 <= 9500)
        return 9000;

    if (heading_x100 >= 17500 && heading_x100 <= 18500)
        return 18000;

    if (heading_x100 >= 26500 && heading_x100 <= 27500)
        return 27000;

    return heading_x100;
}

uint8_t BNO055_Init(void)
{
    uint8_t chip_id = 0;

    HAL_Delay(700);

    if (!BNO055_Read8(BNO055_CHIP_ID_REG, &chip_id))
        return 0;

    if (chip_id != BNO055_CHIP_ID)
        return 0;

    if (!BNO055_Write8(BNO055_PAGE_ID_REG, 0x00))
        return 0;
    HAL_Delay(10);

    if (!BNO055_Write8(BNO055_OPR_MODE_REG, OPERATION_MODE_CONFIG))
        return 0;
    HAL_Delay(30);

    if (!BNO055_Write8(BNO055_PWR_MODE_REG, 0x00))
        return 0;
    HAL_Delay(20);

    if (!BNO055_Write8(BNO055_OPR_MODE_REG, OPERATION_MODE_NDOF))
        return 0;
    HAL_Delay(600);

    return 1;
}
