#ifndef __I2C2_H
#define __I2C2_H

#include "main.h"

void I2C2_Init(void);
void I2C2_Start(void);
void I2C2_Stop(void);
void I2C2_Address(uint8_t address);
void I2C2_Write(uint8_t data);
uint8_t I2C2_Read_Ack(void);
uint8_t I2C2_Read_Nack(void);

#endif