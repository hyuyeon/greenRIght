/*
 * i2c2.h
 *
 *  Created on: 2026. 7. 6.
 *      Author: 한국전파진흥협회
 */

#ifndef INC_I2C2_H_
#define INC_I2C2_H_

#include "main.h"

void I2C2_Init(void);
void I2C2_Start(void);
void I2C2_Stop(void);
void I2C2_Address(uint8_t address);
void I2C2_Write(uint8_t data);
uint8_t I2C2_Read_Ack(void);
uint8_t I2C2_Read_Nack(void);


#endif /* INC_I2C2_H_ */
