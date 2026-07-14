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
uint8_t I2C2_Start(void);
void I2C2_Stop(void);
uint8_t I2C2_Address(uint8_t address);
uint8_t I2C2_Write(uint8_t data);
uint8_t I2C2_Read_Ack(uint8_t *data);
uint8_t I2C2_Read_Nack(uint8_t *data);


#endif /* INC_I2C2_H_ */
