/*
 * i2c1_bus.h
 *
 *  Created on: 2026. 7. 3.
 *      Author: 한국전파진흥협회
 */

#ifndef INC_I2C1_BUS_H_
#define INC_I2C1_BUS_H_

#include <stdint.h>

void GPIO_Init(void);
void I2C1_Init(void);
uint8_t I2C1_WriteReg(uint8_t dev_addr_7bit, uint8_t reg, uint8_t data);
uint8_t I2C1_ReadReg(uint8_t dev_addr_7bit, uint8_t reg, uint8_t *data);

#endif /* INC_I2C1_BUS_H_ */
