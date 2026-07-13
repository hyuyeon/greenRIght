/*
 * i2c2.c
 *
 *  Created on: 2026. 7. 6.
 *      Author: 한국전파진흥협회
 */


#include "i2c2.h"

void I2C2_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

    GPIOF->MODER &= ~((3 << (0 * 2)) | (3 << (1 * 2)));
    GPIOF->MODER |=  ((2 << (0 * 2)) | (2 << (1 * 2)));

    GPIOF->OTYPER |= (1 << 0) | (1 << 1);
    GPIOF->OSPEEDR |= (3 << (0 * 2)) | (3 << (1 * 2));
    GPIOF->PUPDR &= ~((3 << (0 * 2)) | (3 << (1 * 2)));
    GPIOF->PUPDR |=  ((1 << (0 * 2)) | (1 << (1 * 2)));

    GPIOF->AFR[0] &= ~((0xF << (0 * 4)) | (0xF << (1 * 4)));
    GPIOF->AFR[0] |=  ((4 << (0 * 4)) | (4 << (1 * 4)));

    I2C2->CR1 &= ~I2C_CR1_PE;
    I2C2->CR2 = 42;
    I2C2->CCR = 210;
    I2C2->TRISE = 43;
    I2C2->CR1 |= I2C_CR1_PE;
}

void I2C2_Start(void)
{
    I2C2->CR1 |= I2C_CR1_START;
    while (!(I2C2->SR1 & I2C_SR1_SB));
    (void)I2C2->SR1;
}

void I2C2_Stop(void)
{
    I2C2->CR1 |= I2C_CR1_STOP;
}

void I2C2_Address(uint8_t address)
{
    I2C2->DR = address;

    while (!(I2C2->SR1 & I2C_SR1_ADDR));

    (void)I2C2->SR1;
    (void)I2C2->SR2;
}

void I2C2_Write(uint8_t data)
{
    while (!(I2C2->SR1 & I2C_SR1_TXE));

    I2C2->DR = data;
}

uint8_t I2C2_Read_Ack(void)
{
    I2C2->CR1 |= I2C_CR1_ACK;

    while (!(I2C2->SR1 & I2C_SR1_RXNE));

    return (uint8_t)I2C2->DR;
}

uint8_t I2C2_Read_Nack(void)
{
    I2C2->CR1 &= ~I2C_CR1_ACK;

    while (!(I2C2->SR1 & I2C_SR1_RXNE));

    return (uint8_t)I2C2->DR;
}
