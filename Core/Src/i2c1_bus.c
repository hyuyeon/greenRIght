/*
 * i2c1_bus.c
 *
 *  Created on: 2026. 7. 3.
 *      Author: 한국전파진흥협회
 */

#include "main.h"
#include "i2c1_bus.h"

#define I2C_TIMEOUT_MS    100

void GPIO_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    GPIOB->MODER &= ~((3U << (8 * 2)) | (3U << (9 * 2)));
    GPIOB->MODER |=  ((2U << (8 * 2)) | (2U << (9 * 2)));

    GPIOB->OTYPER |= (1U << 8) | (1U << 9);
    GPIOB->PUPDR &= ~((3U << (8 * 2)) | (3U << (9 * 2)));
    GPIOB->PUPDR |=  ((1U << (8 * 2)) | (1U << (9 * 2)));

    GPIOB->AFR[1] &= ~((0xFU << ((8 - 8) * 4)) | (0xFU << ((9 - 8) * 4)));
    GPIOB->AFR[1] |=  ((4U   << ((8 - 8) * 4)) | (4U   << ((9 - 8) * 4)));
}

void I2C1_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;

    I2C1->CR1 &= ~I2C_CR1_PE;

    I2C1->CR2 = 42;
    I2C1->CCR = 210;
    I2C1->TRISE = 43;

    I2C1->CR1 |= I2C_CR1_ACK;
    I2C1->CR1 |= I2C_CR1_PE;
}

static uint8_t I2C1_WaitSet(uint32_t flag, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while ((I2C1->SR1 & flag) == 0)
    {
        if ((HAL_GetTick() - start) > timeout_ms)
            return 0;
    }

    return 1;
}

static uint8_t I2C1_WaitClearSR2(uint32_t flag, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while ((I2C1->SR2 & flag) != 0)
    {
        if ((HAL_GetTick() - start) > timeout_ms)
            return 0;
    }

    return 1;
}

static void I2C1_ClearAddr(void)
{
    volatile uint32_t temp;

    temp = I2C1->SR1;
    temp = I2C1->SR2;
    (void)temp;
}

uint8_t I2C1_WriteReg(uint8_t dev_addr_7bit, uint8_t reg, uint8_t data)
{
    if (!I2C1_WaitClearSR2(I2C_SR2_BUSY, I2C_TIMEOUT_MS))
        return 0;

    I2C1->CR1 |= I2C_CR1_START;
    if (!I2C1_WaitSet(I2C_SR1_SB, I2C_TIMEOUT_MS))
        return 0;

    I2C1->DR = dev_addr_7bit << 1;
    if (!I2C1_WaitSet(I2C_SR1_ADDR, I2C_TIMEOUT_MS))
        return 0;
    I2C1_ClearAddr();

    if (!I2C1_WaitSet(I2C_SR1_TXE, I2C_TIMEOUT_MS))
        return 0;
    I2C1->DR = reg;

    if (!I2C1_WaitSet(I2C_SR1_TXE, I2C_TIMEOUT_MS))
        return 0;
    I2C1->DR = data;

    if (!I2C1_WaitSet(I2C_SR1_BTF, I2C_TIMEOUT_MS))
        return 0;

    I2C1->CR1 |= I2C_CR1_STOP;

    return 1;
}

uint8_t I2C1_ReadReg(uint8_t dev_addr_7bit, uint8_t reg, uint8_t *data)
{
    if (!I2C1_WaitClearSR2(I2C_SR2_BUSY, I2C_TIMEOUT_MS))
        return 0;

    I2C1->CR1 |= I2C_CR1_START;
    if (!I2C1_WaitSet(I2C_SR1_SB, I2C_TIMEOUT_MS))
        return 0;

    I2C1->DR = dev_addr_7bit << 1;
    if (!I2C1_WaitSet(I2C_SR1_ADDR, I2C_TIMEOUT_MS))
        return 0;
    I2C1_ClearAddr();

    if (!I2C1_WaitSet(I2C_SR1_TXE, I2C_TIMEOUT_MS))
        return 0;
    I2C1->DR = reg;

    if (!I2C1_WaitSet(I2C_SR1_TXE, I2C_TIMEOUT_MS))
        return 0;

    I2C1->CR1 |= I2C_CR1_START;
    if (!I2C1_WaitSet(I2C_SR1_SB, I2C_TIMEOUT_MS))
        return 0;

    I2C1->DR = (dev_addr_7bit << 1) | 1;
    if (!I2C1_WaitSet(I2C_SR1_ADDR, I2C_TIMEOUT_MS))
        return 0;

    I2C1->CR1 &= ~I2C_CR1_ACK;
    I2C1_ClearAddr();

    I2C1->CR1 |= I2C_CR1_STOP;

    if (!I2C1_WaitSet(I2C_SR1_RXNE, I2C_TIMEOUT_MS))
        return 0;

    *data = I2C1->DR;

    I2C1->CR1 |= I2C_CR1_ACK;

    return 1;
}
