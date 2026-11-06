/*
 * MPU6050.c
 *
 *  Created on: Oct 31, 2026
 *      Author: Tuan Dung Nguyen
 */
#include "stm32f401xe.h"
#include "MPU6050.h"

void i2c3_init(void)
{
	RCC->AHB1ENR |= 1u << 0u;
	RCC->AHB1ENR |= 1u << 2u;
	RCC->APB1ENR |= 1u << 23u; // I2C3 Clock EN
	GPIOA->MODER &= ~(3u << (2u * 8u));
	GPIOA->MODER |= 2u << (2u * 8u);
	GPIOC->MODER &= ~(3u << (2u * 9u));
	GPIOC->MODER |= 2u << (2u * 9u);
	// I2C3 uses AF4 on PA8 (SCL) and PC9 (SDA).
	GPIOA->AFR[1] &= ~(0xFu << 0u);
	GPIOA->AFR[1] |=  (4u << 0u);
	GPIOC->AFR[1] &= ~(0xFu << 4u);
	GPIOC->AFR[1] |=  (4u << 4u);

	GPIOA->OTYPER |= 1u << 8u;
	GPIOC->OTYPER |= 1u << 9u;

	// Cấu hình tốc độ GPIO
	GPIOA->OSPEEDR &= ~(3u << (2u * 8u));
	GPIOA->OSPEEDR |=  3u << (2u * 8u);

	GPIOC->OSPEEDR &= ~(3u << (2u * 9u));
	GPIOC->OSPEEDR |=  3u << (2u * 9u);

	// Cấu hình I2C
	I2C3->CR1 &= ~I2C_CR1_PE;

	// main() leaves the MCU on its 16 MHz HSI clock (APB1 = 16 MHz).
	I2C3->CR2 = 16u;
	I2C3->CCR = 80u;  // 100 kHz standard mode: 16 MHz / (2 * 80)
	I2C3->TRISE = 17u; // 1000 ns rise time at 16 MHz

	I2C3->CR1 |= I2C_CR1_PE;
}

void I2C3_write(uint8_t sensor_addr, uint8_t sensor_reg, uint8_t data)
{
	while(I2C3->SR2 & I2C_SR2_BUSY);
	I2C3->CR1 |= 1u << 8u;
	while (!(I2C3->SR1 & (1u << 0)));

	I2C3->DR = (sensor_addr << 1u);
	while (!(I2C3->SR1 & I2C_SR1_ADDR));
//	Delete ADDR flag by reading SR2 after reading SR1
	(void)I2C3->SR1;
	(void)I2C3->SR2;

//	Send the sensor register address
	while (!(I2C3->SR1 & I2C_SR1_TXE)); // Wait for Tx buffer to be empty
	I2C3->DR = sensor_reg;

	while (!(I2C3->SR1 & I2C_SR1_TXE));
	I2C3->DR = data;

	while (!(I2C3->SR1 & I2C_SR1_BTF));
	I2C3->CR1 |= I2C_CR1_STOP;
}

void I2C3_read(uint8_t sensor_addr, uint8_t sensor_reg,
               uint8_t *buffer, uint8_t length)
{
    uint8_t index = 0U;
    uint8_t remaining = length;

    if ((buffer == 0) || (length == 0U)) {
        return;
    }

    while (I2C3->SR2 & I2C_SR2_BUSY);

    I2C3->CR1 &= ~I2C_CR1_POS;
    I2C3->CR1 |= I2C_CR1_ACK;

    // Phase 1: START + Slave Address (Write)
    I2C3->CR1 |= I2C_CR1_START;
    while (!(I2C3->SR1 & I2C_SR1_SB));

    I2C3->DR = (uint8_t)(sensor_addr << 1U);
    while (!(I2C3->SR1 & I2C_SR1_ADDR));

    (void)I2C3->SR1;
    (void)I2C3->SR2;

    // Send register address
    while (!(I2C3->SR1 & I2C_SR1_TXE));
    I2C3->DR = sensor_reg;

    while (!(I2C3->SR1 & I2C_SR1_BTF));

    // Phase 2: Repeated START + Slave Address (Read)
    I2C3->CR1 |= I2C_CR1_START;
    while (!(I2C3->SR1 & I2C_SR1_SB));

    I2C3->DR = (uint8_t)((sensor_addr << 1U) | 0x01U);
    while (!(I2C3->SR1 & I2C_SR1_ADDR));

    // Case 1: Read 1 byte
    if (length == 1U) {

        I2C3->CR1 &= ~I2C_CR1_ACK;

        (void)I2C3->SR1;
        (void)I2C3->SR2;

        I2C3->CR1 |= I2C_CR1_STOP;

        while (!(I2C3->SR1 & I2C_SR1_RXNE));

        buffer[0] = (uint8_t)I2C3->DR;
    }

    // Case 2: Read 2 bytes
    else if (length == 2U) {

        I2C3->CR1 |= I2C_CR1_POS;
        I2C3->CR1 &= ~I2C_CR1_ACK;

        (void)I2C3->SR1;
        (void)I2C3->SR2;

        while (!(I2C3->SR1 & I2C_SR1_BTF));

        I2C3->CR1 |= I2C_CR1_STOP;

        buffer[0] = (uint8_t)I2C3->DR;
        buffer[1] = (uint8_t)I2C3->DR;
    }

    // Case 3: Read more than 2 bytes
    else {

        I2C3->CR1 |= I2C_CR1_ACK;

        (void)I2C3->SR1;
        (void)I2C3->SR2;

        // Read until only 3 bytes remain
        while (remaining > 3U) {

            while (!(I2C3->SR1 & I2C_SR1_RXNE));

            buffer[index++] = (uint8_t)I2C3->DR;
            remaining--;
        }

        // Read byte thứ N-2
        while (!(I2C3->SR1 & I2C_SR1_BTF));

        I2C3->CR1 &= ~I2C_CR1_ACK;

        buffer[index++] = (uint8_t)I2C3->DR;
        remaining--;

        // Read 2 bytes cuối
        while (!(I2C3->SR1 & I2C_SR1_BTF));

        I2C3->CR1 |= I2C_CR1_STOP;

        buffer[index++] = (uint8_t)I2C3->DR;
        remaining--;

        buffer[index] = (uint8_t)I2C3->DR;
    }

    // Restore ACK and POS
    I2C3->CR1 &= ~I2C_CR1_POS;
    I2C3->CR1 |= I2C_CR1_ACK;
}

uint8_t MPU6050_init(void)
{
	uint8_t check = 0;
	I2C3_read(MPU_ADDR, WHO_AM_I_reg, &check, 1);
	// MPU6050 reports 0x68; MPU9250 reports 0x71 (some revisions 0x73).
	if(check == 0x68 || check == 0x71 || check == 0x73)
	{
		I2C3_write(MPU_ADDR, 0x6B, 0x00);
		I2C3_write(MPU_ADDR, 0x19, 0x07);
		I2C3_write(MPU_ADDR, 0x1B, 0x00);
		I2C3_write(MPU_ADDR, 0x1C, 0x00);
		return 0;
	}
	return 1;
}

void MPU6050_ReadAll(mpu6050_data_t *Data)
{
    uint8_t rawData[14];

    I2C3_read(MPU_ADDR, 0x3B, rawData, 14);

    int16_t raw_ax   = (int16_t)((rawData[0]  << 8) | rawData[1]);
    int16_t raw_ay   = (int16_t)((rawData[2]  << 8) | rawData[3]);
    int16_t raw_az   = (int16_t)((rawData[4]  << 8) | rawData[5]);

    int16_t raw_temp = (int16_t)((rawData[6]  << 8) | rawData[7]);

    int16_t raw_gx   = (int16_t)((rawData[8]  << 8) | rawData[9]);
    int16_t raw_gy   = (int16_t)((rawData[10] << 8) | rawData[11]);
    int16_t raw_gz   = (int16_t)((rawData[12] << 8) | rawData[13]);

    Data->aX = raw_ax / 16384.0f;
    Data->aY = raw_ay / 16384.0f;
    Data->aZ = raw_az / 16384.0f;

    Data->temp = (raw_temp / 340.0f) + 36.53f;

    Data->gX = raw_gx / 131.0f;
    Data->gY = raw_gy / 131.0f;
    Data->gZ = raw_gz / 131.0f;
}

void Heading_Init(heading_estimator_t *estimator, float gyro_z_bias_dps)
{
	if (estimator == 0) {
		return;
	}
	estimator->heading_deg = 0.0f;
	estimator->gyro_z_bias_dps = gyro_z_bias_dps;
	estimator->last_cycle = 0U;
	estimator->timer_started = 0U;
}

void Heading_Reset(heading_estimator_t *estimator)
{
	if (estimator != 0) {
		estimator->heading_deg = 0.0f;
		estimator->timer_started = 0U;
	}
}

void Heading_TimerInit(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0U;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void Heading_UpdateNow(heading_estimator_t *estimator,
				      const mpu6050_data_t *imu)
{
	uint32_t now;
	uint32_t cycles;
	float dt_s;

	if ((estimator == 0) || (imu == 0)) {
		return;
	}

	now = DWT->CYCCNT;
	if (estimator->timer_started == 0U) {
		estimator->last_cycle = now;
		estimator->timer_started = 1U;
		return;
	}

	/* Unsigned subtraction handles the 32-bit counter wrap-around. */
	cycles = now - estimator->last_cycle;
	estimator->last_cycle = now;
	dt_s = (float)cycles / (float)SystemCoreClock;
	Heading_Update(estimator, imu, dt_s);
}

void Heading_Update(heading_estimator_t *estimator,
				   const mpu6050_data_t *imu, float dt_s)
{
	if ((estimator == 0) || (imu == 0) || (dt_s <= 0.0f)) {
		return;
	}

	/* gZ is converted to degrees/second in MPU6050_ReadAll(). */
	estimator->heading_deg += (imu->gZ - estimator->gyro_z_bias_dps) * dt_s;

	/* Keep the value bounded while preserving the direction of rotation. */
	while (estimator->heading_deg >= 360.0f) {
		estimator->heading_deg -= 360.0f;
	}
	while (estimator->heading_deg < 0.0f) {
		estimator->heading_deg += 360.0f;
	}
}
