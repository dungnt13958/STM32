#include "mpu6500.h"
#include <math.h>

volatile float live_gz_dps = 0.0f;
volatile float raw_gz_bias = 0.0f;

#define MPU_CS_LOW()   (GPIOB->BSRR = (1U << (12 + 16)))
#define MPU_CS_HIGH()  (GPIOB->BSRR = (1U << 12))

static void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 2666; i++) {
        __NOP();
    }
}

static uint8_t SPI2_Transfer(uint8_t byte) {
    while (!(SPI2->SR & SPI_SR_TXE));
    SPI2->DR = byte;
    while (!(SPI2->SR & SPI_SR_RXNE));
    return (uint8_t)SPI2->DR;
}

static void MPU_Write(uint8_t reg, uint8_t val) {
    MPU_CS_LOW();
    SPI2_Transfer(reg & 0x7F);
    SPI2_Transfer(val);
    MPU_CS_HIGH();
}

static void MPU_Read(uint8_t reg, uint8_t *buf, uint8_t len) {
    MPU_CS_LOW();
    SPI2_Transfer(reg | 0x80);
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = SPI2_Transfer(0xFF);
    }
    MPU_CS_HIGH();
}

void MPU_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    // CS: PB12 -> Output
    GPIOB->MODER &= ~(3U << (12 * 2));
    GPIOB->MODER |=  (1U << (12 * 2));
    MPU_CS_HIGH();

    // SPI2 Pin: PB13 (SCK), PB14 (MISO), PB15 (MOSI) -> AF5
    for (uint32_t p = 13; p <= 15; p++) {
        GPIOB->MODER &= ~(3U << (p * 2));
        GPIOB->MODER |=  (2U << (p * 2));
        GPIOB->AFR[1] |= (5U << ((p - 8) * 4));
    }

    SPI2->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI |
                SPI_CR1_BR_1 | SPI_CR1_BR_0 | SPI_CR1_CPOL | SPI_CR1_CPHA;
    SPI2->CR1 |= SPI_CR1_SPE;

    delay_ms(10);
    MPU_Write(0x6B, 0x80); // Reset chip
    delay_ms(100);
    MPU_Write(0x6B, 0x01); // Clock PLL
    MPU_Write(0x1A, 0x03); // DLPF ~42Hz
    MPU_Write(0x1B, 0x18); // +-2000 dps
}

void MPU_Calibrate_Zero(void) {
    uint8_t buf[2];
    int64_t sum = 0;
    const uint16_t samples = 1000;

    for (uint16_t i = 0; i < samples; i++) {
        MPU_Read(0x47, buf, 2);
        sum += (int16_t)((buf[0] << 8) | buf[1]);
        delay_ms(2);
    }
    raw_gz_bias = ((float)sum / (float)samples) / GYRO_SCALE_DIVISOR;
}

float MPU_Get_Calibrated_GZ(void) {
    uint8_t buf[2];
    MPU_Read(0x47, buf, 2);
    int16_t raw_gz = (int16_t)((buf[0] << 8) | buf[1]);

    float rate = ((float)raw_gz / GYRO_SCALE_DIVISOR) - raw_gz_bias;
    if (fabsf(rate) < 0.10f) rate = 0.0f;
    return rate;
}
