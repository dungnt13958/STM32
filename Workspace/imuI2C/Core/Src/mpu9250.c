#include "mpu9250.h"
#include <math.h>

#define MPU_PI               3.14159265f
#define MPU_TWO_PI           6.28318531f
#define DEG_TO_RAD           0.0174532925f // pi / 180

/* Thanh ghi MPU6500 / MPU9265 */
#define MPU_REG_SMPLRT_DIV   0x19
#define MPU_REG_CONFIG       0x1A
#define MPU_REG_GYRO_CONFIG  0x1B
#define MPU_REG_GYRO_ZOUT_H  0x47
#define MPU_REG_PWR_MGMT_1   0x6B

/* Chân CS: PB12 */
#define MPU_CS_LOW()         (GPIOB->BSRR = (1U << (12 + 16)))
#define MPU_CS_HIGH()        (GPIOB->BSRR = (1U << 12))

/* Ngưỡng cắt nhiễu tĩnh: 0.3 độ/s ~ 0.005236 rad/s */
#define GYRO_DEADBAND_RAD_S  0.005236f
#define GYRO_SENSITIVITY_LSB 16.4f

volatile float live_gz_rad_s = 0.0f;
volatile float mpu_yaw_rad   = 0.0f;
volatile float mpu_turn_rad  = 0.0f;

static float gyro_z_bias     = 0.0f;
static float prev_gz_rad_s   = 0.0f;

/**
 * @brief Chuẩn hóa góc bất kỳ về đúng đoạn [-pi, pi]
 */
float MPU_Wrap_To_Pi(float angle_rad) {
    while (angle_rad > MPU_PI)  angle_rad -= MPU_TWO_PI;
    while (angle_rad < -MPU_PI) angle_rad += MPU_TWO_PI;
    return angle_rad;
}

static void mpu_delay_loop(volatile uint32_t count) {
    while (count--) { __NOP(); }
}

static uint8_t SPI2_Transfer(uint8_t data) {
    while (!(SPI2->SR & SPI_SR_TXE));
    *((__IO uint8_t *)&SPI2->DR) = data;
    while (!(SPI2->SR & SPI_SR_RXNE));
    return *((__IO uint8_t *)&SPI2->DR);
}

static void MPU_Write_Reg(uint8_t reg, uint8_t val) {
    MPU_CS_LOW();
    SPI2_Transfer(reg & 0x7F);
    SPI2_Transfer(val);
    MPU_CS_HIGH();
    mpu_delay_loop(1000);
}

void MPU_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    // PB12 (CS) -> Output Push-Pull
    GPIOB->MODER &= ~(3U << (12 * 2));
    GPIOB->MODER |=  (1U << (12 * 2));
    GPIOB->OSPEEDR |= (3U << (12 * 2));
    MPU_CS_HIGH();

    // PB13 (SCK), PB14 (MISO), PB15 (MOSI) -> AF5
    GPIOB->MODER &= ~((3U << (13 * 2)) | (3U << (14 * 2)) | (3U << (15 * 2)));
    GPIOB->MODER |=  ((2U << (13 * 2)) | (2U << (14 * 2)) | (2U << (15 * 2)));
    GPIOB->AFR[1] &= ~((0xFU << ((13 - 8) * 4)) | (0xFU << ((14 - 8) * 4)) | (0xFU << ((15 - 8) * 4)));
    GPIOB->AFR[1] |=  ((5U   << ((13 - 8) * 4)) | (5U   << ((14 - 8) * 4)) | (5U   << ((15 - 8) * 4)));

    // SPI2 Master, Mode 3, fPCLK/16
    SPI2->CR1 = SPI_CR1_MSTR | SPI_CR1_CPOL | SPI_CR1_CPHA |
                SPI_CR1_SSM  | SPI_CR1_SSI  | (3U << SPI_CR1_BR_Pos);
    SPI2->CR1 |= SPI_CR1_SPE;

    mpu_delay_loop(50000);

    MPU_Write_Reg(MPU_REG_PWR_MGMT_1, 0x80);
    mpu_delay_loop(100000);
    MPU_Write_Reg(MPU_REG_PWR_MGMT_1, 0x01);
    MPU_Write_Reg(MPU_REG_CONFIG, 0x03);      // DLPF = 41Hz
    MPU_Write_Reg(MPU_REG_SMPLRT_DIV, 0x04);  // 200Hz
    MPU_Write_Reg(MPU_REG_GYRO_CONFIG, 0x18); // +-2000 dps
}

static int16_t MPU_Read_Raw_GZ(void) {
    uint8_t h, l;
    MPU_CS_LOW();
    SPI2_Transfer(MPU_REG_GYRO_ZOUT_H | 0x80);
    h = SPI2_Transfer(0xFF);
    l = SPI2_Transfer(0xFF);
    MPU_CS_HIGH();
    return (int16_t)((h << 8) | l);
}

void MPU_Calibrate_Zero(void) {
    float sum = 0.0f;
    const int samples = 1000;

    for (int i = 0; i < samples; i++) {
        sum += (float)MPU_Read_Raw_GZ();
        mpu_delay_loop(3200);
    }
    gyro_z_bias   = sum / (float)samples;
    mpu_yaw_rad   = 0.0f;
    mpu_turn_rad  = 0.0f;
    prev_gz_rad_s = 0.0f;
}

/**
 * @brief Đọc vận tốc góc trục Z theo đơn vị Radian/giây (rad/s)
 */
float MPU_Get_Calibrated_GZ_RadS(void) {
    float raw = (float)MPU_Read_Raw_GZ();
    float dps = (raw - gyro_z_bias) / GYRO_SENSITIVITY_LSB;
    float rad_s = dps * DEG_TO_RAD;

    if (fabsf(rad_s) < GYRO_DEADBAND_RAD_S) {
        rad_s = 0.0f;
    }
    return rad_s;
}

/**
 * @brief Tích phân hình thang và giới hạn góc luôn nằm trong [-pi, pi]
 */
void MPU_Update_Angle(float dt_sec) {
    live_gz_rad_s = MPU_Get_Calibrated_GZ_RadS();

    // Tích phân hình thang (đổi dấu trừ theo chiều gắn cảm biến)
    float delta_rad = -((prev_gz_rad_s + live_gz_rad_s) * 0.5f) * dt_sec;
    prev_gz_rad_s = live_gz_rad_s;

    // Luôn ép góc trong phạm vi [-pi, pi]
    mpu_yaw_rad  = MPU_Wrap_To_Pi(mpu_yaw_rad + delta_rad);
    mpu_turn_rad = MPU_Wrap_To_Pi(mpu_turn_rad + delta_rad);
}

void MPU_Reset_Turn_Angle(void) {
    mpu_turn_rad = 0.0f;
}

float MPU_Get_Turn_Angle_Rad(void) {
    return mpu_turn_rad;
}
