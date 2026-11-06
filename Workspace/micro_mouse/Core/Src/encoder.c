#include "encoder.h"
#include <math.h>

void Encoder_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN  | RCC_APB1ENR_TIM3EN;

    // ENCODER TRÁI (TIM2): PA15 (AF1) & PB3 (AF1)
    GPIOA->MODER &= ~(3U << (15 * 2));
    GPIOA->MODER |=  (2U << (15 * 2));
    GPIOA->AFR[1] &= ~(0xFU << ((15 - 8) * 4));
    GPIOA->AFR[1] |=  (1U   << ((15 - 8) * 4));

    GPIOB->MODER &= ~(3U << (3 * 2));
    GPIOB->MODER |=  (2U << (3 * 2));
    GPIOB->AFR[0] &= ~(0xFU << (3 * 4));
    GPIOB->AFR[0] |=  (1U   << (3 * 4));

    TIM2->SMCR  &= ~TIM_SMCR_SMS;
    TIM2->SMCR  |= (3U << TIM_SMCR_SMS_Pos);
    TIM2->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);
    TIM2->CCMR1 |= (1U << TIM_CCMR1_CC1S_Pos) | (1U << TIM_CCMR1_CC2S_Pos);
    TIM2->ARR    = 0xFFFFFFFF;
    TIM2->CR1   |= TIM_CR1_CEN;

    // ENCODER PHẢI (TIM3): PB4 (AF2) & PB5 (AF2)
    GPIOB->MODER &= ~((3U << (4 * 2)) | (3U << (5 * 2)));
    GPIOB->MODER |=  ((2U << (4 * 2)) | (2U << (5 * 2)));
    GPIOB->AFR[0] &= ~((0xFU << (4 * 4)) | (0xFU << (5 * 4)));
    GPIOB->AFR[0] |=  ((2U   << (4 * 4)) | (2U   << (5 * 4)));

    TIM3->SMCR  &= ~TIM_SMCR_SMS;
    TIM3->SMCR  |= (3U << TIM_SMCR_SMS_Pos);
    TIM3->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);
    TIM3->CCMR1 |= (1U << TIM_CCMR1_CC1S_Pos) | (1U << TIM_CCMR1_CC2S_Pos);
    TIM3->ARR    = 0xFFFF;
    TIM3->CR1   |= TIM_CR1_CEN;

    Encoder_Reset();
}

void Encoder_Reset(void) {
    TIM2->CNT = 0;
    TIM3->CNT = 0;
}

int32_t Encoder_Get_Left_Count(void) {
    return (int32_t)TIM2->CNT;
}

int32_t Encoder_Get_Right_Count(void) {
    return (int32_t)((int16_t)TIM3->CNT);
}

float Encoder_Get_Left_Distance_MM(void) {
    return (float)Encoder_Get_Left_Count() / PULSES_PER_MM;
}

float Encoder_Get_Right_Distance_MM(void) {
    return (float)Encoder_Get_Right_Count() / PULSES_PER_MM;
}

float Encoder_Get_Distance_MM(void) {
    return (Encoder_Get_Left_Distance_MM() + Encoder_Get_Right_Distance_MM()) * 0.5f;
}

/**
 * @brief Tính góc xoay (Radian) từ vi sai 2 bánh Encoder, ép trong khoảng [-pi, pi]
 *        Công thức: theta = (S_right - S_left) / Wheel_Track
 */
float Encoder_Get_Turn_Angle_Rad(void)
{
    float dist_l = (float)Encoder_Get_Left_Count()  / PULSES_PER_MM;
    float dist_r = (float)Encoder_Get_Right_Count() / PULSES_PER_MM;

    float angle_rad = (dist_r - dist_l) / WHEEL_TRACK_MM;

    while (angle_rad > PI_FLOAT)  angle_rad -= TWO_PI_FLOAT;
    while (angle_rad < -PI_FLOAT) angle_rad += TWO_PI_FLOAT;

    return angle_rad;
}
