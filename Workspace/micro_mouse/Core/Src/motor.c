#include "motor.h"

/* ================= MACRO ĐIỀU KHIỂN CẦU H ================= */
// Bánh Trái: PA12 và PA1
#define MOTOR_L_FWD()    (GPIOA->BSRR = (1U << 12) | (1U << (1 + 16))) // PA12 = 1, PA1 = 0
#define MOTOR_L_REV()    (GPIOA->BSRR = (1U << (12 + 16)) | (1U << 1)) // PA12 = 0, PA1 = 1
#define MOTOR_L_BRAKE()  (GPIOA->BSRR = (1U << 12) | (1U << 1))        // PA12 = 1, PA1 = 1 (Short Brake)
#define MOTOR_L_COAST()  (GPIOA->BSRR = (1U << (12 + 16)) | (1U << (1 + 16))) // PA12 = 0, PA1 = 0 (Thả trôi)

// Bánh Phải: PB9 và PB10
#define MOTOR_R_FWD()    (GPIOB->BSRR = (1U << 9)  | (1U << (10 + 16))) // PB9 = 1, PB10 = 0
#define MOTOR_R_REV()    (GPIOB->BSRR = (1U << (9 + 16)) | (1U << 10))  // PB9 = 0, PB10 = 1
#define MOTOR_R_BRAKE()  (GPIOB->BSRR = (1U << 9)  | (1U << 10))        // PB9 = 1, PB10 = 1 (Short Brake)
#define MOTOR_R_COAST()  (GPIOB->BSRR = (1U << (9 + 16)) | (1U << (10 + 16))) // PB9 = 0, PB10 = 0 (Thả trôi)

void Motor_Init(void) {
    // 1. Cấp clock cho GPIOA, GPIOB và TIM1
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    // 2. Cấu hình chân hướng Bánh Trái (PA1, PA12) -> Output Push-Pull (01)
    GPIOA->MODER &= ~((3U << (1 * 2)) | (3U << (12 * 2)));
    GPIOA->MODER |=  ((1U << (1 * 2)) | (1U << (12 * 2)));

    // 3. Cấu hình chân hướng Bánh Phải (PB9, PB10) -> Output Push-Pull (01)
    GPIOB->MODER &= ~((3U << (9 * 2)) | (3U << (10 * 2)));
    GPIOB->MODER |=  ((1U << (9 * 2)) | (1U << (10 * 2)));

    // 4. Cấu hình chân PWM: PA8 (TIM1_CH1) và PA11 (TIM1_CH4) -> Alternate Function (10), AF1
    GPIOA->MODER &= ~((3U << (8 * 2)) | (3U << (11 * 2)));
    GPIOA->MODER |=  ((2U << (8 * 2)) | (2U << (11 * 2)));

    GPIOA->AFR[1] &= ~((0xFU << ((8 - 8) * 4)) | (0xFU << ((11 - 8) * 4)));
    GPIOA->AFR[1] |=  ((1U   << ((8 - 8) * 4)) | (1U   << ((11 - 8) * 4)));

    // 5. Cấu hình bộ định thời TIM1 tạo PWM 10 kHz
    TIM1->PSC = 0;
    TIM1->ARR = PWM_PERIOD_ARR;

    // PWM Mode 1 (110) cho CH1 và CH4, bật Preload
    TIM1->CCMR1 &= ~TIM_CCMR1_OC1M;
    TIM1->CCMR1 |= (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;

    TIM1->CCMR2 &= ~TIM_CCMR2_OC4M;
    TIM1->CCMR2 |= (6U << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;

    // Bật ngõ ra CH1, CH4 và Main Output Enable (Bắt buộc với Advanced Timer TIM1)
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC4E;
    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->CR1  |= TIM_CR1_ARPE | TIM_CR1_CEN;

    Motor_Short_Brake();
}

/**
 * @brief Phanh điện từ khóa cứng 2 bánh ngay lập tức
 */
void Motor_Short_Brake(void) {
    MOTOR_L_BRAKE();
    MOTOR_R_BRAKE();
    TIM1->CCR4 = PWM_PERIOD_ARR;
    TIM1->CCR1 = PWM_PERIOD_ARR;
}

/**
 * @brief Thả nổi 2 bánh (dùng khi cần xoay bánh bằng tay)
 */
void Motor_Coast_Stop(void) {
    MOTOR_L_COAST();
    MOTOR_R_COAST();
    TIM1->CCR4 = 0;
    TIM1->CCR1 = 0;
}

/**
 * @brief Điều khiển tốc độ và chiều quay 2 động cơ
 * @param left_pwm:  -1400 đến +1400 (Dương: Tiến, Âm: Lùi, 0: Phanh)
 * @param right_pwm: -1400 đến +1400 (Dương: Tiến, Âm: Lùi, 0: Phanh)
 */
void Motor_Set_Speeds(int16_t left_pwm, int16_t right_pwm) {
    // --- Xử lý Bánh Trái (TIM1_CH4 - PA11) ---
    if (left_pwm == 0) {
        MOTOR_L_BRAKE();
        TIM1->CCR4 = PWM_PERIOD_ARR;
    } else {
        uint32_t duty_l = (uint32_t)(left_pwm > 0 ? left_pwm : -left_pwm);
        if (duty_l > PWM_MAX_LIMIT) duty_l = PWM_MAX_LIMIT;

        if (left_pwm > 0) MOTOR_L_FWD();
        else              MOTOR_L_REV();

        TIM1->CCR4 = duty_l;
    }

    // --- Xử lý Bánh Phải (TIM1_CH1 - PA8) ---
    if (right_pwm == 0) {
        MOTOR_R_BRAKE();
        TIM1->CCR1 = PWM_PERIOD_ARR;
    } else {
        uint32_t duty_r = (uint32_t)(right_pwm > 0 ? right_pwm : -right_pwm);
        if (duty_r > PWM_MAX_LIMIT) duty_r = PWM_MAX_LIMIT;

        if (right_pwm > 0) MOTOR_R_FWD();
        else               MOTOR_R_REV();

        TIM1->CCR1 = duty_r;
    }
}
