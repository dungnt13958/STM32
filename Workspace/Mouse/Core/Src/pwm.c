/*
 * pwm.c
 *
 *  Created on: Oct 16, 2026
 *      Author: nguye
 */
#include "pwm.h"

void PWM_Init(void) {
    // 1. Cấp xung nhịp cho GPIOA và TIM1
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    // 2. Cấu hình chân PA8 (TIM1_CH1) và PA11 (TIM1_CH4) ở chế độ Alternate Function (AF)
    // Mỗi chân chiếm 2 bit trong MODER, giá trị "10" là AF
    GPIOA->MODER &= ~((3U << (8 * 2)) | (3U << (11 * 2)));
    GPIOA->MODER |=  ((2U << (8 * 2)) | (2U << (11 * 2)));

    // 3. Chọn chức năng thay thế (Alternate Function AF1 cho TIM1 trên PA8 và PA11)
    // AFR[0] cho chân từ 0-7, AFR[1] cho chân từ 8-15
    // PA8 -> (8-8)*4 = 0, AF1 = 1
    // PA11 -> (11-8)*4 = 12, AF1 = 1
    GPIOA->AFR[1] &= ~((0xFU << ((8 - 8) * 4)) | (0xFU << ((11 - 8) * 4)));
    GPIOA->AFR[1] |=  ((1U  << ((8 - 8) * 4)) | (1U  << ((11 - 8) * 4)));

    // 4. Cấu hình thông số Timer 1
    TIM1->PSC = PWM_TIMER_PRESCALER;
    TIM1->ARR = PWM_TIMER_PERIOD;

    // 5. Cấu hình chế độ PWM Mode 1 cho Channel 1 (PA8) và Channel 4 (PA11)
    // OC1M = 110 (PWM mode 1), OC1PE = 1 (Preload enable)
    TIM1->CCMR1 |= (6U << 4) | TIM_CCMR1_OC1PE;
    // OC4M = 110 (PWM mode 1), OC4PE = 1 (Preload enable)
    TIM1->CCMR2 |= (6U << 12) | TIM_CCMR2_OC4PE;

    // 6. Kích hoạt ngõ ra PWM cho Channel 1 và Channel 4
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC4E;

    // 7. Bắt buộc bật Main Output Enable (MOE) đối với các dòng Advanced Timer như TIM1
    TIM1->BDTR |= TIM_BDTR_MOE;

    // 8. Đặt giá trị duty cycle ban đầu bằng 0 (dừng động cơ)
    TIM1->CCR1 = 0;
    TIM1->CCR4 = 0;

    // 9. Khởi động Timer 1
    TIM1->CR1 |= TIM_CR1_CEN;
}

void PWM_Set_Left(uint16_t duty) {
    if (duty > PWM_TIMER_PERIOD) duty = PWM_TIMER_PERIOD;
    TIM1->CCR4 = duty; // PWMA tương ứng PA11 (Channel 4)
}

void PWM_Set_Right(uint16_t duty) {
    if (duty > PWM_TIMER_PERIOD) duty = PWM_TIMER_PERIOD;
    TIM1->CCR1 = duty; // PWMB tương ứng PA8 (Channel 1)
}
