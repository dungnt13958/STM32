/*
 * pwm.h
 *
 *  Created on: Oct 16, 2026
 *      Author: nguye
 */
#ifndef INC_PWM_H_
#define INC_PWM_H_

#include "stm32f4xx_hal.h"

// Định nghĩa các thông số PWM
#define PWM_TIMER_PRESCALER     10      // Hệ số chia tần (PSC)
#define PWM_TIMER_PERIOD        255     // Giá trị đỉnh (ARR) tương ứng độ phân giải PWM 8-bit (0 - 255)

void PWM_Init(void);
void PWM_Set_Left(uint16_t duty);   // Điều khiển kênh cho PWMA (PA11 - TIM1_CH4)
void PWM_Set_Right(uint16_t duty);  // Điều khiển kênh cho PWMB (PA8 - TIM1_CH1)

#endif /* INC_PWM_H_ */
