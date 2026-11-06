#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <stdint.h>
#include "stm32f411xe.h"

// Với Clock mặc định HSI = 16MHz, PSC = 0, ARR = 1599 -> Tần số PWM = 16MHz / 1600 = 10 kHz
#define PWM_PERIOD_ARR      1599
#define PWM_MAX_LIMIT       1400   // Giới hạn an toàn (khoảng 87% duty cycle)

void Motor_Init(void);
void Motor_Set_Speeds(int16_t left_pwm, int16_t right_pwm);
void Motor_Short_Brake(void);
void Motor_Coast_Stop(void);

#endif /* INC_MOTOR_H_ */
