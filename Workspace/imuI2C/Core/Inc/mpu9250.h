#ifndef INC_MPU9250_H_
#define INC_MPU9250_H_

#include <stdint.h>
#include "stm32f401xe.h"

extern volatile float live_gz_rad_s;    // Vận tốc góc (rad/s)
extern volatile float mpu_yaw_rad;      // Góc hướng hiện tại thuộc [-pi, pi]
extern volatile float mpu_turn_rad;     // Góc xoay tương đối trong cú rẽ thuộc [-pi, pi]

float MPU_Wrap_To_Pi(float angle_rad);
void MPU_Init(void);
void MPU_Calibrate_Zero(void);
float MPU_Get_Calibrated_GZ_RadS(void);

void MPU_Update_Angle(float dt_sec);
void MPU_Reset_Turn_Angle(void);
float MPU_Get_Turn_Angle_Rad(void);

#endif /* INC_MPU6500_H_ */
