#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "stm32f4xx.h"
#include <stdint.h>
#include <math.h>

#ifndef PI_FLOAT
#define PI_FLOAT                 3.14159265358979323846f
#endif

#ifndef TWO_PI_FLOAT
#define TWO_PI_FLOAT             (2.0f * PI_FLOAT)
#endif

/* Thông số cơ khí của bánh xe và cụm encoder */
#define ENCODER_PPR              2772.0f                                    // Số xung/vòng thực tế
#define WHEEL_DIAMETER_MM        30.0f                                      // Đường kính bánh xe (mm)
#define WHEEL_CIRCUMFERENCE_MM   (PI_FLOAT * WHEEL_DIAMETER_MM)             // Chu vi bánh xe (mm)
#define PULSES_PER_MM            (ENCODER_PPR / WHEEL_CIRCUMFERENCE_MM)     // Số xung trên mỗi mm

#ifndef WHEEL_TRACK_MM
#define WHEEL_TRACK_MM           70.0f                                      // Khoảng cách giữa 2 tâm vệt bánh xe (mm)
#endif

void Encoder_Init(void);
void Encoder_Reset(void);
int32_t Encoder_Get_Left_Count(void);
int32_t Encoder_Get_Right_Count(void);
float Encoder_Get_Turn_Angle_Rad(void);

#endif /* INC_ENCODER_H_ */
