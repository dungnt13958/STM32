#ifndef INC_MPU6500_H_
#define INC_MPU6500_H_

#include <stdint.h>
#include "stm32f411xe.h"

#define GYRO_SCALE_DIVISOR   16.40f // Dải +-2000 dps

extern volatile float live_gz_dps;
extern volatile float raw_gz_bias;

void MPU_Init(void);
void MPU_Calibrate_Zero(void);
float MPU_Get_Calibrated_GZ(void);

#endif /* INC_MPU6500_H_ */
