#include "main.h"
#include "mpu9250.h"
#define mpu 0x71

volatile float raw_imu_data = 0;
volatile float angle = 0;

int main(void)
{
	HAL_Init();
	MPU_Init();
	MPU_Calibrate_Zero();
	uint32_t last_ms = HAL_GetTick();
	while(1)
	{
		uint32_t now_ms = HAL_GetTick();
		uint32_t elapsed_ms = now_ms - last_ms;
		if (elapsed_ms >= 2)  // Mục tiêu: đọc mỗi 5 ms
		{
			last_ms = now_ms;
			float dt_sec = elapsed_ms / 1000.0f;
			raw_imu_data = MPU_Get_Calibrated_GZ_RadS();
			MPU_Update_Angle(dt_sec);
			angle = MPU_Get_Turn_Angle_Rad();

		}

	}
}
