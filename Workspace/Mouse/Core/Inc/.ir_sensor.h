#ifndef INC_IR_SENSOR_H_
#define INC_IR_SENSOR_H_

#include <stdint.h>
#include "stm32f411xe.h"

// Kênh ADC1 cho 4 mắt thu
#define CH_SENFL            2   // PA2 -> ADC1_IN2 (Trước - Trái)
#define CH_SENL             3   // PA3 -> ADC1_IN3 (Hông - Trái)
#define CH_SENR             4   // PA4 -> ADC1_IN4 (Hông - Phải)
#define CH_SENFR            5   // PA5 -> ADC1_IN5 (Trước - Phải)

// 2 Chân kích LED phát hồng ngoại
#define IR1_PIN             6   // PB6 (Cụm Trái: SENL + SENFL)
#define IR2_PIN             7   // PB7 (Cụm Phải: SENR + SENFR)

typedef struct {
    uint16_t senl;
    uint16_t senfl;
    uint16_t senfr;
    uint16_t senr;
} IR_Sensors_t;

typedef struct {
    uint16_t open_fl;
    uint16_t open_fr;
    uint16_t base_sl;
    uint16_t base_sr;
    uint16_t base_front;
    uint16_t thresh_sl;
    uint16_t thresh_sr;
    uint16_t thresh_front;
    int16_t  side_diff_offset;
} IR_Calibration_t;

extern volatile IR_Sensors_t ir_raw;
extern volatile IR_Calibration_t ir_calib;

void IR_Init(void);
void IR_Read_All(void);
void IR_Calibrate_StartCell_Phase1(void);
void IR_Calibrate_StartCell_Phase2_Front(void);
uint8_t IR_Has_Left_Wall(void);
uint8_t IR_Has_Right_Wall(void);
uint8_t IR_Has_Front_Wall(void);

#endif /* INC_IR_SENSOR_H_ */
