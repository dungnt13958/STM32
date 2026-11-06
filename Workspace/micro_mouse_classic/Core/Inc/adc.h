/**
 * @file    adc.h
 * @brief   Driver Bare-metal đọc ADC1 4 kênh cảm biến hồng ngoại kết hợp kích phát PB6/PB7
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "stm32f4xx.h"
#include <stdint.h>

/* ================= SƠ ĐỒ KÊNH ADC1 (PA2 -> PA5) ================= */
#define ADC_CH_SENFL        2U  // PA2 -> ADC1_IN2 (Front Left)
#define ADC_CH_SENL         3U  // PA3 -> ADC1_IN3 (Side Left)
#define ADC_CH_SENR         4U  // PA4 -> ADC1_IN4 (Side Right)
#define ADC_CH_SENFR        5U  // PA5 -> ADC1_IN5 (Front Right)

/* ================= CHÂN KÍCH PHÁT LED HỒNG NGOẠI ================= */
#define IR1_EMIT_PIN        6U  // PB6: Phát cho cặp FL (PA2) & R (PA4)
#define IR2_EMIT_PIN        7U  // PB7: Phát cho cặp L (PA3) & FR (PA5)

/* ================= CẤU TRÚC LƯU TRỮ DỮ LIỆU ADC ================= */
typedef struct {
    volatile uint16_t raw_ambient[4]; // Ánh sáng môi trường khi tắt LED: [0]=FL, [1]=L, [2]=R, [3]=FR
    volatile uint16_t raw_peak[4];    // Tín hiệu tổng khi bật LED:       [0]=FL, [1]=L, [2]=R, [3]=FR
    volatile uint16_t final_val[4];   // Tín hiệu thực (Peak - Ambient):  [0]=FL, [1]=L, [2]=R, [3]=FR
} ADC_Sensor_t;

extern ADC_Sensor_t adc;

/* ================= NGUYÊN MẪU HÀM ================= */
void ADC_Begin(void);
void ADC_Update(void);

uint16_t ADC_Get_FL(void);
uint16_t ADC_Get_L(void);
uint16_t ADC_Get_R(void);
uint16_t ADC_Get_FR(void);

#endif /* INC_ADC_H_ */
