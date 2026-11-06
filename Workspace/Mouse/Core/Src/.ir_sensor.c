#include "ir_sensor.h"

volatile IR_Sensors_t ir_raw = {0};
volatile IR_Calibration_t ir_calib = {0};

#define IR1_ON()    (GPIOB->BSRR = (1U << IR1_PIN))
#define IR1_OFF()   (GPIOB->BSRR = (1U << (IR1_PIN + 16)))
#define IR2_ON()    (GPIOB->BSRR = (1U << IR2_PIN))
#define IR2_OFF()   (GPIOB->BSRR = (1U << (IR2_PIN + 16)))

static void ir_delay_us(uint32_t us) {
    for (uint32_t i = 0; i < us * 3; i++) {
        __NOP();
    }
}

static void ir_delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 2666; i++) {
        __NOP();
    }
}

static uint16_t ADC1_Read_Channel(uint8_t channel) {
    ADC1->SQR3 = channel;
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC));
    return (uint16_t)ADC1->DR;
}

void IR_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // Chân thu Analog: PA2, PA3, PA4, PA5 -> Analog (11)
    GPIOA->MODER |= (3U << (2 * 2)) | (3U << (3 * 2)) |
                    (3U << (4 * 2)) | (3U << (5 * 2));

    // Chân kích LED phát: PB6, PB7 -> Output (01)
    GPIOB->MODER &= ~((3U << (IR1_PIN * 2)) | (3U << (IR2_PIN * 2)));
    GPIOB->MODER |=  ((1U << (IR1_PIN * 2)) | (1U << (IR2_PIN * 2)));

    IR1_OFF();
    IR2_OFF();

    // ADC Prescaler /4 (~4MHz)
    ADC->CCR &= ~(3U << 16);
    ADC->CCR |=  (1U << 16);
    ADC1->CR2 = ADC_CR2_ADON;
    ir_delay_us(50);
}

void IR_Read_All(void) {
    // --- CỤM 1: Phát IR1 (PB6) -> Đo SENL (PA3) & SENFL (PA2) ---
    uint16_t amb_senl  = ADC1_Read_Channel(CH_SENL);
    uint16_t amb_senfl = ADC1_Read_Channel(CH_SENFL);

    IR1_ON();
    ir_delay_us(25);
    uint16_t sig_senl  = ADC1_Read_Channel(CH_SENL);
    uint16_t sig_senfl = ADC1_Read_Channel(CH_SENFL);
    IR1_OFF();

    ir_raw.senl  = (sig_senl  > amb_senl)  ? (sig_senl  - amb_senl)  : 0;
    ir_raw.senfl = (sig_senfl > amb_senfl) ? (sig_senfl - amb_senfl) : 0;

    ir_delay_us(30);

    // --- CỤM 2: Phát IR2 (PB7) -> Đo SENFR (PA5) & SENR (PA4) ---
    uint16_t amb_senfr = ADC1_Read_Channel(CH_SENFR);
    uint16_t amb_senr  = ADC1_Read_Channel(CH_SENR);

    IR2_ON();
    ir_delay_us(25);
    uint16_t sig_senfr = ADC1_Read_Channel(CH_SENFR);
    uint16_t sig_senr  = ADC1_Read_Channel(CH_SENR);
    IR2_OFF();

    ir_raw.senfr = (sig_senfr > amb_senfr) ? (sig_senfr - amb_senfr) : 0;
    ir_raw.senr  = (sig_senr  > amb_senr)  ? (sig_senr  - amb_senr)  : 0;
}

void IR_Calibrate_StartCell_Phase1(void) {
    uint32_t s_fl = 0, s_fr = 0, s_sl = 0, s_sr = 0;
    const uint16_t samples = 200;

    for (uint16_t i = 0; i < samples; i++) {
        IR_Read_All();
        s_fl += ir_raw.senfl;
        s_fr += ir_raw.senfr;
        s_sl += ir_raw.senl;
        s_sr += ir_raw.senr;
        ir_delay_ms(3);
    }

    ir_calib.open_fl = s_fl / samples;
    ir_calib.open_fr = s_fr / samples;
    ir_calib.base_sl = s_sl / samples;
    ir_calib.base_sr = s_sr / samples;

    ir_calib.thresh_sl = (ir_calib.base_sl * 55) / 100;
    ir_calib.thresh_sr = (ir_calib.base_sr * 55) / 100;
    ir_calib.side_diff_offset = (int16_t)ir_calib.base_sl - (int16_t)ir_calib.base_sr;
}

void IR_Calibrate_StartCell_Phase2_Front(void) {
    uint32_t s_front = 0;
    const uint16_t samples = 150;

    for (uint16_t i = 0; i < samples; i++) {
        IR_Read_All();
        s_front += (ir_raw.senfl + ir_raw.senfr) / 2;
        ir_delay_ms(3);
    }

    ir_calib.base_front = s_front / samples;
    uint16_t open_front = (ir_calib.open_fl + ir_calib.open_fr) / 2;
    ir_calib.thresh_front = (open_front + ir_calib.base_front) / 2;
}

uint8_t IR_Has_Left_Wall(void) {
    return (ir_raw.senl > ir_calib.thresh_sl) ? 1 : 0;
}

uint8_t IR_Has_Right_Wall(void) {
    return (ir_raw.senr > ir_calib.thresh_sr) ? 1 : 0;
}

uint8_t IR_Has_Front_Wall(void) {
    uint16_t front_avg = (ir_raw.senfl + ir_raw.senfr) / 2;
    return (front_avg > ir_calib.thresh_front) ? 1 : 0;
}
