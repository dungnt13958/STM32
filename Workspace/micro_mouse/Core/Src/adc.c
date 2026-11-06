/**
 * @file    adc.c
 * @brief   Hiện thực đọc ADC1 siêu tốc bằng thanh ghi và khử nhiễu ánh sáng nền
 */

#include "adc.h"

ADC_Sensor_t adc = {
    .raw_ambient = {0U, 0U, 0U, 0U},
    .raw_peak    = {0U, 0U, 0U, 0U},
    .final_val   = {0U, 0U, 0U, 0U}
};

/**
 * @brief Chờ Phototransistor đạt đỉnh phản xạ hồng ngoại
 */
static inline void ADC_Delay_Peak(void)
{
	__NOP();
}

/**
 * @brief Đọc đơn kênh ADC1 trực tiếp qua thanh ghi
 */
static uint16_t ADC_Read_Channel(uint8_t channel)
{
    ADC1->SQR3 = (uint32_t)channel;
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while ((ADC1->SR & ADC_SR_EOC) == 0U) {
        __NOP();
    }
    return (uint16_t)(ADC1->DR & 0x0FFFU);
}

/**
 * @brief Khởi tạo GPIO Analog (PA2..PA5), GPIO Output (PB6, PB7) và ADC1
 */
void ADC_Begin(void)
{
    // 1. Bật xung nhịp cho GPIOA, GPIOB và ADC1
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // 2. Cấu hình PA2, PA3, PA4, PA5 ở chế độ Analog (MODER = 0b11), không kéo trở
    GPIOA->MODER |= (3U << (2U * 2U)) | (3U << (3U * 2U)) |
                    (3U << (4U * 2U)) | (3U << (5U * 2U));
    GPIOA->PUPDR &= ~((3U << (2U * 2U)) | (3U << (3U * 2U)) |
                      (3U << (4U * 2U)) | (3U << (5U * 2U)));

    // 3. Cấu hình PB6 (IR1) và PB7 (IR2) làm Output Push-Pull, tốc độ Very High
    GPIOB->MODER   &= ~((3U << (6U * 2U)) | (3U << (7U * 2U)));
    GPIOB->MODER   |=  ((1U << (6U * 2U)) | (1U << (7U * 2U)));
    GPIOB->OTYPER  &= ~((1U << 6U) | (1U << 7U));
    GPIOB->OSPEEDR |=  ((3U << (6U * 2U)) | (3U << (7U * 2U)));
    GPIOB->PUPDR   &= ~((3U << (6U * 2U)) | (3U << (7U * 2U)));

    // Đảm bảo tắt cả 2 cụm LED phát ban đầu
    GPIOB->BSRR = GPIO_BSRR_BR6 | GPIO_BSRR_BR7;

    // 4. Cấu hình ADC1: Độ phân giải 12-bit, chuyển đổi đơn kênh
    ADC->CCR &= ~ADC_CCR_ADCPRE; // PCLK2 / 2
    ADC1->CR1 = 0U;
    ADC1->CR2 = ADC_CR2_ADON;
    ADC1->SQR1 = 0U;             // 1 chuyển đổi trong chuỗi

    // Thời gian lấy mẫu 480 chu kỳ (0b111 = 7) cho cả 4 kênh để nạp tụ chính xác nhất
    ADC1->SMPR2 &= ~((7U << (3U * ADC_CH_SENFL)) |
                     (7U << (3U * ADC_CH_SENL))  |
                     (7U << (3U * ADC_CH_SENR))  |
                     (7U << (3U * ADC_CH_SENFR)));
    ADC1->SMPR2 |=  ((7U << (3U * ADC_CH_SENFL)) |
                     (7U << (3U * ADC_CH_SENL))  |
                     (7U << (3U * ADC_CH_SENR))  |
                     (7U << (3U * ADC_CH_SENFR)));

    ADC1->SR = 0U;
}

/**
 * @brief Quét 2 cụm thu phát hồng ngoại và tính giá trị phản xạ thực
 */
void ADC_Update(void)
{
    /* ====================================================================
     * CỤM 1: Kích PB6 (IR1) -> Đọc cặp FL (PA2) và R (PA4)
     * ==================================================================== */
    uint16_t amb_FL = ADC_Read_Channel(ADC_CH_SENFL);
    uint16_t amb_R  = ADC_Read_Channel(ADC_CH_SENR);

    GPIOB->BSRR = GPIO_BSRR_BS6; // Bật LED phát cụm 1 (PB6)
    ADC_Delay_Peak();            // Chờ tín hiệu phản xạ đạt đỉnh

    uint16_t peak_FL = ADC_Read_Channel(ADC_CH_SENFL);
    uint16_t peak_R  = ADC_Read_Channel(ADC_CH_SENR);

    GPIOB->BSRR = GPIO_BSRR_BR6; // Tắt LED phát cụm 1 ngay lập tức

    adc.raw_ambient[0] = amb_FL;
    adc.raw_ambient[2] = amb_R;
    adc.raw_peak[0]    = peak_FL;
    adc.raw_peak[2]    = peak_R;

    int32_t diff_FL = (int32_t)peak_FL - (int32_t)amb_FL;
    int32_t diff_R  = (int32_t)peak_R  - (int32_t)amb_R;

    adc.final_val[0] = (diff_FL > 0) ? (uint16_t)diff_FL : 0U;
    adc.final_val[2] = (diff_R  > 0) ? (uint16_t)diff_R  : 0U;

    /* ====================================================================
     * CỤM 2: Kích PB7 (IR2) -> Đọc cặp L (PA3) và FR (PA5)
     * ==================================================================== */
    uint16_t amb_L  = ADC_Read_Channel(ADC_CH_SENL);
    uint16_t amb_FR = ADC_Read_Channel(ADC_CH_SENFR);

    GPIOB->BSRR = GPIO_BSRR_BS7; // Bật LED phát cụm 2 (PB7)
    ADC_Delay_Peak();            // Chờ tín hiệu phản xạ đạt đỉnh

    uint16_t peak_L  = ADC_Read_Channel(ADC_CH_SENL);
    uint16_t peak_FR = ADC_Read_Channel(ADC_CH_SENFR);

    GPIOB->BSRR = GPIO_BSRR_BR7; // Tắt LED phát cụm 2 ngay lập tức

    adc.raw_ambient[1] = amb_L;
    adc.raw_ambient[3] = amb_FR;
    adc.raw_peak[1]    = peak_L;
    adc.raw_peak[3]    = peak_FR;

    int32_t diff_L  = (int32_t)peak_L  - (int32_t)amb_L;
    int32_t diff_FR = (int32_t)peak_FR - (int32_t)amb_FR;

    adc.final_val[1] = (diff_L  > 0) ? (uint16_t)diff_L  : 0U;
    adc.final_val[3] = (diff_FR > 0) ? (uint16_t)diff_FR : 0U;
}

uint16_t ADC_Get_FL(void) { return adc.final_val[0]; }
uint16_t ADC_Get_L(void)  { return adc.final_val[1]; }
uint16_t ADC_Get_R(void)  { return adc.final_val[2]; }
uint16_t ADC_Get_FR(void) { return adc.final_val[3]; }
