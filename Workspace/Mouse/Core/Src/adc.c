/*
 * adc.c
 *
 *  Created on: Oct 17, 2026
 *      Author: 0865435058
 */

#include "adc.h"

void ADC1_Register_Init(void)
{
    /* 1. Enable clock GPIOA và ADC1 */
    RCC->AHB1ENR |= (1U << 0);   /* GPIOAEN */
    RCC->APB2ENR |= (1U << 8);   /* ADC1EN */

    /* Đọc lại thanh ghi để bảo đảm clock đã được cấp */
    (void)RCC->AHB1ENR;
    (void)RCC->APB2ENR;

    /* 2. PA0..PA3: Analog mode (MODER = 11) */
    GPIOA->MODER |= (3U << 0) |
                    (3U << 2) |
                    (3U << 4) |
                    (3U << 6);

    /* Không pull-up/pull-down */
    GPIOA->PUPDR &= ~((3U << 0) |
                      (3U << 2) |
                      (3U << 4) |
                      (3U << 6));

    /* 3. ADC clock = PCLK2 / 4 */
    ADC->CCR &= ~(3U << 16);
    ADC->CCR |=  (1U << 16);

    /* 4. Cấu hình ADC1 */
    ADC1->CR1 = 0U;       /* Resolution 12 bit, không dùng interrupt */
    ADC1->CR2 = 0U;       /* Single conversion, software trigger */

    /* Sampling time 480 cycles cho các channel 0..3 */
    ADC1->SMPR2 &= ~0xFFFU;
    ADC1->SMPR2 |=  0xFFFU;

    /* Một conversion trong mỗi sequence */
    ADC1->SQR1 &= ~(0xFU << 20);

    /* Enable ADC */
    ADC1->CR2 |= (1U << 0);
}

uint16_t ADC1_ReadChannel(uint8_t channel)
{
    if (channel > 15U)
        return 0U;

    /* Chọn channel cho conversion đầu tiên */
    ADC1->SQR3 = (ADC1->SQR3 & ~0x1FU) |
                 ((uint32_t)channel & 0x1FU);

    /* Xóa cờ EOC cũ bằng cách đọc DR nếu cần */
    (void)ADC1->DR;

    /* Bắt đầu conversion */
    ADC1->CR2 |= (1U << 30);  /* SWSTART */

    /* Chờ conversion hoàn thành */
    while ((ADC1->SR & (1U << 1)) == 0U)
    {
        /* EOC = 1 khi conversion xong */
    }

    return (uint16_t)(ADC1->DR & 0x0FFFU);
}

uint16_t ADC_Read_L(void)
{
    return ADC1_ReadChannel(ADC_CH_LSS);
}

uint16_t ADC_Read_R(void)
{
    return ADC1_ReadChannel(ADC_CH_RSS);
}

uint16_t ADC_Read_FL(void)
{
    return ADC1_ReadChannel(ADC_CH_LFS);
}

uint16_t ADC_Read_FR(void)
{
    return ADC1_ReadChannel(ADC_CH_RFS);
}


