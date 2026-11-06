/*
 * system_ui.c
 *
 *  Created on: Oct 16, 2026
 *      Author: 0865435058
 */
#include "system_ui.h"

void delay_us(uint32_t us){
	for (uint32_t i = 0; i < us * 3; i++){
		__NOP();
	}
}

void delay_ms(uint32_t ms) {
	for (uint32_t i = 0; i < ms * 2666; i++) {
		__NOP();
	}
}

void UI_All_LEDs_Off(void) {
	LED1_OFF();
	LED2_OFF();
	LED3_OFF();
	LED4_OFF();
}

void UI_All_LEDs_On(void) {
	LED1_ON();
	LED2_ON();
	LED3_ON();
	LED4_ON();
}

void UI_Init(void) {
	// cấp xung cho GPIOA v GPIOB
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;

	// cấu hình button: Input (00) + pull-up (01)
	GPIOA->MODER &= ~(3U << (BUTTON_PIN * 2));
	GPIOA->PUPDR &= ~(3U << (BUTTON_PIN * 2));
	GPIOA->PUPDR |=  (1U << (BUTTON_PIN * 2));

	// cấu hình led4: Output (01) + push-pull (01)
	GPIOA->MODER &= ~(3U << (LED4_PIN * 2));
	GPIOA->MODER |=  (1U << (LED4_PIN * 2));
	GPIOA->OTYPER &= ~(3U << (LED4_PIN * 2));

	// Cấu hình Led 1->3
	GPIOB->MODER &= ~((3U << (LED1_PIN * 2)) | (3U << (LED2_PIN * 2)) | (3U << (LED3_PIN * 2)));
	GPIOB->MODER |=  ((1U << (LED1_PIN * 2)) | (1U << (LED2_PIN * 2)) | (1U << (LED3_PIN * 2)));

	UI_All_LEDs_Off();
}

void UI_Show_Mode(uint8_t mode) {
	UI_All_LEDs_Off();
	if (mode == 1) LED1_ON();
	if (mode == 2) LED2_ON();
	if (mode == 3) LED3_ON();
	if (mode == 4) LED4_ON();
}
/**
 * @brief Nhấp nháy LED1 (PB2) với số lần và khoảng thời gian tùy chọn
 */
void UI_Blink_LED1(uint8_t times, uint16_t period_ms) {
    for (uint8_t i = 0; i < times; i++) {
        LED1_ON();
        delay_ms(period_ms);
        LED1_OFF();
        delay_ms(period_ms);
    }
}

ButtonEvent_t UI_Read_Button(void) {
	if (is_button_pressed()) {
		delay_ms(20);
		if (is_button_pressed()) {
			uint32_t hold_ms = 0;

			while(is_button_pressed()) {
				delay_ms(10);
				hold_ms += 10;

				if (hold_ms >= 800) {
					for (int i = 0; i < 4; i++) {
						UI_All_LEDs_On(); delay_ms(50);
						UI_All_LEDs_Off(); delay_ms(50);
					}
					while(is_button_pressed());
					delay_ms(50);
					return BTN_EVENT_LONG_PRESS;
				}
			}
			return BTN_EVENT_SHORT_CLICK;
		}
	}
	return BTN_EVENT_NONE;
}

/**
 * @brief Hiển thị số nguyên 4-bit (0 - 15) lên 4 đèn LED theo hệ nhị phân
 */
void UI_Show_Binary(uint8_t val) {
    // Dùng phép toán AND bit ( & ) để kiểm tra từng bit có bằng 1 hay không
    if (val & 0x01) LED1_ON(); else LED1_OFF();  // Bit 0
    if (val & 0x02) LED2_ON(); else LED2_OFF();  // Bit 1
    if (val & 0x04) LED3_ON(); else LED3_OFF();  // Bit 2
    if (val & 0x08) LED4_ON(); else LED4_OFF();  // Bit 3
}
