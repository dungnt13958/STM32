/*
 * sysm_ui.h
 *
 *  Created on: Oct 16, 2026
 *      Author: 0865435058
 */

#ifndef INC_SYSTEM_UI_H_
#define INC_SYSTEM_UI_H_

#include <stdint.h>
#include "stm32f411xe.h"

/* CONFIG 4 LED & BUTTON */
// LED1 - PB2
#define LED1_PIN 2
#define LED1_ON() (GPIOB->BSRR = (1U << LED1_PIN))
#define LED1_OFF() (GPIOB->BSRR = (1U << (LED1_PIN + 16)))
#define LED1_TOGGLE() (GPIOB->ODR ^= (1U << LED1_PIN))

// LED2 - PB1
#define LED2_PIN 1
#define LED2_ON() (GPIOB->BSRR = (1U << LED2_PIN))
#define LED2_OFF() (GPIOB->BSRR = (1U << (LED2_PIN + 16)))
#define LED2_TOGGLE() (GPIOB->ODR ^= (1U << LED2_PIN))

// LED3 - PB0
#define LED3_PIN 0
#define LED3_ON() (GPIOB->BSRR = (1U << LED3_PIN))
#define LED3_OFF() (GPIOB->BSRR = (1U << (LED3_PIN + 16)))
#define LED3_TOGGLE() (GPIOB->ODR ^= (1U << LED3_PIN))

// LED4 - PA7
#define LED4_PIN 7
#define LED4_ON() (GPIOA->BSRR = (1U << LED4_PIN))
#define LED4_OFF() (GPIOA->BSRR = (1U << (LED4_PIN + 16)))
#define LED4_TOGGLE() (GPIOA->ODR ^= (1U << LED4_PIN))

// BUTTON - PA0
#define BUTTON_PIN 0
#define is_button_pressed() ((GPIOA->IDR & (1U << BUTTON_PIN)) == 0)

typedef enum {
	BTN_EVENT_NONE = 0,
	BTN_EVENT_SHORT_CLICK,
	BTN_EVENT_LONG_PRESS
} ButtonEvent_t;

void UI_Init(void);
void delay_ms(uint32_t ms);
void delay_us(uint32_t us);
void UI_Blink_LED1(uint8_t times, uint16_t period_ms);
void UI_Show_Binary(uint8_t val);
void UI_All_LEDs_Off(void);
void UI_All_LEDs_On(void);
void UI_Show_Mode(uint8_t mode);
ButtonEvent_t UI_Read_Button(void);

#endif /* INC_SYSTEM_UI_H_ */
