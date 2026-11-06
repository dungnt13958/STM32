#include "stm32f4xx_hal.h"

void initLed(void)
{
	RCC -> AHB1ENR |= (1u << 0);
	GPIOA -> MODER &= ~(3u << (2u * 5u));
	GPIOA -> MODER |= (1u << (2u * 5u));
	GPIOA -> PUPDR &= ~(3u << (2u * 5u));
	GPIOA -> PUPDR |= (1u << (2u*5u));
}

void initButton(void)
{
	RCC->AHB1ENR |= (1u << 2u);
	GPIOC -> MODER &= ~(3u << (2u * 13u));
	GPIOC -> PUPDR &= ~(3u << (2u * 13u));
	GPIOC -> PUPDR |= (1u << (2u * 13u));
}

void initInterrupt(void)
{
	RCC->APB2ENR |= (1u << 14u);
	SYSCFG->EXTICR[3] &= ~ (0xFu << 4u);
	SYSCFG->EXTICR[3] |= 0x2u << 4u;
	EXTI -> IMR |= (1u << 13u);
	EXTI -> RTSR &= ~(1u << 13u);
	EXTI -> FTSR |= (1u << 13u);
	EXTI->PR = (1U << 13);
	NVIC_EnableIRQ(EXTI15_10_IRQn);
}
void EXTI15_10_IRQHandler(void)
{
	if(EXTI->PR & (1u << 13u))
	{
			EXTI->PR = (1u << 13u);
			GPIOA -> ODR ^= (1u << 5u);
	}
}
int main(void)
{

	initLed();
	initButton();
	initInterrupt();
	while(1)
	{
	}
}
