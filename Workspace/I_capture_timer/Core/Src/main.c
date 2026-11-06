#include "stm32f4xx_hal.h"

void initEncoder(void)
{
	RCC->AHB1ENR |= 1u << 0;
	GPIOA->MODER &= ~(3u << (2u * 6u));
	GPIOA->MODER |= (2u << (2u * 6u));
	GPIOA->PUPDR &= ~(3u << (2u * 6u));
	GPIOA->PUPDR |= (1u << (2u * 6u));
	GPIOA->MODER &= ~(3u << (2u * 7u));
	GPIOA->MODER |= (2u << (2u * 7u));
	GPIOA->PUPDR &= ~(3u << (2u * 7u));
	GPIOA->PUPDR |= (1u << (2u * 7u));
	GPIOA->AFR[0] &= ~(0xFu <<(4u * 6u));
	GPIOA->AFR[0] |= (2u << (4u * 6u));
	GPIOA->AFR[0] &= ~(0xFu << (4u * 7u));
	GPIOA->AFR[0] |= (2u << (4u * 7u));
}

void timer3_init(void)
{
	RCC->APB1ENR |= 1u << 1u;
	TIM3->PSC = 0;

}
