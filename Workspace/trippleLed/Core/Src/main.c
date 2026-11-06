#include "stm32f4xx.h"

void initLed(void)
{
	RCC->AHB1ENR |= 1u << 0u;
	RCC->AHB1ENR |= 1u << 2u;
	GPIOC->MODER &= ~(3u << (2u*1u));
	GPIOC->MODER &= ~(3u << (2u*2u));
	GPIOC->MODER &= ~(3u << (2u * 3u));
	GPIOC -> PUPDR &= ~(3u << (2u * 1u));
	GPIOC -> PUPDR |= (1u << (2u * 1u));
	GPIOC -> PUPDR &= ~(3u << (2u * 2u));
	GPIOC -> PUPDR |= (1u << (2u * 2u));
	GPIOC -> PUPDR &= ~(3u << (2u * 3u));
	GPIOC -> PUPDR |= (1u << (2u * 3u));

	GPIOA -> MODER &= ~(3u << (2u * 5u));
	GPIOA -> MODER |= (1u << (2u * 5u));
	GPIOA -> MODER &= ~(3u << (2u * 6u));
	GPIOA -> MODER |= (1u << (2u * 6u));
	GPIOA -> MODER &= ~(3u << (2u * 7u));
	GPIOA -> MODER |= (1u << (2u * 7u));
}

int main(void)
{
	initLed();
	while(1)
	{
		uint32_t redLight = (GPIOC->IDR >> 3)&1u;
		uint32_t yellowLight = (GPIOC -> IDR >> 2)&1u;
		uint32_t greenLight = (GPIOC -> IDR >> 1)&1u;
		if(redLight)
		{
			GPIOA->BSRR = (1u << (7u + 16u));
		}
		else
		{
			GPIOA->BSRR = (1u << 7u);
		}
		if(yellowLight)
		{
			GPIOA->BSRR = (1u << (6u + 16u));
		}
		else
		{
			GPIOA->BSRR = (1u << 6u);
		}
		if(greenLight)
		{
			GPIOA -> BSRR = (1u << (5u + 16u));
		}
		else
		{
			GPIOA -> BSRR = (1u << 5u);
		}
	}
}
