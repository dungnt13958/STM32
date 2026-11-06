#include "stm32f4xx_hal.h"

void initLed(void)
{
	RCC -> AHB1ENR |= 1u << 0u;
	GPIOA -> MODER &= ~(3u << (2u * 5u));
	GPIOA -> MODER |= (1u << (2u * 5u));
	GPIOA -> MODER &= ~(3u << (2u * 1u));
	GPIOA -> PUPDR &= ~(3u << (2u * 1u));
	GPIOA -> PUPDR |= (1u << (2u * 1u));
}
int main(void)
{
	initLed();
	while(1)
	{
		uint32_t buttonState = (GPIOA ->IDR >> 1u)&1u;
		if(buttonState)
		{
			GPIOA->BSRR = (1u << (5u + 16u));
		}
		if(!buttonState)
		{
			GPIOA ->BSRR = (1u << 5u);
		}
	}
}
