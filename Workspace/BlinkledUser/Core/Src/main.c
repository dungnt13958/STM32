#include "stm32f4xx_hal.h"

int main(void)
{
	//	Port A clock Enable
	RCC -> AHB1ENR |= 1 << 0;
	GPIOA -> MODER &= ~(3u << (2u*1u));
	GPIOA -> MODER &= ~(3u << (2u*2u));
	GPIOA -> MODER |= (1u << (2u * 2u));
//	GPIOA -> PUPDR &= ~(3u << (2u * 1u));
//	GPIOA -> PUPDR |= (1u << (2u * 1u));
	while(1)
	{
//		uint32_t buttonState = (GPIOA ->IDR >> 1u)&1u;
//		if(buttonState)
//		{
//			GPIOA->BSRR = (1u << (2u + 16u));
//		}
//		if(!buttonState)
//		{
//			GPIOA ->BSRR = (1u << 2u);
//		}
		GPIOA -> ODR |= (1u << 5u);
//		GPIOA -> BSRR = (1u << (2u + 16u));
	}
}
