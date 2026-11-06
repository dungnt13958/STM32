#include "stm32f4xx_hal.h"
#include "config.h"
void initLed(GPIO_TypeDef *GPIOx, uint32_t port, uint32_t pin)
{
	RCC->AHB1ENR |= (1u << port);
	GPIOx -> MODER &= ~(3u << (2u*pin));
	GPIOx -> MODER |= (1u << (2u*pin));
	GPIOx -> PUPDR &= ~(3u << (2u*pin));
	GPIOx -> PUPDR |= (1u << (2u*pin ));
}

void initButton(GPIO_TypeDef *GPIOx, uint32_t port, uint32_t pin)
{
	RCC->AHB1ENR |= (1u << port);
	GPIOx -> MODER &= ~(3u << (2u*pin));
	GPIOx -> PUPDR &= ~(3u << (2u*pin));
	GPIOx -> PUPDR |= (1u << (2u*pin ));
}

void initInt(uint32_t pin)
{
	uint32_t sysExti   = pin / 4u;
	uint32_t intConfig = pin % 4u;
	RCC->APB2ENR |= 1u << 14u;
	SYSCFG->EXTICR[sysExti] &= ~(0xFu << (4u * intConfig));
	SYSCFG->EXTICR[sysExti] |= (0x2u << (4u * intConfig));
	EXTI->IMR |= (1u << pin);
	EXTI->RTSR &= ~(1u << pin);
	EXTI->FTSR |= (1u << pin);
}

void initNVIC(uint32_t pin, IRQn_Type IRQn)
{
	EXTI->PR = (1u << pin);
	NVIC_EnableIRQ(IRQn);
}

void EXTI1_IRQHandler(void)
{
    if (EXTI->PR & (1u << greenButton))
    {
        EXTI->PR = (1u << greenButton);

        // Tắt Yellow và Red
        GPIOA->BSRR = (1u << (yellowLed + 16u));
        GPIOA->BSRR = (1u << (redLed + 16u));

        // Green ON
        GPIOA->BSRR = (1u << greenLed);

        for (volatile uint32_t i = 0; i < 1000000u; i++)
        {
        }

        // Green OFF
        GPIOA->BSRR = (1u << (greenLed + 16u));
    }
}

void EXTI2_IRQHandler(void)
{
    if (EXTI->PR & (1u << yellowButton))
    {
        EXTI->PR = (1u << yellowButton);

        // Tắt Green và Red
        GPIOA->BSRR = (1u << (greenLed + 16u));
        GPIOA->BSRR = (1u << (redLed + 16u));

        // Yellow ON
        GPIOA->BSRR = (1u << yellowLed);

        for (volatile uint32_t i = 0; i < 1000000u; i++)
        {
        }

        // Yellow OFF
        GPIOA->BSRR = (1u << (yellowLed + 16u));
    }
}

void EXTI3_IRQHandler(void)
{
    if (EXTI->PR & (1u << redButton))
    {
        EXTI->PR = (1u << redButton);

        // Tắt Green và Yellow
        GPIOA->BSRR = (1u << (greenLed + 16u));
        GPIOA->BSRR = (1u << (yellowLed + 16u));

        // Red ON
        GPIOA->BSRR = (1u << redLed);

        for (volatile uint32_t i = 0; i < 1000000u; i++)
        {
        }

        // Red OFF
        GPIOA->BSRR = (1u << (redLed + 16u));
    }
}
int main(void)
{
	initLed(GPIOA, 0u, greenLed);
	initLed(GPIOA, 0u, yellowLed);
	initLed(GPIOA, 0u, redLed);
	initButton(GPIOC, 2u, greenButton);
	initButton(GPIOC, 2u, yellowButton);
	initButton(GPIOC, 2u, redButton);
	initInt(greenButton);
	initInt(yellowButton);
	initInt(redButton);
	NVIC_SetPriority(EXTI1_IRQn, 0);
	NVIC_SetPriority(EXTI2_IRQn, 1);
	NVIC_SetPriority(EXTI3_IRQn, 2);
	initNVIC(greenButton, EXTI1_IRQn);
	initNVIC(yellowButton, EXTI2_IRQn);
	initNVIC(redButton, EXTI3_IRQn);
	while(1)
	{

	}
}
