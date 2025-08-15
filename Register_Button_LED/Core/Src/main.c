#include "main.h"

extern uint32_t SystemCoreClock;

uint32_t systemClock;

void RCC_Config(void) {
    // 1. HSE ON
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    // 2. Flash latency ayarla ??????
    FLASH->ACR |= FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN;
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_5WS;

    // 3. PLL ayarları (8MHz HSE)
    RCC->PLLCFGR = 0;
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;
    RCC->PLLCFGR |= (4 << RCC_PLLCFGR_PLLM_Pos);    // PLLM = 4 (8MHz HSE)
    RCC->PLLCFGR |= (168 << RCC_PLLCFGR_PLLN_Pos);  // PLLN = 168
    RCC->PLLCFGR &= ~(2 << RCC_PLLCFGR_PLLP_Pos);   // PLLP = 2 (00b)
    RCC->PLLCFGR |= (7 << RCC_PLLCFGR_PLLQ_Pos);    // PLLQ = 7

    // 4. PLL ON
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    // 5. AHB/APB prescaler ayarları
    RCC->CFGR &= ~RCC_CFGR_HPRE;     // Önce temizle
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1; // AHB = 168MHz

    RCC->CFGR &= ~RCC_CFGR_PPRE1;    // Önce temizle
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4; // APB1 = 42MHz

    RCC->CFGR &= ~RCC_CFGR_PPRE2;    // Önce temizle
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2; // APB2 = 84MHz

    // 6. PLL'i sistem clock yap
    RCC->CFGR &= ~(3 << RCC_CFGR_SW_Pos);
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->AHB1ENR |=RCC_AHB1ENR_GPIODEN;

}

void GPIO_Config(void){

	GPIOA->MODER &= ~(3<<0*2);
	for (int pin = 12; pin < 16; ++pin) {

		GPIOD->MODER &=~(3<<pin*2);
		GPIOD->MODER |= (1<<pin*2);
	}

	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	SYSCFG->EXTICR[0] |=SYSCFG_EXTICR1_EXTI0_PA;
	EXTI->IMR |=(1<<0);
	EXTI->RTSR |=(1<<0);
	NVIC_EnableIRQ(EXTI0_IRQn);

}

void EXTI0_IRQHandler(void){

	if(EXTI->PR & (1<<0)){

		for (int pin = 12; pin < 16; ++pin){

			GPIOD->ODR ^= (1<<pin);
		}
			EXTI->PR |= (1<<0); // EXT interrupt flagı temizlendii.
	}
}
int main(void){

	RCC_Config();
	GPIO_Config();

	while(1)
	{
	}

}



