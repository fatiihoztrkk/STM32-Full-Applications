#include "main.h"

extern uint32_t SystemCoreClock;

uint32_t systemClock;

void RCC_Config(void) {
    // 1. HSE ON
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    // 2. Flash latency ayarla
    FLASH->ACR |= FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN;
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_5WS;

    // 3. PLL ayarları (8MHz HSE için)
    RCC->PLLCFGR = 0;
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;
    RCC->PLLCFGR |= (4 << RCC_PLLCFGR_PLLM_Pos);    // PLLM = 4 (8MHz HSE için)
    RCC->PLLCFGR |= (168 << RCC_PLLCFGR_PLLN_Pos);  // PLLN = 168
    RCC->PLLCFGR &= ~(2 << RCC_PLLCFGR_PLLP_Pos);   // PLLP = 2 (00b)
    RCC->PLLCFGR |= (7 << RCC_PLLCFGR_PLLQ_Pos);    // PLLQ = 7

    // 4. PLL ON
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    // 6. PLL'i sistem clock yap
    RCC->CFGR &= ~(3 << RCC_CFGR_SW_Pos);
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

}


void GPIO_Config(void){

	RCC->AHB1ENR |= (1<<3); // GPIOD Clock Enable

	for (int pin = 12; pin < 16; ++pin) {

		GPIOD->MODER &= ~(3 << (pin * 2));	// Pinleri output olarak ayarladık.
		GPIOD->MODER |= (1 << (pin*2));
	}

	GPIOD->OSPEEDR |= 0xFF000000;

}

void Timer_Init(){

	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // Bu satırda, TIM3’ün saat kaynağını açıyoruz. Saat kaynağı olmadan perifer çalışmaz.
	TIM3->PSC =8399;  // Prescaler (önbölücü). Timer’ın sayma hızını belirler.
	TIM3->ARR =9999; // Auto Reload Register. Sayaç bu değere ulaştığında sıfırlanır ve kesme oluşturulabilir.

	NVIC_EnableIRQ(TIM3_IRQn); // NVIC: Nested Vectored Interrupt Controller.Bu satır, TIM3 interrupt'ını aktif eder. Yani TIM3 bir kesme üretirse, işlemci bunu yakalayabilir.
	TIM3->DIER |=TIM_DIER_UIE; // Update interrupt enable anlamına gelir. Yani “sayaç sıfırlandığında interrupt oluşsun” demek.
	TIM3->CR1 |= TIM_CR1_CEN; // Control Register 1, timer’ı başlatır (enable).

}

void TIM3_IRQHandler(void) /*Bu fonksiyona "Interrupt Handler" denir. Timer kesme üretince buraya sıçrar.
							Eğer bu fonksiyon yazılmazsa, kesme geldiğinde mikrodenetleyici ne yapacağını bilemez.*/
{
	if(TIM3->SR & TIM_SR_UIF) // Birden fazla interrupt aynı fonksiyon ile tetiklenebilir. Bu yüzden if bloğu içinde hangi interruptdan geldiğini kontrol ediyoruz.
	{
		TIM3->SR &= ~TIM_SR_UIF; // Interrupt flagını sıfırlıyoruz.

		for (int pin = 12; pin < 16; ++pin) {

			GPIOD->ODR ^= (1<<pin);
		}

	}

}

int main(void){

	RCC_Config();

	GPIO_Config();

	Timer_Init();

	while(1){



	}
}
