#include "main.h"

void RCC_Config(void){

	RCC-> CR &=~(1<<0);  // HSI OFF
	RCC-> CR |= 1<<16;   // HSE ON
	while(!(RCC->CR & (1<<17))); // HSE aktif olmasını bekledik.
	RCC-> CR |= 1<<19;
	//RCC->PLLCFGR = 0x00000000; // PLLCFGR tüm bitleri sıfırladık. // PPL P =2
	RCC ->PLLCFGR |=(1<<22); // PLL Soruce HSE
	RCC-> PLLCFGR |= (4<<0); // PLLM değeri 4
	RCC -> PLLCFGR |= (168<<6); // PLLN değeri 168
	RCC -> CR |= (1<<24);   // PLL Açıldı.
	while(!(RCC->CR & (1<<25))); // PLL aktif olana kadar bekle

	RCC -> CFGR &= ~(1<<0);
	RCC -> CFGR |=(1<<1); // Sistem clock PLL
	while(!(RCC->CFGR & (1<<1))); // Sistem clock u PLL clock olarak seçtik.
}

void GPIO_Config(void){

	RCC->AHB1ENR |= (1<<3); // GPIOD Clock Enable

	GPIOD->MODER |=(1<<24);  // GPIOD 12.Pini output oldu.
	GPIOD->MODER &= ~(1<<25);

	GPIOD->MODER |=(1<<26);  // GPIOD 12.Pini output oldu.
	GPIOD->MODER &= ~(1<<27);

	GPIOD->MODER |=(1<<28);  // GPIOD 12.Pini output oldu.
	GPIOD->MODER &= ~(1<<29);

	GPIOD->MODER |=(1<<30);  // GPIOD 12.Pini output oldu.
	GPIOD->MODER &= ~(1<<31);

	GPIOD->OSPEEDR |= 0xFF000000;


}


int main(void){

	RCC_Config();

	GPIO_Config();

	while(1){

		GPIOD->ODR |=(1<<12);	// 12.Pin SET
		GPIOD->ODR |=(1<<13);	// 13.Pin SET
		GPIOD->ODR |=(1<<14); 	// 14.Pin SET
		GPIOD->ODR |=(1<<15);	// 15.Pin SET


	}
}

