#include "main.h"

uint16_t adc_value;


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

}

void GPIO_Config(void){

	RCC->AHB1ENR |=RCC_AHB1ENR_GPIOAEN; // GPIOA Clock Enable
	GPIOA->MODER |= (3<<0); 			// Pin 0 Analog
	GPIOA->OSPEEDR |= (3<<0);			// Pin 0 100MHz
}

void ADC_Config(void){

	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // ADC1 Clock Enable

	ADC1->CR1 &= ~(ADC_CR1_RES_0 | ADC_CR1_RES_1); // RES=00 12-bit resolation
	ADC1->CR2 |= ADC_CR2_ADON;					   // ADC Enable
	ADC1->SMPR2 |= (ADC_SMPR2_SMP0_0 | ADC_SMPR2_SMP0_1 | ADC_SMPR2_SMP0_2 ); // 111 480 cycles
	ADC->CCR &= ~(ADC_CCR_ADCPRE_0 | ADC_CCR_ADCPRE_1); // bitleri temizledik 00 oldu.
	ADC->CCR &= ~(ADC_CCR_ADCPRE_0);	 // Div 4

}

uint16_t Read_ADC(){

	uint16_t value;

	ADC1->CR2 |= ADC_CR2_SWSTART; // Yazılımsal olarak adc okuma çevrimini başlatır.

	while(!(ADC1->SR & ADC_SR_EOC)); // Bayrağı kontrol ediyoruz.

	value = ADC1->DR; // DR registerında okuma yapıyoruz.

	return value;
}


int main(void){

	RCC_Config();

	GPIO_Config();

	ADC_Config();

	while(1){

		adc_value = Read_ADC();

	}
}
