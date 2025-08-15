#include "main.h"

uint32_t adc_value[1];
uint32_t adc;
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

	RCC->AHB1ENR |=RCC_AHB1ENR_GPIOAEN;

	GPIOA->MODER &= ~(GPIO_MODER_MODE0_0 | GPIO_MODER_MODE0_1);// 0. Portu temizledik.
	GPIOA->MODER |= GPIO_MODER_MODE0_0 | GPIO_MODER_MODE0_1 ; // Analog pin modunu aktif ettik. 11
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0_0 | GPIO_OSPEEDER_OSPEEDR0_1);
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0_0 | GPIO_OSPEEDER_OSPEEDR0_1;  // 100MHz


}

void ADC_Config(void){

	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;  // ADC1 Clock Enable
	ADC1->CR1 |= ADC_CR1_SCAN;			// ADC SCAN Mode enable. Birden fazla adc okunurken enable edilir.
	ADC1->CR1 &= ~(ADC_CR1_RES_0 | ADC_CR1_RES_1); //Resolutation 12-Bit
	ADC1->CR2 |= ADC_CR2_ADON;                      // ADC Enable
	ADC1->CR2 |= ADC_CR2_CONT;                      // ADC Continues Mode Enable
	ADC1->CR2 |= ADC_CR2_DMA;						// DMA Mode Enable
	ADC1->CR2 |= ADC_CR2_DDS;						// DDS Enable
	ADC1->CR2 |= ADC_CR2_EOCS;						// EOCS Enable
	ADC1->CR2 |= ADC_CR2_SWSTART;
	ADC1->SQR1 |= (0<<20);
	ADC1->SQR3 &= ~ADC_SQR3_SQ1_0;
	ADC1->SMPR2 &= ~(ADC_SMPR2_SMP0_0 | ADC_SMPR2_SMP0_1 | ADC_SMPR2_SMP0_2); // temizledik
	ADC1->SMPR2 |= ADC_SMPR2_SMP0_0 | ADC_SMPR2_SMP0_1 | ADC_SMPR2_SMP0_2; // 480 cycles
	ADC->CCR &= ~(ADC_CCR_ADCPRE_0 | ADC_CCR_ADCPRE_1); // bitleri temizledik 00 oldu.
	ADC->CCR &= ~(ADC_CCR_ADCPRE_0);	 // Div 4

}

void DMA_Config(void){

	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;  // DMA2 Clock Enable

	DMA2_Stream4->CR &= ~DMA_SxCR_EN;
	while((DMA2_Stream4->CR & (1<<0)) == 1 ); // Stream4 0 olana kadar bekle.

	DMA2_Stream4->PAR = (uint32_t) &ADC1->DR;

	DMA2_Stream4->M0AR = (uint32_t) &adc_value;

	DMA2_Stream4->NDTR = 1;

	DMA2_Stream4->CR &= ~(DMA_SxCR_CHSEL_0 | DMA_SxCR_CHSEL_1 | DMA_SxCR_CHSEL_2);  // Channel 0 Selected

	DMA2_Stream4->CR &= ~DMA_SxCR_DIR;			// Peripheral to memory
	DMA2_Stream4->CR |= DMA_SxCR_CIRC;			// Circuler Mode Enable
	DMA2_Stream4->CR &= ~DMA_SxCR_PINC;			// Periperial address fixed
	DMA2_Stream4->CR |= DMA_SxCR_MINC;			// Memory address incremented

    // Data sizes: 32-bit
    DMA2_Stream4->CR &= ~(DMA_SxCR_PSIZE | DMA_SxCR_MSIZE);
    DMA2_Stream4->CR |= DMA_SxCR_PSIZE_1 | DMA_SxCR_MSIZE_1;

	DMA2_Stream4->CR |= DMA_SxCR_PL_0 | DMA_SxCR_PL_1;    // Priority level 11

	  DMA2_Stream4->FCR &= ~(DMA_SxFCR_DMDIS);	// FIFO: disable
	DMA2_Stream4->CR |= DMA_SxCR_EN;			// Enable DMA stream
}



int main(void){

	RCC_Config();
	GPIO_Config();
	ADC_Config();
	DMA_Config();
	ADC1->CR2 |= ADC_CR2_SWSTART;

	while(1){


		adc = adc_value[0];

	}


}
