#include "main.h"

#define VREFINT 1.21      /* Internal Reference Voltage */
#define ADCMAX 4095.0
#define V25 0.76          /* 25 derecedeki sensor voltajı */
#define AVG_Slope 0.0025  /* 2.5mV/C */

uint16_t adc_value[2];  // 0: TS, 1: VREFINT

uint8_t flag;
double VrefInt = 0;
double VTmpSensor = 0;
double Temp = 0;

void RCC_Config(void){
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    NVIC_EnableIRQ(DMA2_Stream0_IRQn); // Interruptı görmesi için aktifledik.
}

void ADC_Config(void){
    ADC1->CR1 = 0;
    ADC1->CR1 |= ADC_CR1_SCAN;  // Çoklu kanal için scan modu

    ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_CONT | ADC_CR2_DMA | ADC_CR2_DDS | ADC_CR2_EOCS;

    // 2 dönüşüm için, SQR1[23:20] = 1 -> 2 conversions
    ADC1->SQR1 = (1 << 20);

    // SQR3: sıra ile kanal 16 ve 17
    ADC1->SQR3 = (16) | (17 << 5);

    // Kanal 16 ve 17 için örnekleme süresi (480 cycles)
    ADC1->SMPR1 |= (7 << (3 * (16 - 10))) | (7 << (3 * (17 - 10)));

    // ADC prescaler div4
    ADC->CCR &= ~ADC_CCR_ADCPRE;
    ADC->CCR |= ADC_CCR_ADCPRE_0;

    // TSVREF Sıcaklık sensörünü aktif et
    ADC->CCR |= ADC_CCR_TSVREFE;
}

void DMA_Config(void){
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;   // Disable DMA stream
    while(DMA2_Stream0->CR & DMA_SxCR_EN);  // Disable olana kadar bekle

    DMA2_Stream0->PAR = (uint32_t)&ADC1->DR;
    DMA2_Stream0->M0AR = (uint32_t)&adc_value[0];
    DMA2_Stream0->NDTR = 2;  // 2 veri

    DMA2_Stream0->CR = 0;
    DMA2_Stream0->CR |= DMA_SxCR_TCIE;

    DMA2_Stream0->CR |= DMA_SxCR_MINC           // Memory increment
                     | DMA_SxCR_CIRC           // Circular mode
                     | (0 << DMA_SxCR_DIR_Pos) // Peripheral-to-memory
                     | (2 << DMA_SxCR_PL_Pos)  // Priority High
                     | (1 << DMA_SxCR_MSIZE_Pos) // Memory data size 16 bit
                     | (1 << DMA_SxCR_PSIZE_Pos); // Peripheral data size 16 bit

    DMA2_Stream0->CR &= ~DMA_SxCR_CHSEL;  // Channel 0 seçili

    DMA2_Stream0->CR |= DMA_SxCR_EN;     // Enable DMA stream
}



void DMA2_Stream0_IRQHandler(void){

	if(DMA2->LISR & DMA_LISR_TCIF0){ // flag kontrol et

		flag=1;

        DMA2->LIFCR |= DMA_LIFCR_CTCIF0; // flag sıfırla

	}

}


int main(void){
    RCC_Config();
    ADC_Config();
    DMA_Config();

    ADC1->CR2 |= ADC_CR2_SWSTART;

    while(1){

    	if(flag){

            VrefInt = (VREFINT * ADCMAX) / adc_value[1];

            VTmpSensor = (VrefInt * adc_value[0]) / ADCMAX;

            Temp = ((VTmpSensor - V25) / AVG_Slope) + 25;

    	}


    }
}

