#include "main.h"

void ADC1_init()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; 	// clock to Port A
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE1, GPIO_MODER_MODE1_0 | GPIO_MODER_MODE1_1); // set PA1 to analog mode (MODER1 = 0b11)
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // enable clock to ADC1
	MODIFY_REG(ADC1->SQR3, ADC_SQR3_SQ1, ADC_SQR3_SQ1_0); // first ADC channel to scan is Channel 1 (PA1)
	ADC1->CR2 |= ADC_CR2_ADON; // enable ADC1
}

float ADC1_read()
{
 	ADC1->CR2 |= ADC_CR2_SWSTART; // start conversion
	while (!(ADC1->SR & ADC_SR_EOC)); // wait until end of conversion is reached	
	return (ADC1->DR / 4095.0f) * ADC_REFVOLT;// read data register
}
