#include "main.h"
#include "DAC.h"

void DAC1_init()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // clock to port A
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE4, GPIO_MODER_MODE4_0 | GPIO_MODER_MODE4_1); // set PA4 to analog mode (MODER1 = 0b11)
	RCC->APB1ENR |= RCC_APB1ENR_DACEN; // clock to DAC
	DAC->CR |= DAC_CR_EN1; // enable DAC
	// DAC in normal mode with output buffer by default
}

void DAC1_writeOutput(float fraction)
{
	uint32_t regData = fraction * 0xFFF; // convert input signal to DAC register data
	DAC->DHR12R1 = regData; // write to 12 bit right alligned DAC output data
}



void PWM_init(uint32_t dutyCycle)
{
	// init PA5
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; 	// clock to Port A
	GPIOA->MODER |= GPIO_MODER_MODE5_1; 	// set PA5 to alternate
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL5_0; 	// set PA5 to AF1 => TIM2 channel 1
	// init Timer 2
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; 	// clock to TIM2
	TIM2->CCER |= TIM_CCER_CC1E; 			// enable Timer 2 channel 1
	TIM2->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1); // OC1M = 110 => PWM mode 1
	// PWM mode 1: In upcounting, channel 1 is active as long as TIMx_CNT<TIMx_CCR1
	// else inactive. In downcounting, channel 1 is inactive (OC1REF=‘0) as long as
	// TIMx_CNT>TIMx_CCR1 else active (OC1REF=1)
	// timer channel configured as output by default
	TIM2->PSC = 0ul; 			// no prescaler
	TIM2->ARR = 99ul;			// reload value = 99
	TIM2->CNT = 0ul; 			// clear counter
	TIM2->CR1 |= TIM_CR1_CEN; 	// enable counter
}


void PWM_setDutyCycle(uint32_t dutyCycle)
{
	TIM2->CCR1 = dutyCycle;		// write duty cycle
}
