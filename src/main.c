#include "stm32f4xx.h"                  // Device header
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/*
ToDo:
- change PWM pin, check PWM frequency for servo, higher PWM duty cycle resolution
- set up PID parameters and standard text for USART communication
- analog input

*/

//___________
// MACROS:
#define maxAmountOfInputNumbers 10


//___________________
// GLOBAL VARIABLES:

char command_char;
struct inputString_struct_type
{
	int8_t index;
	char inputChar[maxAmountOfInputNumbers];
} inputString_struct;
char valueOfParameter_string[maxAmountOfInputNumbers];

enum boolean
{
	FALSE,
	TRUE
} helpMessageWasSent = FALSE;


// this varialbe specifies the interpreted aim of the USART input:
enum
{
	PARAMETER = 1,	// the USART input describes the parameter
	VALUE = 2		// the USART input describes the value to which the parameter will be set
} USART_InputSpecifier;


struct PIDparams_type
{
	int32_t P;
	int32_t I;
	int32_t D;
	int32_t S;
} PIDparams;


const char* helpMessage = "Type\n"
						"P for setting P of PID\n"
						"I for setting I of PID\n"
						"D for setting I of PID\n"
						"S for setting of setpoint of PID\n"
						"A for reading analog input\n";


//_____________
// FUNCTIONS:
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

void usart2_init(void)
{
	// USART2_RX uses port A2 and USART2_TX uses port A3 if each are set to alternate function 7 (p.59 datasheet)
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;	// enable clock to USART2
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // enable clock to Port A
	GPIOA->MODER |= (GPIO_MODER_MODE2_1|GPIO_MODER_MODE3_1); // set PA2 and PA3 to alternate function
	GPIOA->AFR[0] |= 
		(GPIO_AFRL_AFRL2_2 | GPIO_AFRL_AFRL2_1 | GPIO_AFRL_AFRL2_0 | GPIO_AFRL_AFRL3_2 
		| GPIO_AFRL_AFRL3_1 | GPIO_AFRL_AFRL3_0); // set PA2 to AF7 (USART2_TX) and PA3 to AF7 (USART2_RX)
	USART2->CR1 |= (USART_CR1_TE|USART_CR1_RE);// enable transmitter and receiver
	// Baud rate Infos can be found on page 808 of reference manual)
	USART2->BRR = 0x0683;// set baud rate to 9600 (with 16 MHz clock)
	// enable USART2 interrupt:
	USART2->CR1 |= USART_CR1_RXNEIE; // enable receive interrupt
	NVIC_EnableIRQ(USART2_IRQn); // enabl USART2 interrupt in NVIC
	USART2->CR1 |= USART_CR1_UE;// enable USART2 module
}


void usart2_writeChar(char msg_char)
{
	while(!(USART2->SR & USART_SR_TXE)); // wait while transmit data register is full (register empty => TXE=1)
	USART2->DR = (uint8_t) msg_char; // write char in data register
}


void usart2_writeString(char *msg_string)
{
	size_t msg_string_len = strlen(msg_string); // get size of string pointed to by *msg_string
	// print each character of input string
	for (uint32_t i = 0; i < msg_string_len; i++)
		usart2_writeChar(msg_string[i]);
}

char usart2_readChar(void){
	if(USART2->SR & USART_SR_RXNE){ // if data is ready to be read
		return USART2->DR; // read received char and return it
	}
	else return '\0';
}


void processReceivedCommandChar(char c)
{
	switch (c)
	{
	case 'p':
		usart2_writeString("Please type value of P of PID:\n");
		USART_InputSpecifier = VALUE;
		helpMessageWasSent = FALSE;
		
		//strtod();
		break;

	case 'i':
		usart2_writeString("Please type value of I of PID:\n");
		helpMessageWasSent = FALSE;
		break;

	case 'd':
		usart2_writeString("Please type value of D of PID:\n");
		helpMessageWasSent = FALSE;
		break;

	case 's':
		usart2_writeString("Please type value of setpoint of PID:\n");
		helpMessageWasSent = FALSE;
		break;

	case 'a':
		
		helpMessageWasSent = FALSE;
		break;

	default:
		usart2_writeString(helpMessage);
		helpMessageWasSent = TRUE;

	}
};


void resetInputStringStruct(struct inputString_struct_type* iStrStr_Pointer )
{
	size_t i = 0;
	for (i = 0; i < maxAmountOfInputNumbers; i++)
	{
		valueOfParameter_string[i] = '/0';
	}
	iStrStr_Pointer->index = 0;
}



// interrupt request handler:
void USART2_IRQHandler(void)
{
	if (USART_InputSpecifier == PARAMETER)
	{
		command_char = usart2_readChar();
	}
	else if (USART_InputSpecifier == VALUE)
	{
		/* code */
	}
	

}


void main()
{
	USART_InputSpecifier = PARAMETER;
	resetInputStringStruct(&inputString_struct);

	usart2_init();	
	usart2_writeString(helpMessage);


}