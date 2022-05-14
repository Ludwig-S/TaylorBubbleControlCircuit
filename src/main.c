#include "stm32f4xx.h"                  // Device header
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/*
ToDo:
- Test analog input and PID
- PID output to DAC; alternatively PID to PWM: change PWM pin, check PWM frequency for servo, higher PWM duty cycle resolution
*/



/*
_______________________________________________
USART settings: 
Baudrate: 9600 BdpS
8 data bits
1 stop bit
no parity, no flow control
_______________________________________________
_______________________________________________
GPIO pin overview:

PA1		Analog Input
PA2		USART2_RX
PA3		USART2_TX
PA15	PWM output => AF1 (TIM2_CH1/ TIM2_ETR)

Important:
Do not overwrite PA13 and PA14, because the uC uses these pins for flashing!
______________________________________________
*/


//___________
// MACROS:
#define MAX_AMOUNT_INPUT_DIGITS 10
#define ADC_REFVOLT 3.3
#define ADC_SAMPLEPERIOD (15/90e6) // ADC stabilisation time (3 cycles) + conversion time (12 cycles)
// ADC1 gets clock from APB2 (90MHz)

// REGISTER EDITING MACROS:
#define SET_BIT(REG, BIT)     ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)    ((REG) & (BIT))
#define CLEAR_REG(REG)        ((REG) = (0x0))
#define WRITE_REG(REG, VAL)   ((REG) = (VAL))
#define READ_REG(REG)         ((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))
#define POSITION_VAL(VAL)     (__CLZ(__RBIT(VAL)))

//___________________
// GLOBAL VARIABLES:
char inputParameter_char;
struct inputValue_struct_type
{
	int8_t index;
	char inputChar[MAX_AMOUNT_INPUT_DIGITS];
} inputValue_struct;
char valueOfParameter_string[MAX_AMOUNT_INPUT_DIGITS];
enum boolean
{
	FALSE,
	TRUE
} helpMessageWasSent = FALSE;
// this varialbe specifies the interpreted meaning of the USART input:
enum USART_InputSpecifier_type
{
	PARAMETER = 1,	// the USART input describes the parameter
	VALUE = 2		// the USART input describes the value to which the parameter will be set
} inputSpecifier;
struct PIDparams_type
{
	double P;
	double I;
	double D;
	double S; // setpoint
	double W; // wind up limit
	int8_t E; // sign of error
} PIDparams;
const char* helpMessage = "Type 'P' OR 'I' OR 'D' for writing PID parameters OR 'S' for writing setpoint OR 'W' for writing wind up limit OR 'A' for reading analog input\n";


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
	NVIC_EnableIRQ(USART2_IRQn); // enable USART2 interrupt in NVIC
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
	if (USART2->SR & USART_SR_RXNE){ // if data is ready to be read
		return USART2->DR; // read received char and return it
	}
	else return '\0';
}


void resetInputValueStruct(struct inputValue_struct_type* iValStr_pointer, size_t stringSize)
{
	size_t i = 0;
	for (i = 0; i < stringSize; i++)
	{
		iValStr_pointer->inputChar[i] = '/0';
	}
	iValStr_pointer->index = 0;
}


void printParameterWasSetMessage(char parameterChar)
{
	char message[40];
	sprintf(message, " was written to %c ", parameterChar);
	usart2_writeString(message);
}


// similar to: https://stackoverflow.com/questions/905928/using-floats-with-sprintf-in-embedded-c:
char *convertDoubleToString(double input)
{
	static char str[50];
	// reset static char array:
	size_t sizeOfStr = sizeof(str)/sizeof(str[0]);
	size_t i;
	for (i = 0; i < sizeOfStr; i++)
	{
		str[i] = 0;
	}
	// building the output string:
	char *tmpSign = (input < 0) ? "-" : "";
	double tmpVal = (input < 0) ? -input : input;

	int32_t tmpInt1 = tmpVal;                  // Get the integer (678).
	double tmpFrac = tmpVal - tmpInt1;      // Get fraction (0.0123).
	int32_t tmpInt2 = trunc(tmpFrac * 10000);  // Turn into integer (123).

	// Print as parts, note that you need 0-padding for fractional bit.
	sprintf (str, "%s%ld.%04ld", tmpSign, tmpInt1, tmpInt2);
	return str;
}


void usart2_writePIDParameters(struct PIDparams_type PID)
{
	usart2_writeString("P: ");
	usart2_writeString(convertDoubleToString(PID.P));
	usart2_writeString("  I: ");
	usart2_writeString(convertDoubleToString(PID.I));
	usart2_writeString("  D: ");
	usart2_writeString(convertDoubleToString(PID.D));
	usart2_writeString("  S: ");
	usart2_writeString(convertDoubleToString(PID.S));
	usart2_writeString("  W: ");
	usart2_writeString(convertDoubleToString(PID.W));
	if (PID.E==1)
	{
		usart2_writeString("  Error = setpoint - actual value\n");
	}
	else if (PID.E==1)
	{
		usart2_writeString("  Error = actual value - setpoint\n");
	}
}


// interrupt request handler:
void USART2_IRQHandler(void)
{
	char c = usart2_readChar();

	if (inputSpecifier == PARAMETER)
	{
		inputParameter_char = c;
		switch (inputParameter_char)
		{
			case 'P':
			case 'p':
				usart2_writeString("Please type P value of PID: ");
				inputSpecifier = VALUE;
				helpMessageWasSent = FALSE;
				
				//strtod();
				break;
			
			case 'I':
			case 'i':
				usart2_writeString("Please type I value of PID: ");
				inputSpecifier = VALUE;
				helpMessageWasSent = FALSE;
				break;

			case 'D':
			case 'd':
				usart2_writeString("Please type D value of PID: ");
				inputSpecifier = VALUE;
				helpMessageWasSent = FALSE;
				break;

			case 'S':
			case 's':
				usart2_writeString("Please type setpoint of PID: ");
				inputSpecifier = VALUE;
				helpMessageWasSent = FALSE;
				break;

			case 'W':
			case 'w':
				usart2_writeString("Please type wind up limit of PID: ");
				inputSpecifier = VALUE;
				helpMessageWasSent = FALSE;
				break;

			case 'A':
			case 'a':
				// TODO: PRINT ANALOG INPUT			
				helpMessageWasSent = FALSE;
				break;

			case '-':
				PIDparams.E = -PIDparams.E;
				usart2_writeString("Sign of Error was inverted! ");
				helpMessageWasSent = FALSE;

			default:
				if (helpMessageWasSent == FALSE)
				{
					usart2_writeString(helpMessage);
					usart2_writePIDParameters(PIDparams);
				}				
				helpMessageWasSent = TRUE;
		}
	}
	else if (inputSpecifier == VALUE)
	{

		// check if inputChar is digit or period and input buffer not full:
		if (inputValue_struct.index < MAX_AMOUNT_INPUT_DIGITS && ((c >= 48 && c <= 57)||c==46))
		{
			// process input digit:
			inputValue_struct.inputChar[inputValue_struct.index] = c;
			inputValue_struct.index++;
			usart2_writeChar(c);
		}		

		// input stream of digits finished when enter pressed or buffer full
		else if ((c == 13) || (inputValue_struct.index >= MAX_AMOUNT_INPUT_DIGITS))
		{
			char* inputChar_part = calloc(inputValue_struct.index, sizeof(char));
			size_t i;
			for (i = 0; i < inputValue_struct.index; i++)
			{
				inputChar_part[i] = inputValue_struct.inputChar[i];
			}
			char* cptr;
			double finalValue = strtod(inputChar_part, cptr);
			switch (inputParameter_char)
			{
				case 'P':
				case 'p':
					PIDparams.P = finalValue;
					printParameterWasSetMessage('P');
					break;

				case 'I':
				case 'i':
					PIDparams.I = finalValue;
					printParameterWasSetMessage('I');
					break;

				case 'D':
				case 'd':
					PIDparams.D = finalValue;
					printParameterWasSetMessage('D');
					break;

				case 'S':
				case 's':
					PIDparams.S = finalValue;
					printParameterWasSetMessage('S');
					break;

				case 'W':
				case 'w':
					PIDparams.W = finalValue;
					printParameterWasSetMessage('W');
					break;


				default:
					usart2_writeString("Oopsie Daisey! Something went wrong! :(\n");
			
			
			}
			resetInputValueStruct(&inputValue_struct, MAX_AMOUNT_INPUT_DIGITS);
			inputSpecifier = PARAMETER;	
			usart2_writePIDParameters(PIDparams);

		}

		// value input cancelled if escape is pressed:
		else if (c == 27)
		{
			resetInputValueStruct(&inputValue_struct, MAX_AMOUNT_INPUT_DIGITS);
			inputSpecifier = PARAMETER;
			usart2_writeString(" Input cancelled! ");
			usart2_writePIDParameters(PIDparams);
		}
	}
	// ToDo: reset pending bit
}

void ADC1_init()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; 	// clock to Port A
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE1, GPIO_MODER_MODE1_0 || GPIO_MODER_MODE1_1); // set PA1 to analog mode (MODER1 = 0b11)
	// GPIOA->MODER |= (GPIO_MODER_MODE1_0 || GPIO_MODER_MODE1_1);
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // enable clock to ADC1
	MODIFY_REG(ADC1->SQR3, ADC_SQR3_SQ1, ADC_SQR3_SQ1_0); // first ADC channel to scan is Channel 1 (PA1)
	// ADC1->CR2 |= ADC_CR2_CONT; // enable continuous mode
	ADC1->CR2 |= ADC_CR2_ADON; // enable ADC1
	// ADC1->CR1 |= ADC_CR1_SCAN; // select scan mode
}

uint32_t ADC1_read()
{
	ADC1->CR2 |= ADC_CR2_SWSTART; // start conversion
	while (!(ADC1->SR & ADC_SR_EOC)) // wait until end of conversion is reached	
	return ADC1->DR;// read data register		
}

int main()
{
	inputSpecifier = PARAMETER;
	resetInputValueStruct(&inputValue_struct, MAX_AMOUNT_INPUT_DIGITS);

	usart2_init();	
	usart2_writeString(helpMessage);

	while (1)
	{
		double error[2];
		double actualValue = ADC1_read(); // read new actual value
		error[1] = error[0]; // shift old error
		error[0] = (PIDparams.S - actualValue) * PIDparams.E; // calculate new error
		double pOutput = PIDparams.P * error[0];
		double dOutput = PIDparams.D * (error[0] - error[1]) / ADC_SAMPLEPERIOD;
		// calculate integer part of output
		static double iOutput; // static to keep value between loop increments
		if (iOutput < PIDparams.W)
		{
			iOutput = PIDparams.I * error[0] * ADC_SAMPLEPERIOD;
		}
		else
		{
			iOutput = PIDparams.W;
		}
		double output = pOutput + iOutput + dOutput;
		// ToDo: DAC
	}
}
