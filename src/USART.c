#include "main.h"
#include "USART.h"
#include "PID.h"
#include "ADC.h"
#include "DAC.h"

#include <stdint.h>
#include <stdlib.h>
#include <math.h>  
#include <stdio.h>
#include <stddef.h>
#include <string.h>


// variable definitions:
char inputParameter_char;
// this varialbe specifies the interpreted meaning of the USART input:

struct inputValue_struct_type inputValue_struct;
char valueOfParameter_string[MAX_AMOUNT_INPUT_DIGITS];
enum USART_InputSpecifier_type inputSpecifier;
// help message:
// char* helpMessage = "Type 'P', 'I', 'D' for writing PID gain parameters, 'A' for reading analog input, 'S' for writing setpoint, 'W' for writing wind up limit, 'T' for writing time constant of low pass filter of derivative\n";
char* helpMessage = "";
enum boolean
{
	FALSE,
	TRUE
} helpMessageWasSent = FALSE;


// function definitions:
void usart2_init(void)
{
	// USART2_RX uses port A2 and USART2_TX uses port A3 if each are set to alternate function 7 (p.59 datasheet)
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;	// enable clock to USART2
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // enable clock to Port A
	GPIOA->MODER |= (GPIO_MODER_MODE2_1|GPIO_MODER_MODE3_1); // set PA2 and PA3 to alternate function 0b10
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


void usart2_printString(char *msg_string)
{
	size_t msg_string_len = strlen(msg_string); // get size of string pointed to by *msg_string
	// print each character of input string
	for (uint32_t i = 0; i < msg_string_len; i++)
		usart2_writeChar(msg_string[i]);
}


char usart2_readChar(void)
{
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


void printParameterWasSetMessage(char* parameterString)
{
	char message[40];
	sprintf(message, " was written to %s ", parameterString);
	usart2_printString(message);
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


void usart2_printPIDParameters(PIDController PID)
{
	usart2_printString("  Setpoint: ");
	usart2_printString(convertDoubleToString(PID.setpoint));
	usart2_printString("  Kp: ");
	usart2_printString(convertDoubleToString(PID.Kp));
	usart2_printString("  Ki: ");
	usart2_printString(convertDoubleToString(PID.Ki));
	usart2_printString("  Kd: ");
	usart2_printString(convertDoubleToString(PID.Kd));
	usart2_printString("  Integrator windup limit: ");
	usart2_printString(convertDoubleToString(PID.limMaxIntegrator));
	usart2_printString("  Low pass time constant: ");
	usart2_printString(convertDoubleToString(PID.tau));

	if (PID.signOfPID==1)
	{
		usart2_printString("  Error = setpoint - measurement\n");
	}
	else
	{
		usart2_printString("  Error = measurement - setpoint\n");
	}
}

//___________________________________________________________________________________
// interrupt request handlers:
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
				usart2_printString("Please type Kp value of PID: ");
				inputSpecifier = VALUE;
				helpMessageWasSent = FALSE;
				break;
			
			case 'I':
			case 'i':
				usart2_printString("Please type Ki value of PID: ");
				inputSpecifier = VALUE;
				helpMessageWasSent = FALSE;
				break;

			case 'D':
			case 'd':
				usart2_printString("Please type Kd value of PID: ");
				inputSpecifier = VALUE;
				helpMessageWasSent = FALSE;
				break;

			case 'S':
			case 's':
				usart2_printString("Please type setpoint of PID: ");
				inputSpecifier = VALUE;
				helpMessageWasSent = FALSE;
				break;

			case 'W':
			case 'w':
				usart2_printString("Please type wind up limit of PID: ");
				inputSpecifier = VALUE;
				helpMessageWasSent = FALSE;
				break;

			case 'T':
			case 't':
				usart2_printString("Please type time constant of lowpass for D: ");
				inputSpecifier = VALUE;
				helpMessageWasSent = FALSE;
				break;

			case 'A':
			case 'a':
				usart2_printString(convertDoubleToString(ADC1_read()));
				usart2_printString("\n");
				helpMessageWasSent = FALSE;
				break;

			case '-':
				pid.signOfPID = -pid.signOfPID;
				usart2_printString("Sign of PID was inverted! ");
				helpMessageWasSent = FALSE;

			default:
				if (helpMessageWasSent == FALSE)
				{
					usart2_printString(helpMessage);
					usart2_printPIDParameters(pid);
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
			char* cptr = 0;
			double finalValue = strtod(inputChar_part, cptr);
			switch (inputParameter_char)
			{
				case 'P':
				case 'p':
					pid.Kp = finalValue;
					printParameterWasSetMessage("Kp");
					break;

				case 'I':
				case 'i':
					pid.Ki = finalValue;
					printParameterWasSetMessage("Ki");
					break;

				case 'D':
				case 'd':
					pid.Kd = finalValue;
					printParameterWasSetMessage("Kd");
					break;

				case 'S':
				case 's':
					pid.setpoint = finalValue;
					printParameterWasSetMessage("Setpoint");
					break;

				case 'W':
				case 'w':
					pid.limMaxIntegrator = finalValue;
					printParameterWasSetMessage("Wind up limit");
					break;

				case 'T':
				case 't':
					pid.tau = finalValue;
					printParameterWasSetMessage("tau");
					break;
	
				default:
					usart2_printString("Oopsie Daisey! Something went wrong! :(\n");
			
			
			}
			resetInputValueStruct(&inputValue_struct, MAX_AMOUNT_INPUT_DIGITS);
			inputSpecifier = PARAMETER;	
			//usart2_printPIDParameters(PIDparams);

		}

		// value input cancelled if escape is pressed:
		else if (c == 27)
		{
			resetInputValueStruct(&inputValue_struct, MAX_AMOUNT_INPUT_DIGITS);
			inputSpecifier = PARAMETER;
			usart2_printString(" Input cancelled! ");
			//usart2_printPIDParameters(PIDparams);
		}
	}
}
