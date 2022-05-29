#if !defined(USART_H)
#define USART_H

#include "main.h"
#include "PID.h"
#include <stdint.h>
#include <stddef.h>

// global variable declarations:
extern char inputParameter_char;
extern struct inputValue_struct_type
{
	int8_t index;
	char inputChar[MAX_AMOUNT_INPUT_DIGITS];
} inputValue_struct;
// extern struct inputValue_struct_type inputValue_struct;
extern char valueOfParameter_string[MAX_AMOUNT_INPUT_DIGITS];
extern enum USART_InputSpecifier_type
{
	PARAMETER = 1,	// the USART input describes the parameter
	VALUE = 2		// the USART input describes the value to which the parameter will be set
} inputSpecifier;
// extern enum USART_InputSpecifier_type inputSpecifier;
extern char* helpMessage;
extern enum boolean helpMessageWasSent;

// function declarations:
void usart2_init(void);
void usart2_writeChar(char msg_char);
void usart2_printString(char *msg_string);
char usart2_readChar(void);
void resetInputValueStruct(struct inputValue_struct_type* iValStr_pointer, size_t stringSize);
void printParameterWasSetMessage(char* parameterString);
char *convertDoubleToString(double input);
void usart2_printPIDParameters(PIDController PID);
void USART2_IRQHandler(void);

#endif // USART_H
