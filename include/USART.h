#if !defined(USART_H)
#define USART_H

#include "main.h"
#include "PID.h"
#include <stdint.h>
#include <stddef.h>

// variable declarations:
extern char inputParameter_char;
extern struct inputValue_struct_type inputValue_struct;
extern char valueOfParameter_string[MAX_AMOUNT_INPUT_DIGITS];
extern enum inputSpecifier;
extern const char* helpMessage;
extern enum boolean helpMessageWasSent;


// function prototypes
void usart2_init(void);
void usart2_writeChar(char msg_char);
void usart2_writeString(char *msg_string);
char usart2_readChar(void);
void resetInputValueStruct(struct inputValue_struct_type* iValStr_pointer, size_t stringSize);
void printParameterWasSetMessage(char* parameterString);
char *convertDoubleToString(double input);
void usart2_writePIDParameters(PIDController PID);
void USART2_IRQHandler(void);

#endif // USART_H
