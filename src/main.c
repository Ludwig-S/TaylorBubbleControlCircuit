#include "main.h"
#include "PID.h"
#include "USART.h"
#include "ADC.h"
#include "DAC.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
ToDo:
- convert data input of tau to low pass frequency
- make subfiles
- test PID (in process)

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
PA4		DAC_OUT1

PA15	PWM output => AF1 (TIM2_CH1/ TIM2_ETR)

Important:
Do not overwrite PA13 and PA14, because the uC uses these pins for flashing!

Active IRQHandlers:
- USART2_IRQHandler()

*/

// GLOBAL INITIALISATIONS:
PIDController pid = {PID_SETPOINT, PID_SIGN, PID_KP, PID_KI, PID_KD,
						PID_TAU,
						PID_OUT_LIM_MIN, PID_OUT_LIM_MAX,
			PID_INT_LIM_MIN, PID_INT_LIM_MAX,
						ADC_SAMPLEPERIOD };


int main()
{
	inputSpecifier = PARAMETER;
	resetInputValueStruct(&inputValue_struct, MAX_AMOUNT_INPUT_DIGITS);

	usart2_init();	
	ADC1_init();
	DAC1_init();
	PIDController_Init(&pid);

	usart2_writeString(helpMessage);
	float outputSignal_float;

	while (1)
	{
		float measurement = ADC1_read();
		outputSignal_float = PIDController_Update(&pid, measurement);
		DAC1_writeOutput(outputSignal_float / pid.limMaxOut);
	}
}
