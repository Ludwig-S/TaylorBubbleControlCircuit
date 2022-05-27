#include "stm32f4xx.h"                  // Device header
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/*
ToDo:
- convert data input of tau to low pass frequency
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
______________________________________________
*/


//___________
// MACROS:
#define MAX_AMOUNT_INPUT_DIGITS 10
#define ADC_REFVOLT 3.3f
#define ADC_SAMPLEPERIOD (15/90e6) // ADC stabilisation time (3 cycles) + conversion time (12 cycles) = 15 cycles
									// ADC1 gets clock from APB2 (90MHz)
/* Initial controller parameters */
#define PID_SIGN 1
#define PID_SETPOINT 1.5f
#define PID_KP  2.0f
#define PID_KI  0.5f
#define PID_KD  0.25f

#define PID_TAU 0.02f // Tau = RC, f_lowpass = 1/(2*pi*Tau)

#define PID_OUT_LIM_MIN  0
#define PID_OUT_LIM_MAX  3.3f

#define PID_INT_LIM_MIN  0
#define PID_INT_LIM_MAX  5.0f

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
// PID
typedef struct {

	// PID parameters
	float setpoint;
	uint8_t signOfPID;

	/* Controller gains */
	float Kp;
	float Ki;
	float Kd;

	/* Derivative low-pass filter time constant */
	float tau;

	/* Output limits */
	float limMinOut;
	float limMaxOut;
	
	/* Integrator limits */
	float limMinIntegrator;
	float limMaxIntegrator;

	/* Sample time (in seconds) */
	float T;

	/* Controller "memory" */
	float integrator;
	float prevError;			/* Required for integrator */
	float differentiator;
	float prevMeasurement;		/* Required for differentiator */

	/* Controller output */
	float out;

} PIDController;

PIDController pid = {PID_SETPOINT, PID_SIGN, PID_KP, PID_KI, PID_KD,
						PID_TAU,
						PID_OUT_LIM_MIN, PID_OUT_LIM_MAX,
			PID_INT_LIM_MIN, PID_INT_LIM_MAX,
						ADC_SAMPLEPERIOD };

char inputParameter_char;
struct inputValue_struct_type
{
	int8_t index;
	char inputChar[MAX_AMOUNT_INPUT_DIGITS];
} inputValue_struct;
char valueOfParameter_string[MAX_AMOUNT_INPUT_DIGITS];

// communication:
// this varialbe specifies the interpreted meaning of the USART input:
enum USART_InputSpecifier_type
{
	PARAMETER = 1,	// the USART input describes the parameter
	VALUE = 2		// the USART input describes the value to which the parameter will be set
} inputSpecifier;

const char* helpMessage = "Type 'P', 'I', 'D' for writing PID gain parameters, 'A' for reading analog input, 'S' for writing setpoint, 'W' for writing wind up limit, 'T' for writing time constant of low pass filter of derivative\n";

enum boolean
{
	FALSE,
	TRUE
} helpMessageWasSent = FALSE;

//_____________
// FUNCTION PROTOTYPES:
void PWM_init(uint32_t dutyCycle);
void PWM_setDutyCycle(uint32_t dutyCycle);
void usart2_init(void);
void usart2_writeChar(char msg_char);
void usart2_writeString(char *msg_string);
char usart2_readChar(void);
void resetInputValueStruct(struct inputValue_struct_type* iValStr_pointer, size_t stringSize);
void printParameterWasSetMessage(char* parameterString);
char *convertDoubleToString(double input);
void usart2_writePIDParameters(PIDController PID);
void USART2_IRQHandler(void);
void ADC1_init();
float ADC1_read();
void DAC1_init();
void DAC1_writeOutput(float fraction);
void  PIDController_Init(PIDController *pid);
float PIDController_Update(PIDController *pid, float measurement);

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


void usart2_writeString(char *msg_string)
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


void usart2_writePIDParameters(PIDController PID)
{
	usart2_writeString("  Setpoint: ");
	usart2_writeString(convertDoubleToString(PID.setpoint));
	usart2_writeString("  Kp: ");
	usart2_writeString(convertDoubleToString(PID.Kp));
	usart2_writeString("  Ki: ");
	usart2_writeString(convertDoubleToString(PID.Ki));
	usart2_writeString("  Kd: ");
	usart2_writeString(convertDoubleToString(PID.Kd));
	usart2_writeString("  Integrator windup limit: ");
	usart2_writeString(convertDoubleToString(PID.limMaxIntegrator));
	usart2_writeString("  Low pass time constant: ");
	usart2_writeString(convertDoubleToString(PID.tau));

	if (PID.signOfPID==1)
	{
		usart2_writeString("  Error = setpoint - measurement\n");
	}
	else
	{
		usart2_writeString("  Error = measurement - setpoint\n");
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
				usart2_writeString("Please type Kp value of PID: ");
				inputSpecifier = VALUE;
				helpMessageWasSent = FALSE;
				break;
			
			case 'I':
			case 'i':
				usart2_writeString("Please type Ki value of PID: ");
				inputSpecifier = VALUE;
				helpMessageWasSent = FALSE;
				break;

			case 'D':
			case 'd':
				usart2_writeString("Please type Kd value of PID: ");
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

			case 'T':
			case 't':
				usart2_writeString("Please type time constant of lowpass for D: ");
				inputSpecifier = VALUE;
				helpMessageWasSent = FALSE;
				break;

			case 'A':
			case 'a':
				usart2_writeString(convertDoubleToString(ADC1_read()));
				usart2_writeString("\n");
				helpMessageWasSent = FALSE;
				break;

			case '-':
				pid.signOfPID = -pid.signOfPID;
				usart2_writeString("Sign of PID was inverted! ");
				helpMessageWasSent = FALSE;

			default:
				if (helpMessageWasSent == FALSE)
				{
					usart2_writeString(helpMessage);
					usart2_writePIDParameters(pid);
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
					usart2_writeString("Oopsie Daisey! Something went wrong! :(\n");
			
			
			}
			resetInputValueStruct(&inputValue_struct, MAX_AMOUNT_INPUT_DIGITS);
			inputSpecifier = PARAMETER;	
			//usart2_writePIDParameters(PIDparams);

		}

		// value input cancelled if escape is pressed:
		else if (c == 27)
		{
			resetInputValueStruct(&inputValue_struct, MAX_AMOUNT_INPUT_DIGITS);
			inputSpecifier = PARAMETER;
			usart2_writeString(" Input cancelled! ");
			//usart2_writePIDParameters(PIDparams);
		}
	}
	// ToDo: reset pending bit
}

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

void PIDController_Init(PIDController *pid) {

	/* Clear controller variables */
	pid->signOfPID = 1;
	pid->integrator = 0.0f;
	pid->prevError  = 0.0f;

	pid->differentiator  = 0.0f;
	pid->prevMeasurement = 0.0f;

	pid->out = 0.0f;

}

float PIDController_Update(PIDController *pid, float measurement) {

	/*
	* Error signal
	*/
    float error = (pid->setpoint - measurement) * pid->signOfPID;


	/*
	* Proportional
	*/
    float proportional = pid->Kp * error;


	/*
	* Integral
	*/
    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);

	/* Anti-wind-up via integrator clamping */
    if (pid->integrator > pid->limMaxIntegrator) {

        pid->integrator = pid->limMaxIntegrator;

    } else if (pid->integrator < pid->limMinIntegrator) {

        pid->integrator = pid->limMinIntegrator;

    }


	/*
	* Derivative (band-limited differentiator)
	*/
		
    pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement)	/* Note: derivative on measurement, therefore minus sign in front of equation! */
                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
                        / (2.0f * pid->tau + pid->T);


	/*
	* Compute output and apply limits
	*/
    pid->out = proportional + pid->integrator + pid->differentiator;

    if (pid->out > pid->limMaxOut) {

        pid->out = pid->limMaxOut;

    } else if (pid->out < pid->limMinOut) {

        pid->out = pid->limMinOut;

    }

	/* Store error and measurement for later use */
    pid->prevError       = error;
    pid->prevMeasurement = measurement;

	/* Return controller output */
    return pid->out;

}


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
