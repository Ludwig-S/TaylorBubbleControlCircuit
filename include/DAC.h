#if !defined(DAC_H)
#define DAC_H

#include "main.h"

void DAC1_init();
void DAC1_writeOutput(float fraction);

void PWM_init(uint32_t dutyCycle);
void PWM_setDutyCycle(uint32_t dutyCycle);


#endif // DAC_H
