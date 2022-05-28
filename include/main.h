#if !defined(MAIN_H)
#define MAIN_H

#include "stm32f4xx.h"                  // Device header
#include "PID.h"

//______________________________________________________________
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

#endif // MAIN_H
