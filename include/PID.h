#if !defined(PID_H)
#define PID_H

#include <stdint.h>

// global variable declarations:
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

extern PIDController pid;

// function declarations:
void  PIDController_Init(PIDController *pid);
float PIDController_Update(PIDController *pid, float measurement);

#endif // PID_H
