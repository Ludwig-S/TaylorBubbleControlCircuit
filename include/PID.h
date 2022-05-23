#include<stdint.h>
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

typedef struct {

	// Set point
	float setpoint;

	// Sign of error
	int8_t signOfPID;

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

void  PIDController_Init(PIDController *pid);
float PIDController_Update(PIDController *pid, float measurement);

#endif
