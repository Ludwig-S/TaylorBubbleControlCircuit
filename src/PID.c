#include "PID.h"

// variable definitions:
PIDController pid;


// function definitions
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
