#include "Pid.h"
#include "main.h"


#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

PIDController::PIDController(float KP, float KI, float KD, float Ramp, float Limit) {
	P = KP;
	I = KI;
	D = KD;
	output_ramp = Ramp; // output derivative limit [volts/second]
	limit = Limit; // output supply limit     [volts]

	error_prev = 0.0f;
	output_prev = 0.0f;
	integral_prev = 0.0f;

	timestamp_prev = 0;
}

// PID controller function
float PIDController::update(float error) {
    uint32_t timestamp = HAL_GetTick();
    float Ts = (timestamp - (uint32_t)timestamp_prev) / 1000.0f;
    // quick fix for strange cases (overflow or zero)
    if (Ts <= 0 || Ts > 0.5f)
        Ts = 1e-3f;

    // u(s) = (P + I/s + Ds)e(s)
    // Discrete implementations
    // proportional part
    // u_p  = P *e(k)
	float proportional = P * error;
	// Tustin transform of the integral part
	// u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
	float integral = integral_prev + I * Ts * 0.5f * (error + error_prev);
	// antiwindup - limit the output
	integral = _constrain(integral, -limit, limit);
	// Discrete derivation
	// u_dk = D(ek - ek_1)/Ts
	float derivative = D * (error - error_prev) / Ts;

	// sum all the components
	float output = proportional + integral + derivative;
	// antiwindup - limit the output variable
	output = _constrain(output, -limit, limit);

	// if output ramp defined
	if (output_ramp > 0) {
		// limit the acceleration by ramping the output
		float output_rate = (output - output_prev) / Ts;
		if (output_rate > output_ramp)
			output = output_prev + output_ramp * Ts;
		else if (output_rate < -output_ramp)
			output = output_prev - output_ramp * Ts;
	}
	// saving for the next pass
	integral_prev = integral;
	output_prev = output;
	error_prev = error;
	timestamp_prev = timestamp;
	return output;
}

void PIDController::reset() {
	integral_prev = 0.0f;
	output_prev = 0.0f;
	error_prev = 0.0f;
}
