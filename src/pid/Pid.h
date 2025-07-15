#ifndef PID_H
#define PID_H
#define PID_KP 45.0f
#define PID_KI 1.0f
#define PID_KD 1.0f
#define PID_RAMP 2000.0f
#define PID_LIMIT 250.0f
#include "math.h"
#include "main.h"
/**
 *  PID controller class
 */

class PIDController {
public:
	/**
	 *
	 * @param P - Proportional gain
	 * @param I - Integral gain
	 * @param D - Derivative gain
	 * @param ramp - Maximum speed of change of the output value
	 * @param limit - Maximum output value
	 */
	float P; //!< Proportional gain
	float I; //!< Integral gain
	float D; //!< Derivative gain
	float output_ramp; //!< Maximum speed of change of the output value
	float limit; //!< Maximum output value

	PIDController(float KP, float KI, float KD, float Ramp, float Limit);
	~PIDController() = default;
	float update(float error);
	void reset();

protected:
	float error_prev; //!< last tracking error value
	float output_prev;  //!< last pid output value
	float integral_prev; //!< last integral component value
	unsigned long long timestamp_prev; //!< Last execution timestamp
};

#endif // PID_H
