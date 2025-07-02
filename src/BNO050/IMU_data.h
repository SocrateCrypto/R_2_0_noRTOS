#pragma once
#include "bno055.h"
#include <cstdint>




typedef struct {
	float Rotation_YRP[3];
	bno055_vector_t euler;
	bno055_calibration_state_t state;
	bno055_calibration_data_t calibrData;
	bool isCalibrated = false;
} compasData_t;

#define DegToRad 							0.0174533f	// degrees/sec to radians/sec
#define RadToDeg 							57.2958f 	// radians to degres

typedef struct {
    float old_degs;
    int8_t rotations;
} unwrap_t;
extern unwrap_t up[3]; // up[0] - yaw, up[1] - roll, up[2] - pitch
float unwrap(unwrap_t *up, float value);




extern compasData_t compData;


