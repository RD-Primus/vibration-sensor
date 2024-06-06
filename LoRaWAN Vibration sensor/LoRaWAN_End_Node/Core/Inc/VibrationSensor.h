/*
 * VibrationSensor.h
 *
 *  Created on: Mar 21, 2024
 *      Author: muple
 */

#ifndef VIBRATIONSENSOR_H_
#define VIBRATIONSENSOR_H_

#include "main.h"
#include "tim.h"
#include <stdio.h>
#include <stdbool.h>
#include "math.h"

#include "iks01a3_motion_sensors.h"
#include "iks01a3_motion_sensors_ex.h"
#include "iks01a3_env_sensors.h"
#include "iks01a3_env_sensors_ex.h"

#define ARM_MATH_CM4
#include "arm_math.h"

void X_NUCLEO_IKS01A3_Sensors_Init(void);

void VibrationSensor_Process(void);

typedef struct
{
	float x;
	float y;
	float z;
} Vibrations_float;

typedef struct
{
	double x;
	double y;
	double z;
} Vibrations_double;

typedef struct
{
	Vibrations_float LIS2DW12_RMS_Gravity;
	Vibrations_float LIS2DW12_RMS_Acceleration;
	Vibrations_float LIS2DW12_RMS_Velocity;

	float HTTS751_Temperature;
} VibrationSensor_Node;

extern VibrationSensor_Node VibrationSensor_Output;

#endif /* VIBRATIONSENSOR_H_ */
