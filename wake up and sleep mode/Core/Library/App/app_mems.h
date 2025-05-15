/**
  ******************************************************************************
  * File Name          : app_mems.h
  * Description        : This file provides code for the configuration
  *                      of the STMicroelectronics.X-CUBE-MEMS1.11.2.0 instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_MEMS_H
#define __APP_MEMS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "iks01a3_motion_sensors.h"

extern IKS01A3_MOTION_SENSOR_Axes_t acceleration;

typedef struct {

float_t Arms_x, Arms_y, Arms_z ;
float_t Vrms_x, Vrms_y, Vrms_z;
float frequency;
float delta_magnitude;
float max_delta;
float average_delta;

} type_data;
extern type_data data;

//extern float_t Arms_x, Arms_y, Arms_z;
//extern float_t Vrms_x, Vrms_y, Vrms_z;


/* Includes ------------------------------------------------------------------*/

/* Exported defines ----------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
void MX_MEMS_Init(void);
void MX_MEMS_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_MEMS_H */
