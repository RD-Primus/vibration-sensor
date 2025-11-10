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

// ประกาศ extern เพื่อบอกว่า 'acceleration' ถูกประกาศที่อื่น
/* Includes ------------------------------------------------------------------*/

/* Exported defines ----------------------------------------------------------*/

#include "iks01a3_motion_sensors.h"
#include "stm32wlxx_hal.h"
#include <math.h>
#include <stdio.h>


extern IKS01A3_MOTION_SENSOR_Axes_t acceleration;
/* Exported functions --------------------------------------------------------*/
void MX_MEMS_Init(void);
void MX_MEMS_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_MEMS_H */
