///**
//  ******************************************************************************
//  * File Name          : app_mems.c
//  * Description        : This file provides code for the configuration
//  *                      of the STMicroelectronics.X-CUBE-MEMS1.11.2.0 instances.
//  ******************************************************************************
//  * @attention
//  *
//  * Copyright (c) 2025 STMicroelectronics.
//  * All rights reserved.
//  *
//  * This software is licensed under terms that can be found in the LICENSE file
//  * in the root directory of this software component.
//  * If no LICENSE file comes with this software, it is provided AS-IS.
//  *
//  ******************************************************************************
//  */
//
#ifdef __cplusplus
extern "C" {
#endif

#include "app_mems.h"
#include "main.h"
#include "iks01a3_motion_sensors.h"
#include <stdio.h>

#define ACC_FS  2 /* FS = <-4g, 4g> */
#define BUFFER_SIZE 6664

//int16_t data_buffer[BUFFER_SIZE];
IKS01A3_MOTION_SENSOR_Axes_t acceleration;
IKS01A3_MOTION_SENSOR_Axes_t angular_velocity;

void MX_MEMS_Init(void) {
    //BSP_LED_Init(LED2);
    BSP_COM_Init(COM1);
    IKS01A3_MOTION_SENSOR_Init(IKS01A3_LSM6DSO_0, MOTION_ACCELERO | MOTION_GYRO);
    //IKS01A3_MOTION_SENSOR_Init(IKS01A3_LIS2MDL_0, MOTION_MAGNETO);

    IKS01A3_MOTION_SENSOR_SetOutputDataRate(IKS01A3_LSM6DSO_0, MOTION_ACCELERO, 26u);
    IKS01A3_MOTION_SENSOR_SetFullScale(IKS01A3_LSM6DSO_0, MOTION_ACCELERO, ACC_FS);

//    IKS01A3_MOTION_SENSOR_SetOutputDataRate(IKS01A3_LSM6DSO_0, MOTION_GYRO, 6664u);
//    IKS01A3_MOTION_SENSOR_SetFullScale(IKS01A3_LSM6DSO_0, MOTION_GYRO, 250u);


}

void Accelero_Sensor_Handler(uint32_t Instance) {
    if (IKS01A3_MOTION_SENSOR_GetAxes(Instance, MOTION_ACCELERO, &acceleration) == 0) {


    }

//    if (IKS01A3_MOTION_SENSOR_GetAxes(Instance, MOTION_GYRO, &angular_velocity) == 0) {
//
//    	//printf("nGYR_X: %d, GYR_Y: %d, GYR_Z: %d\r\n" ,  (int)angular_velocity.x,  (int)angular_velocity.y  ,  (int)angular_velocity.z);
//
//    }
}
void MX_MEMS_Process(void) {
    Accelero_Sensor_Handler(0);
}

#ifdef __cplusplus
}
#endif
