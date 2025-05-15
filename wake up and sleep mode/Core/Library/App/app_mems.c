/**
  ******************************************************************************
  * File Name          : app_mems.c
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

#ifdef __cplusplus
extern "C" {
#endif

#include "app_mems.h"
#include "main.h"
#include "iks01a3_motion_sensors.h"
#include <stdio.h>

#define ARM_MATH_CM4
#include "arm_math.h"

//#define ACC_FS  4 /* FS = <-4g, 4g> */
#define BUFFER_SIZE 3200
#define FFT_SIZE 1024
#define ACC_FS 16

type_data data;
int16_t data_buffer[BUFFER_SIZE];
IKS01A3_MOTION_SENSOR_Axes_t acceleration;
float_t test_qq;
float_t x_buffer[BUFFER_SIZE];
float_t y_buffer[BUFFER_SIZE];
float_t z_buffer[BUFFER_SIZE];

arm_cfft_radix4_instance_f32	FFThandler;

//int16_t FFT_size = 1024;
//float_t input_x[BUFFER_SIZE] , input_y[BUFFER_SIZE] , input_x[BUFFER_SIZE];
//int index1 , index2 , index3 ;

uint16_t index1 = 0;
uint16_t index2 = 0;
uint16_t index3 = 0;

float prev_magnitude;
float magnitude;
float delta_magnitude;

//#define MAX_SAMPLES FFT_SIZE
#define MAX_SAMPLES 1600

float delta_magnitude_array[MAX_SAMPLES];
int filled = 0;
float max_delta = 0;


void MX_MEMS_Init(void) {
    //BSP_LED_Init(LED2);
    BSP_COM_Init(COM1);
    IKS01A3_MOTION_SENSOR_Init(IKS01A3_LSM6DSO_0, MOTION_ACCELERO | MOTION_GYRO);
//    IKS01A3_MOTION_SENSOR_SetOutputDataRate(IKS01A3_LSM6DSO_0, MOTION_ACCELERO, 6664u);
//    IKS01A3_MOTION_SENSOR_SetFullScale(IKS01A3_LSM6DSO_0, MOTION_ACCELERO, ACC_FS);

    	uint8_t threshold = 0x94;  // ODR = 3332kHz , FS = 16g
        HAL_I2C_Mem_Write(&hi2c2,  LSM6DSO_I2C_ADD_H ,  LSM6DSO_CTRL1_XL , 1 , &threshold, 1, HAL_MAX_DELAY);
//        uint8_t readBack = 0x00;
//        HAL_I2C_Mem_Read(&hi2c2, LSM6DSO_I2C_ADD_H , LSM6DSO_CTRL1_XL, 1, &readBack, 1, HAL_MAX_DELAY);

        uint8_t WAKE_UP_DUR = 0x0F;
        HAL_I2C_Mem_Write(&hi2c2,  LSM6DSO_I2C_ADD_H ,  LSM6DSO_WAKE_UP_DUR   , 1 , &WAKE_UP_DUR, 1, HAL_MAX_DELAY); // 2.458 -> sleep mode

    	uint8_t WAKE_UP_THS = 0x01;
    	HAL_I2C_Mem_Write(&hi2c2,  LSM6DSO_I2C_ADD_H ,  LSM6DSO_WAKE_UP_THS  , 1 , &WAKE_UP_THS, 1, HAL_MAX_DELAY); //250mg
//    	uint8_t readBack = 0x00;
//    	HAL_I2C_Mem_Read(&hi2c2, LSM6DSO_I2C_ADD_H , LSM6DSO_WAKE_UP_THS, 1, &readBack, 1, HAL_MAX_DELAY);
//
//    	printf("---------- %x --------- \r\n" , readBack);

    	uint8_t TAP_CFG0  = 0x10; 	// slope -> 0x00 , HPF -> 0x10 // latched 0x41(slope) 0x51(HPF)
    	HAL_I2C_Mem_Write(&hi2c2,  LSM6DSO_I2C_ADD_H ,LSM6DSO_TAP_CFG0 , 1 , &TAP_CFG0, 1, HAL_MAX_DELAY);

    	uint8_t TAP_CFG2  = 0xC0;
    	HAL_I2C_Mem_Write(&hi2c2,  LSM6DSO_I2C_ADD_H , LSM6DSO_TAP_CFG2 , 1 , &TAP_CFG2, 1, HAL_MAX_DELAY); // enable wake up and active/inactive

    	uint8_t MD1_CFG  = 0xE0;
    	HAL_I2C_Mem_Write(&hi2c2,  LSM6DSO_I2C_ADD_H , LSM6DSO_MD1_CFG , 1 , &MD1_CFG, 1, HAL_MAX_DELAY); // enable wake up , active/inactive


}

void Accelero_Sensor_Handler(uint32_t Instance) {
    if (IKS01A3_MOTION_SENSOR_GetAxes(Instance, MOTION_ACCELERO, &acceleration) == 0) {

//    	magnitude = sqrtf((acceleration.x * acceleration.x ) + (acceleration.y * acceleration.y) + (acceleration.z * acceleration.z));
    	magnitude = acceleration.z;

   	    data.delta_magnitude = magnitude - prev_magnitude ;

    	//printf("Current: %.2f, Previous: %.2f, d: %.2f\r\n", magnitude, prev_magnitude, delta_magnitude);


    	prev_magnitude = magnitude;

    	delta_magnitude_array[index1] = data.delta_magnitude;
    	index1++;

    	if (index1 >= MAX_SAMPLES)
    	{
    	    index1 = 0;
    	    filled = 1;

    	    max_delta = delta_magnitude_array[0];
    	    float sum_delta = 0.0f;
    	    for (int i = 0; i < MAX_SAMPLES; i++)
    	    {
    	        sum_delta += fabs(delta_magnitude_array[i]);
    	    }
    	    float average_delta = sum_delta / MAX_SAMPLES;
    	    data.average_delta = average_delta;

    	}

    }
}

void MX_MEMS_Process(void) {
    Accelero_Sensor_Handler(0);
}


#ifdef __cplusplus
}
#endif
