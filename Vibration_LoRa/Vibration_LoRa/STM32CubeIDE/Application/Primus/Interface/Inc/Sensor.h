/*
 * Sensor.h
 *
 *  Created on: Apr 23, 2025
 *      Author: Nilniz
 */

#ifndef METERS_H_
#define METERS_H_

#define ARM_MATH_CM4
#include "arm_math.h"

#include <b_wl5m_subg_motion_sensors_ex.h>
#include <b_wl5m_subg_env_sensors_ex.h>

#define STTS22H_ONESHOT_ENABLE             		false
#define SENSOR_MEASURE_PERIOD					20
#define SENSOR_LOG_PERIOD						1000
#define FFT_ENABLE             					true

#define ISM330DHCX_TIM							&htim17
#define Ennable_magnitude 1

#if FFT_ENABLE
//#define Acc_bufffer_size 2048  //......................
//#define FFT_BUFFER_SIZE	Acc_bufffer_size
//#define FFT_LENGTH FFT_BUFFER_SIZE/2

#define GRAVITATIONAL_ACCELERATION	9.80665f
#define NOISE_THRESHOLD 0.01f // Threshold to consider as noise

#define bufffer_size  2048// (FFT_SIZE * 2)
#define FFT_SIZE 1024

typedef struct{
	float 	Sum_sqrt_X;
	float 	Sum_sqrt_Y;
	float	Sum_sqrt_Z;

	float 	x;
	float 	y;
	float 	z;
	int16_t		sample_size;
	uint16_t	index;
}type_rms;

typedef struct {
	//MOTION_SENSOR_Axes_t AccValue[Acc_bufffer_size];

//	float32_t aFFT_Input_f32[FFT_BUFFER_SIZE];
//	float32_t aFFT_Output_f32[FFT_BUFFER_SIZE];

	arm_status status;

	float32_t maxValue;    /* Max FFT value is stored here */
	uint32_t maxIndex;    /* Index in Output array where max value is */

//	type_rms rms_acc;
//	type_rms rms_gyr;
//	type_rms rms_vel;

	uint16_t index;
	uint16_t index_buff;
	uint16_t index_afft;
	uint8_t req_cmd;

	bool Calculate;
}type_FFT_acc;

#endif





typedef struct
{
	MOTION_SENSOR_AxesRaw_t data_raw_acceleration;   /**<Raw accelerometer output*/
  int16_t temperature;  /**<Last measured temperature [0.01 `C]*/
  float32_t ACC_X[bufffer_size];
  float32_t ACC_Y[bufffer_size];
  float32_t ACC_Z[bufffer_size];

  float32_t output_fft_mag_z[FFT_SIZE];
  float32_t output_fft_mag_y[FFT_SIZE];
  float32_t output_fft_mag_x[FFT_SIZE];

  uint32_t  index2 ;
  bool status ;
  int Update_values;

  bool temperature_sensor_good;     /**<True if the sensor is good to measure, false if something failed*/
  bool accelero_sensor_good;  /**<True if the sensor is good to measure, false if something failed*/

//  uint16_t ISM330DHCX_1Sec;
//  uint16_t STTS22H_1Sec;
//
//  uint16_t ISM330DHCX_Tick;
//  uint16_t STTS22H_Tick;

  uint16_t ISM330DHCX_fail;
  uint16_t STTS22H_fail;
  uint16_t sampling;

  uint8_t bussy;
  uint8_t Wake_up_cnt;

} Sensor_t;
extern Sensor_t Sensor;

//------------------------------------------------------------------------------------
#define TOP_N 16
typedef struct {
    float value_z;
    float value_y;
    float value_x;
    float_t peakFrequency_z ;
    float_t peakFrequency_y ;
    float_t peakFrequency_x ;
    uint32_t index_x;
    uint32_t index_y;
    uint32_t index_z;
} Peak_t;
extern Peak_t peaks_acc[TOP_N];

typedef struct {
	float E_sse_av_x;
	float E_sse_av_y;
	float E_sse_av_z;

	float E_sse_sum_x ;
	float E_sse_sum_y ;
	float E_sse_sum_z ;

	float E_sse1_x[(FFT_SIZE / 2)];
	float E_sse1_y[(FFT_SIZE / 2)];
	float E_sse1_z[(FFT_SIZE / 2)];

	uint32_t Ratio_TOP_N;
	float Ratio_z , Ratio_y , Ratio_x;

}type_threshold_Base;
extern type_threshold_Base TH;

typedef struct{
	float32_t rms_x, rms_y , rms_z ;
	float32_t rms_z_sum , rms_z_sqr;
	float32_t rms_y_sum , rms_y_sqr;
	float32_t rms_x_sum , rms_x_sqr;

}Velocity_t;
extern Velocity_t v;



extern uint32_t Sensor_Init(void);
extern void Sensor_DeInit(void);
extern void Sensor_Run();
extern void Sensor_Log();
extern uint64_t GET_uTick();
//extern void u_Tick(TIM_HandleTypeDef *htim) ;
extern void ACCELERO_GYRO_INT_RUN();
extern void Enable_Temp_STTS22H();
extern void Measure_Temp_STTS22H();
extern void Measure_Acc_ISM330DHCX();
extern void Acc_Calculation();
extern void wake_up_ISM330DHCX();

#endif /* METERS_H_ */
