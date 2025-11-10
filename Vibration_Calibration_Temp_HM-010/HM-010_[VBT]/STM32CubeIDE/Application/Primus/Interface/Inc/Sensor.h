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

#define STTS22H_ONESHOT_ENABLE     			 	False
#define SENSOR_MEASURE_PERIOD					20
#define SENSOR_LOG_PERIOD						1000
#define FFT_ENABLE             					true

#define ISM330DHCX_TIM							&htim17


#if FFT_ENABLE
//#define Acc_bufffer_size 2048  //......................
//#define FFT_BUFFER_SIZE	Acc_bufffer_size
//#define FFT_LENGTH FFT_BUFFER_SIZE/2

#define GRAVITATIONAL_ACCELERATION	9.80665f
#define NOISE_THRESHOLD 0.01f // Threshold to consider as noise

#define bufffer_size  2048// (FFT_SIZE * 2)
#define FFT_SIZE 1024


typedef struct {
	uint16_t index_afft;
	arm_status status;
	bool Calculate;
}type_FFT_acc;

#endif



typedef struct
{
  MOTION_SENSOR_AxesRaw_t data_raw_acceleration;   /**<Raw accelerometer output*/
  int16_t temperature;  /**<Last measured temperature [0.01 `C]*/
  int8_t temperature_alarm;

  float32_t ACC_X[bufffer_size];
  float32_t ACC_Y[bufffer_size];
  float32_t ACC_Z[bufffer_size];
  float32_t output_fft_mag_z[FFT_SIZE];
  float32_t output_fft_mag_y[FFT_SIZE];
  float32_t output_fft_mag_x[FFT_SIZE];
  uint32_t  index2 ;
//  void (*input_and_Apeak)(void);

  bool status ;
  bool status_wk;
  int Update_values;

  bool temperature_sensor_good;     /**<True if the sensor is good to measure, false if something failed*/
  bool accelero_sensor_good;  /**<True if the sensor is good to measure, false if something failed*/

  uint16_t ISM330DHCX_1Sec_1;
  uint16_t ISM330DHCX_1Sec_2;
//  uint16_t STTS22H_1Sec;

  uint16_t ISM330DHCX_Tick;
//  uint16_t STTS22H_Tick;

  uint16_t ISM330DHCX_fail;
  uint16_t STTS22H_fail;
  uint16_t sampling;

  uint8_t bussy;
  uint8_t Wake_up_cnt;

} Sensor_t;
extern Sensor_t Sensor;;

#define TOP_N	8

typedef struct{
	bool wake_up ;
	bool offset ;
	bool offset_done;
	bool Train ;
	float32_t input_Vrms ; // Threshold->wake_up
	float32_t input_RPM , freq;
	int input_FS_g;
	bool zeroCross ;

	float32_t Arms ;

//	void(*offset_working)(void);
//	void(*wake_up_now_working)(void);
//	void(*wake_up_working)(void);
//	void(*Train_working)(void);

    struct{
    	float64_t sum_x, sum_y ,sum_z;
    	float64_t x, y, z;
    	float64_t scale ;
    	int8_t reg_x , reg_y , reg_z;
    }offset1;

    struct{
    	float32_t av_x, av_y ,av_z;
    	float32_t x, y, z;
    	int count;
    	float32_t max_val;
    }Train1;

}sensor_App_t;
extern sensor_App_t sensor_App;


typedef struct{
	uint8_t WAKE_UP_THS ; // Threshold->wake_up
	float32_t slope ;
	float32_t calibration ;


	struct {
		float32_t x;
		float32_t y;
		float32_t z;
		uint8_t reg ;
	}Fs; // Threshold->Full scale

	struct {
		uint8_t reg ;
		uint32_t time ;
	}ODR_sampling;

	struct {
		uint32_t freq;
		float32_t Sampling_Rate;
	}machine; //Threshold->frequency_machine

//	void (*Full_scale)(void);
//	void (*ODR_FS_setting)(void);

}threshold_t;
extern threshold_t threshold ;

typedef struct{
	float E_sse_av_x;
	float E_sse_av_y;
	float E_sse_av_z;

	float E_sse_sum_x ;
	float E_sse_sum_y ;
	float E_sse_sum_z ;

	float E_sse1_x[(FFT_SIZE / 2)];
	float E_sse1_y[(FFT_SIZE / 2)];
	float E_sse1_z[(FFT_SIZE / 2)];


	void (*threshold_noise)(void);


	uint32_t Ratio_TOP_N;
	float Ratio_z , Ratio_y , Ratio_x ;
}Base_noise_t;
extern Base_noise_t Base_noise;

typedef struct{

	struct {
	    float Apeak_S1_z;
	    float Apeak_S1_y;
	    float Apeak_S1_x;
	    float Apeak_use_z;
	    float Apeak_use_y;
	    float Apeak_use_x;
	    float_t output_Hz_z ;
	    float_t output_Hz_y ;
	    float_t output_Hz_x ;
	    uint32_t index_z;
	    uint32_t index_y;
	    uint32_t index_x;
	} peaks_acc[TOP_N] ;

//	void (*peak)(void);

	struct {
		float x , y , z;
	}Interpolated_bin;

	float64_t f_sum_x ,  f_sum_y ,  f_sum_z;

}freq_t;
extern freq_t freq;

typedef struct {
	float32_t rms_x, rms_y , rms_z , output_axis , output_axis_num ;
	float64_t rms_z_sum , rms_z_sqr[TOP_N] , output_sqr_z;
	float64_t rms_y_sum , rms_y_sqr[TOP_N] , output_sqr_y;
	float64_t rms_x_sum , rms_x_sqr[TOP_N] , output_sqr_x , output_Addition_vectors /*,output_axis_num*/;


//	void(*velocity)(void);
}v_t;
extern v_t v;

typedef struct {
	float32_t rms_x, rms_y , rms_z , output_axis ;
	float64_t rms_z_sum , rms_z_sqr[TOP_N] , output_sqr_z;
	float64_t rms_y_sum , rms_y_sqr[TOP_N] , output_sqr_y;
	float64_t rms_x_sum , rms_x_sqr[TOP_N] , output_sqr_x , output_Addition_vectors ;


//	void(*displacement_c)(void);

}displacement_t;
extern displacement_t displacement;

typedef struct {
    float32_t rms_x, rms_y, rms_z, output_Addition_vectors , output_axis , sum_x, sum_y, sum_z;
    float32_t output_rms_x, output_rms_y, output_rms_z;
    float32_t rms_x1, rms_y1, rms_z1;


    float32_t addition_A1 ;
} acc_t;
extern acc_t acc;

typedef struct {
	int Count_x ;
	int Count_y ;
	int Count_z ;

	float32_t freq_x , freq_y , freq_z ;
}zeroCross_t;
extern zeroCross_t zeroCross;

typedef struct {
	float32_t freq ;
	//float32_t x[TOP_N] , y[TOP_N] ,z[TOP_N] ;
	float32_t x1, y1 ,z1 ;
	float64_t peak_x[TOP_N], peak_y[TOP_N] ,peak_z[TOP_N] ;

	struct {
		uint8_t input_cal_enable ;
		uint8_t Freq_cal;/* 15.92Hz : 0, 40.00Hz : 1, 80.00Hz : 2 */
		float Freq_errorCal[5];
		float error;
		float peak_z;
		float error_av;
		float new_Han;
		float check;
	}Hann ;
	uint8_t cal_enable ;

	struct {
		float input_Gain ; //recommend ->" 1.00 - 2.00"
		bool Gain ;
	}Regession_peak ;

} calibration_t;
extern calibration_t calibration ;

typedef struct {
	uint8_t Temp_H_OUT ;
	uint8_t Temp_L_OUT ;
	int16_t Temp_OUT ;
	int16_t Output ;

	uint8_t OVER;
	//bool Reset;

	int16_t H_limit;
	int16_t L_limit;

	bool interrupt;


}Temp_t;
extern Temp_t Temp;

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
