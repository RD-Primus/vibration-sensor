/*
 * Sensor.c
 *
 *  Created on: Apr 23, 2025
 *      Author: Nilniz
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>

#define FFT_INVERSE_FLAG        ((uint8_t)0)
#define FFT_Normal_OUTPUT_FLAG  ((uint8_t)1)

arm_cfft_radix4_instance_f32	FFThandler;
//arm_rfft_fast_instance_f32 fft_instance;

#define Acc_FS 16
#define FFT_SIZE 1024

Sensor_t Sensor;
#if FFT_ENABLE
type_FFT_acc FFT_Acc;
#endif

static uint32_t Sensor_STTS22HInit(void);
static uint32_t Sensor_ISM330DHCXInit(void);

uint32_t index1 , index2 ,index3;

float_t Arms_x, Arms_y, Arms_z;
float_t Vrms_x, Vrms_y, Vrms_z;

float32_t maxValue_x , maxValue_y , maxValue_z;
uint32_t maxIndex_x ,maxIndex_y ,maxIndex_z;

float32_t FFT_output_z[2 * FFT_SIZE];
float32_t output_fft_mag_z[FFT_SIZE];

uint32_t time1 , time2 , time3;
float frequency;

#define TOP_N 3

typedef struct {
    float value;
    float_t peakFrequency_z ;
    uint32_t index;
} Peak_t;

Peak_t peaks[TOP_N];



void Sensor_Errors(void)
{
  UTILS_ENTER_CRITICAL_SECTION();
  Sensor_DeInit();
  Sensor_Init();
  UTILS_EXIT_CRITICAL_SECTION();
}

uint32_t Sensor_Init(void)
{

#if FFT_ENABLE
	FFT_Acc.status = ARM_MATH_SUCCESS;
//	arm_cfft_radix4_init_f32(&FFThandler, FFT_BUFFER_SIZE, FFT_INVERSE_FLAG, FFT_Normal_OUTPUT_FLAG);
	arm_cfft_radix4_init_f32(&FFThandler, FFT_SIZE , 0, 1);

	//arm_rfft_fast_init_f32(&fft_instance, Acc_bufffer_size);

	FFT_Acc.index_afft = 0;
#endif

  Sensor.temperature = 0;

  /*Init sensors*/
  while(!Sensor.temperature_sensor_good && !Sensor.accelero_sensor_good){
  	if(!Sensor.temperature_sensor_good)
  		Sensor.temperature_sensor_good = (Sensor_STTS22HInit() == 0) ? true : false;
  	if(!Sensor.accelero_sensor_good)
      Sensor.accelero_sensor_good = (Sensor_ISM330DHCXInit() == 0) ? true : false;

  }
  return 0;
}


void Sensor_DeInit(void)
{
  BSP_ENV_SENSOR_DeInit(ENV_SENSOR_STTS22H_0);
  BSP_MOTION_SENSOR_DeInit(MOTION_SENSOR_ISM330DHCX_0);
  BSP_I2C2_DeInit();
  BSP_I2C2_DeInit();
  BSP_I2C2_DeInit();
  BSP_I2C2_DeInit();
}

static uint32_t Sensor_STTS22HInit(void)
{
  if (BSP_ENV_SENSOR_Init(ENV_SENSOR_STTS22H_0, ENV_TEMPERATURE))
  {
    return 1;
  }
  if (BSP_ENV_SENSOR_Enable(ENV_SENSOR_STTS22H_0, ENV_TEMPERATURE))
  {
    return 2;
  }
#if STTS22H_ONESHOT_ENABLE
  if (BSP_ENV_SENSOR_Set_One_Shot(ENV_SENSOR_STTS22H_0))
  {
    return 4;
  }
#endif

  /* USER CODE BEGIN Init */

//  uint8_t Data;
//  BSP_ENV_SENSOR_Write_Register(ENV_SENSOR_STTS22H_0, Reg, Data);
//  BSP_ENV_SENSOR_Read_Register(ENV_SENSOR_STTS22H_0, Reg, &Data);

  /* USER CODE END Init */


  return 0;
}


static uint32_t Sensor_ISM330DHCXInit(void)
{
  if (BSP_MOTION_SENSOR_Init(MOTION_SENSOR_ISM330DHCX_0, MOTION_ACCELERO))
  {
    return 1;
  }
  if (BSP_MOTION_SENSOR_SetFullScale(MOTION_SENSOR_ISM330DHCX_0, MOTION_ACCELERO, Acc_FS))
  {
    return 2;
  }
  if (BSP_MOTION_SENSOR_SetOutputDataRate(MOTION_SENSOR_ISM330DHCX_0, MOTION_ACCELERO, 6667u))
  {
    return 3;
  }
  /* USER CODE BEGIN Init */

//  uint8_t Data;
//  Data =0x21;
//  BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, 0x08, Data);
//  Data =111;
//  BSP_MOTION_SENSOR_Read_Register(MOTION_SENSOR_ISM330DHCX_0, 0x08, &Data);

  /* USER CODE END Init */

  return 0;
}

void Enable_Temp_STTS22H(void)
{
  /*Trigger temperature sensor*/
  if (Sensor.temperature_sensor_good == true)
  {
    if (BSP_ENV_SENSOR_Enable(ENV_SENSOR_STTS22H_0, ENV_TEMPERATURE))
    {
      Sensor.temperature_sensor_good = false;
    }

#if STTS22H_ONESHOT_ENABLE
    else if (BSP_ENV_SENSOR_Set_One_Shot(ENV_SENSOR_STTS22H_0))
    {
      Sensor.temperature_sensor_good = false;
    }
#endif
  }

}

void Measure_Temp_STTS22H()
{
    Sensor.STTS22H_Tick++;
  /*Temperature*/
  if (Sensor.temperature_sensor_good == true)
  {
    float temperature = 0;
    if (BSP_ENV_SENSOR_GetValue(ENV_SENSOR_STTS22H_0, ENV_TEMPERATURE, &temperature))
    {
//      Sensor.temperature_sensor_good = false;
        Sensor.STTS22H_fail++;
    }

#if STTS22H_ONESHOT_ENABLE
    else if (BSP_ENV_SENSOR_Disable(ENV_SENSOR_STTS22H_0, ENV_TEMPERATURE))
    {
      Sensor.temperature_sensor_good = false;
    }
#endif
    else
    {
      Sensor.temperature = (int16_t)(100 * temperature);    /*[0.01 `C]*/

    }
  }
  else
  {
//    Sensor_Errors();
//    Sensor.temperature = 0;
  }
}

void Measure_Acc_ISM330DHCX()
{
	Sensor.ISM330DHCX_Tick++;

	if ((Sensor.accelero_sensor_good == true)) /*Accelerometer is working*/
	{
		/*Get accelerometer data*/
		if (BSP_MOTION_SENSOR_GetAxesRaw(MOTION_SENSOR_ISM330DHCX_0,
				MOTION_ACCELERO, &(Sensor.data_raw_acceleration))) {
//      Sensor.accelero_sensor_good = false;
//      Sensor_Errors();
	        Sensor.ISM330DHCX_fail++;
		} else {

#if FFT_ENABLE
//			if (FFT_Acc.index_buff < Acc_bufffer_size && !FFT_Acc.Calculate) {
			if (index2 < 1024 && !FFT_Acc.Calculate) {
//				FFT_Acc.AccValue[FFT_Acc.index_buff].x = Sensor.data_raw_acceleration.x;
//				FFT_Acc.AccValue[FFT_Acc.index_buff].y = Sensor.data_raw_acceleration.y;
//				FFT_Acc.AccValue[FFT_Acc.index_buff].z = Sensor.data_raw_acceleration.z;
//				FFT_Acc.index_buff += 1;

				Sensor.ACC_X[index1++] = (ism330dhcx_from_fs16g_to_mg(Sensor.data_raw_acceleration.x) / (float) 1000) *  9.80665;
				Sensor.ACC_Y[index2++] = (ism330dhcx_from_fs16g_to_mg(Sensor.data_raw_acceleration.y) / (float) 1000) *  9.80665;
				Sensor.ACC_Z[index3++] = ((ism330dhcx_from_fs16g_to_mg(Sensor.data_raw_acceleration.z) / (float) 1000) *  9.80665) - 9.80665 ;

			} else if (!FFT_Acc.Calculate) {
				FFT_Acc.Calculate = true;
				HAL_TIM_Base_Stop_IT(ISM330DHCX_TIM);
			}
#endif
		}

	}
}

#if FFT_ENABLE


void calculate_velocity(float *x, float *y, float *z, uint16_t size, float sampling_rate, float frequency,
                        float *Arms_x, float *Arms_y, float *Arms_z,
                        float *Vrms_x, float *Vrms_y, float *Vrms_z) {


    float sum_ax2 = 0, sum_ay2 = 0, sum_az2 = 0;

    for (uint16_t i = 0; i < size; i++) {
    	sum_ax2 += x[i] * x[i];
    	sum_ay2 += y[i] * y[i];
    	sum_az2 += z[i] * z[i];
    }

       *Arms_x = sqrtf(sum_ax2 / size);
       *Arms_y = sqrtf(sum_ay2 / size);
       *Arms_z = sqrtf(sum_az2 / size);


    // คำนวณ Vrms
    float factor = 2.0f * M_PI * frequency;
    *Vrms_x = (*Arms_x / factor) * 1000; // mm/s
    *Vrms_y = (*Arms_y / factor) * 1000;
    *Vrms_z = (*Arms_z / factor) * 1000;
}


//void calculate_velocity(float *x, float *y, float *z, uint16_t size, float sampling_rate, float frequency,
//                        float *Arms_x, float *Arms_y, float *Arms_z,
//                        float *Vrms_x, float *Vrms_y, float *Vrms_z) {
//
//    float sum_ax2 = 0, sum_ay2 = 0, sum_az2 = 0;
//
//    // Step 1: หาค่าเฉลี่ย (mean) ของข้อมูลเพื่อใช้ลบ offset
//    float mean_x = 0, mean_y = 0, mean_z = 0;
//    for (uint16_t i = 0; i < size; i++) {
//        mean_x += x[i];
//        mean_y += y[i];
//        mean_z += z[i];
//    }
//    mean_x /= size;
//    mean_y /= size;
//    mean_z /= size;
//
//    // ลบ mean ออกจากข้อมูล (Remove DC offset)
//    for (uint16_t i = 0; i < size; i++) {
//        x[i] -= mean_x;
//        y[i] -= mean_y;
//        z[i] -= mean_z;
//    }
//
//    // Step 2: Arms
//    for (uint16_t i = 0; i < size; i++) {
//        sum_ax2 += x[i] * x[i];
//        sum_ay2 += y[i] * y[i];
//        sum_az2 += z[i] * z[i];
//    }
//    *Arms_x = sqrtf(sum_ax2 / size);
//    *Arms_y = sqrtf(sum_ay2 / size);
//    *Arms_z = sqrtf(sum_az2 / size);
//
//    // Step 3: a(t) → v(t)
//    float vx[size], vy[size], vz[size];
//    float sum_vx = 0, sum_vy = 0 , sum_vz = 0;
//    float dt = 1.0f / sampling_rate;
//
//    vx[0] = 0;
//    vy[0] = 0;
//    vz[0] = 0;
//
//    for (uint16_t i = 1; i < size; i++) {
//        vx[i] = vx[i - 1] + x[i] * dt;
//        vy[i] = vy[i - 1] + y[i] * dt;
//        vz[i] = vz[i - 1] + z[i] * dt;
//    }
//
//    // Step 4: คำนวณ Vrms จาก v(t)
//    for (uint16_t i = 0; i < size; i++) {
//        sum_vx += vx[i] * vx[i];
//        sum_vy += vy[i] * vy[i];
//        sum_vz += vz[i] * vz[i];
//    }
//
//    *Vrms_x = sqrtf(sum_vx / size)* 1000;
//    *Vrms_y = sqrtf(sum_vy / size)* 1000;
//    *Vrms_z = sqrtf(sum_vz / size)* 1000;
//}



void calculateMagnitudeArray(float *input, float *output, int length) {
    for (int i = 0; i < length ; i++) {
        float real = input[2 * i];  ////////////////////////////////////////////////////////
        float imag = input[2 * i + 1];
        output[i] = sqrtf(real * real + imag * imag);
    }
}

void find_top_peaks(float mag[], Peak_t peaks[], uint32_t fft_size)
{
    for (uint32_t i = 0; i < TOP_N; i++) {
        peaks[i].value = 0.0f;
        peaks[i].index = 0;
    }

    for (uint32_t i = 1; i < fft_size; i++) {
        float val = mag[i];

        int min_idx = 0;
        for (int j = 1; j < TOP_N; j++) {
            if (peaks[j].value < peaks[min_idx].value) {
                min_idx = j;
            }
        }

        if (val > peaks[min_idx].value) {
            peaks[min_idx].value = val;
            peaks[min_idx].index = i;
        }
    }
}

#endif

void Acc_Calculation() {

	/*Convert accelerometer, mg/LSB  to m/s^2*/
//	Sensor.ACC_X[index1++] = (ism330dhcx_from_fs16g_to_mg(Sensor.data_raw_acceleration.x) / (float) 1000) *  9.80665;
//	Sensor.ACC_Y[index2++] = (ism330dhcx_from_fs16g_to_mg(Sensor.data_raw_acceleration.y) / (float) 1000) *  9.80665;
//	Sensor.ACC_Z[index3++] = (ism330dhcx_from_fs16g_to_mg(Sensor.data_raw_acceleration.z) / (float) 1000) *  9.80665;

	/* USER CODE BEGIN */

	/* USER CODE END */

#if FFT_ENABLE

	if (FFT_Acc.Calculate) {

//		Cal_Rms_value(&FFT_Acc.rms_acc, FFT_Acc.AccValue, FFT_Acc.index_buff);
//		Cal_Rms_velocity(&FFT_Acc.rms_vel, FFT_Acc.AccValue,
//				FFT_Acc.index_buff);
		if(index1 >= FFT_SIZE){

//		arm_rfft_fast_f32(&fft_instance, Sensor.ACC_Z, FFT_output_z, 0);

			for (int i = 0; i < FFT_SIZE; i++) {

			    FFT_output_z[(uint16_t)(2*i)] = Sensor.ACC_Z[i];    // real
			    FFT_output_z[(uint16_t)(2 * i + 1)] = 0.0f;       // imag = 0

			}

		arm_cfft_radix4_f32(&FFThandler, FFT_output_z);
		calculateMagnitudeArray(FFT_output_z, output_fft_mag_z, FFT_SIZE);
		//arm_cmplx_mag_f32(FFT_output_z, output_fft_mag_z , FFT_SIZE/2);
		//----------------------------------------------------------------------------------------------------------------------------------
		arm_max_f32(output_fft_mag_z, FFT_SIZE, &maxValue_z, &maxIndex_z);
		frequency = (float)maxIndex_z * (3200.0f / FFT_SIZE);

		find_top_peaks(output_fft_mag_z, peaks , FFT_SIZE); //peaks[i].value ; peaks[i].index ;
						for (int i = 0; i < TOP_N; i++) {
							peaks[i].peakFrequency_z = (float_t)((float)peaks[i].index) * ((float_t)((float)3200) / FFT_SIZE);
						}

		memset(FFT_output_z, 0, sizeof(FFT_output_z));
		memset(output_fft_mag_z, 0, sizeof(output_fft_mag_z)); //Reset arry

		}

		//-----------------------------------------------------------------------------------------------------------------------------------

		if(index2 >= 1024){

		calculate_velocity(Sensor.ACC_X,  Sensor.ACC_Y, Sensor.ACC_Z , 1024 , 3200 , frequency , &Arms_x, &Arms_y, &Arms_z, &Vrms_x, &Vrms_y, &Vrms_z);

		index1 = index2 = index3 = 0;
		}


		//Process_FFT_Input_buffer(&FFT_Acc, FFT_Acc.AccValue, 2);
		//arm_cfft_radix4_f32(&FFThandler, FFT_Acc.aFFT_Input_f32);
		//calculateMagnitude(FFT_Acc.aFFT_Input_f32, FFT_Acc.aFFT_Output_f32,
//				FFT_LENGTH);
//		FFT_Acc.aFFT_Output_f32[0] = 0;
//		arm_max_f32(FFT_Acc.aFFT_Output_f32, FFT_LENGTH, &FFT_Acc.maxValue,
//				&FFT_Acc.maxIndex);
//
		FFT_Acc.index_buff = FFT_Acc.maxValue = 0;
		FFT_Acc.Calculate = false;

		HAL_TIM_Base_Start_IT(ISM330DHCX_TIM);

	}

#endif
}

void Sensor_Run() {

		Measure_Temp_STTS22H() ;
		Acc_Calculation();

}

void Sensor_Log(){



	//MW_LOG(TS_OFF, VLEVEL_M, "Temp : %d | X : %d | Y : %d | Z : %d  \r\n", Sensor.temperature, Sensor.data_raw_acceleration.x, Sensor.data_raw_acceleration.y, Sensor.data_raw_acceleration.z) ;

//   float odr;
//   char odr_str[16];
//    if (BSP_MOTION_SENSOR_GetOutputDataRate(MOTION_SENSOR_ISM330DHCX_0, MOTION_ACCELERO, &odr) != BSP_ERROR_NONE) {
//    }else{
//    	snprintf(odr_str, sizeof(odr_str), "%.3f", odr);
//    	MW_LOG(TS_OFF, VLEVEL_M, "ODR[%d]: %s Hz\r\n", MOTION_SENSOR_ISM330DHCX_0, odr_str);
//    }


//	int32_t fullScale;
//
//	BSP_MOTION_SENSOR_GetFullScale(MOTION_SENSOR_ISM330DHCX_0, MOTION_ACCELERO, &fullScale);
//	MW_LOG(TS_OFF, VLEVEL_M, "Fullscal [%d]: %d mg\r\n", MOTION_SENSOR_ISM330DHCX_0, fullScale);

}

