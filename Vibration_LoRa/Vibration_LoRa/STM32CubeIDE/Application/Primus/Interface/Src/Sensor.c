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

#define START_MCU_LOAD			false

#define Acc_FS 16
#define Sampling_Rate 3200
#define SUPPRESSION_WIDTH 3

#if FFT_ENABLE
type_FFT_acc FFT_Acc;
#endif

static uint32_t Sensor_STTS22HInit(void);
static uint32_t Sensor_ISM330DHCXInit(void);

//--------------------------------------------------------
Peak_t peaks_acc[TOP_N]; //  Prak_acc

type_threshold_Base TH; // Threshold_noise

Velocity_t v;

Sensor_t Sensor;

float32_t Arms_x, Arms_y, Arms_z;
//--------------------------------------------------------


#define millis() __HAL_TIM_GET_COUNTER(ISM330DHCX_TIM)

uint32_t start_time = 0;
bool status_prev = 0;
int toggle_count;
bool tracking = false;
bool count = 0;

void Sensor_Errors(void)
{
  UTILS_ENTER_CRITICAL_SECTION();
  Sensor_DeInit();
  Sensor_Init();
  UTILS_EXIT_CRITICAL_SECTION();
}

uint32_t Sensor_Init(void) {

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
  if (BSP_MOTION_SENSOR_SetOutputDataRate(MOTION_SENSOR_ISM330DHCX_0, MOTION_ACCELERO, 3332u))
  {
    return 3;
  }
  /* USER CODE BEGIN Init */

   uint8_t WAKE_UP_DUR = 0x1F; //delay interrupt -> 2.459 sec
   BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_WAKE_UP_DUR , WAKE_UP_DUR);//  -> sleep mode

//   uint8_t CTRL8_XL = 0xE4; //HPF
//   BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_CTRL8_XL  ,  CTRL8_XL); //---------------------

   uint8_t WAKE_UP_THS = 0x01;
   BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_WAKE_UP_THS, WAKE_UP_THS);

   uint8_t TAP_CFG0  = 0x00; 	// slope -> 0x00 , HPF -> 0x10 // latched 0x41(slope) 0x51(HPF)
   BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_TAP_CFG0, TAP_CFG0); //----------------------------

   uint8_t TAP_CFG2  = 0xC0;
   BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0,ISM330DHCX_TAP_CFG2 , TAP_CFG2 );

   uint8_t MD1_CFG  = 0xE0;
   BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_MD1_CFG , MD1_CFG);

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

void wake_up_ISM330DHCX()
{

}
uint8_t Read_Status =0;
void Measure_Acc_ISM330DHCX() {
	if ( Sensor.bussy )
		return ;

	Sensor.bussy = 1 ;

//	return;

	if ( (Sensor.accelero_sensor_good == true) ) /*Accelerometer is working*/
	{

		/*Get accelerometer data*/
		if ( BSP_MOTION_SENSOR_GetAxesRaw(MOTION_SENSOR_ISM330DHCX_0, MOTION_ACCELERO, &(Sensor.data_raw_acceleration)) ) {
//      Sensor.accelero_sensor_good = false;
//			  BSP_I2C2_Init();
//			  Sensor_Errors();
			MX_I2C2_Init(&hi2c2) ;
			if ( Sensor.ISM330DHCX_fail++ >= 10 ) {
				Sensor.ISM330DHCX_fail = 0 ;
				FFT_Acc.Calculate = 0 ;
				HAL_TIM_Base_Stop_IT(ISM330DHCX_TIM) ;
#if START_MCU_LOAD
				UTIL_SEQ_PauseTask(1 << CFG_SEQ_Task_ApplicationLoop) ;
#endif
				MW_LOG(TS_OFF, VLEVEL_M, "###### ACCELERO_GYRO Sleep ######\r\n")
				;

			} else {

				MW_LOG(TS_OFF, VLEVEL_M, "###### ACCELERO_GYRO_Error : %d ######\r\n", Sensor.ISM330DHCX_fail)
				;
			}

		} else {
			Sensor.ISM330DHCX_fail = 0 ;
#if FFT_ENABLE

			if ( FFT_Acc.Calculate ) {

				uint8_t WAKE_UP_SRC ;
				BSP_MOTION_SENSOR_Read_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_WAKE_UP_SRC, &WAKE_UP_SRC) ;


				Sensor.status = (WAKE_UP_SRC & 0x10) ? 0 : 1 ;

				if ( Sensor.index2 < FFT_SIZE ) {

					int out_index =  Sensor.index2 * 2 ;

					Sensor.ACC_X[out_index] = ((ism330dhcx_from_fs16g_to_mg(Sensor.data_raw_acceleration.x) / 1000.0f) * 9.80665f) ;
					Sensor.ACC_Y[out_index] = ((ism330dhcx_from_fs16g_to_mg(Sensor.data_raw_acceleration.y) / 1000.0f) * 9.80665f) ;
					Sensor.ACC_Z[out_index] = (((ism330dhcx_from_fs16g_to_mg(Sensor.data_raw_acceleration.z) / 1000.0f) * 9.80665f) - 9.80665f);

					Sensor.ACC_X[out_index + 1] = 0.0f ;
					Sensor.ACC_Y[out_index + 1] = 0.0f ;
					Sensor.ACC_Z[out_index + 1] = 0.0f ;

#if Ennable_magnitude
					MW_LOG(TS_OFF, VLEVEL_M," $%d %d %d;",Sensor.data_raw_acceleration.x , Sensor.data_raw_acceleration.y , Sensor.data_raw_acceleration.z);
#endif
					 Sensor.index2 ++ ;

				} else {

					Acc_Calculation() ;
					if ( Sensor.status  == 0 ) {

						FFT_Acc.Calculate = 0 ;
						HAL_TIM_Base_Stop_IT(ISM330DHCX_TIM) ;
#if START_MCU_LOAD
						UTIL_SEQ_PauseTask(1 << CFG_SEQ_Task_ApplicationLoop) ;
#endif
						MW_LOG(TS_OFF, VLEVEL_M, "###### ACCELERO_GYRO Sleep ######\r\n")
						;

					}
				}

			}
#endif
		}

	}

	Sensor.bussy = 0 ;
}

#if FFT_ENABLE

void calculate_acc(float *x, float *y, float *z, uint16_t size, float *Arms_x, float *Arms_y, float *Arms_z){

    float sum_ax2 = 0, sum_ay2 = 0, sum_az2 = 0;

    for (uint16_t i = 0; i < size; i++) {
      	sum_ax2 += x[2*i] * x[2*i];
      	sum_ay2 += y[2*i] * y[2*i];
      	sum_az2 += z[2*i] * z[2*i];
      }

       *Arms_x = sqrtf(sum_ax2 / size);
       *Arms_y = sqrtf(sum_ay2 / size);
       *Arms_z = sqrtf(sum_az2 / size);
}

void calculate_velocity(Peak_t *peaks_acc, Velocity_t *v, uint16_t TOP) {

    for (int i = 0; i < TOP; i++) {
        float factor_x = 2.0f * M_PI * peaks_acc[i].peakFrequency_x * sqrtf(2.0f);
        float factor_y = 2.0f * M_PI * peaks_acc[i].peakFrequency_y * sqrtf(2.0f);
        float factor_z = 2.0f * M_PI * peaks_acc[i].peakFrequency_z * sqrtf(2.0f);

        v[i].rms_x = (peaks_acc[i].value_x / factor_x) * 1000.0f;
        v[i].rms_y = (peaks_acc[i].value_y / factor_y) * 1000.0f;
        v[i].rms_z = (peaks_acc[i].value_z / factor_z) * 1000.0f;

        v->rms_x_sum += v[i].rms_x  * v[i].rms_x ;
        v->rms_y_sum += v[i].rms_y  * v[i].rms_y ;
        v->rms_z_sum += v[i].rms_z  * v[i].rms_z;
    }
}

void calculateMagnitudeArray(float *input, float *output, int length) {
    for (int i = 0; i < length ; i++) {
        float real = input[2 * i];  ////////////////////////////////////////////////////////
        float imag = input[2 * i + 1];
        output[i] = sqrtf(real * real + imag * imag);
    }
}

void find_top_peaks_3axis(float mag_x[], float mag_y[], float mag_z[], Peak_t peaks_acc[], uint32_t fft_size )
{
    uint8_t used[fft_size];
    for (uint32_t i = 0; i < fft_size; i++) {
        used[i] = 0;
    }

    for (uint32_t n = 0; n < TOP_N; n++) {
        float max_val_x = 0.0f;
        float max_val_y = 0.0f;
        float max_val_z = 0.0f;

        uint32_t max_idx_x = 0;
        uint32_t max_idx_y = 0;
        uint32_t max_idx_z = 0;

        // find peak
        for (uint32_t i = 1; i < fft_size; i++) {
            if (!used[i]) {
                if (mag_x[i] > max_val_x) {
                    max_val_x = mag_x[i];
                    max_idx_x = i;
                }
                if (mag_y[i] > max_val_y) {
                    max_val_y = mag_y[i];
                    max_idx_y = i;
                }
                if (mag_z[i] > max_val_z) {
                    max_val_z = mag_z[i];
                    max_idx_z = i;
                }
            }
        }

        peaks_acc[n].value_x = max_val_x;
        peaks_acc[n].value_y = max_val_y;
        peaks_acc[n].value_z = max_val_z;

        peaks_acc[n].index_x = max_idx_x;
        peaks_acc[n].index_y = max_idx_y;
        peaks_acc[n].index_z = max_idx_z;

        for (int j = -(int)SUPPRESSION_WIDTH; j <= (int)SUPPRESSION_WIDTH; j++) {
            int idx_x = (int)max_idx_x + j;
            int idx_y = (int)max_idx_y + j;
            int idx_z = (int)max_idx_z + j;

            if (idx_x >= 0 && idx_x < fft_size) used[idx_x] = 1;
            if (idx_y >= 0 && idx_y < fft_size) used[idx_y] = 1;
            if (idx_z >= 0 && idx_z < fft_size) used[idx_z] = 1;
        }
    }
}

void Threshold_noise(float mag_z[], float E_sse_av , float  E_sse_sum , float E_sse1[] ) {

    float  R[FFT_SIZE / 2], S1[FFT_SIZE / 2]/*, R1[FFT_SIZE / 2]*/;
    float  t_peak_s[(FFT_SIZE / 2) + 2] /*, t_peak_r[(FFT_SIZE / 2) + 2]*/;
    float  Nsse = 51;

    for (int i = 0; i < FFT_SIZE / 2; i++) {
            R[i] = 0.0f;
            S1[i] = 0.0f;
            //R1[i] = 0.0f;
            E_sse1[i] = 0.0f;
        }

    t_peak_s[0] = mag_z[(FFT_SIZE / 2) - 1];
    t_peak_s[(FFT_SIZE / 2) + 1] = mag_z[0];

    for (int i = 1; i <= (FFT_SIZE / 2); i++) {
        t_peak_s[i] = mag_z[i - 1];
    }
    // Moving average
    for (int i = 0; i < (FFT_SIZE / 2); i++) {
        S1[i] = (t_peak_s[i] + t_peak_s[i + 1] + t_peak_s[i + 2]) / 3.0f;
         R[i] = (S1[i] != 0.0f) ? 1.0f / S1[i] : 0.0f;
    }
    // Smooth by FIR filter
    for(int i = 0 ; i < (FFT_SIZE / 2) ; i ++ ){
    	R[i] = (i < (Nsse-1)) ? R[i] * (1.0f / Nsse) : 0.0f;
    	E_sse1[i] = (R[i] != 0.0f) ? 1.0f / R[i] : 0.0f;
    }

}

#endif

void Acc_Calculation() {

	/*Convert accelerometer, mg/LSB  to m/s^2*/
//	Sensor.ACC_X[index1++] = (ism330dhcx_from_fs16g_to_mg(Sensor.data_raw_acceleration.x) / (float) 1000) *  9.80665;
//	Sensor.ACC_Y[index2++] = (ism330dhcx_from_fs16g_to_mg(Sensor.data_raw_acceleration.y) / (float) 1000) *  9.80665;
//	Sensor.ACC_Z[index3++] = (ism330dhcx_from_fs16g_to_mg(Sensor.data_raw_acceleration.z) / (float) 1000) *  9.80665;

	/* USER CODE BEGIN */

	MW_LOG(TS_OFF, VLEVEL_M, "###### ACCELERO_GYRO_Calculation ######\r\n") ;
	/* USER CODE END */

#if FFT_ENABLE

		if(Sensor.index2 == FFT_SIZE){
		calculate_acc(Sensor.ACC_X, Sensor.ACC_Y, Sensor.ACC_Z , FFT_SIZE , &Arms_x, &Arms_y, &Arms_z);
//-----------------------------------------------------------------------------------------------------------
		float32_t Hann_w ;
		for(int i = 0 ; i < FFT_SIZE ; i++){
				Hann_w  = 0.5f * (1.0f - arm_cos_f32((2.0f * PI * i) / (FFT_SIZE - 1)));
				Sensor.ACC_Z[i*2] *= Hann_w; Sensor.ACC_Y[i*2] *= Hann_w; Sensor.ACC_X[i*2] *= Hann_w;
		}

		arm_cfft_radix4_f32(&FFThandler, Sensor.ACC_Z);
		arm_cfft_radix4_f32(&FFThandler, Sensor.ACC_Y);
		arm_cfft_radix4_f32(&FFThandler, Sensor.ACC_X);

		arm_cmplx_mag_f32(Sensor.ACC_Z, Sensor.output_fft_mag_z , FFT_SIZE);
		arm_cmplx_mag_f32(Sensor.ACC_Y, Sensor.output_fft_mag_y , FFT_SIZE);
		arm_cmplx_mag_f32(Sensor.ACC_X, Sensor.output_fft_mag_x , FFT_SIZE);

		for(int i = 0 ; i < FFT_SIZE / 2 ; i++){
			Sensor.output_fft_mag_z[i] *=  2.0f / (FFT_SIZE / 2.0f);
			Sensor.output_fft_mag_y[i] *=  2.0f / (FFT_SIZE / 2.0f);
			Sensor.output_fft_mag_x[i] *=  2.0f / (FFT_SIZE / 2.0f); //A_peak
			}

		 Threshold_noise(Sensor.output_fft_mag_x , TH.E_sse_av_x , TH.E_sse_sum_x , TH.E_sse1_x); //------------------------
		 Threshold_noise(Sensor.output_fft_mag_y , TH.E_sse_av_y , TH.E_sse_sum_y , TH.E_sse1_y);
		 Threshold_noise(Sensor.output_fft_mag_z , TH.E_sse_av_z , TH.E_sse_sum_z , TH.E_sse1_z);
		 TH.E_sse_sum_z =  TH.E_sse_sum_x =  TH.E_sse_sum_y = TH.E_sse_av_z = TH.E_sse_av_x = TH.E_sse_av_y =0.0f;
		 for(int i = 0 ; i < (FFT_SIZE / 2) ; i++){
			 TH.E_sse_sum_x += TH.E_sse1_x[i];
			 TH.E_sse_sum_y += TH.E_sse1_y[i];
			 TH.E_sse_sum_z += TH.E_sse1_z[i];

		 }TH.E_sse_av_x = TH.E_sse_sum_x / (FFT_SIZE / 2) ;
		  TH.E_sse_av_y = TH.E_sse_sum_y / (FFT_SIZE / 2) ;
		  TH.E_sse_av_z = TH.E_sse_sum_z / (FFT_SIZE / 2) ;

		 TH.Ratio_x =  3 * TH.E_sse_av_x;
		 TH.Ratio_y =  3 * TH.E_sse_av_y;
		 TH.Ratio_z =  3 * TH.E_sse_av_z;//---------------------- threshold noise ------------

		find_top_peaks_3axis(Sensor.output_fft_mag_x ,Sensor.output_fft_mag_y , Sensor.output_fft_mag_z , peaks_acc, FFT_SIZE / 2 );

		for(int i = 0 ; i < TOP_N ; i++){
			if(peaks_acc[i].value_z > TH.Ratio_z && peaks_acc[i].value_y > TH.Ratio_y && peaks_acc[i].value_x > TH.Ratio_x ){
				TH.Ratio_TOP_N++;
			}
		}if(TH.Ratio_TOP_N == 0){
			TH.Ratio_TOP_N = 1 ;
		}

		for(int i = 0 ; i < TH.Ratio_TOP_N ; i++){
			peaks_acc[i].peakFrequency_z = ((float)peaks_acc[i].index_z) * (3200.0f / FFT_SIZE);
			peaks_acc[i].peakFrequency_y = ((float)peaks_acc[i].index_y) * (3200.0f / FFT_SIZE);
			peaks_acc[i].peakFrequency_x = ((float)peaks_acc[i].index_x) * (3200.0f / FFT_SIZE);
		}
		//----------------------------------------------------------------------------------------------------------------------------------

		calculate_velocity(peaks_acc, &v, TH.Ratio_TOP_N);
		v.rms_z_sqr = sqrtf(v.rms_z_sum);
		v.rms_y_sqr = sqrtf(v.rms_y_sum);
		v.rms_x_sqr = sqrtf(v.rms_x_sum);

		memset(Sensor.ACC_Z, 0, sizeof(Sensor.ACC_Z));
		memset(Sensor.ACC_Y, 0, sizeof(Sensor.ACC_Y));
		memset(Sensor.ACC_X, 0, sizeof(Sensor.ACC_X));
		memset(Sensor.output_fft_mag_z, 0, sizeof(Sensor.output_fft_mag_z)); //Reset array
		memset(Sensor.output_fft_mag_y, 0, sizeof(Sensor.output_fft_mag_y)); //Reset array
		memset(Sensor.output_fft_mag_x, 0, sizeof(Sensor.output_fft_mag_x)); //Reset array
		v.rms_z_sum = 0.0f; v.rms_y_sum = 0.0f; v.rms_x_sum = 0.0f; TH.Ratio_TOP_N = 0;


		 Sensor.index2 = 0;

	}

#endif
}

void Sensor_Run() {
//		Measure_Temp_STTS22H();
//		Acc_Calculation();

}

void Sensor_Log(){
	 //MW_LOG(TS_OFF, VLEVEL_M, " ------------------- \r\n" );
}
uint8_t Wake_up_cnt = 0;
uint32_t Wake_up_tick = 1000;
void ACCELERO_GYRO_INT_RUN() {

	if(HAL_GetTick() < Wake_up_tick)
		return;


	if (!FFT_Acc.Calculate ) {

		if(HAL_GetTick() > Wake_up_tick + 100){
			Wake_up_cnt = 0;
		}

		if(Wake_up_cnt++ < 10){
			Wake_up_tick = HAL_GetTick() + 500;
			MW_LOG(TS_OFF, VLEVEL_M, "###### ACCELERO_GYRO_Wake Up Cnt : %d ######\r\n", Wake_up_cnt) ;
			return;
		}

		Wake_up_cnt = 0;

		MW_LOG(TS_OFF, VLEVEL_M, "###### ACCELERO_GYRO_Wake Up ######\r\n") ;
		//BSP_MOTION_SENSOR_SetFullScale(MOTION_SENSOR_ISM330DHCX_0, MOTION_ACCELERO, 16 ); // 62.5 mg

//		uint8_t MD1_CFG  = 0xE0;
//		BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_MD1_CFG , MD1_CFG);
//		if(index1 >= 20){
//		index1 = 0;
//		}

		memset(Sensor.ACC_Z, 0, sizeof(Sensor.ACC_Z));
		memset(Sensor.ACC_Y, 0, sizeof(Sensor.ACC_Y));
		memset(Sensor.ACC_X, 0, sizeof(Sensor.ACC_X));
		memset(Sensor.output_fft_mag_z, 0, sizeof(Sensor.output_fft_mag_z)); //Reset array
		memset(Sensor.output_fft_mag_y, 0, sizeof(Sensor.output_fft_mag_y)); //Reset array
		memset(Sensor.output_fft_mag_x, 0, sizeof(Sensor.output_fft_mag_x)); //Reset array
		Sensor.index2 = 0;

		FFT_Acc.Calculate = 1;
		Sensor.Update_values = 0;


//		MX_I2C2_Init(&hi2c2);
#if START_MCU_LOAD
		MX_I2C2_Init(&hi2c2);
		UTIL_SEQ_ResumeTask(1 << CFG_SEQ_Task_ApplicationLoop);
#endif
		HAL_TIM_Base_Start_IT(ISM330DHCX_TIM);

	  }else{

			Wake_up_tick = HAL_GetTick() + 1000;
	  }
}
/*



 */

