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

#define START_MCU_LOAD	false
#define SUPPRESSION_WIDTH 2

#if FFT_ENABLE
type_FFT_acc FFT_Acc;
#endif
float Interpolated_bin_z ;
float Interpolated_bin_y ;
float Interpolated_bin_x ;

static uint32_t Sensor_STTS22HInit(void);
static uint32_t Sensor_ISM330DHCXInit(void);

void Full_scale (void);
void ODR_FS_setting (void);
void offset_working(void);
void wake_up_now_working(void);
void wake_up_working(void);

void input_and_Apeak(void);
void threshold_noise(void);
void peak(void);
void velocity(void);
void Reset(void);

float S1_x[512];
float S1_y[512];
float S1_z[512];
//float t_peak_s[512 + 2];  // Temporary buffer for edge mirroring
//float R[512];       // Inverse of S1[k]
//float R1[512];      // Smoothed inverse spectrum (after FIR)

bool count;
//--------------------------------------------------------
Sensor_t Sensor ={.input_and_Apeak = &input_and_Apeak};

threshold_t threshold = {
	.WAKE_UP_THS = 0x01,
//	.input_Arms = 0.001,
	.Fs = {.input_g = 4 , .reg = 0 },
	.ODR_sampling = { .reg = 0},
	.machine = {.freq = 1600},
		.Full_scale = &Full_scale ,
		.ODR_FS_setting = &ODR_FS_setting
};

Base_noise_t Base_noise = {
		  .threshold_noise = &threshold_noise
};

freq_t freq = {
    .peak = &peak,
};
sensor_App_t sensor_App ={
		.wake_up = 0,
		.offset = 0,
		.offset_working = &offset_working ,
		.wake_up_now_working = &wake_up_now_working,
		.wake_up_working = &wake_up_working,

		.offset1 = {.scale = 0.015625 , .reg_x = 0 , .reg_y = 0 , .reg_z = 0 }
};
v_t v = {.velocity = &velocity };

Sensor_t Sensor;
acc_t acc ;

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
//  if (BSP_MOTION_SENSOR_SetFullScale(MOTION_SENSOR_ISM330DHCX_0, MOTION_ACCELERO, threshold.Fs.input_g))
//  {
//    return 2;
//  }
//  if (BSP_MOTION_SENSOR_SetOutputDataRate(MOTION_SENSOR_ISM330DHCX_0, MOTION_ACCELERO, 1666u))
//  {
//    return 3;
//  }
  /* USER CODE BEGIN Init */

//   uint8_t CTRL8_XL = 0xE4; //HPF
//   BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_CTRL8_XL  ,  CTRL8_XL); //---------------------

  threshold.ODR_FS_setting();
  uint8_t CTRL1_XL = (threshold.Fs.reg | threshold.ODR_sampling.reg) ; //Setting -> Fs , sampling Rate
  BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_CTRL1_XL , CTRL1_XL);

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

	Sensor.ISM330DHCX_Tick++;

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
			threshold.Full_scale();
			//threshold.ODR();

			sensor_App.offset_working();
			sensor_App.wake_up_now_working();
			sensor_App.wake_up_working();

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
void calculate_velocity( freq_t *freq, uint16_t TOP) {

    for (int i = 0; i < TOP ; i++) {
        float factor_x = 2.0f * M_PI * freq->peaks_acc[i].output_Hz_x * sqrtf(2.0f);
        float factor_y = 2.0f * M_PI * freq->peaks_acc[i].output_Hz_y * sqrtf(2.0f);
        float factor_z = 2.0f * M_PI * freq->peaks_acc[i].output_Hz_z * sqrtf(2.0f);

        freq->peaks_acc[i].Apeak_use_x = Sensor.output_fft_mag_x[freq->peaks_acc[i].index_x - 1] + Sensor.output_fft_mag_x[freq->peaks_acc[i].index_x + 1];
        freq->peaks_acc[i].Apeak_use_y = Sensor.output_fft_mag_y[freq->peaks_acc[i].index_y - 1] + Sensor.output_fft_mag_y[freq->peaks_acc[i].index_y + 1];
        freq->peaks_acc[i].Apeak_use_z = Sensor.output_fft_mag_z[freq->peaks_acc[i].index_z - 1] + Sensor.output_fft_mag_z[freq->peaks_acc[i].index_z + 1];

        v.rms_x_sqr[i] = ( freq->peaks_acc[i].Apeak_use_x/ factor_x);
        v.rms_y_sqr[i] = ( freq->peaks_acc[i].Apeak_use_y/ factor_y);
        v.rms_z_sqr[i] = ( freq->peaks_acc[i].Apeak_use_z/ factor_z);

        v.rms_x = (v.rms_x_sqr[i] < DBL_MIN) ? 0 : v.rms_x_sqr[i];
        v.rms_y = (v.rms_y_sqr[i] < DBL_MIN) ? 0 : v.rms_y_sqr[i];
        v.rms_z = (v.rms_z_sqr[i] < DBL_MIN) ? 0 : v.rms_z_sqr[i];

        v.rms_x_sum += v.rms_x * v.rms_x;
        v.rms_y_sum += v.rms_y * v.rms_y;
        v.rms_z_sum += v.rms_z * v.rms_z;
    }
}
//void calculate_velocity_time( float *z, uint16_t size, float sampling_rate, Velocity_t *v) {
//
//	float32_t vz[size];	 vz[0] = 0;
//	float32_t sum_vz = 0.0f ;
//	float32_t dt = 1.0f / sampling_rate;
//
//	float32_t mean_z = 0.0f;
//	    for (uint16_t i = 0; i < size; i++) {
//	        mean_z += z[i*2];
//	    }mean_z /= size;
//
//	    for (uint16_t i = 0; i < size; i++) {
//	        z[i*2] -= mean_z;
//	    }
////	    float32_t init_velocity = 0.0f;
////	    for (uint16_t i = 1; i < size; i++) {
////	        init_velocity += z[i*2] * dt;
////	    }vz[0] = init_velocity;
//	   for (uint16_t i = 1; i < size; i++) {
//	        vz[i] =  vz[i - 1] +  (z[2*i] * dt);
//	        sum_vz +=  vz[i] * vz[i];
//	    }
//	    v->rms_z_sqr =  sqrtf(sum_vz / size)* 1000;
//}
//void high_pass_filter(float *data, uint16_t size_FFT, float sampling_rate, float cutoff_freq) {
//
//    float RC = 1.0f / (2.0f * M_PI * cutoff_freq);
//    float dt = 1.0f / sampling_rate;
//    float alpha = RC / (RC + dt);
//
//    float prev_input = data[0];
//    float prev_output = data[0];
//
//    for (uint16_t i = 1; i < size_FFT * 2; i++) {
//        float current_input = data[i * 2];
//        data[i * 2] = alpha * (prev_output + current_input - prev_input);
//
//        prev_input = current_input;
//        prev_output = data[i * 2];
//    }
//}

void calculateMagnitudeArray(float *input, float *output, int length) {
    for (int i = 0; i < length ; i++) {
        float real = input[2 * i];
        float imag = input[2 * i + 1];
        output[i] = sqrtf(real * real + imag * imag);
    }
}
void find_top_peaks_3axis(float mag_x[], float mag_y[], float mag_z[], freq_t *freq, uint32_t fft_size )
{
    uint8_t used[fft_size];
    for (uint32_t i = 0; i < fft_size; i++) {
        used[i] = 0;
    }
    // haven't index 1
    used[1] = 1;

    for (uint32_t n = 0; n < TOP_N; n++) {
        float max_val_x = 0.0f;
        float max_val_y = 0.0f;
        float max_val_z = 0.0f;

        uint32_t max_idx_x = 0;
        uint32_t max_idx_y = 0;
        uint32_t max_idx_z = 0;

        // find peak
        for (uint32_t i = 3; i < fft_size; i++) { // 9.375
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

        freq->peaks_acc[n].Apeak_S1_x = max_val_x;
        freq->peaks_acc[n].Apeak_S1_y = max_val_y;
        freq->peaks_acc[n].Apeak_S1_z = max_val_z;

        freq->peaks_acc[n].index_x = max_idx_x;
        freq->peaks_acc[n].index_y = max_idx_y;
        freq->peaks_acc[n].index_z = max_idx_z;

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


void Threshold_E_sse(float mag_z[], float E_sse1[], float S1[]) {
    int half_size = FFT_SIZE / 2;
    int Nsse = 51;  // Length of the FIR smoothing filter

    // Step 1–2: Combined – 3-point moving average with manual edge wrapping
    for (int i = 0; i < half_size; i++) {
        float prev = (i == 0) ? mag_z[half_size - 1] : mag_z[i - 1];
        float curr = mag_z[i];
        float next = (i == half_size - 1) ? mag_z[0] : mag_z[i + 1];
        S1[i] = (prev + curr + next) / 3.0f;
    }

    // Step 3–5: Inverse, FIR smoothing, and final SSE combined
    for (int k = 0; k < half_size; k++) {
        float sum = 0.0f;
        for (int n = 0; n < Nsse; n++) {
            int idx = (k - n + half_size) % half_size;
            float inv = (S1[idx] > 0.0f) ? (1.0f / S1[idx]) : 0.0f;
            sum += inv;
        }

        float avg_inv = sum / Nsse;
        E_sse1[k] = (avg_inv > 0.0f) ? (1.0f / avg_inv) : 0.0f;
    }
}

void Reset(){
	memset(Sensor.ACC_Z, 0, sizeof(Sensor.ACC_Z));
	memset(Sensor.ACC_Y, 0, sizeof(Sensor.ACC_Y));
	memset(Sensor.ACC_X, 0, sizeof(Sensor.ACC_X));
	memset(Sensor.output_fft_mag_z, 0, sizeof(Sensor.output_fft_mag_z)); //Reset array
	memset(Sensor.output_fft_mag_y, 0, sizeof(Sensor.output_fft_mag_y)); //Reset array
	memset(Sensor.output_fft_mag_x, 0, sizeof(Sensor.output_fft_mag_x)); //Reset array
//	memset(freq.peaks_acc, 0, sizeof(freq.peaks_acc));
	Sensor.index2 = 0;
}

//---------------------------------------------------------------------------------------------------------------

void Full_scale (){
switch (threshold.Fs.input_g) {
	case 2:
		threshold.Fs.x = ism330dhcx_from_fs2g_to_mg(Sensor.data_raw_acceleration.x);
		threshold.Fs.y = ism330dhcx_from_fs2g_to_mg(Sensor.data_raw_acceleration.y);
		threshold.Fs.z = ism330dhcx_from_fs2g_to_mg(Sensor.data_raw_acceleration.z);
		break;
	case 4:
		threshold.Fs.x = ism330dhcx_from_fs4g_to_mg(Sensor.data_raw_acceleration.x);
		threshold.Fs.y = ism330dhcx_from_fs4g_to_mg(Sensor.data_raw_acceleration.y);
		threshold.Fs.z = ism330dhcx_from_fs4g_to_mg(Sensor.data_raw_acceleration.z);
		break;
	case 8 :
		threshold.Fs.x = ism330dhcx_from_fs8g_to_mg(Sensor.data_raw_acceleration.x);
		threshold.Fs.y = ism330dhcx_from_fs8g_to_mg(Sensor.data_raw_acceleration.y);
		threshold.Fs.z = ism330dhcx_from_fs8g_to_mg(Sensor.data_raw_acceleration.z);
		break;
	case 16 :
		threshold.Fs.x = ism330dhcx_from_fs16g_to_mg(Sensor.data_raw_acceleration.x);
		threshold.Fs.y = ism330dhcx_from_fs16g_to_mg(Sensor.data_raw_acceleration.y);
		threshold.Fs.z = ism330dhcx_from_fs16g_to_mg(Sensor.data_raw_acceleration.z);
		break;
	default:
		break;
 }

//threshold.test = ((threshold.input_Arms * sqrtf(2)) / 9.80665) * (256.0f / (float32_t)threshold.Fs.input_g);
//threshold.WAKE_UP_THS = (uint8_t)(threshold.test);
//if(threshold.test <= 1.0f){
//	threshold.WAKE_UP_THS = 0x01;
//}


//threshold.slope = (threshold.input_Arms * sqrtf(2) * (256.0f / 4.0f) *  arm_sin_f32(2 * PI * 640 * (1.0f / 6667.0f))) / (9.80665 *  2.0f);
//threshold.WAKE_UP_THS = (uint8_t)((threshold.slope));
//if(threshold.slope  <= 1.0f){
//	threshold.WAKE_UP_THS = 0x01;
//}

}

void ODR_FS_setting(){

	threshold.ODR_sampling.reg = 0b10010000; // 6667 Hz

	switch (threshold.Fs.input_g) {
		case 2:threshold.Fs.reg = 0b00000000;break;
		case 4:threshold.Fs.reg = 0b00001000;break;
		case 8:threshold.Fs.reg = 0b00001100;break;
		case 16:threshold.Fs.reg = 0b00000100;break;
		default:break;
	 }
}

void offset_working(){
	if( Sensor.index2 < FFT_SIZE && sensor_App.offset){
		sensor_App.offset1.sum_x += (threshold.Fs.x / 1000.0f) ;
		sensor_App.offset1.sum_y += (threshold.Fs.y / 1000.0f) ;
		sensor_App.offset1.sum_z += (threshold.Fs.z / 1000.0f) ;

		Sensor.index2++;

	}else if(sensor_App.offset){

		sensor_App.offset1.x = sensor_App.offset1.sum_x / (float)FFT_SIZE;
		sensor_App.offset1.y = sensor_App.offset1.sum_y / (float)FFT_SIZE;
		sensor_App.offset1.z = sensor_App.offset1.sum_z / (float)FFT_SIZE;

		uint8_t CTRL7_G = 0x02 ; // accelerometer user offset correction block enabled
		BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_CTRL7_G  ,  CTRL7_G );
	    uint8_t CTRL6_C = 0x08 ; // 0x00: 2^-10 g/LSB     0x08: 2^-6 g/LSB
	    BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_CTRL6_C  ,  CTRL6_C);
	    sensor_App.offset1.reg_x = (int8_t)(sensor_App.offset1.x / sensor_App.offset1.scale) ;
	    sensor_App.offset1.reg_y = (int8_t)(sensor_App.offset1.y / sensor_App.offset1.scale) ;
	    sensor_App.offset1.reg_z = (int8_t)(sensor_App.offset1.z / sensor_App.offset1.scale) ;
	    BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_X_OFS_USR  ,  sensor_App.offset1.reg_x );
	    BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_Y_OFS_USR  ,  sensor_App.offset1.reg_y );
	    BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_Z_OFS_USR  ,  sensor_App.offset1.reg_z );

		   uint8_t WAKE_UP_DUR = 0x1F; //delay interrupt -> x sec
		   BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_WAKE_UP_DUR , WAKE_UP_DUR); // -> sleep mode

		   uint8_t WAKE_UP_THS = threshold.WAKE_UP_THS;
		   BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_WAKE_UP_THS, WAKE_UP_THS);

		   uint8_t TAP_CFG0  = 0x00; 	// slope -> 0x00 , HPF -> 0x10 // latched 0x41(slope) 0x51(HPF)
		   BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_TAP_CFG0, TAP_CFG0); //----------------------------

		   uint8_t TAP_CFG2  = 0xC0;
		   BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0,ISM330DHCX_TAP_CFG2 , TAP_CFG2 );

		   uint8_t MD1_CFG  = 0xE0;
		   BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_MD1_CFG , MD1_CFG);

		 if(!Sensor.status_wk){HAL_TIM_Base_Stop_IT(ISM330DHCX_TIM) ;}

		 Sensor.index2 = 0;
		 sensor_App.offset  = 0;
	}
}
void wake_up_now_working(){
	if( Sensor.index2 < FFT_SIZE && sensor_App.wake_up ){
		int out_index =  Sensor.index2 * 2 ;

//		Sensor.ACC_X[out_index] = threshold.Fs.x ;
//		Sensor.ACC_Y[out_index] = threshold.Fs.y ;
//		Sensor.ACC_Z[out_index] = threshold.Fs.z ;
		Sensor.ACC_X[out_index] = ((threshold.Fs.x / 1000.0f) * 9.80665f) ;
		Sensor.ACC_Y[out_index] = ((threshold.Fs.y / 1000.0f) * 9.80665f) ;
		Sensor.ACC_Z[out_index] = ((threshold.Fs.z / 1000.0f) * 9.80665f) ;
		Sensor.ACC_X[out_index + 1] = 0.0f ;
		Sensor.ACC_Y[out_index + 1] = 0.0f ;
		Sensor.ACC_Z[out_index + 1] = 0.0f ;

		Sensor.index2++ ;

	}else if(sensor_App.wake_up ){

		   uint8_t WAKE_UP_DUR = 0x1F; //delay interrupt -> x sec
		   BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_WAKE_UP_DUR , WAKE_UP_DUR); // -> sleep mode

		   uint8_t WAKE_UP_THS = threshold.WAKE_UP_THS;
		   BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_WAKE_UP_THS, WAKE_UP_THS);

		   uint8_t TAP_CFG0  = 0x00; 	// slope -> 0x00 , HPF -> 0x10 // latched 0x41(slope) 0x51(HPF)
		   BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_TAP_CFG0, TAP_CFG0); //----------------------------

		   uint8_t TAP_CFG2  = 0xC0;
		   BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0,ISM330DHCX_TAP_CFG2 , TAP_CFG2 );

		   uint8_t MD1_CFG  = 0xE0;
		   BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_MD1_CFG , MD1_CFG);

		 Acc_Calculation();
		 if(!Sensor.status_wk){HAL_TIM_Base_Stop_IT(ISM330DHCX_TIM) ; }
	}
}
void wake_up_working(){
	if ( FFT_Acc.Calculate && !(sensor_App.wake_up || sensor_App.offset) ) {

		uint8_t WAKE_UP_SRC ;
		BSP_MOTION_SENSOR_Read_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_WAKE_UP_SRC, &WAKE_UP_SRC) ;

		Sensor.status = (WAKE_UP_SRC & 0x10) ? 0 : 1 ;

		if ( Sensor.index2 < FFT_SIZE ) {

			int out_index =  Sensor.index2 * 2 ;

//			Sensor.ACC_X[out_index] = threshold.Fs.x ;
//			Sensor.ACC_Y[out_index] = threshold.Fs.y ;
//			Sensor.ACC_Z[out_index] = threshold.Fs.z ;
			Sensor.ACC_X[out_index] = ((threshold.Fs.x / 1000.0f) * 9.80665f) ;
			Sensor.ACC_Y[out_index] = ((threshold.Fs.y / 1000.0f) * 9.80665f) ;
			Sensor.ACC_Z[out_index] = ((threshold.Fs.z / 1000.0f) * 9.80665f) ;
			Sensor.ACC_X[out_index + 1] = 0.0f ;
			Sensor.ACC_Y[out_index + 1] = 0.0f ;
			Sensor.ACC_Z[out_index + 1] = 0.0f ;

#if Ennable_magnitude
			MW_LOG(TS_OFF, VLEVEL_M," $%d %d %d;",(int)(Sensor.ACC_X[out_index]*1000.0f) , (int)(Sensor.ACC_Y[out_index]*1000.0f) , (int)(Sensor.ACC_Z[out_index] * 1000.0f));
#endif
			 Sensor.index2++ ;

		} else {

			Acc_Calculation() ;
			if ( Sensor.status  == 0 ) {

				FFT_Acc.Calculate = 0 ;
				Sensor.status_wk = 0;
				HAL_TIM_Base_Stop_IT(ISM330DHCX_TIM) ;
				//HAL_TIM_Base_Stop_IT(&htim16) ;
#if START_MCU_LOAD
				UTIL_SEQ_PauseTask(1 << CFG_SEQ_Task_ApplicationLoop) ;
#endif
				MW_LOG(TS_OFF, VLEVEL_M, "###### ACCELERO_GYRO Sleep ######\r\n")
				;

			}
		}

	}
}

//-----------------------------------------------------------------------------------------------------------------

void input_and_Apeak(){
	float32_t Hann_w ;
	for(int i = 0 ; i < FFT_SIZE ; i++){
			Hann_w  = 0.5f * (1.0f - arm_cos_f32((2.0f * PI * i) / (FFT_SIZE - 1)));
			Sensor.ACC_Z[i*2] *= Hann_w; Sensor.ACC_Y[i*2] *= Hann_w; Sensor.ACC_X[i*2] *= Hann_w;
	}

	arm_cfft_radix4_f32(&FFThandler , Sensor.ACC_Z);
	arm_cfft_radix4_f32(&FFThandler , Sensor.ACC_Y);
	arm_cfft_radix4_f32(&FFThandler , Sensor.ACC_X);

	arm_cmplx_mag_f32(Sensor.ACC_Z , Sensor.output_fft_mag_z , FFT_SIZE);
	arm_cmplx_mag_f32(Sensor.ACC_Y , Sensor.output_fft_mag_y , FFT_SIZE);
	arm_cmplx_mag_f32(Sensor.ACC_X , Sensor.output_fft_mag_x , FFT_SIZE);

	for(int i = 0 ; i < FFT_SIZE / 2 ; i++){
		Sensor.output_fft_mag_z[i] *=  2.0f / (FFT_SIZE / 2.0f);
		Sensor.output_fft_mag_y[i] *=  2.0f / (FFT_SIZE / 2.0f);
		Sensor.output_fft_mag_x[i] *=  2.0f / (FFT_SIZE / 2.0f); //A_peak

	}
}
void threshold_noise(){
	Threshold_E_sse(Sensor.output_fft_mag_x , Base_noise.E_sse1_x ,S1_x );
	Threshold_E_sse(Sensor.output_fft_mag_y , Base_noise.E_sse1_y ,S1_y );
	Threshold_E_sse(Sensor.output_fft_mag_z , Base_noise.E_sse1_z ,S1_z );

	Base_noise.E_sse_sum_z =  Base_noise.E_sse_sum_x = Base_noise.E_sse_sum_y =
	Base_noise.E_sse_av_z = Base_noise.E_sse_av_x = Base_noise.E_sse_av_y =0.0f;

	for(int i = 0 ; i < (FFT_SIZE / 2) ; i++){
		Base_noise.E_sse_sum_x += Base_noise.E_sse1_x[i];
		Base_noise.E_sse_sum_y += Base_noise.E_sse1_y[i];
		Base_noise.E_sse_sum_z += Base_noise.E_sse1_z[i];

	}Base_noise.E_sse_av_x = Base_noise.E_sse_sum_x / (FFT_SIZE / 2) ;
	Base_noise.E_sse_av_y = Base_noise.E_sse_sum_y / (FFT_SIZE / 2) ;
	Base_noise.E_sse_av_z = Base_noise.E_sse_sum_z / (FFT_SIZE / 2) ;

	Base_noise.Ratio_x =  5 * Base_noise.E_sse_av_x;
	Base_noise.Ratio_y =  5 * Base_noise.E_sse_av_y;
	Base_noise.Ratio_z =  5 * Base_noise.E_sse_av_z;

}

void peak(){
	//find_top_peaks_3axis(Sensor.output_fft_mag_x ,Sensor.output_fft_mag_y , Sensor.output_fft_mag_z , &freq , FFT_SIZE / 2 );
	find_top_peaks_3axis(S1_x , S1_y , S1_z , &freq , FFT_SIZE / 2 );

#if Ennable_s
	if(!count){
	 for (int i = 0; i < 512; i++) {
	MW_LOG(TS_OFF, VLEVEL_M," $%d %d %d;",(int)(S1_x[i] * 1000.0f) , (int)(S1_y[i] * 1000.0f) , (int)(S1_z[i] * 1000.0f));
//	MW_LOG(TS_OFF, VLEVEL_M," $%d %d %d;",(int)(Sensor.output_fft_mag_x[i] * 1000.0f) , (int)(Sensor.output_fft_mag_y[i] * 1000.0f)
//			, (int)(Sensor.output_fft_mag_z[i] * 1000.0f));
	 }
	 count = 1;
	}
#endif

	threshold.machine.Sampling_Rate = (float32_t)threshold.machine.freq * 2;

	for(int i = 0 ; i < TOP_N ; i++){
		if(freq.peaks_acc[i].Apeak_S1_x > Base_noise.Ratio_x
				&& freq.peaks_acc[i].Apeak_S1_y > Base_noise.Ratio_y
				&& freq.peaks_acc[i].Apeak_S1_z > Base_noise.Ratio_z){
			Base_noise.Ratio_TOP_N++;
		}
	}if(Base_noise.Ratio_TOP_N == 0){
		Base_noise.Ratio_TOP_N = 1 ;
	}

	for(int i = 0 ; i < Base_noise.Ratio_TOP_N ; i++){

		freq.Interpolated_bin.x = ((Sensor.output_fft_mag_x[freq.peaks_acc[i].index_x - 1] - Sensor.output_fft_mag_x[freq.peaks_acc[i].index_x + 1])/
				((Sensor.output_fft_mag_x[freq.peaks_acc[i].index_x - 1] - (2.0f *(float)freq.peaks_acc[i].index_x) + Sensor.output_fft_mag_x[freq.peaks_acc[i].index_x + 1])))* (1.0f/2.0f);
		freq.Interpolated_bin.y = ((Sensor.output_fft_mag_y[freq.peaks_acc[i].index_y - 1] - Sensor.output_fft_mag_y[freq.peaks_acc[i].index_y + 1])/
				((Sensor.output_fft_mag_y[freq.peaks_acc[i].index_y - 1] - (2.0f *(float)freq.peaks_acc[i].index_y) + Sensor.output_fft_mag_y[freq.peaks_acc[i].index_y + 1])))* (1.0f/2.0f);
		freq.Interpolated_bin.z = ((Sensor.output_fft_mag_z[freq.peaks_acc[i].index_z - 1] - Sensor.output_fft_mag_z[freq.peaks_acc[i].index_z + 1])/
				((Sensor.output_fft_mag_z[freq.peaks_acc[i].index_z - 1] - (2.0f * Sensor.output_fft_mag_z[freq.peaks_acc[i].index_z]) + Sensor.output_fft_mag_z[freq.peaks_acc[i].index_z + 1]))) * (1.0f/2.0f);

		freq.peaks_acc[i].output_Hz_x = (freq.Interpolated_bin.x * (threshold.machine.Sampling_Rate / (float)FFT_SIZE))
				+ (((float)freq.peaks_acc[i].index_x) * (threshold.machine.Sampling_Rate / (float)FFT_SIZE));

		freq.peaks_acc[i].output_Hz_y = (freq.Interpolated_bin.y * (threshold.machine.Sampling_Rate / (float)FFT_SIZE))
				+ (((float)freq.peaks_acc[i].index_y) * (threshold.machine.Sampling_Rate / (float)FFT_SIZE));

		freq.peaks_acc[i].output_Hz_z = (freq.Interpolated_bin.z * (threshold.machine.Sampling_Rate / (float)FFT_SIZE))
				+ (((float)freq.peaks_acc[i].index_z ) * (threshold.machine.Sampling_Rate / (float)FFT_SIZE));

	}
}

void velocity(){
	calculate_velocity(&freq, Base_noise.Ratio_TOP_N);
	v.output_sqr_z = sqrtf(v.rms_z_sum)* 1000.0f;
	v.output_sqr_y = sqrtf(v.rms_y_sum)* 1000.0f;
	v.output_sqr_x = sqrtf(v.rms_x_sum)* 1000.0f;

	v.output_Addition_vectors = sqrtf(v.rms_x_sum + v.rms_y_sum + v.rms_z_sum) * 1000.0f;
}

#endif

void Acc_Calculation() {

	/*Convert accelerometer, mg/LSB  to m/s^2*/

	/* USER CODE BEGIN */

	MW_LOG(TS_OFF, VLEVEL_M, "###### ACCELERO_GYRO_Calculation ######\r\n") ;
	/* USER CODE END */

#if FFT_ENABLE

		if(Sensor.index2 == FFT_SIZE){
		calculate_acc(Sensor.ACC_X, Sensor.ACC_Y, Sensor.ACC_Z , FFT_SIZE , &acc.output_rms_x, &acc.output_rms_y, &acc.output_rms_z);
		acc.output_Addition_vectors = sqrtf((acc.output_rms_x * acc.output_rms_x) + (acc.output_rms_y * acc.output_rms_y)
				+ (acc.output_rms_z * acc.output_rms_z));

//-----------------------------------------------------------------------------------------------------------
#if Ennable_HPF
		HPF.CUTOFF_FREQ = 2.0f * ((float)Sampling_Rate / (float)FFT_SIZE);
		high_pass_filter(Sensor.ACC_Z , FFT_SIZE , Sampling_Rate , HPF.CUTOFF_FREQ);
#endif
//		high_pass_filter(Sensor.ACC_Z , 1024, 3200 , 50) ;
//		high_pass_filter(Sensor.ACC_Y , 1024, 3200 , 50) ;
//		high_pass_filter(Sensor.ACC_X , 1024, 3200 , 50) ;

		Sensor.input_and_Apeak();

		Base_noise.threshold_noise();

		freq.peak();

		v.velocity();


		Reset();
		v.rms_z_sum = 0.0f; v.rms_y_sum = 0.0f; v.rms_x_sum = 0.0f; Base_noise.Ratio_TOP_N = 0;
		sensor_App.wake_up  = 0;
		sensor_App.offset  = 0;

	}

#endif
}

void Sensor_Run() {
//		Measure_Temp_STTS22H();
//		Acc_Calculation();

}

void Sensor_Log(){
//	 MW_LOG(TS_OFF, VLEVEL_M, " ---------- minggggggggggg  --------- \r\n");

	if(sensor_App.wake_up || sensor_App.offset ){

//		uint8_t CTRL1_XL = (threshold.Fs.reg | threshold.ODR_sampling.reg) ; //Setting -> Fs , sampling Rate
//		BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_CTRL1_XL , CTRL1_XL);

		if( sensor_App.offset ){
		BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_CTRL7_G  ,  0x00 );
	    BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_CTRL6_C  ,  0x00);
	    BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_X_OFS_USR  , 0x00 );
	    BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_Y_OFS_USR  , 0x00 );
	    BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_Z_OFS_USR  ,  0x00 );
		}

		BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_WAKE_UP_DUR , 0x00);
		BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_WAKE_UP_THS, 0x00);
		BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_TAP_CFG0, 0x00);
		BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0,ISM330DHCX_TAP_CFG2 , 0x00 );
		BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_MD1_CFG , 0x00);

	sensor_App.offset1.sum_x = sensor_App.offset1.sum_y = sensor_App.offset1.sum_z = 0;
	HAL_TIM_Base_Start_IT(ISM330DHCX_TIM);
	}
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

		Reset();
		FFT_Acc.Calculate = 1;
		Sensor.status_wk = 1;
		Sensor.Update_values = 0;


//		MX_I2C2_Init(&hi2c2);
#if START_MCU_LOAD
		MX_I2C2_Init(&hi2c2);
		UTIL_SEQ_ResumeTask(1 << CFG_SEQ_Task_ApplicationLoop);
#endif
		HAL_TIM_Base_Start_IT(ISM330DHCX_TIM);
		//HAL_TIM_Base_Start_IT(&htim16) ;

	  }else{

			Wake_up_tick = HAL_GetTick() + 1000;
	  }
}
/*



 */

