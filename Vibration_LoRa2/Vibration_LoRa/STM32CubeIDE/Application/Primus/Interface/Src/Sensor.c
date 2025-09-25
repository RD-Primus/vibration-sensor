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
void Train_working(void);
void wake_up_now_working(void);
void wake_up_working(void);

void input_and_Apeak(void);
void threshold_noise(void);
void peak(void);
void velocity(void);
void displacement_c(void);
void Reset(void);

float S1_x[512];
float S1_y[512];
float S1_z[512];
float val_x , val_y , val_z , max_val;
//float t_peak_s[512 + 2];  // Temporary buffer for edge mirroring
//float R[512];       // Inverse of S1[k]
//float R1[512];      // Smoothed inverse spectrum (after FIR)


int count_cycle = 0;
//--------------------------------------------------------
Sensor_t Sensor ={.input_and_Apeak = &input_and_Apeak};

threshold_t threshold = {
	.WAKE_UP_THS = 0x01,

	.Fs = { .reg = 0 },
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
		.Train = 0,

		//TODO User config
		.input_Vrms = 0,
		.input_RPM = 9552,  // 159.2Hz
		//cal
		.input_FS_g = 4 ,
		//

		.offset_working = &offset_working ,
		.wake_up_now_working = &wake_up_now_working,
		.wake_up_working = &wake_up_working,
		.Train_working = &Train_working,

		.offset1 = {.scale = 0.015625 , .reg_x = 0 , .reg_y = 0 , .reg_z = 0 }

};
v_t v = {.velocity = &velocity};

displacement_t displacement = {
		.displacement_c = &displacement_c
};

calibration_t calibration;
Sensor_t Sensor;
acc_t acc ;
zeroCross_t zeroCross;

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

  sensor_App.Arms = (sensor_App.input_Vrms / 1000.0f) * (2.0f * PI * (sensor_App.input_RPM * 0.016667)) ; // Arms = Vrms * 2 pi f

  threshold.slope = (sensor_App.Arms * sqrtf(2) * (256.0f /(float)sensor_App.input_FS_g ) *
  		arm_sin_f32(2 * PI * (sensor_App.input_RPM * 0.016667) * (1.0f / 6667.0f))) / (9.80665 *  2.0f);

  threshold.WAKE_UP_THS = (uint8_t)((threshold.slope))| 0x40;
  if(threshold.slope  <= 1.0f){
  	threshold.WAKE_UP_THS = 0x01 | 0x40;
  }else if(threshold.slope >= 63.0f){
  	threshold.WAKE_UP_THS = 0x3F | 0x40;
  }

// uint8_t CTRL8_XL  = 0x01;
// BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_CTRL8_XL, CTRL8_XL); // LPF , 6667 / 4

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

			sensor_App.offset_working();//offset = หาค่าออฟเซต
			sensor_App.Train_working();//Train = หาค่า g ใหม่
			sensor_App.wake_up_now_working();//Wake up = บังคับตื่น
			sensor_App.wake_up_working();

#endif
		}

	}

	Sensor.bussy = 0 ;
}
#if FFT_ENABLE

void calibration1(float freq , float *calibration){

//	if(freq < 15.625){
//		*calibration = 100 - (110.8099857 * expf((-0.024695105) * freq));
//	}else if(freq >= 15.625 && freq < 40.625){
//		*calibration = 100 - (109.8316476 * expf((-0.024135185) * freq));
//	}else if(freq >= 40.625 && freq < 81.25){
//		*calibration = 100 - (78.60833343 * expf((-0.015803623) * freq));
//	}else if(freq >= 81.25 && freq < 159.375){
//		*calibration = 100 - (43.7482519 * expf((-0.008489938) * freq));
//	}else if(freq >= 159.375 && freq < 321.875){
//		*calibration = 100 - (22.93071884 * expf((-0.004432591) * freq));
//	}else if(freq >= 321.875 && freq < 640.625){
//		*calibration = ( 0.009812 * freq ) + 91.3 ;
//	}else if(freq >= 640.625){
//		*calibration = ( 0.002797 * freq ) + 95.79 ;
//	}

	if(freq < 15.625){
			*calibration = 1;
		}else if(freq >= 15.625 && freq < 40.625){
			*calibration = 1;
		}else if(freq >= 40.625 && freq < 81.25){
			*calibration = 1;
		}else if(freq >= 81.25 && freq < 159.375){
			*calibration = 1;
		}else if(freq >= 159.375 && freq < 321.875){
			//*calibration = (0.0163 * freq ) - 0.1179584;
			*calibration = 1;
		}else if(freq >= 321.875 && freq < 640.625){
			//*calibration = (0.0466 * freq ) - 9.8194838;
			*calibration = 1;
		}else if(freq >= 640.625){
			//*calibration = (0.0703 * freq ) - 24.992;
			*calibration = 1;
		}
}

void calculate_acc(float *x, float *y, float *z, uint16_t size, float *Arms_x, float *Arms_y, float *Arms_z){

    float sum_ax2 = 0, sum_ay2 = 0, sum_az2 = 0;

    for (uint16_t i = 0; i < size; i++) {
      	sum_ax2 += x[2*i] * x[2*i];
      	sum_ay2 += y[2*i] * y[2*i];
      	sum_az2 += z[2*i] * z[2*i];
      }

//       *Arms_x = sum_ax2 / size;
//       *Arms_y = sum_ay2 / size;
//       *Arms_z = sum_az2 / size;

       *Arms_x =  sqrtf(sum_ax2 / size);
       *Arms_y =  sqrtf(sum_ay2 / size);
       *Arms_z =  sqrtf(sum_az2 / size);

}

void calculate_accver2(){
	for(int i = 0 ; i < Base_noise.Ratio_TOP_N ; i++){

		acc.sum_x = calibration.peak_x[i] / sqrtf(2);
		acc.sum_y = calibration.peak_y[i] / sqrtf(2);
		acc.sum_z = calibration.peak_z[i] / sqrtf(2);

		acc.rms_x += acc.sum_x * acc.sum_x;
		acc.rms_y += acc.sum_y * acc.sum_y;
		acc.rms_z += acc.sum_z * acc.sum_z;
	}

	acc.output_rms_x = sqrtf(acc.rms_x) ;
	acc.output_rms_y = sqrtf(acc.rms_y) ;
	acc.output_rms_z = sqrtf(acc.rms_z) ;
	acc.output_Addition_vectors = sqrtf((acc.rms_x ) + (acc.rms_y) + (acc.rms_z ));

}

void velocity_and_displacement( freq_t *freq, uint16_t TOP) {

	if(!sensor_App.zeroCross){
    for (int i = 0; i < TOP ; i++) {
        float factor_x = 2.0f * M_PI * freq->peaks_acc[i].output_Hz_x ;
        float factor_y = 2.0f * M_PI * freq->peaks_acc[i].output_Hz_y ;
        float factor_z = 2.0f * M_PI * freq->peaks_acc[i].output_Hz_z ;

        freq->peaks_acc[i].Apeak_use_x = calibration.peak_x[i] ;
        freq->peaks_acc[i].Apeak_use_y = calibration.peak_y[i];
        freq->peaks_acc[i].Apeak_use_z = calibration.peak_z[i];


//        freq->peaks_acc[i].Apeak_use_x = Sensor.output_fft_mag_x[freq->peaks_acc[i].index_x] ;
//        freq->peaks_acc[i].Apeak_use_y = Sensor.output_fft_mag_y[freq->peaks_acc[i].index_y] ;
//        freq->peaks_acc[i].Apeak_use_z = Sensor.output_fft_mag_z[freq->peaks_acc[i].index_z] ;

//        freq->peaks_acc[i].Apeak_use_x = Sensor.output_fft_mag_x[freq->peaks_acc[i].index_x - 1] + Sensor.output_fft_mag_x[freq->peaks_acc[i].index_x + 1] ;
//        freq->peaks_acc[i].Apeak_use_y = Sensor.output_fft_mag_y[freq->peaks_acc[i].index_y - 1] +  Sensor.output_fft_mag_y[freq->peaks_acc[i].index_y + 1]  ;
//        freq->peaks_acc[i].Apeak_use_z = Sensor.output_fft_mag_z[freq->peaks_acc[i].index_z - 1] + Sensor.output_fft_mag_z[freq->peaks_acc[i].index_z + 1];

        v.rms_x_sqr[i] = (freq->peaks_acc[i].Apeak_use_x / (factor_x * sqrtf(2.0f)));
        v.rms_y_sqr[i] = (freq->peaks_acc[i].Apeak_use_y / (factor_y * sqrtf(2.0f)));
        v.rms_z_sqr[i] = (freq->peaks_acc[i].Apeak_use_z / (factor_z * sqrtf(2.0f)));

        displacement.rms_x_sqr[i] =  v.rms_x_sqr[i]  / factor_x ;
        displacement.rms_y_sqr[i] =  v.rms_y_sqr[i]  / factor_y ;
        displacement.rms_z_sqr[i] =  v.rms_z_sqr[i]  / factor_z ;

        v.rms_x = (v.rms_x_sqr[i] < DBL_MIN) ? 0 : v.rms_x_sqr[i];
        v.rms_y = (v.rms_y_sqr[i] < DBL_MIN) ? 0 : v.rms_y_sqr[i];
        v.rms_z = (v.rms_z_sqr[i] < DBL_MIN) ? 0 : v.rms_z_sqr[i];

        displacement.rms_x = (displacement.rms_x_sqr[i] < DBL_MIN) ? 0 : displacement.rms_x_sqr[i] * 1000.0f;
        displacement.rms_y = (displacement.rms_y_sqr[i] < DBL_MIN) ? 0 : displacement.rms_y_sqr[i] * 1000.0f;
        displacement.rms_z = (displacement.rms_z_sqr[i] < DBL_MIN) ? 0 : displacement.rms_z_sqr[i] * 1000.0f;

        v.rms_x_sum += v.rms_x * v.rms_x;
        v.rms_y_sum += v.rms_y * v.rms_y;
        v.rms_z_sum += v.rms_z * v.rms_z;

        displacement.rms_x_sum +=  displacement.rms_x * displacement.rms_x ;
        displacement.rms_y_sum +=  displacement.rms_y * displacement.rms_y ;
        displacement.rms_z_sum +=  displacement.rms_z * displacement.rms_z ;
    	}
	}else {
		zeroCross.freq_x = (zeroCross.Count_x/2.0f) * (((float32_t)threshold.machine.freq * 2)/ FFT_SIZE ) ;
		zeroCross.freq_y = (zeroCross.Count_y/2.0f) * (((float32_t)threshold.machine.freq * 2)/ FFT_SIZE ) ;
		zeroCross.freq_z = (zeroCross.Count_z/2.0f) * (((float32_t)threshold.machine.freq * 2)/ FFT_SIZE ) ;

		float factor_x = 2.0f * M_PI * zeroCross.freq_x ;
		float factor_y = 2.0f * M_PI * zeroCross.freq_y ;
		float factor_z = 2.0f * M_PI * zeroCross.freq_z ;

		if(zeroCross.freq_x > 3.125 ) {
			v.rms_x_sqr[0] =   acc.rms_x1/ factor_x ;
			displacement.rms_x_sqr[0] =  v.rms_x_sqr[0] / factor_x ;
			v.rms_x_sum += v.rms_x_sqr[0] * v.rms_x_sqr[0];
			displacement.rms_x_sum +=  displacement.rms_x_sqr[0] * 1000.0f * displacement.rms_x_sqr[0] * 1000.0f;
		}
		if(zeroCross.freq_y > 3.125) {
			v.rms_y_sqr[0] =  acc.rms_y1/ factor_y ;
			displacement.rms_y_sqr[0] =  v.rms_y_sqr[0] / factor_y ;
			v.rms_y_sum += v.rms_y_sqr[0] * v.rms_y_sqr[0];
			displacement.rms_y_sum +=  displacement.rms_y_sqr[0] * 1000.0f * displacement.rms_y_sqr[0] * 1000.0f;
		}
		if(zeroCross.freq_z > 3.125) {
			v.rms_z_sqr[0] =    acc.rms_z1/ factor_z ;
			displacement.rms_z_sqr[0] =  v.rms_z_sqr[0]   / factor_z ;
			v.rms_z_sum += v.rms_z_sqr[0] * v.rms_z_sqr[0];
			displacement.rms_z_sum +=  displacement.rms_z_sqr[0] * 1000.0f * displacement.rms_z_sqr[0] * 1000.0f;
		}

        v.rms_x_sum += v.rms_x_sqr[0] * v.rms_x_sqr[0];
        v.rms_y_sum += v.rms_y_sqr[0] * v.rms_y_sqr[0];
        v.rms_z_sum += v.rms_z_sqr[0] * v.rms_y_sqr[0];

        displacement.rms_x_sum +=  displacement.rms_x_sqr[0] * displacement.rms_x_sqr[0];
        displacement.rms_y_sum +=  displacement.rms_y_sqr[0] * displacement.rms_y_sqr[0];
        displacement.rms_z_sum +=  displacement.rms_z_sqr[0] * displacement.rms_z_sqr[0];

		v.rms_x_sqr[0] *= 1000;
		v.rms_y_sqr[0] *= 1000;
		v.rms_z_sqr[0] *= 1000;

		displacement.rms_x_sqr[0] *= 1000000.0f;
		displacement.rms_y_sqr[0] *= 1000000.0f;
		displacement.rms_z_sqr[0] *= 1000000.0f;
	}

}

void calculateMagnitudeArray(float *input, float *output, int length) {
    for (int i = 0; i < length ; i++) {
        float real = input[2 * i];
        float imag = input[2 * i + 1];
        output[i] = sqrtf(real * real + imag * imag);
    }
}
void find_top_peaks_3axis(float mag_x[], float mag_y[], float mag_z[], freq_t *freq, uint32_t fft_size)
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

        // find local peak (strict): mag[i] > mag[i-1] && mag[i] > mag[i+1]
        // NOTE: ช่วงค้นหาเริ่มที่ 4 และจบที่ fft_size-30 ตามของเดิม (พอสำหรับ i-1, i+1)
        for (uint32_t i = 4; i < fft_size - 30; i++) {
            if (!used[i]) {
                // X axis
                if (mag_x[i] > max_val_x &&
                    mag_x[i] > mag_x[i - 1] &&    // ซ้ายต่ำกว่า
                    mag_x[i] > mag_x[i + 1]) {    // ขวาต่ำกว่า
                    max_val_x = mag_x[i];
                    max_idx_x = i;
                }
                // Y axis
                if (mag_y[i] > max_val_y &&
                    mag_y[i] > mag_y[i - 1] &&
                    mag_y[i] > mag_y[i + 1]) {
                    max_val_y = mag_y[i];
                    max_idx_y = i;
                }
                // Z axis
                if (mag_z[i] > max_val_z &&
                    mag_z[i] > mag_z[i - 1] &&
                    mag_z[i] > mag_z[i + 1]) {
                    max_val_z = mag_z[i];
                    max_idx_z = i;
                }
            }
        }

        // save results
        freq->peaks_acc[n].Apeak_S1_x = max_val_x;
        freq->peaks_acc[n].Apeak_S1_y = max_val_y;
        freq->peaks_acc[n].Apeak_S1_z = max_val_z;

        freq->peaks_acc[n].index_x = max_idx_x;
        freq->peaks_acc[n].index_y = max_idx_y;
        freq->peaks_acc[n].index_z = max_idx_z;

        // suppress neighborhood around chosen peaks
        for (int j = -(int)SUPPRESSION_WIDTH; j <= (int)SUPPRESSION_WIDTH; j++) {
            int idx_x = (int)max_idx_x + j;
            int idx_y = (int)max_idx_y + j;
            int idx_z = (int)max_idx_z + j;

            if (idx_x >= 0 && idx_x < (int)fft_size) used[idx_x] = 1;
            if (idx_y >= 0 && idx_y < (int)fft_size) used[idx_y] = 1;
            if (idx_z >= 0 && idx_z < (int)fft_size) used[idx_z] = 1;
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
	acc.rms_x = acc.rms_y = acc.rms_z = 0.0f;
	Sensor.index2 = 0;
}

static inline int axis_code(int ax) {              // 0->1, 1->2, 2->3
    return ax == 0 ? 1 : (ax == 1 ? 2 : 3);
}
static inline int pair_code(int a, int b) {        //  12, 13, 23
    int A = axis_code(a), B = axis_code(b);
    return (A < B) ? (A*10 + B) : (B*10 + A);
}

void frequency(float x_sum, float y_sum, float z_sum , float *output_axis
				, float freq_x ,  float freq_y ,  float freq_z){

	float rms_sum[3]   = {x_sum, y_sum, z_sum};

    float hz[3]   = {freq_x , freq_y , freq_z};


    float val[3]  = {val_x, val_y, val_z};

    int imax = 0;
    if (val[1] > val[imax]) imax = 1; //max index freq
    if (val[2] > val[imax]) imax = 2;

    int o1 = (imax == 0) ? 1 : 0;   // 2-3 index freq
    int o2 = (imax == 2) ? 1 : 2;
    if (o1 == o2) o2 = 2;
    const int close1 = fabsf(hz[imax] - hz[o1]) <= 5.0f; // rang freq
    const int close2 = fabsf(hz[imax] - hz[o2]) <= 5.0f;

    if (close1 && close2) {
    	*output_axis    = sqrtf(rms_sum[0] + rms_sum[1] + rms_sum[2]) * 1000.0f;
        v.output_axis_num = 123 ;
    } else if (close1) {
    	*output_axis    = sqrtf(rms_sum[imax] + rms_sum[o1]) * 1000.0f;
        v.output_axis_num = pair_code(imax, o1);
    } else if (close2) {
    	*output_axis     = sqrtf(rms_sum[imax] + rms_sum[o2]) * 1000.0f;
        v.output_axis_num = pair_code(imax, o2);
    } else {
    	*output_axis = sqrtf(rms_sum[imax]) * 1000.0f;
        v.output_axis_num = axis_code(imax);

    }
}

//---------------------------------------------------------------------------------------------------------------
void ODR_FS_setting(){

	threshold.ODR_sampling.reg = 0b10010000; // 6667 Hz

//if(sensor_App.zeroCross){
	switch (sensor_App.input_FS_g) {
		case 2:threshold.Fs.reg = 0b00000000;break;
		case 4:threshold.Fs.reg = 0b00001000;break;
		case 8:threshold.Fs.reg = 0b00001100;break;
		case 16:threshold.Fs.reg = 0b00000100;break;
		default:break;
	}
//	uint8_t CTRL8_XL  = 0x00;
//	 BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_CTRL8_XL, CTRL8_XL); // LPF1
//} else {
//	 switch (sensor_App.input_FS_g) {
//			case 2:threshold.Fs.reg = 0b00000000;break;
//			case 4:threshold.Fs.reg = 0b00001000;break;
//			case 8:threshold.Fs.reg = 0b00001100;break;
//			case 16:threshold.Fs.reg = 0b00000100;break;
//			default:break;
//	}  uint8_t CTRL8_XL  = 0x01;
//	BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_CTRL8_XL, CTRL8_XL); // LPF1 + LPF2 (ODR / 4)
//
//}
}

void Full_scale (){
//ทำเฉพาะเปลี่ยนค่า sensor_App.input_FS_g
switch (sensor_App.input_FS_g) {
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

}

void offset_working(){
	if( Sensor.index2 < FFT_SIZE && sensor_App.offset){

		sensor_App.offset1.sum_x += (threshold.Fs.x / 1000.0f) * (threshold.Fs.x / 1000.0f);
		sensor_App.offset1.sum_y += (threshold.Fs.y / 1000.0f) * (threshold.Fs.y / 1000.0f);
		sensor_App.offset1.sum_z += (threshold.Fs.z / 1000.0f) * (threshold.Fs.z / 1000.0f);

		Sensor.index2++;

	}else if(sensor_App.offset){

		sensor_App.offset1.x = sqrtf(sensor_App.offset1.sum_x / (float)FFT_SIZE);
		sensor_App.offset1.y = sqrtf(sensor_App.offset1.sum_y / (float)FFT_SIZE);
		sensor_App.offset1.z = sqrtf(sensor_App.offset1.sum_z / (float)FFT_SIZE);

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

		   uint8_t WAKE_UP_DUR = 0x71; //delay interrupt -> x sec
		   BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_WAKE_UP_DUR , WAKE_UP_DUR); // -> sleep mode

		   uint8_t WAKE_UP_THS = threshold.WAKE_UP_THS ;
		   BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_WAKE_UP_THS, WAKE_UP_THS);

		   uint8_t TAP_CFG0  = 0x00; 	// slope -> 0x00 , HPF -> 0x10 // latched 0x41(slope) 0x51(HPF)
		   BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_TAP_CFG0, TAP_CFG0); //----------------------------

		   uint8_t TAP_CFG2  = 0xC0;
		   BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0,ISM330DHCX_TAP_CFG2 , TAP_CFG2 );

		   uint8_t MD1_CFG  = 0xE0;
		   BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_MD1_CFG , MD1_CFG);

			 Sensor.index2 = 0;
			 sensor_App.offset  = 0;
		 if(!Sensor.status_wk){HAL_TIM_Base_Stop_IT(ISM330DHCX_TIM) ;}
	}
}

void Train_working(){
 //TODO หาช่วงเวลาที่ควรเทรน
	if((Sensor.index2 < FFT_SIZE && sensor_App.Train) &&  sensor_App.Train1.count < 4){ // 4 cycle

		sensor_App.Train1.x = fabsf(threshold.Fs.x) ;
		sensor_App.Train1.y = fabsf(threshold.Fs.y) ;
		sensor_App.Train1.z = fabsf(threshold.Fs.z) ;

		if(sensor_App.Train1.x > sensor_App.Train1.max_x)sensor_App.Train1.max_x = sensor_App.Train1.x;
		if(sensor_App.Train1.y > sensor_App.Train1.max_y)sensor_App.Train1.max_y = sensor_App.Train1.y;
		if(sensor_App.Train1.z > sensor_App.Train1.max_z)sensor_App.Train1.max_z = sensor_App.Train1.z;

		Sensor.index2++ ;
	}else if(sensor_App.Train  &&  sensor_App.Train1.count < 4){

		sensor_App.Train1.max_val = fmaxf(fmaxf(sensor_App.Train1.max_x, sensor_App.Train1.max_y), sensor_App.Train1.max_z); // max value

		if (sensor_App.Train1.max_val >= 5600.0f) {
		    sensor_App.input_FS_g = 16;
		} else if (sensor_App.Train1.max_val >= 2800.0f) {
		    sensor_App.input_FS_g = 8;
		} else if (sensor_App.Train1.max_val >= 1400.0f) {
		    sensor_App.input_FS_g = 4;
		} else {
		    sensor_App.input_FS_g = 2;
		}

		ODR_FS_setting();
		uint8_t CTRL1_XL = (threshold.Fs.reg | threshold.ODR_sampling.reg) ; //Setting -> Fs , sampling Rate
		BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_CTRL1_XL , CTRL1_XL);
		sensor_App.Train1.count++;

	}

	if(sensor_App.Train  &&  sensor_App.Train1.count == 4){

		 //--------------------------------- update threshold slope ------------------------------------------------
			  sensor_App.Arms = (sensor_App.input_Vrms / 1000.0f) * (2.0f * PI * (sensor_App.input_RPM * 0.016667)) ; // Arms = Vrms * 2 pi f

			  threshold.slope = (sensor_App.Arms * sqrtf(2) * (256.0f /(float)sensor_App.input_FS_g ) *
			  		arm_sin_f32(2 * PI * (sensor_App.input_RPM * 0.016667) * (1.0f / 6667.0f))) / (9.80665 *  2.0f);

			  threshold.WAKE_UP_THS = (uint8_t)((threshold.slope))| 0x40;
			  if(threshold.slope  <= 1.0f){
			  	threshold.WAKE_UP_THS = 0x01 | 0x40;
			  }else if(threshold.slope >= 63.0f){
			  	threshold.WAKE_UP_THS = 0x3F | 0x40;
			  }
	     //----------------------------------------------------------------------------------------------------------

				 sensor_App.Train1.x = sensor_App.Train1.y = sensor_App.Train1.z = 0;
				 sensor_App.Train1.max_x = sensor_App.Train1.max_y = sensor_App.Train1.max_z = sensor_App.Train1.max_val = 0;

			   uint8_t WAKE_UP_DUR = 0x71; //delay interrupt -> x sec
			   BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_WAKE_UP_DUR , WAKE_UP_DUR); // -> sleep mode

			   uint8_t WAKE_UP_THS = threshold.WAKE_UP_THS;
			   BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_WAKE_UP_THS, WAKE_UP_THS);

			   uint8_t TAP_CFG0  = 0x00; 	// slope -> 0x00 , HPF -> 0x10 // latched 0x41(slope) 0x51(HPF)
			   BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_TAP_CFG0, TAP_CFG0); //----------------------------

			   uint8_t TAP_CFG2  = 0xC0;
			   BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0,ISM330DHCX_TAP_CFG2 , TAP_CFG2 );

			   uint8_t MD1_CFG  = 0xE0;
			   BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_MD1_CFG , MD1_CFG);

				 Sensor.index2 = 0;
				 sensor_App.Train  = 0 ;
				 sensor_App.Train1.count = 0;

	if(!Sensor.status_wk){HAL_TIM_Base_Stop_IT(ISM330DHCX_TIM) ;}
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


		//TODO เปิดอินเตอรับเซนเซอร์
		   uint8_t WAKE_UP_DUR = 0x71; //delay interrupt -> x sec
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

	for(int i = 0 ; i < FFT_SIZE  ; i++){
		Sensor.output_fft_mag_z[i] *=  2.1f / (FFT_SIZE / 2.0f);
		Sensor.output_fft_mag_y[i] *=  2.1f / (FFT_SIZE / 2.0f);
		Sensor.output_fft_mag_x[i] *=  2.1f / (FFT_SIZE / 2.0f); //A_peak
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

	Base_noise.Ratio_x =  60 * Base_noise.E_sse_av_x;
	Base_noise.Ratio_y =  60 * Base_noise.E_sse_av_y;
	Base_noise.Ratio_z =  60 * Base_noise.E_sse_av_z;

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

	//------------------------------------------------ Interpolated ------------------------------------------------
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

//		freq.peaks_acc[i].output_Hz_x = (float)freq.peaks_acc[i].index_x * (threshold.machine.Sampling_Rate / (float)FFT_SIZE) ;
//		freq.peaks_acc[i].output_Hz_y = (float)freq.peaks_acc[i].index_y * (threshold.machine.Sampling_Rate / (float)FFT_SIZE) ;
//		freq.peaks_acc[i].output_Hz_z = (float)freq.peaks_acc[i].index_z * (threshold.machine.Sampling_Rate / (float)FFT_SIZE) ;

		calibration1(freq.peaks_acc[i].output_Hz_x , &calibration.x1);
		calibration1(freq.peaks_acc[i].output_Hz_y , &calibration.y1);
		calibration1(freq.peaks_acc[i].output_Hz_z , &calibration.z1);

		calibration.peak_x[i] = Sensor.output_fft_mag_x[freq.peaks_acc[i].index_x] -
	    (0.25 * (Sensor.output_fft_mag_x[freq.peaks_acc[i].index_x - 1] - Sensor.output_fft_mag_x[freq.peaks_acc[i].index_x + 1]) * freq.Interpolated_bin.x);
		calibration.peak_y[i] = Sensor.output_fft_mag_y[freq.peaks_acc[i].index_y] -
	    (0.25 * (Sensor.output_fft_mag_y[freq.peaks_acc[i].index_y - 1] - Sensor.output_fft_mag_y[freq.peaks_acc[i].index_y + 1]) * freq.Interpolated_bin.y);
		calibration.peak_z[i] = Sensor.output_fft_mag_z[freq.peaks_acc[i].index_z] -
		(0.25 * (Sensor.output_fft_mag_z[freq.peaks_acc[i].index_z - 1] - Sensor.output_fft_mag_z[freq.peaks_acc[i].index_z + 1]) * freq.Interpolated_bin.z);

//		calibration.peak_x[i] = Sensor.output_fft_mag_x[freq.peaks_acc[i].index_x] / ((100.0f - calibration.x1) / 100.0f) ;
//		calibration.peak_y[i] = Sensor.output_fft_mag_y[freq.peaks_acc[i].index_y] / ((100.0f - calibration.y1) / 100.0f) ;
//		calibration.peak_z[i] = Sensor.output_fft_mag_z[freq.peaks_acc[i].index_z] / ((100.0f - calibration.z1) / 100.0f) ;

//		calibration.peak_x[i] = (Sensor.output_fft_mag_x[freq.peaks_acc[i].index_x - 1] + Sensor.output_fft_mag_x[freq.peaks_acc[i].index_x + 1]) / ((100.0f - calibration.x1) / 100.0f) ;
//		calibration.peak_y[i] = (Sensor.output_fft_mag_y[freq.peaks_acc[i].index_y - 1] + Sensor.output_fft_mag_y[freq.peaks_acc[i].index_y + 1]) / ((100.0f - calibration.y1) / 100.0f) ;
//		calibration.peak_z[i] = (Sensor.output_fft_mag_z[freq.peaks_acc[i].index_z - 1] + Sensor.output_fft_mag_z[freq.peaks_acc[i].index_z + 1]) / ((100.0f - calibration.z1) / 100.0f) ;

	}

	val_x = Sensor.output_fft_mag_x[freq.peaks_acc[0].index_x];
	val_y = Sensor.output_fft_mag_y[freq.peaks_acc[0].index_y];
	val_z = Sensor.output_fft_mag_z[freq.peaks_acc[0].index_z]; // for frequency V&D

	//------------------test---------------------------

	freq.f_sum_x += freq.peaks_acc[0].output_Hz_x ; // for average
	freq.f_sum_y += freq.peaks_acc[0].output_Hz_y ;
	freq.f_sum_z += freq.peaks_acc[0].output_Hz_z ;

	//------------------test---------------------------

}

void velocity(){ // mm/s
	v.output_sqr_z = sqrtf(v.rms_z_sum)* 1000.0f;
	v.output_sqr_y = sqrtf(v.rms_y_sum)* 1000.0f;
	v.output_sqr_x = sqrtf(v.rms_x_sum)* 1000.0f;

  frequency(v.rms_x_sum , v.rms_y_sum , v.rms_z_sum , &v.output_axis ,
		  freq.peaks_acc[0].output_Hz_x , freq.peaks_acc[0].output_Hz_y , freq.peaks_acc[0].output_Hz_z);

	v.output_Addition_vectors =  sqrtf(v.rms_x_sum + v.rms_y_sum + v.rms_z_sum) * 1000.0f;

}

void displacement_c() { //um
	displacement.output_sqr_z = sqrtf(displacement.rms_z_sum)* 1000.0f;
	displacement.output_sqr_y = sqrtf(displacement.rms_y_sum)* 1000.0f;
	displacement.output_sqr_x = sqrtf(displacement.rms_x_sum)* 1000.0f;

	frequency(displacement.rms_x_sum, displacement.rms_y_sum , displacement.rms_z_sum , &displacement.output_axis ,
			freq.peaks_acc[0].output_Hz_x , freq.peaks_acc[0].output_Hz_y , freq.peaks_acc[0].output_Hz_z);

	displacement.output_Addition_vectors = sqrtf(displacement.rms_x_sum + displacement.rms_y_sum + displacement.rms_z_sum) * 1000.0f;

}

#endif

void Acc_Calculation() {

	/*Convert accelerometer, mg/LSB  to m/s^2*/

	/* USER CODE BEGIN */

	MW_LOG(TS_OFF, VLEVEL_M, "###### ACCELERO_GYRO_Calculation ######\r\n") ;
	/* USER CODE END */

#if FFT_ENABLE
		//TODO ถ้าเพิ่มอุปกรณ์ ให้เช็คแกน Z
		if(Sensor.index2 == FFT_SIZE){
       calculate_acc(Sensor.ACC_X, Sensor.ACC_Y, Sensor.ACC_Z , FFT_SIZE , &acc.rms_x1, &acc.rms_y1, &acc.rms_z1);
//		acc.output_Addition_vectors = sqrtf((acc.rms_x1 ) + (acc.rms_y1) + (acc.rms_z1 ));
       acc.addition_A1 = sqrtf((acc.rms_x1 *acc.rms_x1 ) + (acc.rms_y1 * acc.rms_y1) + (acc.rms_z1 * acc.rms_z1 ));
//		acc.Arms_total += acc.output_Addition_vectors ;
//		acc.Ax_total += sqrtf(acc.rms_x1);
//		acc.Ay_total += sqrtf(acc.rms_y1);
//		acc.Az_total += sqrtf(acc.rms_z1);
//-----------------------------------------------------------------------------------------------------------
		float p_CZ = 0.000f ;
		float n_CZ = -0.000f ;
		if(sensor_App.zeroCross){
		   for(int i = 1; i < FFT_SIZE; i++) {
		       if((Sensor.ACC_Z[2*(i-1)] >= n_CZ && Sensor.ACC_Z[2*i] < p_CZ) ||
		          (Sensor.ACC_Z[2*(i-1)] < p_CZ && Sensor.ACC_Z[2*i] >= n_CZ)) {
		           zeroCross.Count_z++;
		       }
		       if((Sensor.ACC_Y[2*(i-1)] >= n_CZ && Sensor.ACC_Y[2*i] < p_CZ ) ||
		       	  (Sensor.ACC_Y[2*(i-1)] < p_CZ && Sensor.ACC_Y[2*i] >= n_CZ)) {
		       	   zeroCross.Count_y++;
		       }
		       if((Sensor.ACC_X[2*(i-1)] >= n_CZ && Sensor.ACC_X[2*i] < p_CZ) ||
		       	(Sensor.ACC_X[2*(i-1)] < p_CZ && Sensor.ACC_X[2*i] >= n_CZ)) {
		    	   zeroCross.Count_x++;
		       }
		   }
		}

		if( !sensor_App.zeroCross ){
				input_and_Apeak();
				threshold_noise();
				peak();
				calculate_accver2();
			}


		velocity_and_displacement(&freq, Base_noise.Ratio_TOP_N);

		if(sensor_App.zeroCross){
			val_x = acc.rms_x1 ; val_y = acc.rms_y1; val_z = acc.rms_z1;

			  frequency(v.rms_x_sum , v.rms_y_sum , v.rms_z_sum , &v.output_axis
					  , zeroCross.freq_x , zeroCross.freq_y , zeroCross.freq_z);
			  frequency(displacement.rms_x_sum, displacement.rms_y_sum , displacement.rms_z_sum , &displacement.output_axis
					  , zeroCross.freq_x , zeroCross.freq_y , zeroCross.freq_z);
			  frequency((acc.rms_x1 * acc.rms_x1 ) , (acc.rms_y1 * acc.rms_y1) , (acc.rms_z1 * acc.rms_z1 ) , &acc.output_axis
					  , zeroCross.freq_x , zeroCross.freq_y , zeroCross.freq_z);

		acc.output_axis = acc.output_axis /( 1000 );
		v.output_Addition_vectors =  sqrtf(v.rms_x_sum  + v.rms_y_sum + v.rms_z_sum ) * 1000.0f;
		displacement.output_Addition_vectors =  sqrtf( displacement.rms_x_sum +  displacement.rms_y_sum + displacement.rms_z_sum) * 1000.0f; //Zero

		}else if (!sensor_App.zeroCross){

		frequency((acc.rms_x1 * acc.rms_x1 ) , (acc.rms_y1 * acc.rms_y1) , (acc.rms_z1 * acc.rms_z1 ) , &acc.output_axis
				,freq.peaks_acc[0].output_Hz_x , freq.peaks_acc[0].output_Hz_y , freq.peaks_acc[0].output_Hz_z);

		acc.output_axis = acc.output_axis /( 1000 );

		velocity();
		displacement_c();

		}

		Reset();
		v.rms_z_sum = 0.0f; v.rms_y_sum = 0.0f; v.rms_x_sum = 0.0f; Base_noise.Ratio_TOP_N = 0;
		displacement.rms_z_sum = 0.0f; displacement.rms_y_sum = 0.0f; displacement.rms_x_sum = 0.0f;
		sensor_App.wake_up  = 0;

		zeroCross.Count_x = zeroCross.Count_y = zeroCross.Count_z = 0 ;
		displacement.rms_x_sqr[0] = displacement.rms_y_sqr[0] = displacement.rms_z_sqr[0] = 0.0f ;
		v.rms_x_sqr[0] = v.rms_y_sqr[0] = v.rms_z_sqr[0] = 0.0f ;
	}

#endif
}

void Sensor_Run() {
//		Measure_Temp_STTS22H();
//		Acc_Calculation();

}

void Sensor_Log(){
//	 MW_LOG(TS_OFF, VLEVEL_M, " ---------- minggggggggggg  --------- \r\n");

	if((sensor_App.wake_up || sensor_App.offset) || sensor_App.Train ){

//		uint8_t CTRL1_XL = (threshold.Fs.reg | threshold.ODR_sampling.reg) ; //Setting -> Fs , sampling Rate
//		BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_CTRL1_XL , CTRL1_XL);

		if( sensor_App.offset ){

		//TODO รีเซตค่าออฟเซต
		BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_CTRL7_G  ,  0x00 );
	    BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_CTRL6_C  ,  0x00);
	    BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_X_OFS_USR  , 0x00 );
	    BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_Y_OFS_USR  , 0x00 );
	    BSP_MOTION_SENSOR_Write_Register(MOTION_SENSOR_ISM330DHCX_0, ISM330DHCX_Z_OFS_USR  ,  0x00 );
		}

		//TODO ปิดค่า อินเตอรับเซนเซอร์
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

		if(Wake_up_cnt++ < 1){
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

