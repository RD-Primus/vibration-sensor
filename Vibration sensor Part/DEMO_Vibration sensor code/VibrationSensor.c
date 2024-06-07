/*
 * VibrationSensor.c
 *
 *  Created on: Mar 21, 2024
 *      Author: muple
 */
/* Includes ------------------------------------------------------------------*/
#include "VibrationSensor.h"

/* Private typedef -----------------------------------------------------------*/
VibrationSensor_Node VibrationSensor_Output;

typedef enum
{
	MotorSTOP,
	MotorRUN
} MotorSTATUS;
MotorSTATUS MotorStatus = MotorSTOP;

typedef enum
{
	Alarm_OFF,
	Alarm_ON
} AlarmStatus;
AlarmStatus Acceleration_AlarmStatus = Alarm_OFF;
AlarmStatus Velocity_AlarmStatus = Alarm_OFF;
AlarmStatus Temperature_AlarmStatus = Alarm_OFF;

typedef struct
{
	uint8_t			CTRL[6];
	const float		So;
	const float		ODR;
	const float		Ts;
	const uint8_t	Discard;
	const uint16_t	FFT;
} LIS2DW12_Setting;
static LIS2DW12_Setting LIS2DW12_LowPower =
{
		.CTRL		=	{0x42, 0x8C, 0x00, 0x00, 0x00, 0xE4},
		/* CTRL1	ODR_50Hz, Low-Power Mode 3
		 * CTRL2	BOOT_Enable, BDU_Enable, IF_ADD_INC_Enable
		 * CTRL6	BW_ODR/20, FS_8g, LPF, Low-Noise_Enable */

		.So			=	0.976f,
		/* Sensitivity[mg/digit] at FS_8g in low power mode 3. */

		.ODR		=	50.0f,
		/* Output data rate frequency (Sampling frequency). */

		.Ts			=	0.02f,
		/* Sampling period. */

		.Discard	=	1U,
		/* Data samples to be discarded. */

		.FFT	=	0U
		/* Initial Fast Fourier Transform size. */
};

static LIS2DW12_Setting LIS2DW12_HighPerformance50Hz =
{
		.CTRL		=	{0x47, 0x8C, 0x00, 0x00, 0x00, 0xEC},
		/* CTRL1	ODR_50Hz, High-Performance Mode
		 * CTRL2	BOOT_Enable, BDU_Enable, IF_ADD_INC_Enable
		 * CTRL6	BW_ODR/20, FS_8g, HPF, Low-Noise_Enable */

		.So			=	0.976f,
		/* Sensitivity[mg/digit] at FS_8g in high performance mode. */

		.ODR		=	50.0f,
		/* Output data rate frequency (Sampling frequency). */

		.Ts			=	0.02f,
		/* Sampling period. */

		.Discard	=	1U,
		/* Data samples to be discarded. */

		.FFT	=	0U
		/* Initial Fast Fourier Transform size. */
};

static LIS2DW12_Setting LIS2DW12_HighPerformance100Hz =
{
		.CTRL		=	{0x57, 0x8C, 0x00, 0x00, 0x00, 0xEC},
		/* CTRL1	ODR_100Hz, High-Performance Mode
		 * CTRL2	BOOT_Enable, BDU_Enable, IF_ADD_INC_Enable
		 * CTRL6	BW_ODR/20, FS_8g, HPF, Low-Noise_Enable */

		.So			=	0.976f,
		/* Sensitivity[mg/digit] at FS_8g in high performance mode. */

		.ODR		=	100.0f,
		/* Output data rate frequency (Sampling frequency). */

		.Ts			=	0.01f,
		/* Sampling period. */

		.Discard	=	1U,
		/* Data samples to be discarded. */

		.FFT	=	0U
		/* Initial Fast Fourier Transform size. */
};

static LIS2DW12_Setting LIS2DW12_HighPerformance200Hz =
{
		.CTRL		=	{0x67, 0x8C, 0x00, 0x00, 0x00, 0xDC},
		/* CTRL1	ODR_200Hz, High-Performance Mode
		 * CTRL2	BOOT_Enable, BDU_Enable, IF_ADD_INC_Enable
		 * CTRL6	BW_ODR/20, FS_4g, HPF, Low-Noise_Enable */

		.So			=	0.488f,
		/* Sensitivity[mg/digit] at FS_4g in high performance mode. */

		.ODR		=	200.0f,
		/* Output data rate frequency (Sampling frequency). */

		.Ts			=	0.005f,
		/* Sampling period. */

		.Discard	=	1U,
		/* Data samples to be discarded. */

		.FFT	=	128U
		/* Initial Fast Fourier Transform size. */
};

static LIS2DW12_Setting LIS2DW12_HighPerformance400Hz =
{
		.CTRL		=	{0x77, 0x8C, 0x00, 0x00, 0x00, 0xDC},
		/* CTRL1	ODR_400Hz, High-Performance Mode
		 * CTRL2	BOOT_Enable, BDU_Enable, IF_ADD_INC_Enable
		 * CTRL6	BW_ODR/20, FS_4g, HPF, Low-Noise_Enable */

		.So			=	0.488f,
		/* Sensitivity[mg/digit] at FS_4g in high performance mode. */

		.ODR		=	400.0f,
		/* Output data rate frequency (Sampling frequency). */

		.Ts			=	0.0025f,
		/* Sampling period. */

		.Discard	=	1U,
		/* Data samples to be discarded. */

		.FFT	=	256U
		/* Initial Fast Fourier Transform size. */
};

static LIS2DW12_Setting LIS2DW12_HighPerformance800Hz =
{
		.CTRL		=	{0x87, 0x8C, 0x00, 0x00, 0x00, 0xCC},
		/* CTRL1	ODR_800Hz, High-Performance Mode
		 * CTRL2	BOOT_Enable, BDU_Enable, IF_ADD_INC_Enable
		 * CTRL6	BW_ODR/20, FS_2g, HPF, Low-Noise_Enable */

		.So			=	0.244f,
		/* Sensitivity[mg/digit] at FS_2g in high performance mode. */

		.ODR		=	800.0f,
		/* Output data rate frequency (Sampling frequency). */

		.Ts			=	0.00125f,
		/* Sampling period. */

		.Discard	=	1U,
		/* Data samples to be discarded. */

		.FFT	=	512U
		/* Initial Fast Fourier Transform size. */
};

static LIS2DW12_Setting LIS2DW12_HighPerformance1600Hz =
{
		.CTRL		=	{0x97, 0x8C, 0x00, 0x00, 0x00, 0xCC},
		/* CTRL1	ODR_1600Hz, High-Performance Mode
		 * CTRL2	BOOT_Enable, BDU_Enable, IF_ADD_INC_Enable
		 * CTRL6	BW_ODR/20, FS_2g, HPF, Low-Noise_Enable */

		.So			=	0.244f,
		/* Sensitivity[mg/digit] at FS_2g in high performance mode. */

		.ODR		=	1600.0f,
		/* Output data rate frequency (Sampling frequency). */

		.Ts			=	0.000625f,
		/* Sampling period. */

		.Discard	=	2U,
		/* Data samples to be discarded. */

		.FFT	=	1024U
		/* Initial Fast Fourier Transform size. */
};

Vibrations_float RMS_Acceleration, RMS_Velocity;

typedef struct
{
	float x[1600];
	float y[1600];
	float z[1600];
} DATA_BUFFER;
DATA_BUFFER AccData_BUF;

typedef struct
{
	float x[3];
	float y[3];
	float z[3];
} Vibrations_DataBuffer;
Vibrations_DataBuffer rmsAcc, rmsVel;

typedef struct
{
	float32_t	x[1024];
	float32_t	y[1024];
	float32_t	z[1024];
} FFT_TimeDomain;

typedef struct
{
	float		Frequency[512];
	float32_t	x[512];
	float32_t	y[512];
	float32_t	z[512];
} FFT_FrequencyDomain;

typedef struct
{
	uint8_t					rmsNum[3];
	Vibrations_float		rmsAcc[3];
	Vibrations_float		rmsVel[3];
	Vibrations_DataBuffer	rmsAccBUF[3];
	Vibrations_DataBuffer	rmsVelBUF[3];

	Vibrations_DataBuffer	RMS_AccOUT;
	Vibrations_DataBuffer	RMS_VelOUT;
} Vibrations_RMS;

typedef struct displayFloatToInt_s
{
  uint8_t	sign; /* 0 means positive, 1 means negative */
  uint32_t	out_int;
  uint32_t	out_dec;
} displayFloatToInt_t;

/* Private define ------------------------------------------------------------*/
#define UARTx	huart2
/* Define used UART serial port communication. */

#define TIMx1s	htim16
/* Define timer used for counting 1 second. */

#define TIMxIT	htim17
/* Define timer interrupt used for sampling data. */

#define UART_BUF_SIZE	256

#define DATA_BUFFER_SIZE	1600

#define VIBRATION_BUFFER_SIZE	3

#define ITERATION_BUFFER_SIZE	3

#define LIS2DW12_REG_COUNT	(sizeof(LIS2DW12_reg_addr)/sizeof(uint8_t))

#define HTTS751_REG_COUNT	(sizeof(HTTS751_reg_addr)/sizeof(uint8_t))

#define GRAVITATIONAL_ACCELERATION	9.80665f

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
arm_rfft_fast_instance_f32 FFThandler;

IKS01A3_MOTION_SENSOR_AxesRaw_t RawData;

char msg[UART_BUF_SIZE];

static uint8_t VibrationDRDY = 0U;
static uint8_t EnvironmentalDRDY = 0U;

const uint8_t LIS2DW12_reg_addr[]		=	{0x20, 0x21, 0x22, 0x23, 0x24, 0x25};
/* CTRL1, CTRL2, CTRL3, CTRL4, CTRL5, CTRL6 */

const uint8_t HTTS751_reg_addr[]	=	{0x03, 0x04};
/* CONF, CONV */
const uint8_t HTTS751_reg[]	=	{0x80, 0x04};
/* Res10bit_0.25, 1Hz */

static uint8_t Read_RDY = 0U;

float Sensitivity = 0.0f;
float ODR_Hz = 0.0f;
float SAMPLING_PERIOD = 0.0f;
uint8_t numDISCARD = 0U;
uint16_t FFT_SIZE = 0U;

uint8_t Num_Samples = 0U;

uint16_t numRawData = 0U;
uint16_t numData = 0U;

float Temperature = 0.0f;

/* Private function prototypes -----------------------------------------------*/
static int32_t LIS2DW12_Sensor_Handler(LIS2DW12_Setting *ptr);

static void LIS2DW12_Data_Handler(Vibrations_RMS *RMS_Data, uint8_t SamplingRound);

static void LIS2DW12_Data_Process();

static int32_t LIS2DW12_Get_Acc_Data(IKS01A3_MOTION_SENSOR_AxesRaw_t *RawData);

void RC_Filter(float fc, float Ts, uint16_t numDATA);

void IIR_Filter(float Alpha, uint16_t numDATA);

void MovingAverage_Filter(uint16_t WINDOW_SIZE, uint16_t numDATA, uint16_t *Filtered_numDATA);

void RMS_Gravity(uint16_t numDATA);

void FastFourierTransform(void);

void Vibration_Calculate();

void RootMeanSquare_Iteration(Vibrations_RMS *ptr, uint8_t Iterations);

void Delta_Vibration(Vibrations_RMS *ptr);

static int32_t HTTS751_Sensor_Handler();

static int32_t HTTS751_Get_ENV_Data(float *Temp);

static void floatToInt(float Input_value, displayFloatToInt_t *Output_value, int32_t dec_prec);

void UART_printData(void);

/* Private user code ---------------------------------------------------------*/
void X_NUCLEO_IKS01A3_Sensors_Init(void)
{
	(void)IKS01A3_MOTION_SENSOR_Init(IKS01A3_LIS2DW12_0, MOTION_ACCELERO);

	(void)IKS01A3_ENV_SENSOR_Init(IKS01A3_STTS751_0, ENV_TEMPERATURE);
}


/*----------------------------------------------------------------------------------------------------*/


void VibrationSensor_Process(void)
{
	/* Reset the output data. */
	VibrationSensor_Output.LIS2DW12_RMS_Gravity.x = 0.0f;
	VibrationSensor_Output.LIS2DW12_RMS_Gravity.y = 0.0f;
	VibrationSensor_Output.LIS2DW12_RMS_Gravity.z = 0.0f;

	VibrationSensor_Output.LIS2DW12_RMS_Velocity.x = 0.0f;
	VibrationSensor_Output.LIS2DW12_RMS_Velocity.y = 0.0f;
	VibrationSensor_Output.LIS2DW12_RMS_Velocity.z = 0.0f;

	VibrationSensor_Output.LIS2DW12_RMS_Acceleration.x = 0.0f;
	VibrationSensor_Output.LIS2DW12_RMS_Acceleration.y = 0.0f;
	VibrationSensor_Output.LIS2DW12_RMS_Acceleration.z = 0.0f;

	VibrationSensor_Output.HTTS751_Temperature = 0.0f;

	LIS2DW12_Sensor_Handler(&LIS2DW12_HighPerformance1600Hz);

	HTTS751_Sensor_Handler();

	VibrationSensor_Output.LIS2DW12_RMS_Velocity.x = RMS_Velocity.x;
	VibrationSensor_Output.LIS2DW12_RMS_Velocity.y = RMS_Velocity.y;
	VibrationSensor_Output.LIS2DW12_RMS_Velocity.z = RMS_Velocity.z;

	VibrationSensor_Output.LIS2DW12_RMS_Acceleration.x = RMS_Acceleration.x;
	VibrationSensor_Output.LIS2DW12_RMS_Acceleration.y = RMS_Acceleration.y;
	VibrationSensor_Output.LIS2DW12_RMS_Acceleration.z = RMS_Acceleration.z;

	VibrationSensor_Output.HTTS751_Temperature = Temperature;

//	(void)UART_printData();
}


/*----------------------------------------------------------------------------------------------------*/


static int32_t LIS2DW12_Sensor_Handler(LIS2DW12_Setting *ptr)
{
	uint8_t i;
	int32_t ret;
	uint8_t *LIS2DW12_CTRL_Register;
	static Vibrations_RMS RMS_Data;

	/* Reset buffers. */
	LIS2DW12_CTRL_Register	= 0U;
	Sensitivity				= 0.0f;
	ODR_Hz					= 0.0f;
	SAMPLING_PERIOD			= 0.0f;
	numDISCARD				= 0U;
	FFT_SIZE				= 0U;

	/* Initial the settings parameter. */
	LIS2DW12_CTRL_Register	= ptr->CTRL;
	Sensitivity				= ptr->So;
	ODR_Hz					= ptr->ODR;
	SAMPLING_PERIOD			= ptr->Ts;
	numDISCARD				= ptr->Discard;
	FFT_SIZE				= ptr->FFT;

	/* Set the sensor register. */
	for (i = 0; i < LIS2DW12_REG_COUNT; i++)
	{
		if ((ret = IKS01A3_MOTION_SENSOR_Write_Register(IKS01A3_LIS2DW12_0, LIS2DW12_reg_addr[i], LIS2DW12_CTRL_Register[i])) != BSP_ERROR_NONE)
		{
			return ret;
		}
	}

	if (FFT_SIZE > 0U)
	{
		arm_rfft_fast_init_f32(&FFThandler, FFT_SIZE);
	}
	else
	{}

	/* Delay for stable operating. */
	HAL_Delay(100);

	/* Data handle function */
	LIS2DW12_Data_Handler(&RMS_Data, 1);

    /* Vibration sensor data ready flag. */
	VibrationDRDY = 1U;

	return ret;
}


/*----------------------------------------------------------------------------------------------------*/


static void LIS2DW12_Data_Handler(Vibrations_RMS *RMS_Data, uint8_t SamplingRound)
{
	uint8_t i;

	/* Reset buffers. */
	Num_Samples = 0U;

	for (i = 0; i < VIBRATION_BUFFER_SIZE; i++)
	{
		rmsAcc.x[i] = 0.0f,	rmsAcc.y[i] = 0.0f,	rmsAcc.z[i] = 0.0f;

		rmsVel.x[i] = 0.0f,	rmsVel.y[i] = 0.0f,	rmsVel.z[i] = 0.0f;
	}

	/* Prevent the vibration data from overflowing the buffer. */
	if (SamplingRound > VIBRATION_BUFFER_SIZE)
	{
		SamplingRound = VIBRATION_BUFFER_SIZE;
	}
	else
	{}

	/* Data process function. */
	while (Num_Samples < SamplingRound)
	{

		LIS2DW12_Data_Process();

		Vibration_Calculate();

	}

	RMS_Acceleration.x = rmsAcc.x[0];
	RMS_Acceleration.y = rmsAcc.y[0];
	RMS_Acceleration.z = rmsAcc.z[0];

	RMS_Velocity.x = rmsVel.x[0] * 1000;
	RMS_Velocity.y = rmsVel.y[0] * 1000;
	RMS_Velocity.z = rmsVel.z[0] * 1000;

//	for (i = 0; i < Num_Samples; i++)
//	{
////		sprintf(msg, "\r\n(%d)rmsAcc\r\nX = %f\r\nY = %f\r\nZ = %f",
////				i, (rmsAcc.x[i]), (rmsAcc.y[i]), (rmsAcc.z[i]));
////		HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);
//
//		sprintf(msg, "\r\n(%d)Vel\r\nX = %f\r\nY = %f\r\nZ = %f\r\n",
//				i, (rmsVel.x[i]*1000), (rmsVel.y[i]*1000), (rmsVel.z[i]*1000));
//		HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);
//	}

//	Vibrations_float deltaVel, deltaAcc;
//
//	deltaVel.x = 0.0f,	deltaVel.y = 0.0f,	deltaVel.z = 0.0f;
//	deltaAcc.x = 0.0f,	deltaAcc.y = 0.0f,	deltaAcc.z = 0.0f;
//
////	deltaVel.x = fabs(rmsVel.x[1] - rmsVel.x[0]) * 1000;
////	deltaVel.y = fabs(rmsVel.y[1] - rmsVel.y[0]) * 1000;
////	deltaVel.z = fabs(rmsVel.z[1] - rmsVel.z[0]) * 1000;
//
//	deltaVel.x = rmsVel.x[0] * 1000;
//	deltaVel.y = rmsVel.y[0] * 1000;
//	deltaVel.z = rmsVel.z[0] * 1000;
//
//	deltaAcc.x = fabs(rmsAcc.x[1] - rmsAcc.x[0]) * 1000;
//	deltaAcc.y = fabs(rmsAcc.y[1] - rmsAcc.y[0]) * 1000;
//	deltaAcc.z = fabs(rmsAcc.z[1] - rmsAcc.z[0]) * 1000;
//
//	sprintf(msg, "\r\ndelta2Vel\r\nX = %f\r\nY = %f\r\nZ = %f\r\ndelta2Acc\r\nX = %f\r\nY = %f\r\nZ = %f\r\n",
//			(deltaVel.x), (deltaVel.y), (deltaVel.z), (deltaAcc.x), (deltaAcc.y), (deltaAcc.z));
//	HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);

//	RootMeanSquare_Iteration(RMS_Data, 3);

//	sprintf(msg, "\r\nNumIteration0(%d)", (RMS_Data->rmsNum[0]));
//	HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);
//
//	sprintf(msg, "\r\nNumIteration1(%d)", (RMS_Data->rmsNum[1]));
//	HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);
//
//	sprintf(msg, "\r\nNumIteration2(%d)", (RMS_Data->rmsNum[2]));
//	HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);
//
//	sprintf(msg, "\r\nNumIteration3(%d)", (RMS_Data->rmsNum[3]));
//	HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);
//
//	sprintf(msg, "\r\nNumIteration4(%d)", (RMS_Data->rmsNum[4]));
//	HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);
//
//	sprintf(msg, "\r\nNumIteration5(%d)\r\n", (RMS_Data->rmsNum[5]));
//	HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);

//	sprintf(msg, "\r\nrms2Acc\r\nX = %f\r\nY = %f\r\nZ = %f",
//			(RMS_Data->rmsAcc[2].x), (RMS_Data->rmsAcc[2].y), (RMS_Data->rmsAcc[2].z));
//	HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);
//
//	sprintf(msg, "\r\nrms2Vel\r\nX = %f\r\nY = %f\r\nZ = %f",
//			(RMS_Data->rmsVel[2].x), (RMS_Data->rmsVel[2].y), (RMS_Data->rmsVel[2].z));
//	HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);

//	for (i = 0; i < VIBRATION_BUFFER_SIZE; i++)
//	{
//		sprintf(msg, "\r\n(%d)rms1Acc\r\nX = %f\r\nY = %f\r\nZ = %f",
//				i, (RMS_Data->rmsAccBUF[1].x[i]), (RMS_Data->rmsAccBUF[1].y[i]), (RMS_Data->rmsAccBUF[1].z[i]));
//		HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);
//
//		sprintf(msg, "\r\n(%d)rms1Vel\r\nX = %f\r\nY = %f\r\nZ = %f\r\n",
//				i, (RMS_Data->rmsVelBUF[1].x[i]), (RMS_Data->rmsVelBUF[1].y[i]), (RMS_Data->rmsVelBUF[1].z[i]));
//		HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);
//	}

//	for (i = 0; i < VIBRATION_BUFFER_SIZE; i++)
//	{
//		sprintf(msg, "\r\n(%d)OutRMSAcc\r\nX = %f\r\nY = %f\r\nZ = %f",
//				i, (RMS_Data->RMS_AccOUT.x[i]), (RMS_Data->RMS_AccOUT.y[i]), (RMS_Data->RMS_AccOUT.z[i]));
//		HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);
//
//		sprintf(msg, "\r\n(%d)OutRMSVel\r\nX = %f\r\nY = %f\r\nZ = %f\r\n",
//				i, (RMS_Data->RMS_VelOUT.x[i]), (RMS_Data->RMS_VelOUT.y[i]), (RMS_Data->RMS_VelOUT.z[i]));
//		HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);
//	}

	Delta_Vibration(RMS_Data);
}


/*----------------------------------------------------------------------------------------------------*/


static void LIS2DW12_Data_Process()
{
	uint16_t i;
	uint16_t TIM16_cnt;
	uint16_t CounterPeriod;

	/* Reset buffers. */
    TIM16_cnt = 0U;
	CounterPeriod = 0U;

	Read_RDY = 0U;
	numRawData = 0U;

	RawData.x = 0,	RawData.y = 0,	RawData.z = 0;

    for (i = 0; i < DATA_BUFFER_SIZE; i++)
    {
    	AccData_BUF.x[i] = 0.0f,	AccData_BUF.y[i] = 0.0f,	AccData_BUF.z[i] = 0.0f;
    }

    /* Determine the timer counter period for interrupt. */
    CounterPeriod = (((HAL_RCC_GetPCLK1Freq()) / (TIMxIT.Instance->PSC)) / ODR_Hz) - 1;
    __HAL_TIM_SET_AUTORELOAD(&TIMxIT, CounterPeriod);

//	sprintf(msg, "\r\n(%d)\r\n", CounterPeriod);
//	HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);

    /* Start timers. */
    HAL_TIM_Base_Start(&TIMx1s);

    HAL_TIM_Base_Start_IT(&TIMxIT);

    /* Wait for the timer to work properly. */
    HAL_Delay(30);

	/* Reset and update timer count. */
    __HAL_TIM_SET_COUNTER(&TIMx1s, 0x0000);
    TIM16_cnt = __HAL_TIM_GET_COUNTER(&TIMx1s);

    __HAL_TIM_SET_COUNTER(&TIMxIT, 0x0000);

    /* Set the read ready flag for 1 second. (Timer clk 10kHz) */
    while ((__HAL_TIM_GET_COUNTER(&TIMx1s) - TIM16_cnt) <= 10000)
    {
        Read_RDY = 1U;
    }

    /* Clear read ready flag after finished reading the data. */
    Read_RDY = 0U;

	/* Stop timers. */
    HAL_TIM_Base_Stop(&TIMx1s);

	HAL_TIM_Base_Stop_IT(&TIMxIT);

    /* Check if the read ready flag is clear to proceed to the next process. */
    if (Read_RDY == 0U)
    {
//    	for (i = 0; i < numRawData; i++)
//    	{
////    		sprintf(msg, "\r\n(%d)[Raw Data]\r\nX = %f\r\nY = %f\r\nZ = %f\r\n",
////    				i, AccData_BUF.x[i], AccData_BUF.y[i], AccData_BUF.z[i]);
////    		HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);
//
//    		sprintf(msg, "%f,%f,%f\r\n", AccData_BUF.x[i], AccData_BUF.y[i], AccData_BUF.z[i]);
//    		HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);
//    	}

    	/* Discard first and last data to avoiding errors. */
    	for (i = numDISCARD; i < (numRawData - 1); i++)
    	{
    		AccData_BUF.x[i - numDISCARD] = AccData_BUF.x[i];
    		AccData_BUF.y[i - numDISCARD] = AccData_BUF.y[i];
    		AccData_BUF.z[i - numDISCARD] = AccData_BUF.z[i];
    	}
    	/* Update the number of raw data samples. */
    	numRawData = i - numDISCARD;

    	RC_Filter(300.0, SAMPLING_PERIOD, numRawData);

//    	IIR_Filter(0.500, numRawData);

//    	for (i = 0; i < numRawData; i++)
//    	{
////    		sprintf(msg, "\r\n(%d)[Raw Data]\r\nX = %f\r\nY = %f\r\nZ = %f\r\n",
////    				i, AccData_BUF.x[i], AccData_BUF.y[i], AccData_BUF.z[i]);
////    		HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);
//
//    		sprintf(msg, "%f,%f,%f\r\n", AccData_BUF.x[i], AccData_BUF.y[i], AccData_BUF.z[i]);
//    		HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);
//    	}

    	/* Convert the filtered raw data to acceleration[g]. */
    	for (i = 0; i < numRawData; i++)
    	{
    		AccData_BUF.x[i] = round(AccData_BUF.x[i]);
    		AccData_BUF.y[i] = round(AccData_BUF.y[i]);
    		AccData_BUF.z[i] = round(AccData_BUF.z[i]);

//    		sprintf(msg, "\r\n(%d)[Raw Filtered Data]\r\nX = %f\r\nY = %f\r\nZ = %f\r\n",
//    				i, AccData_BUF.x[i], AccData_BUF.y[i], AccData_BUF.z[i]);
//    		HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);

    		AccData_BUF.x[i] = (round(AccData_BUF.x[i] * Sensitivity) / 1000.0f);
    		AccData_BUF.y[i] = (round(AccData_BUF.y[i] * Sensitivity) / 1000.0f);
    		AccData_BUF.z[i] = (round(AccData_BUF.z[i] * Sensitivity) / 1000.0f);

//    		sprintf(msg, "\r\n(%d)Acceleration[g]\r\nX = %f\r\nY = %f\r\nZ = %f\r\n",
//    				i, AccData_BUF.x[i], AccData_BUF.y[i], AccData_BUF.z[i]);
//    		HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);

//    		sprintf(msg, "%f,%f,%f\r\n", AccData_BUF.x[i], AccData_BUF.y[i], AccData_BUF.z[i]);
//    		HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);
    	}
    	/* Update the number of data samples. */
    	numData = i;

    	RMS_Gravity(numData);

    	if (FFT_SIZE > 0U)
    	{
        	FastFourierTransform();
    	}
    	else
    	{}

    	for (i = 0; i < numData; i++)
    	{
//    		sprintf(msg, "\r\n(%d)Acceleration[g]\r\nX = %f\r\nY = %f\r\nZ = %f\r\n",
//    				i, AccData_BUF.x[i], AccData_BUF.y[i], AccData_BUF.z[i]);
//    		HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);

    		AccData_BUF.x[i] = AccData_BUF.x[i] * GRAVITATIONAL_ACCELERATION;
    		AccData_BUF.y[i] = AccData_BUF.y[i] * GRAVITATIONAL_ACCELERATION;
    		AccData_BUF.z[i] = AccData_BUF.z[i] * GRAVITATIONAL_ACCELERATION;

//    		sprintf(msg, "\r\n(%d)Acceleration[m/s^2]\r\nX = %f\r\nY = %f\r\nZ = %f\r\n",
//    				i, AccData_BUF.x[i], AccData_BUF.y[i], AccData_BUF.z[i]);
//    		HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);

//    		sprintf(msg, "%f,%f,%f\r\n", AccData_BUF.x[i], AccData_BUF.y[i], AccData_BUF.z[i]);
//    		HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);
    	}

//    	MovingAverage_Filter(6, numData, &numData);

//    	for (i = 0; i < numData; i++)
//    	{
////    		sprintf(msg, "\r\n(%d)Filtered Acceleration[m/s^2]\r\nX = %f\r\nY = %f\r\nZ = %f\r\n",
////    				i, AccData_BUF.x[i], AccData_BUF.y[i], AccData_BUF.z[i]);
////    		HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);
//
//    		sprintf(msg, "%f,%f,%f\r\n", AccData_BUF.x[i], AccData_BUF.y[i], AccData_BUF.z[i]);
//    		HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);
//    	}
    }
}


/* Use timer interrupt to sampling data */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM17)
	{
		/* Check read ready flag. */
		if (Read_RDY == 1U)
		{
			/* Check whether the array buffer is full or empty. */
			if (numRawData < DATA_BUFFER_SIZE)
			{
				if (numRawData < ((uint16_t)ODR_Hz))
				{
					/* Reading raw data. */
					if (LIS2DW12_Get_Acc_Data(&RawData) != BSP_ERROR_NONE)
					{
						Error_Handler();
					}

					/* Store raw data in the arrays. */
					AccData_BUF.x[numRawData] = RawData.x;
					AccData_BUF.y[numRawData] = RawData.y;
					AccData_BUF.z[numRawData] = RawData.z;

					/* Count the samples data. */
					numRawData ++;
				}
				/* If the data reaches the frequency sampling. */
				else
				{
					/* Clear read ready flag. */
					Read_RDY = 0U;
				}
			}
			/* If the buffer is full. */
			else
			{
				/* Clear read ready flag. */
				Read_RDY = 0U;
			}
		}
		else
		{
			/* To make sure that read ready flag is '0'. */
			Read_RDY = 0U;
		}
	}
}


/*----------------------------------------------------------------------------------------------------*/


static int32_t LIS2DW12_Get_Acc_Data(IKS01A3_MOTION_SENSOR_AxesRaw_t *RawData)
{
	uint8_t status;
	int32_t ret;

	/* Wait for data ready. */
	do
	{
		if ((ret = IKS01A3_MOTION_SENSOR_Get_DRDY_Status(IKS01A3_LIS2DW12_0, MOTION_ACCELERO, &status)) != BSP_ERROR_NONE)
		{
			return ret;
		}
	}
	while (status == 0U);

	/* Read raw data. */
	if ((ret = IKS01A3_MOTION_SENSOR_GetAxesRaw(IKS01A3_LIS2DW12_0, MOTION_ACCELERO, RawData)) != BSP_ERROR_NONE)
	{
		return ret;
	}

	return ret;
}


/*----------------------------------------------------------------------------------------------------*/


void RC_Filter(float fc, float Ts, uint16_t numDATA)
{
	uint16_t i;
	float RC;
	float Coeff1, Coeff2;

	/* Reset buffers. */
	RC = 0.0f;
	Coeff1 = 0.0f;
	Coeff2 = 0.0f;

	/* Compute RC constant from cut-off frequency. */
	RC = (1.0f / (2.0f * 3.141592f * fc));

	/* Coefficient of RC low-pass filter difference equation. */
	Coeff1 = Ts / (Ts + RC);
	Coeff2 = RC / (Ts + RC);

	for (i = 1; i < numDATA; i++)
	{
		/* Compute the RC low-pass filter difference equation. */
		AccData_BUF.x[i] = ((Coeff1 * AccData_BUF.x[i]) + (Coeff2 * AccData_BUF.x[i - 1]));
		AccData_BUF.y[i] = ((Coeff1 * AccData_BUF.y[i]) + (Coeff2 * AccData_BUF.y[i - 1]));
		AccData_BUF.z[i] = ((Coeff1 * AccData_BUF.z[i]) + (Coeff2 * AccData_BUF.z[i - 1]));
	}
}


/*----------------------------------------------------------------------------------------------------*/


void IIR_Filter(float Alpha, uint16_t numDATA)
{
	uint16_t i;

	/* Alpha is in the range of 0.0 to 1.0. */
	if (Alpha < 0.0f)
	{
		Alpha = 0.0f;
	}
	else if (Alpha > 1.0f)
	{
		Alpha = 1.0f;
	}
	else
	{}

	for (i = 1; i < numDATA; i++)
	{
		AccData_BUF.x[i] = (((1.0f - Alpha) * AccData_BUF.x[i]) + (Alpha * AccData_BUF.x[i - 1]));
		AccData_BUF.y[i] = (((1.0f - Alpha) * AccData_BUF.y[i]) + (Alpha * AccData_BUF.y[i - 1]));
		AccData_BUF.z[i] = (((1.0f - Alpha) * AccData_BUF.z[i]) + (Alpha * AccData_BUF.z[i - 1]));
	}
}


/*----------------------------------------------------------------------------------------------------*/


void MovingAverage_Filter(uint16_t WINDOW_SIZE, uint16_t numDATA, uint16_t *Filtered_numDATA)
{
    uint16_t i, j;
    Vibrations_double SUM;

    /* Reset buffers. */
    i = 0U;
    j = 0U;

    *Filtered_numDATA = 0U;

    /* Compute the moving average filter. */
    for (i = (WINDOW_SIZE - 1); i < numDATA; i++)
    {
    	SUM.x = 0.0f,	SUM.y = 0.0f,	SUM.z = 0.0f;

    	/* Summation of raw data. */
    	for (j = 0; j < WINDOW_SIZE; j++)
        {
            SUM.x += AccData_BUF.x[i - j];
            SUM.y += AccData_BUF.y[i - j];
            SUM.z += AccData_BUF.z[i - j];
        }

    	/* Average the sum of raw data. */
    	AccData_BUF.x[i - (WINDOW_SIZE - 1)] = SUM.x / WINDOW_SIZE;
    	AccData_BUF.y[i - (WINDOW_SIZE - 1)] = SUM.y / WINDOW_SIZE;
    	AccData_BUF.z[i - (WINDOW_SIZE - 1)] = SUM.z / WINDOW_SIZE;
    }

	/* Count the filtered data. */
    *Filtered_numDATA = i - (WINDOW_SIZE - 1);
}


/*----------------------------------------------------------------------------------------------------*/


void RMS_Gravity(uint16_t numDATA)
{
	uint16_t i;
	Vibrations_float SumSq;

	/* Reset buffers. */
	SumSq.x = 0.0f,	SumSq.y = 0.0f,	SumSq.z = 0.0f;

	for (i = 0; i < numDATA; i++)
	{
		SumSq.x += AccData_BUF.x[i] * AccData_BUF.x[i];
		SumSq.y += AccData_BUF.y[i] * AccData_BUF.y[i];
		SumSq.z += AccData_BUF.z[i] * AccData_BUF.z[i];
	}

	VibrationSensor_Output.LIS2DW12_RMS_Gravity.x = (float)sqrt(SumSq.x / numDATA);
	VibrationSensor_Output.LIS2DW12_RMS_Gravity.y = (float)sqrt(SumSq.y / numDATA);
	VibrationSensor_Output.LIS2DW12_RMS_Gravity.z = (float)sqrt(SumSq.z / numDATA);
}


/*----------------------------------------------------------------------------------------------------*/


void FastFourierTransform(void)
{
	uint16_t i;
	float Mag_x, Mag_y, Mag_z;
	FFT_TimeDomain FFT_Input, FFT_Output;
	FFT_FrequencyDomain VibrationSpectrum;

	/* Reset buffers. */
	for (i = 0; i < FFT_SIZE; i++)
	{
		FFT_Input.x[i] = 0.0f,	FFT_Input.y[i] = 0.0f,	FFT_Input.z[i] = 0.0f;

		FFT_Output.x[i] = 0.0f,	FFT_Output.y[i] = 0.0f,	FFT_Output.z[i] = 0.0f;
	}

	for (i = 0; i < (FFT_SIZE / 2); i++)
	{
		VibrationSpectrum.Frequency[i] = 0.0f;

		VibrationSpectrum.x[i] = 0.0f,	VibrationSpectrum.y[i] = 0.0f,	VibrationSpectrum.z[i] = 0.0f;
	}

	/* Store FFT input buffers. */
	for (i = 0; i < FFT_SIZE; i++)
	{
		FFT_Input.x[i] = AccData_BUF.x[i];
		FFT_Input.y[i] = AccData_BUF.y[i];
		FFT_Input.z[i] = AccData_BUF.z[i];

//		FFT_Input.x[i] = arm_sin_f32((2 * 3.14 * 200 * i) / ODR_Hz);
//		FFT_Input.y[i] = arm_sin_f32((2 * 3.14 * 400 * i) / ODR_Hz);
//		FFT_Input.z[i] = arm_sin_f32((2 * 3.14 * 600 * i) / ODR_Hz);
	}

	/* Compute Fast Fourier Transform algorithm. */
	arm_rfft_fast_f32(&FFThandler, FFT_Input.x, FFT_Output.x, 0);

	arm_rfft_fast_f32(&FFThandler, FFT_Input.y, FFT_Output.y, 0);

	arm_rfft_fast_f32(&FFThandler, FFT_Input.z, FFT_Output.z, 0);

	/* Magnitude spectrum of frequency domain. */
	for (i = 0; i < (FFT_SIZE / 2); i++)
	{
		Mag_x = 0.0f;
		Mag_y = 0.0f;
		Mag_z = 0.0f;

		/* Calculate the magnitude of complex number. */
		Mag_x = sqrtf((FFT_Output.x[i * 2] * FFT_Output.x[i * 2]) + (FFT_Output.x[(i * 2) + 1] * FFT_Output.x[(i * 2) + 1]));
		Mag_y = sqrtf((FFT_Output.y[i * 2] * FFT_Output.y[i * 2]) + (FFT_Output.y[(i * 2) + 1] * FFT_Output.y[(i * 2) + 1]));
		Mag_z = sqrtf((FFT_Output.z[i * 2] * FFT_Output.z[i * 2]) + (FFT_Output.z[(i * 2) + 1] * FFT_Output.z[(i * 2) + 1]));

		VibrationSpectrum.Frequency[i] = ((float)i * ODR_Hz) / FFT_SIZE;

		VibrationSpectrum.x[i] = Mag_x;
		VibrationSpectrum.y[i] = Mag_y;
		VibrationSpectrum.z[i] = Mag_z;
	}

//	for (i = 0; i < (FFT_SIZE / 2); i++)
//	{
////		sprintf(msg, "\r\n(%d)Frequency[%.4f Hz]\r\nX = %f\r\nY = %f\r\nZ = %f\r\n",
////				i, VibrationSpectrum.Frequency[i], VibrationSpectrum.x[i], VibrationSpectrum.y[i], VibrationSpectrum.z[i]);
////		HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);
//
//		sprintf(msg, "%f,%f,%f\r\n", VibrationSpectrum.x[i], VibrationSpectrum.y[i], VibrationSpectrum.z[i]);
//		HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);
//	}
}


/*----------------------------------------------------------------------------------------------------*/


void Vibration_Calculate()
{
	uint16_t i;
	DATA_BUFFER VelData_BUF;
	Vibrations_float MidPoint, Vel;
	Vibrations_double Acc_SumSq, Vel_SumSq;

	/* Reset buffers. */
	Vel.x = 0.0f,	Vel.y = 0.0f,	Vel.z = 0.0f;

	Acc_SumSq.x = 0.0f,	Acc_SumSq.y = 0.0f,	Acc_SumSq.z = 0.0f;

	Vel_SumSq.x = 0.0f,	Vel_SumSq.y = 0.0f,	Vel_SumSq.z = 0.0f;

	for (i = 0; i < DATA_BUFFER_SIZE; i++)
	{
		VelData_BUF.x[i] = 0.0f,	VelData_BUF.y[i] = 0.0f,	VelData_BUF.z[i] = 0.0f;
	}

	/* Integrate acceleration to get velocity by using Mid-point rule. [v = u + a*dt]
	 * (Integrate the area under the curve between two data points) */
//	for (i = 1; i < numData; i++)
//	{
//		MidPoint.x = 0.0f;
//		MidPoint.y = 0.0f;
//		MidPoint.z = 0.0f;
//
//		MidPoint.x = (AccData_BUF.x[i] + AccData_BUF.x[i - 1]) * 0.5f;
//		MidPoint.y = (AccData_BUF.y[i] + AccData_BUF.y[i - 1]) * 0.5f;
//		MidPoint.z = (AccData_BUF.z[i] + AccData_BUF.z[i - 1]) * 0.5f;
//
//		VelData_BUF.x[i] = VelData_BUF.x[i - 1] + (MidPoint.x * SAMPLING_PERIOD);
//		VelData_BUF.y[i] = VelData_BUF.y[i - 1] + (MidPoint.y * SAMPLING_PERIOD);
//		VelData_BUF.z[i] = VelData_BUF.z[i - 1] + (MidPoint.z * SAMPLING_PERIOD);
//	}

	 void StationaryPhase(uint16_t i)
	{
		if (fabs(AccData_BUF.x[i]) < 0.025)
		{
			Vel.x = 0.0f;
		}

		if (fabs(AccData_BUF.y[i]) < 0.025)
		{
			Vel.y = 0.0f;
		}

		if (fabs(AccData_BUF.z[i]) < 0.025)
		{
			Vel.z = 0.0f;
		}
	}

	for (i = 1; i < numData; i++)
	{
		MidPoint.x = 0.0f;
		MidPoint.y = 0.0f;
		MidPoint.z = 0.0f;

		MidPoint.x = (AccData_BUF.x[i] + AccData_BUF.x[i - 1]) * 0.5f;
		MidPoint.y = (AccData_BUF.y[i] + AccData_BUF.y[i - 1]) * 0.5f;
		MidPoint.z = (AccData_BUF.z[i] + AccData_BUF.z[i - 1]) * 0.5f;

		Vel.x += (MidPoint.x * SAMPLING_PERIOD);
		Vel.y += (MidPoint.y * SAMPLING_PERIOD);
		Vel.z += (MidPoint.z * SAMPLING_PERIOD);

		StationaryPhase(i);

		Vel_SumSq.x += (Vel.x * Vel.x);
		Vel_SumSq.y += (Vel.y * Vel.y);
		Vel_SumSq.z += (Vel.z * Vel.z);

//		sprintf(msg, "%f\r\n", Vel.z);
//		HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);
	}

	/* Compute sum of the square. */
    for (i = 0; i < numData; i++)
    {
    	/* Summation of acceleration square. */
    	Acc_SumSq.x += (AccData_BUF.x[i] * AccData_BUF.x[i]);
    	Acc_SumSq.y += (AccData_BUF.y[i] * AccData_BUF.y[i]);
    	Acc_SumSq.z += (AccData_BUF.z[i] * AccData_BUF.z[i]);

    	/* Summation of velocity square. */
//    	Vel_SumSq.x += (VelData_BUF.x[i] * VelData_BUF.x[i]);
//    	Vel_SumSq.y += (VelData_BUF.y[i] * VelData_BUF.y[i]);
//    	Vel_SumSq.z += (VelData_BUF.z[i] * VelData_BUF.z[i]);
    }

    /* Compute square root of mean. */
    rmsAcc.x[Num_Samples] = (float)sqrt(Acc_SumSq.x / numData);
    rmsAcc.y[Num_Samples] = (float)sqrt(Acc_SumSq.y / numData);
    rmsAcc.z[Num_Samples] = (float)sqrt(Acc_SumSq.z / numData);

    rmsVel.x[Num_Samples] = (float)sqrt(Vel_SumSq.x / numData);
    rmsVel.y[Num_Samples] = (float)sqrt(Vel_SumSq.y / numData);
    rmsVel.z[Num_Samples] = (float)sqrt(Vel_SumSq.z / numData);

	/* Count the vibrations data. */
    Num_Samples ++;

//	for (i = 0; i < numData; i++)
//	{
////		sprintf(msg, "\r\n(%d)Filtered Acceleration[m/s^2]\r\nX = %f\r\nY = %f\r\nZ = %f\r\n",
////				i, AccData_BUF.x[i], AccData_BUF.y[i], AccData_BUF.z[i]);
////		HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);
//
////		sprintf(msg, "%f,%f\r\n",AccData_BUF.z[i], VelData_BUF.z[i]);
////		HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);
//
////		sprintf(msg, "%f,%f\r\n",AccData_BUF.z[i], rmsAcc.z[0]);
////		HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);
//
//		sprintf(msg, "%f,%f\r\n", VelData_BUF.z[i], rmsVel.z[0]);
//		HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);
//
////		sprintf(msg, "%f,%f,%f,%f,%f,%f\r\n",
////				AccData_BUF.x[i], AccData_BUF.y[i], AccData_BUF.z[i], VelData_BUF.x[i], VelData_BUF.y[i], VelData_BUF.z[i]);
////		HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);
//	}
}


/*----------------------------------------------------------------------------------------------------*/


void Compute_RootMeanSquare(Vibrations_RMS *ptr, uint8_t ITERATION)
{
	uint8_t i, j;
	Vibrations_float Acc_SumSq, Vel_SumSq;

	/* Reset buffers. */
	Acc_SumSq.x = 0.0f,	Acc_SumSq.y = 0.0f,	Acc_SumSq.z = 0.0f;

	Vel_SumSq.x = 0.0f,	Vel_SumSq.y = 0.0f,	Vel_SumSq.z = 0.0f;

	for (i = 0; i < ITERATION_BUFFER_SIZE; i++)
	{
		ptr->rmsAcc[i].x = 0.0f;
		ptr->rmsAcc[i].y = 0.0f;
		ptr->rmsAcc[i].z = 0.0f;

		ptr->rmsVel[i].x = 0.0f;
		ptr->rmsVel[i].y = 0.0f;
		ptr->rmsVel[i].z = 0.0f;
	}

	/* Compute the first iteration's. */
	if (ITERATION == 0)
	{
		for (j = 0; j < Num_Samples; j++)
		{
			/* Sum of acceleration square. */
			Acc_SumSq.x += fabs(rmsAcc.x[j] * rmsAcc.x[j]);
			Acc_SumSq.y += fabs(rmsAcc.y[j] * rmsAcc.y[j]);
			Acc_SumSq.z += fabs(rmsAcc.z[j] * rmsAcc.z[j]);

			/* Sum of velocity square. */
			Vel_SumSq.x += fabs(rmsVel.x[j] * rmsVel.x[j]);
			Vel_SumSq.y += fabs(rmsVel.y[j] * rmsVel.y[j]);
			Vel_SumSq.z += fabs(rmsVel.z[j] * rmsVel.z[j]);
		}

		/* Root mean square of acceleration. */
		ptr->rmsAcc[ITERATION].x = sqrtf(Acc_SumSq.x / Num_Samples);
		ptr->rmsAcc[ITERATION].y = sqrtf(Acc_SumSq.y / Num_Samples);
		ptr->rmsAcc[ITERATION].z = sqrtf(Acc_SumSq.z / Num_Samples);

		/* Root mean square of velocity. */
		ptr->rmsVel[ITERATION].x = sqrtf(Vel_SumSq.x / Num_Samples);
		ptr->rmsVel[ITERATION].y = sqrtf(Vel_SumSq.y / Num_Samples);
		ptr->rmsVel[ITERATION].z = sqrtf(Vel_SumSq.z / Num_Samples);

		/* Count the data from the first iteration. */
		if (ptr->rmsNum[ITERATION] < VIBRATION_BUFFER_SIZE)
		{
			ptr->rmsNum[ITERATION]++;
		}
		else
		{
			ptr->rmsNum[ITERATION] = VIBRATION_BUFFER_SIZE;
		}
	}

	/* Compute the next sequence iteration's.*/
	else
	{
		for (j = 0; j < VIBRATION_BUFFER_SIZE; j++)
		{
			/* Sum of acceleration square. */
			Acc_SumSq.x += fabs(ptr->rmsAccBUF[ITERATION - 1].x[j] * ptr->rmsAccBUF[ITERATION - 1].x[j]);
			Acc_SumSq.y += fabs(ptr->rmsAccBUF[ITERATION - 1].y[j] * ptr->rmsAccBUF[ITERATION - 1].y[j]);
			Acc_SumSq.z += fabs(ptr->rmsAccBUF[ITERATION - 1].z[j] * ptr->rmsAccBUF[ITERATION - 1].z[j]);

			/* Sum of velocity square. */
			Vel_SumSq.x += fabs(ptr->rmsVelBUF[ITERATION - 1].x[j] * ptr->rmsVelBUF[ITERATION - 1].x[j]);
			Vel_SumSq.y += fabs(ptr->rmsVelBUF[ITERATION - 1].y[j] * ptr->rmsVelBUF[ITERATION - 1].y[j]);
			Vel_SumSq.z += fabs(ptr->rmsVelBUF[ITERATION - 1].z[j] * ptr->rmsVelBUF[ITERATION - 1].z[j]);
		}

		/* Root mean square of acceleration. */
		ptr->rmsAcc[ITERATION].x = sqrtf(Acc_SumSq.x / ptr->rmsNum[ITERATION - 1]);
		ptr->rmsAcc[ITERATION].y = sqrtf(Acc_SumSq.y / ptr->rmsNum[ITERATION - 1]);
		ptr->rmsAcc[ITERATION].z = sqrtf(Acc_SumSq.z / ptr->rmsNum[ITERATION - 1]);

		/* Root mean square of velocity. */
		ptr->rmsVel[ITERATION].x = sqrtf(Vel_SumSq.x / ptr->rmsNum[ITERATION - 1]);
		ptr->rmsVel[ITERATION].y = sqrtf(Vel_SumSq.y / ptr->rmsNum[ITERATION - 1]);
		ptr->rmsVel[ITERATION].z = sqrtf(Vel_SumSq.z / ptr->rmsNum[ITERATION - 1]);

		/* Count the data from each iteration. */
		if (ptr->rmsNum[ITERATION] < VIBRATION_BUFFER_SIZE)
		{
			ptr->rmsNum[ITERATION]++;
		}
		else
		{
			ptr->rmsNum[ITERATION] = VIBRATION_BUFFER_SIZE;
		}
	}
}

void RootMeanSquare_Iteration(Vibrations_RMS *ptr, uint8_t Iterations)
{
	uint8_t i, j;

	/* Prevent the iteration rounds from overflowing the buffer. */
	if (Iterations > ITERATION_BUFFER_SIZE)
	{
		Iterations = ITERATION_BUFFER_SIZE;
	}
	else
	{}

	/* Start the iteration's. */
	for (i = 0; i < Iterations; i++)
	{
		Compute_RootMeanSquare(ptr, i);

		/* Store the final iteration output. */
		if (i == (Iterations - 1))
		{
			/* Shift all elements to the left. */
			for (j = 0; j < (VIBRATION_BUFFER_SIZE - 1); j++)
			{
				ptr->RMS_AccOUT.x[j] = ptr->RMS_AccOUT.x[j + 1];
				ptr->RMS_AccOUT.y[j] = ptr->RMS_AccOUT.y[j + 1];
				ptr->RMS_AccOUT.z[j] = ptr->RMS_AccOUT.z[j + 1];

				ptr->RMS_VelOUT.x[j] = ptr->RMS_VelOUT.x[j + 1];
				ptr->RMS_VelOUT.y[j] = ptr->RMS_VelOUT.y[j + 1];
				ptr->RMS_VelOUT.z[j] = ptr->RMS_VelOUT.z[j + 1];
			}
			/* Store new data on the right. */
			ptr->RMS_AccOUT.x[(VIBRATION_BUFFER_SIZE - 1)] = ptr->rmsAcc[i].x;
			ptr->RMS_AccOUT.y[(VIBRATION_BUFFER_SIZE - 1)] = ptr->rmsAcc[i].y;
			ptr->RMS_AccOUT.z[(VIBRATION_BUFFER_SIZE - 1)] = ptr->rmsAcc[i].z;

			ptr->RMS_VelOUT.x[(VIBRATION_BUFFER_SIZE - 1)] = ptr->rmsVel[i].x;
			ptr->RMS_VelOUT.y[(VIBRATION_BUFFER_SIZE - 1)] = ptr->rmsVel[i].y;
			ptr->RMS_VelOUT.z[(VIBRATION_BUFFER_SIZE - 1)] = ptr->rmsVel[i].z;
		}

		/* Shift the iteration output. */
		else
		{
			/* Shift all elements to the right. */
			for (j = (VIBRATION_BUFFER_SIZE - 1); j > 0; j--)
			{
				ptr->rmsAccBUF[i].x[j] = ptr->rmsAccBUF[i].x[j - 1];
				ptr->rmsAccBUF[i].y[j] = ptr->rmsAccBUF[i].y[j - 1];
				ptr->rmsAccBUF[i].z[j] = ptr->rmsAccBUF[i].z[j - 1];

				ptr->rmsVelBUF[i].x[j] = ptr->rmsVelBUF[i].x[j - 1];
				ptr->rmsVelBUF[i].y[j] = ptr->rmsVelBUF[i].y[j - 1];
				ptr->rmsVelBUF[i].z[j] = ptr->rmsVelBUF[i].z[j - 1];
			}
			/* Store new data on the left. */
			ptr->rmsAccBUF[i].x[0] = ptr->rmsAcc[i].x;
			ptr->rmsAccBUF[i].y[0] = ptr->rmsAcc[i].y;
			ptr->rmsAccBUF[i].z[0] = ptr->rmsAcc[i].z;

			ptr->rmsVelBUF[i].x[0] = ptr->rmsVel[i].x;
			ptr->rmsVelBUF[i].y[0] = ptr->rmsVel[i].y;
			ptr->rmsVelBUF[i].z[0] = ptr->rmsVel[i].z;
		}
	}
}


/*----------------------------------------------------------------------------------------------------*/


void Delta_Vibration(Vibrations_RMS *ptr)
{
	uint8_t i;
	uint8_t numdelta;
	Vibrations_float AccSUM, VelSUM;
	Vibrations_DataBuffer deltaAcc, deltaVel;

	/* Reset buffers. */
	numdelta = 0U;

	AccSUM.x = 0.0f,	AccSUM.y = 0.0f,	AccSUM.z = 0.0f;

	VelSUM.x = 0.0f,	VelSUM.y = 0.0f,	VelSUM.z = 0.0f;

	for (i = 0; i < VIBRATION_BUFFER_SIZE; i++)
	{
		deltaAcc.x[i] = 0.0f,	deltaAcc.y[i] = 0.0f,	deltaAcc.z[i] = 0.0f;

		deltaVel.x[i] = 0.0f,	deltaVel.y[i] = 0.0f,	deltaVel.z[i] = 0.0f;
	}

	for (i = 0; i < (VIBRATION_BUFFER_SIZE - 1); i++)
	{
		deltaAcc.x[i] = fabs(ptr->RMS_AccOUT.x[i] - ptr->RMS_AccOUT.x[i + 1]) * 1000;
		deltaAcc.y[i] = fabs(ptr->RMS_AccOUT.y[i] - ptr->RMS_AccOUT.y[i + 1]) * 1000;
		deltaAcc.z[i] = fabs(ptr->RMS_AccOUT.z[i] - ptr->RMS_AccOUT.z[i + 1]) * 1000;

		deltaVel.x[i] = fabs(ptr->RMS_VelOUT.x[i] - ptr->RMS_VelOUT.x[i + 1]) * 1000;
		deltaVel.y[i] = fabs(ptr->RMS_VelOUT.y[i] - ptr->RMS_VelOUT.y[i + 1]) * 1000;
		deltaVel.z[i] = fabs(ptr->RMS_VelOUT.z[i] - ptr->RMS_VelOUT.z[i + 1]) * 1000;

		numdelta ++;
	}

	for (i = 0; i < numdelta; i++)
	{
		AccSUM.x += deltaAcc.x[i];
		AccSUM.y += deltaAcc.y[i];
		AccSUM.z += deltaAcc.z[i];

		VelSUM.x += deltaVel.x[i];
		VelSUM.y += deltaVel.y[i];
		VelSUM.z += deltaVel.z[i];
	}

//	RMS_Acceleration.x = (AccSUM.x / numdelta);
//	RMS_Acceleration.y = (AccSUM.y / numdelta);
//	RMS_Acceleration.z = (AccSUM.z / numdelta);
//
//	RMS_Velocity.x = (VelSUM.x / numdelta);
//	RMS_Velocity.y = (VelSUM.y / numdelta);
//	RMS_Velocity.z = (VelSUM.z / numdelta);

//	RMS_Acceleration.x = fabs(ptr->RMS_AccOUT.x[(VIBRATION_BUFFER_SIZE - 1)] - ptr->RMS_AccOUT.x[(VIBRATION_BUFFER_SIZE - 2)]) * 1000;
//	RMS_Acceleration.y = fabs(ptr->RMS_AccOUT.y[(VIBRATION_BUFFER_SIZE - 1)] - ptr->RMS_AccOUT.y[(VIBRATION_BUFFER_SIZE - 2)]) * 1000;
//	RMS_Acceleration.z = fabs(ptr->RMS_AccOUT.z[(VIBRATION_BUFFER_SIZE - 1)] - ptr->RMS_AccOUT.z[(VIBRATION_BUFFER_SIZE - 2)]) * 1000;
//
//	RMS_Velocity.x = fabs(ptr->RMS_VelOUT.x[(VIBRATION_BUFFER_SIZE - 1)] - ptr->RMS_VelOUT.x[(VIBRATION_BUFFER_SIZE - 2)]) * 1000;
//	RMS_Velocity.y = fabs(ptr->RMS_VelOUT.y[(VIBRATION_BUFFER_SIZE - 1)] - ptr->RMS_VelOUT.y[(VIBRATION_BUFFER_SIZE - 2)]) * 1000;
//	RMS_Velocity.z = fabs(ptr->RMS_VelOUT.z[(VIBRATION_BUFFER_SIZE - 1)] - ptr->RMS_VelOUT.z[(VIBRATION_BUFFER_SIZE - 2)]) * 1000;
}


/*----------------------------------------------------------------------------------------------------*/


static int32_t HTTS751_Sensor_Handler()
{
	uint8_t i;
	int32_t ret;

	Temperature = 0.0f;

	/* Set the sensor register. */
	for (i = 0; i < HTTS751_REG_COUNT; i++)
	{
		if ((ret = IKS01A3_ENV_SENSOR_Write_Register(IKS01A3_STTS751_0, HTTS751_reg_addr[i], HTTS751_reg[i])) != BSP_ERROR_NONE)
		{
			return ret;
		}
	}

	/* Read the data. */
	if (HTTS751_Get_ENV_Data(&Temperature) != BSP_ERROR_NONE)
	{
		Error_Handler();
	}

	/* Environmental sensor data ready flag. */
	EnvironmentalDRDY = 1U;

	return ret;
}


/*----------------------------------------------------------------------------------------------------*/


static int32_t HTTS751_Get_ENV_Data(float *Temp)
{
	uint8_t status;
	int32_t ret;

	/* Wait for data ready. */
	do
	{
		if ((ret = IKS01A3_ENV_SENSOR_Get_DRDY_Status(IKS01A3_STTS751_0, ENV_TEMPERATURE, &status)) != BSP_ERROR_NONE)
		{
			return ret;
		}
	}
	while (status == 0U);

	/* Read temperature data.*/
	if ((ret = IKS01A3_ENV_SENSOR_GetValue(IKS01A3_STTS751_0, ENV_TEMPERATURE, Temp)) != BSP_ERROR_NONE)
	{
		return ret;
	}

	return ret;
}


/*----------------------------------------------------------------------------------------------------*/


static void floatToInt(float Input_value, displayFloatToInt_t *Output_value, int32_t dec_prec)
{
	/* Check the sign. */
	if (Input_value >= 0.0f)
	{
		Output_value->sign = 0;
	}
	else
	{
		Output_value->sign = 1;
		Input_value = - Input_value;
	}

	Input_value = Input_value + (0.5 / pow(10, dec_prec));
	Output_value->out_int = (int32_t)Input_value;
	Input_value = Input_value - (float)(Output_value->out_int);
	Output_value->out_dec = (int32_t)trunc(Input_value * pow(10, dec_prec));
}


/*----------------------------------------------------------------------------------------------------*/


void UART_printData(void)
{
	displayFloatToInt_t out_value;

	sprintf(msg, "\r\nSampling Rate = %.1f Hz", (float)ODR_Hz);
	HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);

	sprintf(msg, "\r\nRMS Acceleration[g]\r\nX = %.3f\r\nY = %.3f\r\nZ = %.3f"
			, (RMS_Acceleration.x / 9.80665), (RMS_Acceleration.y / 9.80665), (RMS_Acceleration.z / 9.80665));
	HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);

	sprintf(msg, "\r\nRMS Acceleration[mm/s^2]\r\nX = %.2f\r\nY = %.2f\r\nZ = %.2f"
			, (RMS_Acceleration.x), (RMS_Acceleration.y), (RMS_Acceleration.z));
	HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);

	sprintf(msg, "\r\nRMS Velocity[mm/s]\r\nX = %.2f\r\nY = %.2f\r\nZ = %.2f"
			, (RMS_Velocity.x), (RMS_Velocity.y), (RMS_Velocity.z));
	HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);

	floatToInt(Temperature, &out_value, 2);
	sprintf(msg, "\r\nTemperature[degC]: %c%d.%02d\r\n", ((out_value.sign > 0) ? '-' : '+'), (int)out_value.out_int, (int)out_value.out_dec);
	HAL_UART_Transmit(&UARTx, (uint8_t*)msg, strlen(msg), 100);
}
