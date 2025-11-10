/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_mems.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motion_tl2.h"
#include "stm32wlxx_hal.h"
#include <stdbool.h>
#include <math.h>

 MTL2_knobs_t Knobs;
 MTL2_input_t input = {0};
 MTL2_output_t output = {0};

 bool Mode;
 bool Compass;

float Tilt_angle ;
float calibration;
float pitch_angle;
float Roll_angle;

float theta_angle;
float psi_angle;
float phi_angle;

float input_z;

#define VERSION_STR_LENG 35
#define SAMPLE_FREQ_HZ 50.0f




/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//char lib_version[35];
//uint8_t MotionTL2_GetLibVersion(lib_version);

#define VERSION_STR_LENG    35
/* Using tilt algorithm */

void Timer_OR_DataRate_Interrupt_Handler()
{
//	if(!Mode){
//		 Knobs.mode = MTL2_SINGLE_PLANE ;
//	}else{
//		 Knobs.mode = MTL2_DUAL_PLANE ;
//	}MotionTL2_SetKnobs(&Knobs);

	MX_MEMS_Process();
	  input.acc_x = (float)acceleration.x / 1000.0f;
	  input.acc_y = (float)acceleration.y / 1000.0f;
	  input_z = (float)acceleration.z / 1000.0f;

//	input.acc_x = 350.0f/ 1000.0f;
//	input.acc_y = 940.0f / 1000.0f;


//Todo ALL equation --------------------------------------------------
	Tilt_angle = atan2f(input.acc_x , input.acc_y) * (180.0f / M_PI);

	pitch_angle = atan2f((input.acc_x ),
			sqrtf((input.acc_y * input.acc_y) + (input_z * input_z) )) * (180.0f / M_PI);

	Roll_angle = atan2f((input.acc_y * (-1.0f)),
			sqrtf((input.acc_x * input.acc_x) + (input_z * input_z) )) * (180.0f / M_PI);

	theta_angle = asinf(input.acc_x)* (180.0f / M_PI);

	psi_angle = asinf(input.acc_y) * (180.0f / M_PI);

	phi_angle = asinf(sqrtf((input.acc_x * input.acc_x) + (input.acc_y * input.acc_y))) * (180.0f / M_PI);
//-------------------------------------------------------------------

	//MotionTL2_Update(&input, HAL_GetTick(), &output);


}

//void MotionTL2_Setup(void)
//{
//    char    lib_version[35];
//    uint8_t n;
//
//    MotionTL2_Init(MTL2_MCU_STM32);
//
//    n = MotionTL2_GetLibVersion(lib_version);
//    printf("MotionTL2 version: %s (%u)\r\n", lib_version, n);
//}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
  __HAL_RCC_CRC_CLK_ENABLE();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM16_Init();
  MX_MEMS_Init();
  /* USER CODE BEGIN 2 */

 // ****************************   tilt measurement  *****************
//  MotionTL2_Init(MTL2_MCU_STM32);
//
//  char lib_version[VERSION_STR_LENG];
//  MotionTL2_GetLibVersion(lib_version);
//
//  /* OPTIONAL */
//  MotionTL2_GetKnobs(&Knobs);
//  Knobs.fullscale = 2.0f;             // FS
//  Knobs.k         = 20.0f;            // k parameter is the filtering coefficient. The range of k is [0.1 to ODR].
//  Knobs.mode      = MTL2_SINGLE_PLANE  ;
//  // MTL2_SINGLE_PLANE = 0 enables the angle computation in single plane mode
//  // MTL2_DUAL_PLANE = 1 enables the angle computation in dual plane mode
//
//  Knobs.orn[0]    = 'e';  // X
//  Knobs.orn[1]    = 'n';  // Y
//  //n (north) or s (south), w (west)(ตก) or e (east)(ออก).
//
//  /* Update fullscale, k, orientation, mode */
//  MotionTL2_SetKnobs(&Knobs);


//  //*******************************************************************


  //HAL_TIM_Base_Start_IT(&htim16);


  /* Using tilt algorithm */



  /* USER CODE END 2 */

  /* Boot CPU2 */
  HAL_PWREx_ReleaseCore(PWR_CORE_CPU2);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

//	  x = 10 ;
//		printf("55\r\n");
//		x = 22 ;
    /* USER CODE END WHILE */

	  Timer_OR_DataRate_Interrupt_Handler();
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 48-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1000-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_PeriodElapsedHalfCpltCallback could be implemented in the user file
   */
  if (htim->Instance == TIM16)
  {
	 // MX_MEMS_Process();


	//printf("55\r\n");
//		    /* Run tilt sensing algorithm */
//	MotionTL2_Update(&input, HAL_GetTick(), &output);
  }

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
