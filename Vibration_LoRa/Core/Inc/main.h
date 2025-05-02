/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wlxx_hal.h"

#include "st25dvxxkc_conf.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "tim.h"
#include "adc_if.h"
#include "mw_log_conf.h"
#include "stm32_timer.h"
#include "stm32_seq.h"
#include "utilities_def.h"
#include "radio_driver.h"

#include "LmHandler.h"
#include "lora_app.h"
#include "lora_info.h"
#include "app_version.h"
#include "stdlib.h"

#include "custom_nfc07a1_nfctag.h"
#include "sys_app.h"

#include "LoRaMacInterfaces.h"
#include "secure-element.h"
#include "LoRaMacTypes.h"


//#define ADC_Disable_
#include "stdbool.h"
#include "usart.h"
#include "Parameter.h"

#include <Sensor.h>
#include "FLASH_STM32.h"
#include "Alarm_interface.h"
#include "NFC_07A1_Interface.h"
#include "LoRa_Support.h"
#include "Application.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RTC_N_PREDIV_S 10
#define RTC_PREDIV_S ((1<<RTC_N_PREDIV_S)-1)
#define RTC_PREDIV_A ((1<<(15-RTC_N_PREDIV_S))-1)
#define LED3_Pin GPIO_PIN_11
#define LED3_GPIO_Port GPIOB
#define USARTx_RX_Pin GPIO_PIN_3
#define USARTx_RX_GPIO_Port GPIOA
#define USARTx_TX_Pin GPIO_PIN_2
#define USARTx_TX_GPIO_Port GPIOA
#define IRQ_NFC_Pin GPIO_PIN_10
#define IRQ_NFC_GPIO_Port GPIOB
#define LPD_NFC_Pin GPIO_PIN_8
#define LPD_NFC_GPIO_Port GPIOA
#define PROB1_Pin GPIO_PIN_12
#define PROB1_GPIO_Port GPIOB
#define BUT1_Pin GPIO_PIN_0
#define BUT1_GPIO_Port GPIOA
#define BUT1_EXTI_IRQn EXTI0_IRQn
#define PROB2_Pin GPIO_PIN_13
#define PROB2_GPIO_Port GPIOB
#define ACCELERO_GYRO_INT_Pin GPIO_PIN_2
#define ACCELERO_GYRO_INT_GPIO_Port GPIOB
#define ACCELERO_GYRO_INT_EXTI_IRQn EXTI2_IRQn
#define ACCELERO_GYRO_INT2_Pin GPIO_PIN_14
#define ACCELERO_GYRO_INT2_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_9
#define LED2_GPIO_Port GPIOB
#define GPO_Pin GPIO_PIN_8
#define GPO_GPIO_Port GPIOB
#define V_EH_Pin GPIO_PIN_4
#define V_EH_GPIO_Port GPIOB
#define NFC_V_Pin GPIO_PIN_5
#define NFC_V_GPIO_Port GPIOB
#define LPD_Pin GPIO_PIN_3
#define LPD_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
