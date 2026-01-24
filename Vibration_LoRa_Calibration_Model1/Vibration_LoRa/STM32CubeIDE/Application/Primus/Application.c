/*
 * Application.c
 *
 *  Created on: 25 ต.ค. 2565
 *      Author: RD
 */
#include "main.h"

#if !defined   INC_APPLICATION_H_
///TODO Add include Library to "main.h"
#include "Application.h"
#endif

//DEBUG
#define RUN_Programe  //For Test Check Memory Use
//#define Power_Test
#if ( defined RUN_Programe)

UTIL_TIMER_Object_t NFC_Power_OFF;
UTIL_TIMER_Object_t Reboot;
UTIL_TIMER_Object_t Req_sensor;
UTIL_TIMER_Object_t Log;
UTIL_TIMER_Object_t App_Tick;

UTIL_TIMER_Object_t wake_up;
UTIL_TIMER_Object_t TEST_SQE;

ApplicationData_typedef App ;



///FIXME User Configure Pointer Functions
//Function_typedef Display_Func ;


///TODO Example Application
/*
 Add Function Function_Init(X,X,X,X); to Initial Function
 Function_Init(Pointer AppRun_typedef , Function_Run, Function Tick or 0 not use, Interval Loop, Start Program);
 **Function Send interval_ms to Function_Run for functions use
*/

void Function_Init(Function_typedef *app, void *Function_Run, void *Function_Tick, uint16_t Delay_ms, Program_ Run) {

	app->Run = Run ;
	app->Function_Run = Function_Run ;
	app->Function_Tick = Function_Tick ;
	app->Interval_ms = Delay_ms ;
	App.Function_Data[App._index_] = app ;
	App._index_++ ;
}

static uint16_t ms = 0 ;
void Function_Tick(void) {

	if ( ++ms > 1000 ) {
		App.loop_per_ms = App.loop_Tick ;
		App.loop_Tick = 1 ;
		ms = 1 ;
	}

	for ( uint8_t i = 0; i < App._index_; i++ ) {
		App._index_Tick = i ;
		App.Function_Data[i]->sTick++ ;

		if ( ms >= 1000 ) {
			App.Function_Data[i]->loop_per_ms = App.Function_Data[i]->loop_Tick ;
			App.Function_Data[i]->loop_Tick = 0 ;
		}

		if ( App.Function_Data[i]->Function_Tick != 0 && App.Function_Data[i]->Run != _WAIT_)
			App.Function_Data[i]->Function_Tick() ;

	}
}

void Function_Start(void) {

	for ( uint8_t i = 0; i < App._index_; i++ ) {
		App.Function_Data[i]->Run = _START_UP_;
	}

}
void Function_Loop(void) {

	App.loop_Tick++ ;
	////////		Library Run		///////
	for ( uint8_t i = 0; i < App._index_; i++ ) {
		App._index_Func = i ;
		if ( App.Function_Data[i]->sTick >= App.Function_Data[i]->Interval_ms ) {
			if ( App.Function_Data[i]->Function_Run != 0 && App.Function_Data[i]->Run != _WAIT_){
				App.Function_Data[i]->loop_Tick++ ;
				App.Function_Data[i]->sTick = 0 ;
				App.Function_Data[i]->Function_Run(App.Function_Data[i]->Interval_ms) ;
//				App.Function_Data[i]->Function_Run() ;
			}
		}
	}
}
#endif

void Application_Initial(void) {
	///* TODO initial *///
#if ( defined RUN_Programe)
	/* USER CODE BEGIN WHILE */

  UTIL_TIMER_Create(&NFC_Power_OFF, NFC_Power_On_ms, UTIL_TIMER_ONESHOT, NFC_07A1_OFF, NULL);
  UTIL_TIMER_Create(&Reboot, 3000, UTIL_TIMER_ONESHOT, Force_Reboot, NULL);

	MW_LOG(TS_OFF, VLEVEL_M, "###### Primus Init Start ######\r") ;
  NFC_07A1_Init() ;
  Parameter_Init();
  NFC_Config.LoRa_Mode = 1;
  LoRa_Support_Init(NFC_Config.LoRa_Mode,NFC_Config.LoRa_Sampling, NFC_Info.UplinkCounter, &UplinkCounter_Save);



//	UTIL_TIMER_Create(&Req_sensor, 10, UTIL_TIMER_ONESHOT, Sensor_Run, NULL);
	Sensor_Init();


	UTIL_TIMER_Create(&Log, SENSOR_LOG_PERIOD, UTIL_TIMER_PERIODIC, Sensor_Log, NULL);
	UTIL_TIMER_Start(&Log);


//	UTIL_TIMER_Create(&wake_up, 5000, UTIL_TIMER_PERIODIC, wake_up_ISM330DHCX, NULL);
//	UTIL_TIMER_Start(&wake_up);

//  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_ApplicationLoop), UTIL_SEQ_RFU, Measure_Acc_ISM330DHCX);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_ApplicationLoop), UTIL_SEQ_RFU, Application_Run);
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_ApplicationLoop), CFG_SEQ_Prio_Loop);
//  UTIL_SEQ_PauseTask(1 << CFG_SEQ_Task_ApplicationLoop);

	MW_LOG(TS_OFF, VLEVEL_M, "###### Primus Init End ######\r") ;
	/* USER CODE END WHILE */
#endif
}



void Application_Run(void) {
	///* TODO Infinite loop *///
#if ( !defined Power_Test)

//	Function_Loop() ;
	/* USER CODE BEGIN WHILE */

//	Sensor_Run();
//	Measure_Acc_ISM330DHCX();
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_ApplicationLoop), CFG_SEQ_Prio_Loop);
	/* USER CODE END WHILE */
	/**************************************/

#endif
}


void HAL_IncTick(void) {
//	SysTick 1 mS
	uwTick += uwTickFreq ;
	if(uwTick >= UINT32_MAX - 1)
		NVIC_SystemReset();

#if ( defined RUN_Programe)
	Function_Tick() ;
#endif
}
uint8_t iii = 0;
uint16_t Tick;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

#if ( !defined Power_Test)
	if(htim->Instance == TIM17){
		//3200
			Measure_Acc_ISM330DHCX();
	}
	else if(htim->Instance == TIM16){
		Tick++;
			if ( Tick >= 1000 ) {
				Sensor.ISM330DHCX_1Sec_1 = Sensor.ISM330DHCX_Tick ;
				Sensor.ISM330DHCX_Tick = 0 ;
				Tick = 0;
			}
	}


#endif

}

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
void Application_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

#if ( !defined Power_Test)
  switch (GPIO_Pin)
  {


  case  ACCELERO_GYRO_INT_Pin:
	  	ACCELERO_GYRO_INT_RUN();
    break;

    case  NFC_07A1_EH_Pin:
      NFC_07A1_CallBack_EXIT_in(GPIO_Pin);
      break;
    default:
      break;
  }

#endif

}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
void Application_UART_RxCpltCallback(UART_HandleTypeDef *huart){
#if ( !defined Power_Test)


#endif
}
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
void Application_UART_TxCpltCallback(UART_HandleTypeDef *huart){

}

