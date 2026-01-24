/*
 * Application.h
 *
 *  Created on: 25 ต.ค. 2565
 *      Author: RD
 */

#ifndef INC_APPLICATION_H_
#define INC_APPLICATION_H_

#include "main.h"

///FIXME User Configure Max Function Index
#define Index_Function 		10
typedef enum{
	_WAIT_,
	_START_UP_,

	Program_Max = UINT8_MAX
}Program_;

typedef struct {

		uint16_t Interval_ms;
		uint16_t loop_per_ms;
		uint16_t sTick;
		uint16_t loop_Tick;
		Program_ Run;
		void (*Function_Run)(uint16_t Interval_ms);
//		void (*Function_Run)(void);
		void (*Function_Tick)(void);

} Function_typedef;

typedef struct {
		uint8_t _index_;
		uint8_t _index_Tick;
		uint8_t _index_Func;
		uint16_t loop_per_ms;
		uint16_t loop_Tick;
		Function_typedef* Function_Data[Index_Function];
} ApplicationData_typedef;
extern ApplicationData_typedef App ;
extern void Function_Start(void);
extern void Application_Initial(void) ;
extern void Application_Run(void);
extern void Application_GPIO_EXTI_Callback(uint16_t GPIO_Pin);


extern UTIL_TIMER_Object_t Req_sensor;

extern UTIL_TIMER_Object_t NFC_Power_OFF;
extern UTIL_TIMER_Object_t Reboot;
#if (V23037_Program_Run == interupt_Run)
extern UTIL_TIMER_Object_t V23037_Tick_;
#endif

extern void Application_UART_RxCpltCallback(UART_HandleTypeDef *huart);
extern void Application_UART_TxCpltCallback(UART_HandleTypeDef *huart);
///TODO NOTE
/*
	- Timer 17 Fix Freq
	- acc.output_rms_ = ความเร่ง rms
	- v.output_sqr_ = ความเร็ว
	- displacement.output_sqr_ = การกระจัด
	- freq.peaks_acc[i].output_Hz_ = ความถี่


*/
#endif /* INC_APPLICATION_H_ */
