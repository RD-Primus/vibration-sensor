/*
 * LoRa_Support.h
 *
 *  Created on: Oct 7, 2024
 *      Author: Nilniz
 */

#ifndef INC_LORA_SUPPORT_H_
#define INC_LORA_SUPPORT_H_

#define RUN_TEST		0

#if !RUN_TEST
#define TxListMax 					10
#define PayLoadSize 				20
#define JOIN_MAX 						15
#define ACK_MAX 						10
#define Join_Cycle	 				40000 //40 second
#define multiply_lora_Send 	3600000 // 1 hour 3600000
#else
#define TxListMax 					10
#define PayLoadSize 				20
#define JOIN_MAX 						15
#define ACK_MAX 						10
#define Join_Cycle	 				40000 //40 second
#define multiply_lora_Send 	1000 // 1 hour 3600000
#endif
extern ActivationType_t LORAWAN_DEFAULT_ACTIVATION_TYPE;
extern ActivationType_t ActivationType;

#define LPP_DIGITAL_COUNT       			0x65              /* _ 4 byte */
#define LPP_ALARM_STATUS	      			0x66              /* _ 1 byte */

typedef enum LoRa_Status {

	Join_Requests,
	Join_Fail,
	Joined,
	LoRa_Send,

} LoRa_Status ;
typedef enum Op_Status {

	OnJoin,
	Sleep_Mode,
	ACK,
	NACK

} Op_Status ;


typedef enum LoraMode{
	LoRa_Run,
	LoRa_Test,


	LoraMode_Max = 0xFF
}LoRa_operate_Mode;
typedef struct {
		LoRa_operate_Mode operate_mode;
		int16_t RSSI;
		uint8_t ACK_Fail ;
		uint8_t JOIN_Fail ;
		uint8_t TxIndex ;
		uint32_t TxCycle ;
		uint8_t TxList;
		uint8_t TxByte;
		uint8_t List_PayLoad[TxListMax][PayLoadSize];
		uint8_t CursorPayLoad;

		uint16_t Tx_DutyCycle;
		uint8_t LoRa_Mode;
		UTIL_TIMER_Object_t *Timer;
		uint32_t MCU_ID[3] ;
		uint32_t UplinkCounter ;
		uint32_t* buff_FCnts ;
		void (*Uplink_Save)(uint32_t UplinkCounter_);
		uint8_t First_join;
		LoRa_Status LR_Status;
		Op_Status OP_Status;

} LoRa_Support_ ;
extern LoRa_Support_ LoRa_Sup ;
extern void Reset_fCnt_();
extern void Link_LoRa_buff();
extern void Load_fCntUp(uint32_t* FCntUp);
extern void LoRa_Support_Init(uint8_t LoRa_Mode, uint8_t LoRa_Sampling, uint32_t UplinkCounter, void *Uplink_Save);
extern void LoRa_Support_Force_Send ();
extern void LoRa_Support_OnJoin (int16_t status);
extern void LoRa_Support_OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params);
extern uint32_t LoRa_Support_OnTxData(LmHandlerTxParams_t *params, UTIL_TIMER_Object_t *Timer) ;
extern void LoRaSupport_CayenBuffer(uint8_t* _CayenBuffData, uint8_t* _CayenBuffCursor, uint8_t _CayenBuffSize) ;

#endif /* INC_LORA_SUPPORT_H_ */
