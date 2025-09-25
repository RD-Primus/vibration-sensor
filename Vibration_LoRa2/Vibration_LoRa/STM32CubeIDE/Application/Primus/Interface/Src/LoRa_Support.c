/*
 * LoRa_Support.c
 *
 *  Created on: Oct 7, 2024
 *      Author: Nilniz
 */
//LORAWAN_DEFAULT_DATA_RATE
#include "main.h"

#define RETURN_
ActivationType_t LORAWAN_DEFAULT_ACTIVATION_TYPE = ACTIVATION_TYPE_OTAA;
ActivationType_t ActivationType = ACTIVATION_TYPE_OTAA;

LoRa_Support_ LoRa_Sup ;

uint32_t Send_Cycle = 20000 ; //> 0 for Test Send
uint8_t fcnt_init = 0;
uint16_t Random_Time(uint16_t Min, uint16_t Max) {

	uint16_t Random = (rand()) % (Max - Min + 1) + Min ;
	uint32_t Next_random = rand() + ((HAL_GetTick() + 2) / 2) ;
	srand(Next_random) ;
	return Random ;

}

//uint8_t SUBGRF_SetRfTxPower(int8_t power) {
//	power = 22 ;
//	uint8_t paSelect = RFO_HP ;
//
//	SUBGRF_SetTxParams(paSelect, power, RADIO_RAMP_3400_US) ;
//
//	return paSelect ;
//}

void LoRa_Support_Init(uint8_t LoRa_Mode, uint8_t LoRa_Sampling, uint32_t UplinkCounter, void *Uplink_Save) {
	///Read MCUID
	LoRa_Sup.MCU_ID[0] = HAL_GetUIDw0() ;
	LoRa_Sup.MCU_ID[1] = HAL_GetUIDw1() ;
	LoRa_Sup.MCU_ID[2] = HAL_GetUIDw2() ;
	///Set Default Value for Random
	srand(LoRa_Sup.MCU_ID[0] ^ LoRa_Sup.MCU_ID[1] ^ LoRa_Sup.MCU_ID[2]) ;

	LoRa_Sup.UplinkCounter = UplinkCounter + 1000 ;
	LoRa_Sup.Uplink_Save = Uplink_Save ;
	LoRa_Sup.LoRa_Mode = LoRa_Mode ;
	LoRa_Sup.Tx_DutyCycle = LoRa_Sampling ;
	LORAWAN_DEFAULT_ACTIVATION_TYPE = ActivationType = !LoRa_Sup.LoRa_Mode ? ACTIVATION_TYPE_ABP : ACTIVATION_TYPE_OTAA ;

	MW_LOG(TS_OFF, VLEVEL_M, "\r\n###### LoRa Activation Type : %s ######\r\n", !LoRa_Sup.LoRa_Mode ? "ABP" : "OTAA");


	if(fcnt_init){
		*LoRa_Sup.buff_FCnts = !LoRa_Sup.LoRa_Mode ? LoRa_Sup.UplinkCounter : 0 ;
		MW_LOG(TS_OFF, VLEVEL_M, "\r\n######  UplinkCounter Load : %u ######\r\n", *LoRa_Sup.buff_FCnts);
	}


	HAL_Delay(Random_Time(100, 1000)) ;

}

/*
 ResetFCnts

 static void ResetFCnts( void )
{
    CryptoNvm->FCntList.FCntUp = 0;
    CryptoNvm->FCntList.NFCntDown = FCNT_DOWN_INITIAL_VALUE;
    CryptoNvm->FCntList.AFCntDown = FCNT_DOWN_INITIAL_VALUE;
    CryptoNvm->FCntList.FCntDown = FCNT_DOWN_INITIAL_VALUE;
    CryptoNvm->LastDownFCnt = CryptoNvm->FCntList.FCntDown;

    for( int32_t i = 0; i < LORAMAC_MAX_MC_CTX; i++ )
    {
        CryptoNvm->FCntList.McFCntDown[i] = FCNT_DOWN_INITIAL_VALUE;
    }
    Load_fCntUp(&CryptoNvm->FCntList.FCntUp);
}

 */

void Load_fCntUp(uint32_t *FCntUp) {
	fcnt_init = 1;
	LoRa_Sup.buff_FCnts = FCntUp ;

}
void Reset_fCnt_() {

	if ( ActivationType == ACTIVATION_TYPE_ABP ) {
		*LoRa_Sup.buff_FCnts = 0 ;
		LoRa_Sup.UplinkCounter = *LoRa_Sup.buff_FCnts ;
		LoRa_Sup.Uplink_Save(*LoRa_Sup.buff_FCnts) ;
	}

}
void TransmitionCycle(uint32_t Time) {
	if ( LoRa_Sup.operate_mode == LoRa_Test )
		Send_Cycle = 20000 ;
#if RUN_TEST

	Send_Cycle += (Random_Time(5000, 60000)) ; // random 50 - 60000 mS
#endif

	if ( Send_Cycle != 0 ) {
//		LoRa_Sup.TxCycle = LoRa_Sup.JOINED == 0xA1 ? 60000 : Send_Cycle ;
		LoRa_Sup.TxCycle = Send_Cycle ;
	} else {
		LoRa_Sup.TxCycle = Time ;

		LoRa_Sup.TxCycle += (Random_Time(50, 60000)) ; // random 50 - 60000 mS
	}

}
void LogTime_Next_Send() {

	MW_LOG(TS_OFF, VLEVEL_M, "\r\n###### Next Send Time :  %d minute | %d second ######\r\n\n", (LoRa_Sup.TxCycle / 1000) / 60, (LoRa_Sup.TxCycle % 60000) /1000 );
}
uint8_t Timer_check() {
	if ( LoRa_Sup.Timer != NULL ) {
		return 1 ;
	} else {
		MW_LOG(TS_OFF, VLEVEL_M, "\r\n###### Timer Error ######\r\n") ;
		return 0 ;
	}

}

void LoRa_Support_Force_Send() {

	if ( Timer_check() ) {
		LoRa_Sup.TxCycle = 40000 ;
		LoRa_Sup.TxIndex = 0 ;
		LogTime_Next_Send() ;
		UTIL_TIMER_Stop(LoRa_Sup.Timer) ;
		UTIL_TIMER_SetPeriod(LoRa_Sup.Timer, LoRa_Sup.TxCycle) ;
		UTIL_TIMER_Start(LoRa_Sup.Timer) ;
	}
}

void LoRa_Support_OnJoin(int16_t status) {

	if ( !Timer_check() )
		return ;

	if ( status == LORAMAC_HANDLER_SUCCESS ) {

		LoRa_Sup.LR_Status = Joined ;
		LoRa_Sup.TxCycle = 10000 ;
		LoRa_Sup.TxIndex = 0 ;

	} else {
		MW_LOG(TS_OFF, VLEVEL_M, "\r\n###### JOIN Fail : %d  ######\r\n", ++LoRa_Sup.JOIN_Fail);
		if ( LoRa_Sup.JOIN_Fail >= JOIN_MAX ) {
			LoRa_Sup.JOIN_Fail = 0 ;
			TransmitionCycle(LoRa_Sup.Tx_DutyCycle * multiply_lora_Send) ;
			LoRa_Sup.LR_Status = Join_Fail ;
			LoRa_Sup.OP_Status = Sleep_Mode;
		} else {
			TransmitionCycle(Join_Cycle) ;
			LoRa_Sup.LR_Status = Join_Requests ;
			LoRa_Sup.OP_Status = OnJoin;
		}

		Link_LoRa_buff();
		LogTime_Next_Send() ;
	}

	UTIL_TIMER_Stop(LoRa_Sup.Timer) ;
	UTIL_TIMER_SetPeriod(LoRa_Sup.Timer, LoRa_Sup.TxCycle) ;
	UTIL_TIMER_Start(LoRa_Sup.Timer) ;
}

void LoRa_Support_OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params) {

	LoRa_Sup.RSSI = params->Rssi ;
	LoRa_Sup.LR_Status = ACK ;
//	Link_LoRa_buff();

}
uint32_t LoRa_Support_OnTxData(LmHandlerTxParams_t *params, UTIL_TIMER_Object_t *Timer_) {

	LoRa_Sup.Timer = Timer_ ;

	if ( *LoRa_Sup.buff_FCnts >= UINT32_MAX - 10000 ) {
		*LoRa_Sup.buff_FCnts = 0 ;
		if ( ActivationType == ACTIVATION_TYPE_ABP )
			LoRa_Sup.Uplink_Save(*LoRa_Sup.buff_FCnts) ;
		else
			NVIC_SystemReset() ;
	}

	if ( LoRa_Sup.First_join == 0xA1 && params->IsMcpsConfirm ) {
		LoRa_Sup.LR_Status = LoRa_Send ;
		///Join Pass
		if ( !params->AckReceived ) {
			///Confirm Fail
			MW_LOG(TS_OFF, VLEVEL_M, "\r\n###### ACK Fail : %d  ######\r\n", ++LoRa_Sup.ACK_Fail);
			LoRa_Sup.RSSI = 0 ;

			LoRa_Sup.OP_Status = NACK ;

			if ( LoRa_Sup.ACK_Fail >= ACK_MAX ) {
				LoRa_Sup.ACK_Fail = 0 ;

				LoRa_Sup.LR_Status = Joined ;
				if ( ActivationType == ACTIVATION_TYPE_ABP ) {
					*LoRa_Sup.buff_FCnts = LoRa_Sup.UplinkCounter ;
					MW_LOG(TS_OFF, VLEVEL_M, "\r\n###### ABP ACK Fail : Sleep  ######\r\n");
					LoRa_Sup.TxIndex = 0 ;
					TransmitionCycle(LoRa_Sup.Tx_DutyCycle * multiply_lora_Send) ;
				} else {
					Link_LoRa_buff();
					MW_LOG(TS_OFF, VLEVEL_M, "\r\n###### OTAA ACK Fail : REBOOT  ######\r\n");
					HAL_Delay(1000);
					NVIC_SystemReset() ;
				}
			} else {

				TransmitionCycle(Join_Cycle) ;
			}
		} else {
			///Confirm Pass
			LoRa_Sup.ACK_Fail = 0 ;
			MW_LOG(TS_OFF, VLEVEL_M, "\r\n###### ACK Confirm : Pass  ######\r\n");
			LoRa_Sup.OP_Status = ACK ;

			///TODO TEST List Transmition
			if ( ++LoRa_Sup.TxIndex < LoRa_Sup.TxList ) {
				LoRa_Sup.TxCycle = 10000 ;
			} else {
				LoRa_Sup.OP_Status = Sleep_Mode ;
				LoRa_Sup.LR_Status = Joined ;
				LoRa_Sup.TxIndex = 0 ;
				TransmitionCycle(LoRa_Sup.Tx_DutyCycle * multiply_lora_Send) ;
			}

			if ( ActivationType == ACTIVATION_TYPE_ABP ) {
				LoRa_Sup.UplinkCounter = *LoRa_Sup.buff_FCnts ;
				LoRa_Sup.Uplink_Save(*LoRa_Sup.buff_FCnts) ;
			}

		}

		Link_LoRa_buff();
		LogTime_Next_Send() ;

		UTIL_TIMER_Stop(LoRa_Sup.Timer) ;
		UTIL_TIMER_SetPeriod(LoRa_Sup.Timer, LoRa_Sup.TxCycle) ;
		UTIL_TIMER_Start(LoRa_Sup.Timer) ;
	} else if ( params->IsMcpsConfirm ) {

		if ( ActivationType == ACTIVATION_TYPE_ABP )
			params->UplinkCounter = LoRa_Sup.UplinkCounter ;

		LoRa_Sup.First_join = 0xA1 ;
		MW_LOG(TS_OFF, VLEVEL_M, "\r\n###### First Join  ######\r\n");
		LogTime_Next_Send() ;
	} else {
		LoRa_Sup.First_join = 0x00 ;
	}

	return LoRa_Sup.TxCycle ;
}

void LoRaSupport_Add_4Byte(uint8_t channel, uint8_t Type, int32_t val, uint8_t *pPayLoad) {

	/* Data ID : 1 byte + Data Type : 1 byte + Data Size : 4 */
	if ( ((LoRa_Sup.CursorPayLoad) + 6) > PayLoadSize )
		return ;

	*(pPayLoad + (LoRa_Sup.CursorPayLoad++)) = channel ;
	*(pPayLoad + (LoRa_Sup.CursorPayLoad++)) = Type ;
	*(pPayLoad + (LoRa_Sup.CursorPayLoad++)) = (val >> 24) & 0xFF ;
	*(pPayLoad + (LoRa_Sup.CursorPayLoad++)) = (val >> 16) & 0xFF ;
	*(pPayLoad + (LoRa_Sup.CursorPayLoad++)) = (val >> 8) & 0xFF ;
	*(pPayLoad + (LoRa_Sup.CursorPayLoad++)) = val & 0xFF ;

}
void LoRaSupport_Add_3Byte(uint8_t channel, uint8_t Type, int32_t val, uint8_t *pPayLoad) {

	/* Data ID : 1 byte + Data Type : 1 byte + Data Size : 3 */
	if ( ((LoRa_Sup.CursorPayLoad) + 5) > PayLoadSize )
		return ;

	*(pPayLoad + (LoRa_Sup.CursorPayLoad++)) = channel ;
	*(pPayLoad + (LoRa_Sup.CursorPayLoad++)) = Type ;
	*(pPayLoad + (LoRa_Sup.CursorPayLoad++)) = (val >> 16) & 0xFF ;
	*(pPayLoad + (LoRa_Sup.CursorPayLoad++)) = (val >> 8) & 0xFF ;
	*(pPayLoad + (LoRa_Sup.CursorPayLoad++)) = val & 0xFF ;

}
void LoRaSupport_Add_2Byte(uint8_t channel, uint8_t Type, int32_t val, uint8_t *pPayLoad) {

	/* Data ID : 1 byte + Data Type : 1 byte + Data Size : 2 */
	if ( ((LoRa_Sup.CursorPayLoad) + 4) > PayLoadSize )
		return ;

	*(pPayLoad + (LoRa_Sup.CursorPayLoad++)) = channel ;
	*(pPayLoad + (LoRa_Sup.CursorPayLoad++)) = Type ;
	*(pPayLoad + (LoRa_Sup.CursorPayLoad++)) = (val >> 8) & 0xFF ;
	*(pPayLoad + (LoRa_Sup.CursorPayLoad++)) = val & 0xFF ;

}

void LoRaSupport_Add_1Byte(uint8_t channel, uint8_t Type, int32_t val, uint8_t *pPayLoad) {

	/* Data ID : 1 byte + Data Type + Data Size : 1 byte */
	if ( ((LoRa_Sup.CursorPayLoad) + 3) > PayLoadSize )
		return ;

	*(pPayLoad + (LoRa_Sup.CursorPayLoad++)) = channel ;
	*(pPayLoad + (LoRa_Sup.CursorPayLoad++)) = Type ;
	*(pPayLoad + (LoRa_Sup.CursorPayLoad++)) = val & 0xFF ;

}

/*******************************/
void LoRaSupport_Add_Param_4Byte(uint8_t channel, int32_t val, uint8_t *pPayLoad) {

	/* Data ID : 1 byte + Data Type : 1 byte + Data Size : 4 */
	if ( ((LoRa_Sup.CursorPayLoad) + 6) > PayLoadSize )
		return ;

	*(pPayLoad + (LoRa_Sup.CursorPayLoad++)) = channel ;
	*(pPayLoad + (LoRa_Sup.CursorPayLoad++)) = (val >> 24) & 0xFF ;
	*(pPayLoad + (LoRa_Sup.CursorPayLoad++)) = (val >> 16) & 0xFF ;
	*(pPayLoad + (LoRa_Sup.CursorPayLoad++)) = (val >> 8) & 0xFF ;
	*(pPayLoad + (LoRa_Sup.CursorPayLoad++)) = val & 0xFF ;

}
void LoRaSupport_Add_Param_3Byte(uint8_t channel, int32_t val, uint8_t *pPayLoad) {

	/* Data ID : 1 byte + Data Type : 1 byte + Data Size : 3 */
	if ( ((LoRa_Sup.CursorPayLoad) + 5) > PayLoadSize )
		return ;

	*(pPayLoad + (LoRa_Sup.CursorPayLoad++)) = channel ;
	*(pPayLoad + (LoRa_Sup.CursorPayLoad++)) = (val >> 16) & 0xFF ;
	*(pPayLoad + (LoRa_Sup.CursorPayLoad++)) = (val >> 8) & 0xFF ;
	*(pPayLoad + (LoRa_Sup.CursorPayLoad++)) = val & 0xFF ;

}
void LoRaSupport_Add_Param_2Byte(uint8_t channel, int32_t val, uint8_t *pPayLoad) {

	/* Data ID : 1 byte + Data Type : 1 byte + Data Size : 2 */
	if ( ((LoRa_Sup.CursorPayLoad) + 4) > PayLoadSize )
		return ;

	*(pPayLoad + (LoRa_Sup.CursorPayLoad++)) = channel ;
	*(pPayLoad + (LoRa_Sup.CursorPayLoad++)) = (val >> 8) & 0xFF ;
	*(pPayLoad + (LoRa_Sup.CursorPayLoad++)) = val & 0xFF ;

}

void LoRaSupport_Add_Param_1Byte(uint8_t channel, int32_t val, uint8_t *pPayLoad) {

	/* Data ID : 1 byte + Data Type + Data Size : 1 byte */
	if ( ((LoRa_Sup.CursorPayLoad) + 3) > PayLoadSize )
		return ;

	*(pPayLoad + (LoRa_Sup.CursorPayLoad++)) = channel ;
	*(pPayLoad + (LoRa_Sup.CursorPayLoad++)) = val & 0xFF ;

}

//#define Test_PayLoad

#ifdef Test_PayLoad
uint8_t PayLoad = 3 ;
void LoRaSupport_CayenBuffer(uint8_t* _CayenBuffData, uint8_t* _CayenBuffCursor, uint8_t _CayenBuffSize) {

	CayenBuffData = _CayenBuffData;
	CayenBuffSize = _CayenBuffSize;
	CayenBuffCursor = _CayenBuffCursor;

	for (uint8_t i = 0; i < PayLoad; ++i) {

		LoRaSupport_AddWaterMeterCount(i, i + 2100000000);
	}
}
#else

void SyncPayLoad(uint8_t Num, uint8_t *CayenBuffData, uint8_t *pPayLoad) {

	for ( uint8_t i = 1; i <= *pPayLoad; i++ ) {

		*(CayenBuffData + (i - 1)) = *(pPayLoad + i) ;
	}

}

void Link_LoRa_buff() {


	if ( NFC_Config.RSSI != LoRa_Sup.RSSI ) {
		NFC_Config.RSSI = LoRa_Sup.RSSI ;
		NFC_07A1_Memory_Write((uint8_t*) &NFC_Config.RSSI, EEP_NFC_Config_RSSI, 4) ;
	}

	if ( NFC_Config.LR_Status != LoRa_Sup.LR_Status ) {
		NFC_Config.LR_Status = LoRa_Sup.LR_Status ;
		NFC_07A1_Memory_Write((uint8_t*) &NFC_Config.LR_Status, EEP_NFC_Config_LR_Status, 4) ;
	}

	if ( NFC_Config.OP_Status != LoRa_Sup.OP_Status ) {
		NFC_Config.OP_Status = LoRa_Sup.OP_Status ;
		NFC_07A1_Memory_Write((uint8_t*) &NFC_Config.OP_Status, EEP_NFC_Config_OP_Status, 4) ;
	}

	static uint8_t buff_1[][20] = {"Join_Requests", "Join_Fail", "Joined", "LoRa_Send"};
	static uint8_t buff_2[][20] = {"OnJoin", "Sleep_Mode", "ACK", "NACK"};

	MW_LOG(TS_OFF, VLEVEL_M, "\r\n###### LoRa UplinkCounter : %u ######\r\n", *LoRa_Sup.buff_FCnts);
	MW_LOG(TS_OFF, VLEVEL_M, "\r\n###### TxIndex: %d  ######\r\n", (int16_t)LoRa_Sup.TxIndex);
	MW_LOG(TS_OFF, VLEVEL_M, "\r\n###### RSSI: %d  ######\r\n", (int16_t)NFC_Config.RSSI);
	MW_LOG(TS_OFF, VLEVEL_M, "\r\n###### LR_Status: %s  ######\r\n", buff_1[NFC_Config.LR_Status]);
	MW_LOG(TS_OFF, VLEVEL_M, "\r\n###### OP_Status: %s  ######\r\n", buff_2[NFC_Config.OP_Status]);


}

void LoRaSupport_CayenBuffer(uint8_t *_CayenBuffData, uint8_t *_CayenBuffCursor, uint8_t _CayenBuffSize) {

	LoRa_Sup.CursorPayLoad = 1 ;

	/** Parameter Configuration Begin**/
	LoRa_Sup.LoRa_Mode = NFC_Config.LoRa_Mode ;
	LoRa_Sup.Tx_DutyCycle = NFC_Config.LoRa_Sampling ;

	LoRa_Sup.TxList = 3 ;
	/** Add PayLoad **/
//	V23037_Read_Count() ;
	switch ( LoRa_Sup.TxIndex ) {
		default :
			LoRa_Sup.TxIndex = 0 ;
		case 0 :
//			LoRaSupport_Add_Param_4Byte(0,  (uint32_t) acc.output_rms_x, LoRa_Sup.List_PayLoad[LoRa_Sup.TxIndex]) ;
//			LoRaSupport_Add_Param_4Byte(1,  (uint32_t) acc.output_rms_z, LoRa_Sup.List_PayLoad[LoRa_Sup.TxIndex]) ;
		break ;
		case 1 :

			LoRaSupport_Add_Param_4Byte(2,  (uint32_t)0x00, LoRa_Sup.List_PayLoad[LoRa_Sup.TxIndex]) ;
			LoRaSupport_Add_Param_4Byte(3,  (uint32_t)0x00, LoRa_Sup.List_PayLoad[LoRa_Sup.TxIndex]) ;
		break ;
		case 2 :

			LoRaSupport_Add_Param_4Byte(4,  0x00, LoRa_Sup.List_PayLoad[LoRa_Sup.TxIndex]) ;
			LoRaSupport_Add_Param_4Byte(4,  0x00 , LoRa_Sup.List_PayLoad[LoRa_Sup.TxIndex]) ;
		break ;
	}
	/** Parameter Configuration End **/

	LoRa_Sup.LR_Status = LoRa_Sup.First_join == 0xA1 ? LoRa_Send : Join_Requests ;


	*_CayenBuffCursor = LoRa_Sup.List_PayLoad[LoRa_Sup.TxIndex][0] = LoRa_Sup.CursorPayLoad - 1 ;
	SyncPayLoad(LoRa_Sup.TxIndex, _CayenBuffData, LoRa_Sup.List_PayLoad[LoRa_Sup.TxIndex]) ;
//	*_CayenBuffCursor = LoRa_Sup.List_PayLoad[LoRa_Sup.TxIndex][0];
}
#endif

