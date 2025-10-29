/*
 * Alarm_interface.c
 *
 *  Created on: Oct 8, 2024
 *      Author: Nilniz
 */
#include "main.h"


Alarm_Status _Alarm_;

void Alarm_Status_Battery(){
	uint8_t Battery_status;
	uint16_t Battery = SYS_GetBatteryLevel();
	if(Battery <= 2800){	///Low
		_Alarm_.Bit.Low_Battery = true;
		Battery_status = 0;
	}else if(Battery <= 3240){	///Mid
		Battery_status = 1;

	}else	{	///Full
		Battery_status = 2;

	}

	if(NFC_Config.Battery_Level != Battery_status){
		MW_LOG(TS_OFF, VLEVEL_M, "\r\n###### Power : %d mV ######\r\n", Battery) ;
		NFC_Config.Battery_Level = Battery_status;
		NFC_07A1_Memory_Write((uint8_t*) &NFC_Config.Battery_Level, EEP_NFC_Config_Battery_Level, 4) ;
	}
}

void Alarm_Status_Reset(){

//	V23037_Leak_Reset();
	NFC_Config.Alarm_Status_ = _Alarm_.Value = 0;
	NFC_Config.Battery_Level = 2;
	NFC_07A1_Memory_Write((uint8_t*) &NFC_Config.Alarm_Status_, EEP_NFC_Config_Alarm_Status_, 4) ;
	NFC_07A1_Memory_Write((uint8_t*) &NFC_Config.Battery_Level, EEP_NFC_Config_Battery_Level, 4) ;
}

void Alarm_Status_Update(){

	Alarm_Status_Battery();

	if(NFC_Config.Alarm_Status_ != _Alarm_.Value){
		NFC_Config.Alarm_Status_ = _Alarm_.Value;
		NFC_07A1_Memory_Write((uint8_t*) &NFC_Config.Alarm_Status_, EEP_NFC_Config_Alarm_Status_, 4) ;
		MW_LOG(TS_OFF, VLEVEL_M, "Reverse_Flow : %s \r\n", _Alarm_.Bit.Reverse_Flow? "True" : "False") ;
		MW_LOG(TS_OFF, VLEVEL_M, "Leakage : %s \r\n", _Alarm_.Bit.Leakage? "True" : "False") ;
		MW_LOG(TS_OFF, VLEVEL_M, "Low_Battery : %s \r\n", _Alarm_.Bit.Low_Battery? "True" : "False") ;
		MW_LOG(TS_OFF, VLEVEL_M, "Counter_OverFlow : %s \r\n", _Alarm_.Bit.Counter_OverFlow? "True" : "False") ;
		LoRa_Support_Force_Send ();
	}


//	MW_LOG(TS_OFF, VLEVEL_M, "Module_Remove : %s \r\n", _Alarm_.Bit.Module_Remove? "True" : "False") ;

}

