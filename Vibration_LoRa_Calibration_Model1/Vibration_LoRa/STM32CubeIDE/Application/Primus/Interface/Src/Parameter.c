/*
 * Parameter.c
 *
 *  Created on: Jan 13, 2024
 *      Author: Nilniz
 */
#include "main.h"
#if !defined   INC_INTERFACE_PARAMETER_H_
///TODO Add include Library to "main.h"
#include "Parameter.h"
#endif
NFC_Info_Data NFC_Info ;
NFC_Config_Data NFC_Config ;
_V23037_CNT V23037_CNT ;
NFC_Config_Buff NFC_Buff ;
void Force_Reboot() {

	MW_LOG(TS_OFF, VLEVEL_M, "\r\n###### MCU Reboot ######\r\n") ;
	HAL_Delay(500);
	NVIC_SystemReset();
}
void Set_LoRa_Key_TEST_MODE() {

//###### AppKey:      A1:A2:A3:A4:A5:A6:A7:A8:A9:B1:B2:B3:B4:B5:B6:B7
//###### NwkKey:      A1:A2:A3:A4:A5:A6:A7:A8:A9:B1:B2:B3:B4:B5:B6:B7
//###### AppSKey:     A1:A2:A3:A4:A5:A6:A7:A8:A9:B1:B2:B3:B4:B5:B6:B7
//###### NwkSKey:     A1:A2:A3:A4:A5:A6:A7:A8:A9:B1:B2:B3:B4:B5:B6:B7
//###### DevEUI:      11:22:33:44:55:66:77:88
//###### AppEUI:      01:01:01:01:01:01:01:01
//###### DevAddr:     55:66:77:88

	uint32_t devAddr = 1432778632;//

	uint8_t devEui[] = {0x1A, 0x2B, 0x3C, 0x4D, 0x5E, 0x6F, 0x71, 0x82};
	uint8_t joinEui[] = {0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01};
	uint8_t Key[] = {0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7};

	SecureElementSetDevEui(devEui);

	SecureElementSetDevAddr(ACTIVATION_TYPE_ABP, devAddr);
	SecureElementSetDevAddr(ACTIVATION_TYPE_OTAA, devAddr);

	SecureElementSetJoinEui(joinEui) ;
	SecureElementSetKey(APP_KEY, Key) ;
	SecureElementSetKey(NWK_KEY, Key) ;
	SecureElementSetKey(APP_S_KEY, Key) ;
	SecureElementSetKey(NWK_S_KEY, Key) ;

  MW_LOG( TS_OFF, VLEVEL_M, "\r\n###### Key Config By Test Mode Start ######\r\n\n");
	SecureElementPrintKeys();
  MW_LOG( TS_OFF, VLEVEL_M, "\r###### Key Config By Test Mode End ######\r\n\n");

}
void Set_LoRa_Key_Info() {

	SecureElementSetJoinEui(NFC_Info.joinEui) ;
	//TODO Key Bug
	SecureElementSetKey(APP_KEY, NFC_Info.NWK_KEY_) ;
	SecureElementSetKey(NWK_KEY, NFC_Info.APP_KEY_) ;

	SecureElementSetKey(APP_S_KEY, NFC_Info.APP_S_KEY_) ;
	SecureElementSetKey(NWK_S_KEY, NFC_Info.NWK_S_KEY_) ;
  MW_LOG( TS_OFF, VLEVEL_M, "\r\n###### Key Config By Primus Start ######\r\n\n");
	SecureElementPrintKeys();
  MW_LOG( TS_OFF, VLEVEL_M, "\r###### Key Config By Primus End ######\r\n\n");

}
void Get_LoRa_Key_Info() {

	///TODO Get LoRa Key Info

	/*Unique key*/
	SecureElementGetDevEui(NFC_Info.devEui) ;
	SecureElementGetDevAddr(ACTIVATION_TYPE_NONE, &NFC_Info.devAddr) ;


	uint8_t DefaultjoinEui[8] = {0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01};
	uint8_t Default_KEY_[16] = {0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
															0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C};

	memcpy(NFC_Info.joinEui, DefaultjoinEui, sizeof(DefaultjoinEui));
	memcpy(NFC_Info.APP_KEY_, Default_KEY_, sizeof(Default_KEY_));
	memcpy(NFC_Info.NWK_KEY_, Default_KEY_, sizeof(Default_KEY_));
	memcpy(NFC_Info.APP_S_KEY_, Default_KEY_, sizeof(Default_KEY_));
	memcpy(NFC_Info.NWK_S_KEY_, Default_KEY_, sizeof(Default_KEY_));


	NFC_Config.devAddr  = NFC_Info.devAddr;

	for(uint8_t i = 0; i < 8; i++){

		NFC_Config.devEui[i]  = NFC_Info.devEui[i];

		if(!NFC_Info.Firt_init)
			NFC_Config.joinEui[i]  = NFC_Info.joinEui[i];
	}

	if(!NFC_Info.Firt_init){
		for(uint8_t i = 0; i < 16; i++){

			NFC_Config. APP_KEY_[i] = NFC_Info.APP_KEY_[i];
			NFC_Config. NWK_KEY_[i] = NFC_Info.NWK_KEY_[i];

			NFC_Config. APP_KEY_[i] = NFC_Info.APP_KEY_[i];							// 16 Byte//Config
			NFC_Config. NWK_KEY_[i] = NFC_Info.NWK_KEY_[i];							// 16 Byte//Config
			NFC_Config. APP_S_KEY_[i] = NFC_Info.APP_S_KEY_[i];						// 16 Byte
			NFC_Config. NWK_S_KEY_[i] = NFC_Info.NWK_S_KEY_[i];						// 16 Byte

		}
	}
	NFC_Info.Firt_init = 1;
}


void Default_Parameter() {


	///Default NFC Informations
	Get_LoRa_Key_Info();

	NFC_Info. Struct_Size = sizeof(NFC_Info);
	NFC_Info.Init = Init_Again ;
	NFC_Info.Fw_Version = Fw_Version_;
	NFC_Info.Password = 1111;
	NFC_Info.UplinkCounter = 0;
	NFC_Info.Firt_init = 0;

	///Default Count Data
	V23037_CNT. Struct_Size = sizeof(V23037_CNT);
	V23037_CNT. Forward_Raw = 0 ;
	V23037_CNT. Reverse_Raw = 0 ;
	V23037_CNT. Forward_Grand = 0 ;
	V23037_CNT. Reverse_Grand = 0 ;
	V23037_CNT. Leak_Index = 0;

	for(uint8_t i = 0; i < 12; i++){
		V23037_CNT.Leak_Count[i]  = 0;
	}

	///Default Configurations Data
	NFC_Config. Struct_Size = sizeof(NFC_Config);
	NFC_Config. Model = NFC_Model;
	uint8_t i = 0;
	NFC_Config. Name[i++] = 'M';
	NFC_Config. Name[i++] = 'E';
	NFC_Config. Name[i++] = 'T';
	NFC_Config. Name[i++] = 'E';
	NFC_Config. Name[i++] = 'R';
	NFC_Config. Name[i++] = ' ';
	NFC_Config. Name[i++] = '-';
	NFC_Config. Name[i++] = ' ';
	NFC_Config. Name[i++] = 'N';
	NFC_Config. Name[i++] = 'A';
	NFC_Config. Name[i++] = 'M';
	NFC_Config. Name[i++] = 'E';
	NFC_Config. Name[i++] = '.';
	NFC_Config. Name[i++] = '.';
	NFC_Config. Name[i++] = '.';
	NFC_Config. Name[i++] = '?';


	NFC_Config. CNT_Total = 0 ;
	NFC_Config. CNT_Foward  = 0;
	NFC_Config. CNT_Reverse  = 0;
	NFC_Config. RSSI  = 0;
	NFC_Config. Count_Multiply  = 1;
	NFC_Config. Count_Divisor  = 1;
	NFC_Config. LoRa_Sampling  = 12;
	NFC_Config. LoRa_Mode  = 1;
	NFC_Config. Battery_Level  = 3;
	NFC_Config. Leak_Value  = 10;
	NFC_Config. Alarm_Status_  = 0;
	NFC_Config. Old_Password  = 0;
	NFC_Config. New_Password  = 0;
	NFC_Config. Update_Day  = 0;
	NFC_Config. Update_Month  = 0;
	NFC_Config. Update_Year  = 0;
	NFC_Config. Update_Hour  = 0;
	NFC_Config. Update_Minute  = 0;
	NFC_Config. Update_Second  = 0;

	NFC_Config. Fw_Version  = Fw_Version_;


	NFC_Config. Co2_Multiply = 1;// 4 Byte
	NFC_Config. Co2_CNT = 0;// 4 Byte


	NFC_Config. LR_Status = 0 ;
	NFC_Config. OP_Status = 0;
	Alarm_Status_Reset();
	Alarm_Status_Update();
	Parameter_Save(Table_NFC_Info);
	Parameter_Save(Table_NFC_Config);
	Parameter_Save(Table_V23037_CNT);

}

void UplinkCounter_Save(uint32_t value) {
	NFC_Info.UplinkCounter = value;
	NFC_07A1_Memory_Write((uint8_t*) &NFC_Info.UplinkCounter, EEP_NFC_Info_UplinkCounter, 4) ;
}

void Parameter_Init(void) {
	uint16_t Test_Mode = 0;
#if NFC_07A1_Enable
	Parameter_Read(Table_NFC_Info);

	NFC_07A1_Memory_Read((uint8_t*) &Test_Mode, EEP_ADDR_Test_Mode, 2) ;
	if(Test_Mode == 16345){
		NFC_Info.Init = 0;
		Test_Mode = 0;
		NFC_07A1_Memory_Write((uint8_t*) &Test_Mode, EEP_ADDR_Test_Mode, 2) ;
	}

	if(NFC_Info.Init != Init_Again || NFC_Info.Fw_Version != Fw_Version_|| NFC_Info.Struct_Size != sizeof(NFC_Info) ){
		NFC_Info.Firt_init = 0;
		///Configurations
		NFC_07A1_Default_Configurations();
		Default_Parameter();
	}else{

		NFC_07A1_END_Zone_Check();
		Parameter_Read(Table_NFC_Config);
		Parameter_Read(Table_V23037_CNT);
		_Alarm_.Value = NFC_Config.Alarm_Status_;


		///Check New Password
		if ( NFC_Info.Password == NFC_Config.Old_Password && NFC_Config.Old_Password != NFC_Config.New_Password ) {
			NFC_Info.Password = NFC_Config.New_Password ;
			Parameter_Save(Table_NFC_Info);

			MW_LOG(TS_OFF, VLEVEL_M, "\r\n###### New Password : %d ######\r\n", NFC_Info.Password) ;
		}
		if(NFC_Config.Old_Password + NFC_Config.New_Password != 0){

			NFC_Config.Old_Password = NFC_Config.New_Password = 0;
			NFC_07A1_Memory_Write((uint8_t*) &NFC_Config.Old_Password, EEP_NFC_Config_Old_Password, 4) ;
			NFC_07A1_Memory_Write((uint8_t*) &NFC_Config.New_Password, EEP_NFC_Config_New_Password, 4) ;
		}

		NFC_Buff.Update_Hour = NFC_Config.Update_Hour;
		NFC_Buff.Update_Minute = NFC_Config.Update_Minute;
		NFC_Buff.Update_Second = NFC_Config.Update_Second;


		NFC_07A1_Memory_Write((uint8_t*) &NFC_Config.New_Password, EEP_ADDR_Status, 4) ;
		NFC_07A1_Memory_Write((uint8_t*) &NFC_Config.New_Password, EEP_ADDR_Password, 4) ;
		NFC_07A1_Memory_Lock(ST25DVXXKC_PROT_ZONE2, ST25DVXXKC_WRITE_PROT) ; //Lock Zone 1

//		MW_LOG(TS_OFF, VLEVEL_M, "\r\n###### Password : %d ######\r\n", NFC_Info.Password) ;
	}
#else
	LoRa_Sup.LoRa_Mode = 1;
#endif


	///LoRa Configurations

#if !RUN_TEST
	if(Test_Mode == 1634){
		Test_Mode = 16345;
		NFC_07A1_Memory_Write((uint8_t*) &Test_Mode, EEP_ADDR_Test_Mode, 2) ;

		NFC_Config.LoRa_Sampling = 0;
		LoRa_Sup.operate_mode = LoRa_Test;

		Set_LoRa_Key_TEST_MODE();
	}else{
		LoRa_Sup.operate_mode = LoRa_Run;
		Set_LoRa_Key_Info();
	}
#else
	LoRa_Sup.operate_mode = LoRa_Run;
	Set_LoRa_Key_Info();
#endif
}

void Parameter_Save(Save_Table Table) {

	switch (Table) {
		case Table_NFC_Info:

			NFC_07A1_Memory_Write((uint8_t*) &NFC_Info, EEP_ADDR_Init, sizeof(NFC_Info)) ;
			break;
		case Table_V23037_CNT:

			NFC_07A1_Memory_Write((uint8_t*) &V23037_CNT, EEP_ADDR_CNT, sizeof(V23037_CNT)) ;
			break;
		case Table_NFC_Config:

			NFC_07A1_Memory_Write((uint8_t*) &NFC_Config, EEP_ADDR_Config, sizeof(NFC_Config)) ;
			break;
		default:
			break;
	}
}

void Parameter_Read(Save_Table Table) {

	switch (Table) {
		case Table_NFC_Info:

			NFC_07A1_Memory_Read((uint8_t*) &NFC_Info, EEP_ADDR_Init, sizeof(NFC_Info)) ;
			break;
		case Table_V23037_CNT:

			NFC_07A1_Memory_Read((uint8_t*) &V23037_CNT, EEP_ADDR_CNT, sizeof(V23037_CNT)) ;
			break;
		case Table_NFC_Config:

			NFC_07A1_Memory_Read((uint8_t*) &NFC_Config, EEP_ADDR_Config, sizeof(NFC_Config)) ;
			break;
		default:
			break;
	}
}







