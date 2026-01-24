/*
 * NFC_07A1_Interface.c
 *
 *  Created on: Jun 18, 2024
 *      Author: Nilniz
 */
#include "main.h"

#if !defined   INC_NFC_07A1_INTERFACE_H_
///TODO Add include Library to "main.h"
#include "NFC_07A1_Interface.h"
#endif

//DEBUGGER_ENABLED
uint32_t NFCTAG_INSTANCE = 0 ;
const ST25DVxxKC_PASSWD_t st25dv_i2c_password = { .MsbPasswd = 0, .LsbPasswd = 0 } ;
const ST25DVxxKC_PASSWD_t st25dv_i2c_password_Lock = { .MsbPasswd = 1, .LsbPasswd = 1 } ;
ST25DVxxKC_RF_PROT_ZONE_t RfProtZone ;
uint8_t Zone_Size_1 = 5 ;
uint8_t Zone_Size_2 = 245 ;
uint8_t Zone_Size_C1 = 0 ;
uint8_t Zone_Size_C2 = 0 ;
uint16_t NFC_Pin = 33559 ;
uint16_t NFC_Pass_True ;
FunctionalState NFC_Power_Status ;
NFC_07A1_CMD for_test_cmd ;

uint8_t APP_ID_Link[] = {
		0xE1, 0x40, 0x40, 0x01,
		0x03, 0x2A, 0xD4, 0x0F,
		0x18, 0x61, 0x6E, 0x64,
		0x72, 0x6F, 0x69, 0x64,
		0x2E, 0x63, 0x6F, 0x6D,
		0x3A, 0x70, 0x6B, 0x67,
		0x63, 0x6F, 0x6D, 0x2E,
		0x70, 0x72, 0x69, 0x6D,
		0x75, 0x73, 0x74, 0x68,
		0x61, 0x69, 0x2E, 0x70,
		0x6D, 0x5F, 0x6D, 0x65,
		0x74, 0x65, 0x72, 0x73
} ;

void NFC_07A1_Delay(uint16_t delay) {

	for ( uint16_t var = 0; var < delay; ++var ) {
		HAL_GetTick() ;
	}

}
void NFC_07A1_ON() {

//	if(NFC_Power_OFF.IsRunning)
	UTIL_TIMER_Stop(&NFC_Power_OFF) ;
	UTIL_TIMER_Start(&NFC_Power_OFF) ;

	if ( NFC_Power_Status )
		return ;
	NFC_07A1_V_ON ;
	NFC_07A1_LPD_ON ;
	NFC_Power_Status = ENABLE ;
	NFC_07A1_Delay(4000) ;
}
void NFC_07A1_OFF() {
	if ( !NFC_Power_Status )
		return ;

	UTIL_TIMER_Stop(&NFC_Power_OFF) ;
	NFC_07A1_V_OFF ;
	NFC_07A1_LPD_OFF ;
	NFC_Power_Status = DISABLE ;
}

void NFC_07A1_Memory_Read(uint8_t *const pData, const uint16_t EEP_ADDR, const uint16_t Size) {
#if NFC_07A1_Enable
	NFC_07A1_ON() ;
	CUSTOM_NFCTAG_ReadData(NFCTAG_INSTANCE, (uint8_t*) pData, EEP_ADDR, Size) ;
//	NFC_07A1_OFF();
#endif
}

void NFC_07A1_Memory_Write(uint8_t *const pData, const uint16_t EEP_ADDR, const uint16_t Size) {
#if NFC_07A1_Enable
	NFC_07A1_ON() ;
	CUSTOM_NFCTAG_WriteData(NFCTAG_INSTANCE, (uint8_t*) pData, EEP_ADDR, Size) ;
//	NFC_07A1_OFF();
#endif
}

void NFC_07A1_Memory_Lock(const ST25DVxxKC_PROTECTION_ZONE_E Zone, const ST25DVxxKC_PROTECTION_CONF_E RW) {
#if NFC_07A1_Enable
	NFC_07A1_ON() ;
	/* Present configuration password Unlock*/
	CUSTOM_NFCTAG_PresentI2CPassword(NFCTAG_INSTANCE, st25dv_i2c_password) ;

/// Protection Memory
	RfProtZone.PasswdCtrl = ST25DVXXKC_NOT_PROTECTED ;
	RfProtZone.RWprotection = RW ;
	CUSTOM_NFCTAG_WriteRFZxSS(NFCTAG_INSTANCE, Zone, RfProtZone) ;

	/* Present configuration password lock*/
	CUSTOM_NFCTAG_PresentI2CPassword(NFCTAG_INSTANCE, st25dv_i2c_password_Lock) ;
//	NFC_07A1_OFF();
#endif
}

void NFC_07A1_END_Zone_Check() {

#if NFC_07A1_Enable
	NFC_07A1_ON() ;
	CUSTOM_NFCTAG_ReadEndZonex(NFCTAG_INSTANCE, ST25DVXXKC_ZONE_END1, &Zone_Size_C1) ;
	CUSTOM_NFCTAG_ReadEndZonex(NFCTAG_INSTANCE, ST25DVXXKC_ZONE_END2, &Zone_Size_C2) ;
	if ( Zone_Size_1 != Zone_Size_C1 || Zone_Size_2 != Zone_Size_C2 ) {
		NFC_07A1_Default_Configurations() ;
	}

#endif
}
void NFC_07A1_Default_Configurations() {
#if NFC_07A1_Enable
	NFC_07A1_ON() ;
	/* Present configuration password Unlock*/
	CUSTOM_NFCTAG_PresentI2CPassword(NFCTAG_INSTANCE, st25dv_i2c_password) ;

	/* Set GPO Configuration */
///GPO Interrupt on RF Write Done
	CUSTOM_NFCTAG_ConfigIT(NFCTAG_INSTANCE, ST25DVXXKC_GPO1_ENABLE_MASK | ST25DVXXKC_GPO1_RFWRITE_MASK) ;

	/* Set EH_V Configuration */
	CUSTOM_NFCTAG_WriteEHMode(NFCTAG_INSTANCE, ST25DVXXKC_EH_ACTIVE_AFTER_BOOT) ;

	/* Set RF Lock Register Configuration */
	CUSTOM_NFCTAG_WriteLockCFG(NFCTAG_INSTANCE, ST25DVXXKC_LOCKED) ;

	/* Set Size Memory Zone */
///Clear Zone
	CUSTOM_NFCTAG_WriteEndZonex(NFCTAG_INSTANCE, ST25DVXXKC_ZONE_END3, 255) ;
	CUSTOM_NFCTAG_WriteEndZonex(NFCTAG_INSTANCE, ST25DVXXKC_ZONE_END2, 255) ;
	CUSTOM_NFCTAG_WriteEndZonex(NFCTAG_INSTANCE, ST25DVXXKC_ZONE_END1, 255) ;

///Zone 1 for Initial Data
	RfProtZone.RWprotection = ST25DVXXKC_NO_PROT ;
	RfProtZone.RWprotection = ST25DVXXKC_READWRITE_PROT ;
	CUSTOM_NFCTAG_WriteEndZonex(NFCTAG_INSTANCE, ST25DVXXKC_ZONE_END1, Zone_Size_1) ;
	CUSTOM_NFCTAG_WriteRFZxSS(NFCTAG_INSTANCE, ST25DVXXKC_PROT_ZONE1, RfProtZone) ;

///Zone 2 for Product Data
	RfProtZone.RWprotection = ST25DVXXKC_NO_PROT ;
	RfProtZone.RWprotection = ST25DVXXKC_WRITE_PROT ;
	CUSTOM_NFCTAG_WriteEndZonex(NFCTAG_INSTANCE, ST25DVXXKC_ZONE_END2, Zone_Size_2) ;
	CUSTOM_NFCTAG_WriteRFZxSS(NFCTAG_INSTANCE, ST25DVXXKC_PROT_ZONE2, RfProtZone) ;

///Zone 3 for Password Data
	RfProtZone.PasswdCtrl = ST25DVXXKC_NOT_PROTECTED ;
	RfProtZone.RWprotection = ST25DVXXKC_NO_PROT ;
	CUSTOM_NFCTAG_WriteEndZonex(NFCTAG_INSTANCE, ST25DVXXKC_ZONE_END3, 0xFF) ;
	CUSTOM_NFCTAG_WriteRFZxSS(NFCTAG_INSTANCE, ST25DVXXKC_PROT_ZONE3, RfProtZone) ;

	/* Set Link ID Application */
	uint8_t clear = 0x00 ;
	for ( uint8_t i = 0; i < 64; ++i ) {

		NFC_07A1_Memory_Write(&clear, i, 1) ;
	}
	NFC_07A1_Memory_Write(APP_ID_Link, 0x00, sizeof(APP_ID_Link)) ;

	/* Present configuration password lock*/
	CUSTOM_NFCTAG_PresentI2CPassword(NFCTAG_INSTANCE, st25dv_i2c_password_Lock) ;

#endif
}

void NFC_07A1_Init() {

#if NFC_07A1_Enable
	NFC_07A1_ON() ;

	while ( CUSTOM_NFCTAG_Init(NFCTAG_INSTANCE) != NFCTAG_OK ) ;

	CUSTOM_GPO_Init() ;

#endif
}

void NFC_07A1_CMD_Run(NFC_07A1_CMD *CMD) {
#if NFC_07A1_Enable
	if ( *CMD == CMD_NFC_STB )
		return ;
	NFC_07A1_ON() ;
	/* Present configuration password Unlock*/
	CUSTOM_NFCTAG_PresentI2CPassword(NFCTAG_INSTANCE, st25dv_i2c_password) ;

	switch ( (uint16_t) *CMD ) {
		case CMD_NFC_Lock_Memory_1 :

			NFC_07A1_Memory_Lock(ST25DVXXKC_PROT_ZONE2, ST25DVXXKC_WRITE_PROT) ; //Lock Zone 1
		break ;
		case CMD_NFC_UnLock_Memory_1 :

			NFC_07A1_Memory_Lock(ST25DVXXKC_PROT_ZONE2, ST25DVXXKC_NO_PROT) ; //Unlock Zone 1
		break ;

	}

	/* Present configuration password lock*/
	CUSTOM_NFCTAG_PresentI2CPassword(NFCTAG_INSTANCE, st25dv_i2c_password_Lock) ;
	*CMD = CMD_NFC_STB ;

#endif
}

void NFC_07A1_Run() {
	NFC_07A1_CMD_Run(&for_test_cmd) ;
}

void NFC_07A1_CountConfig() {

///Config Count
	if ( NFC_Config.Flag_Count_Update != 0 ) {
		MW_LOG(TS_OFF, VLEVEL_M, "\r\n###### NFC_ Write CNT SAVE ######\r\n")
		;
//		V23037_Reset_Count(NFC_Config.CNT_Foward, NFC_Config.CNT_Reverse) ;
		NFC_Config.Flag_Count_Update = 0 ;
		NFC_07A1_Memory_Write((uint8_t*) &NFC_Config.Flag_Count_Update, 1176, 4) ;

	} else {

		MW_LOG(TS_OFF, VLEVEL_M, "\r\n###### NFC_ Write DATA ######\r\n")
		;
	}
}
void NFC_07A1_KeyConfig() {
	uint8_t update_info = 0 ;
///New LoRa Key

	for ( uint8_t i = 0; i < 8; i++ ) {
		if ( NFC_Info.joinEui[i] != NFC_Config.joinEui[i] ) {
			NFC_Info.joinEui[i] = NFC_Config.joinEui[i] ;
			update_info = 1 ;
		}
	}

	for ( uint8_t i = 0; i < 16; i++ ) {

		if ( NFC_Info.APP_KEY_[i] != NFC_Config.APP_KEY_[i] || NFC_Info.NWK_KEY_[i] != NFC_Config.NWK_KEY_[i]
				|| NFC_Info.APP_S_KEY_[i] != NFC_Config.APP_S_KEY_[i] || NFC_Info.NWK_S_KEY_[i] != NFC_Config.NWK_S_KEY_[i] ) {

			NFC_Info.APP_KEY_[i] = NFC_Config.APP_KEY_[i] ;
			NFC_Info.NWK_KEY_[i] = NFC_Config.NWK_KEY_[i] ;
			NFC_Info.APP_S_KEY_[i] = NFC_Config.APP_S_KEY_[i] ;
			NFC_Info.NWK_S_KEY_[i] = NFC_Config.NWK_S_KEY_[i] ;
			update_info = 1 ;
		}

	}

///TODO Reset MCU for init LoRa
	if ( update_info ) {
		Parameter_Save(Table_NFC_Info) ;
		MW_LOG(TS_OFF, VLEVEL_M, "\r\n###### NFC_ Info SAVE DATA ######\r\n")
		;

	}
}
uint8_t Firt_Write = 1 ;
void NFC_07A1_PasswordTrue() {

	MW_LOG(TS_OFF, VLEVEL_M, "\r\n###### Login : Pass ######\r\n")
	;
	if ( !NFC_Pass_True ) {
		NFC_Pass_True = NFC_Model ;
		NFC_07A1_Memory_Write((uint8_t*) &NFC_Pass_True, EEP_ADDR_Status, 2) ; //Set Status

		HAL_Delay(2000) ;
		NFC_Pass_True = 0 ;
		NFC_07A1_Memory_Write((uint8_t*) &NFC_Pass_True, EEP_ADDR_Status, 2) ; //Set Status
		Firt_Write = 1 ;
	}
}
void NFC_07A1_WRITE_DATA() {

	if ( Firt_Write == 1 ) {
		NFC_07A1_Memory_Lock(ST25DVXXKC_PROT_ZONE2, ST25DVXXKC_NO_PROT) ; //Unlock Zone 2

		MW_LOG(TS_OFF, VLEVEL_M, "\r\n###### NFC Unlock Zone 2 ######\r\n")
		;
		Firt_Write = 2 ;
	} else if ( Firt_Write == 2 ) {

		Parameter_Read(Table_NFC_Config) ;

		NFC_07A1_CountConfig() ;
		NFC_07A1_KeyConfig() ;

		///Reset Status Login
		if ( NFC_Buff.Update_Hour != NFC_Config.Update_Hour || NFC_Buff.Update_Minute != NFC_Config.Update_Minute
				|| NFC_Buff.Update_Second != NFC_Config.Update_Second ) {

			NFC_Buff.Update_Hour = NFC_Config.Update_Hour ;
			NFC_Buff.Update_Minute = NFC_Config.Update_Minute ;
			NFC_Buff.Update_Second = NFC_Config.Update_Second ;

			///Reset Alarm After Write Password
			Alarm_Status_Reset() ;

			NFC_Pin = 0 ;
			NFC_Pass_True = 0 ;
			NFC_07A1_Memory_Write((uint8_t*) &NFC_Pin, EEP_ADDR_Password, 2) ; //Clear NFC_Pin
			NFC_07A1_Memory_Write((uint8_t*) &NFC_Pass_True, EEP_ADDR_Status, 2) ; //Clear Status
			NFC_07A1_Memory_Lock(ST25DVXXKC_PROT_ZONE2, ST25DVXXKC_WRITE_PROT) ; //Lock Zone 2
			MW_LOG(TS_OFF, VLEVEL_M, "\r\n###### NFC Lock Zone 2 ######\r\n")
			;
			Firt_Write = 1 ;
		}

	}

}
void NFC_07A1_FACTORY_RESET() {
	MW_LOG(TS_OFF, VLEVEL_M, "\r\n###### NFC Factory Reset DATA ######\r\n")
	;
	NFC_Pass_True = 0 ;
	NFC_Pin = 0 ;
	NFC_07A1_Memory_Write((uint8_t*) &NFC_Pass_True, EEP_ADDR_Status, 2) ;
	NFC_07A1_Memory_Write((uint8_t*) &NFC_Pin, EEP_ADDR_Password, 2) ;
	NFC_07A1_Memory_Lock(ST25DVXXKC_PROT_ZONE2, ST25DVXXKC_WRITE_PROT) ; //Lock Zone 1
	NFC_07A1_Default_Configurations() ; //Reset
	Default_Parameter() ;

	UTIL_TIMER_Stop(&Reboot) ;
	UTIL_TIMER_Start(&Reboot) ;
}
void NFC_07A1_READ_DATA() {
	MW_LOG(TS_OFF, VLEVEL_M, "\r\n###### NFC_ Read DATA ######\r\n")
	;
	NFC_Pin = 0 ;
	NFC_07A1_Memory_Write((uint8_t*) &NFC_Pin, EEP_ADDR_Password, 2) ; //Clear NFC_Pin
//	V23037_Read_Count() ;
	Link_LoRa_buff();
}

void NFC_07A1_LoRa_SEND() {
	MW_LOG(TS_OFF, VLEVEL_M, "\r\n###### LORA SEND DATA ######\r\n");
	NFC_Pin = 0 ;
	NFC_07A1_Memory_Write((uint8_t*) &NFC_Pin, EEP_ADDR_Password, 2) ; //Clear NFC_Pin
//	V23037_Read_Count() ;
	LoRa_Support_Force_Send() ;
}

void NFC_07A1_Init_MCU() {
	NFC_Pin = 0 ;
	NFC_07A1_Memory_Write((uint8_t*) &NFC_Pin, EEP_ADDR_Password, 2) ; //Clear NFC_Pin

	MW_LOG(TS_OFF, VLEVEL_M, "\r\n###### SET MCU Init ######\r\n") ;

	UTIL_TIMER_Stop(&Reboot) ;
	UTIL_TIMER_Start(&Reboot) ;
}

void NFC_07A1_Password_Check() {
#if NFC_07A1_Enable

	NFC_07A1_Memory_Read((uint8_t*) &NFC_Pin, EEP_ADDR_Password, 2) ; //Read Password Check

//	MW_LOG(TS_OFF, VLEVEL_M, "\r\n###### NFC_Pin Check : %d ######\r\n", NFC_Pin);
	if ( NFC_Pin == NFC_Info.Password ) {
		NFC_07A1_PasswordTrue() ;
	} else {

		switch ( NFC_Pin ) {
			case NFC_Password_Masster_1 :
			case NFC_Password_Masster_2 :
				NFC_07A1_PasswordTrue() ;
			break ;

			case NFC_Password_WRITE :
				NFC_07A1_WRITE_DATA() ;
			break ;

			case NFC_Password_READ :
				NFC_07A1_READ_DATA() ;
			break ;

			case NFC_Password_SEND :
				NFC_07A1_LoRa_SEND() ;
			break ;

			case NFC_Password_Factory :
				NFC_07A1_FACTORY_RESET() ;
				///Start Timer RESET MCU
			break ;

			case NFC_Password_Init :
				NFC_07A1_Init_MCU() ;
				///Start Timer RESET MCU
			break ;

			case NFC_Password_Reset_Alarm :
				///Reset Alarm After Write Password
				Alarm_Status_Reset() ;
			break ;
			case NFC_Password_Reset_FCnt :
				Reset_fCnt_();
			break ;
			case NFC_Password_Test_Mode :

				uint16_t NFC_Test_Mode = 1634 ;
				NFC_07A1_Memory_Write((uint8_t*) &NFC_Test_Mode, EEP_ADDR_Test_Mode, 2) ;
			break ;
			default :

//				MW_LOG(TS_OFF, VLEVEL_M, "\r\n###### NFC_Pin : %d ######\r\n", NFC_Pin);
//				MW_LOG(TS_OFF, VLEVEL_M, "\r\n###### NFC_ Lock  ######\r\n");
				NFC_Pass_True = 0 ;
				NFC_Pin = 0 ;
				NFC_07A1_Memory_Write((uint8_t*) &NFC_Pass_True, EEP_ADDR_Status, 2) ;
				NFC_07A1_Memory_Write((uint8_t*) &NFC_Pin, EEP_ADDR_Password, 2) ;
				NFC_07A1_Memory_Lock(ST25DVXXKC_PROT_ZONE2, ST25DVXXKC_WRITE_PROT) ; //Lock Zone 1
			break ;
		}
	}

#endif
}

void BSP_GPO_Callback() {
#if NFC_07A1_Enable

	NFC_07A1_ON() ;
	NFC_07A1_Password_Check() ;
//	NFC_07A1_OFF();

#endif
}

void NFC_07A1_CallBack_EXIT_in(uint32_t GPIO_Pin) {


#if NFC_07A1_Enable

	if ( !(GPIO_Pin & (NFC_07A1_EH_Pin)) || NFC_Power_Status )
		return ;
	NFC_07A1_ON() ;
#endif

}
