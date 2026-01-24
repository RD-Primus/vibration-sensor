/*
 * NFC_07A1_Library.h
 *
 *  Created on: Jun 18, 2024
 *      Author: Nilniz
 */
#ifndef INC_NFC_07A1_INTERFACE_H_
#define INC_NFC_07A1_INTERFACE_H_

#define NFC_07A1_Enable		0


#define WS_03_L		1
#define KM_24_L		2

#define NFC_Model  									WS_03_L

#define NFC_Password_Masster_1  		1634
#define NFC_Password_Masster_2  		7005

#define NFC_Password_DEBUG  				65530
#define NFC_Password_Factory 				65535
#define NFC_Password_READ  					65534
#define NFC_Password_WRITE  				65533
#define NFC_Password_SEND 					16341
#define NFC_Password_Init						16342
#define NFC_Password_Reset_Alarm		16343
#define NFC_Password_Reset_FCnt			16344
#define NFC_Password_Test_Mode			16345

typedef enum{
	CMD_NFC_STB,
	CMD_NFC_Lock_Memory_1,
	CMD_NFC_UnLock_Memory_1,


	CMD_NFC_End = 65535,
}NFC_07A1_CMD;






#define NFC_Power_On_ms		5000
#define NFC_07A1_EH_Pin		V_EH_Pin

#define NFC_07A1_LPD_ON		HAL_GPIO_WritePin(LPD_GPIO_Port, LPD_Pin, RESET)
//#define NFC_07A1_LPD_OFF	HAL_GPIO_WritePin(LPD_GPIO_Port, LPD_Pin, SET)
#define NFC_07A1_LPD_OFF	HAL_GPIO_WritePin(LPD_GPIO_Port, LPD_Pin, RESET)

#define NFC_07A1_V_ON			HAL_GPIO_WritePin(NFC_V_GPIO_Port, NFC_V_Pin, SET)
#define NFC_07A1_V_OFF		HAL_GPIO_WritePin(NFC_V_GPIO_Port, NFC_V_Pin, RESET)

extern void NFC_07A1_OFF();
extern void NFC_07A1_ON();
extern void NFC_07A1_Run();
extern void NFC_07A1_Init();
extern void NFC_07A1_Default_Configurations() ;
extern void NFC_07A1_END_Zone_Check();
extern void NFC_07A1_CallBack_EXIT_in(uint32_t GPIO_Pin);
extern void NFC_07A1_Memory_Read(uint8_t * const pData, const uint16_t EEP_ADDR, const uint16_t Size);
extern void NFC_07A1_Memory_Write(uint8_t * const pData, const uint16_t EEP_ADDR, const uint16_t Size);
extern void NFC_07A1_Memory_Lock(const ST25DVxxKC_PROTECTION_ZONE_E Zone, const ST25DVxxKC_PROTECTION_CONF_E RW);
#endif /* INC_NFC_07A1_INTERFACE_H_ */
