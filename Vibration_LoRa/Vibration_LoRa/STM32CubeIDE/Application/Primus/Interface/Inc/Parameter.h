/*
 * Parameter.h
 *
 *  Created on: Jan 13, 2024
 *      Author: Nilniz
 */

#ifndef INC_INTERFACE_PARAMETER_H_
#define INC_INTERFACE_PARAMETER_H_

//TODO Firmware Version
#define Fw_Version_    		10003 //1.00.03
/* X.YY.ZZ
  X : New Design, MCU, IC
 YY : Fix Bug, ECN Update Productions
 ZZ : Fix Bug QA Test

#1.00.00
 - New Model
#1.00.01
 - Add LoRa Status, Operate Status, ResetFhCnt
 - Add ABP FCnt Management
 - OTAA Swap AppKey & NwkKey

#1.00.02
 - Fix Bug Carbon Footprint

#1.00.03
 - Fix High Power TX

*/
#define Init_Again				'G' //Charge for New Initial QA Bug
#define CAYENNE_LPP


#if !defined bool_defined
#define bool_defined
typedef enum {
	False = 0U, True
} bool_ ;
#endif

typedef union {
	int8_t Val ;
} i8_Type ;
typedef union {
	int16_t Val ;
	struct {
		int8_t L ;
		int8_t H ;
	} ;
} i16_Type ;

typedef union {
	int32_t Val ;
	struct {
		i16_Type L ;
		i16_Type H ;
	} ;
} i32_Type ;
typedef union {
	int64_t Val ;
	struct {
		i32_Type L ;
		i32_Type H ;
	} ;
} i64_Type ;

typedef union {
	uint8_t Val ;
} u8_Type ;
typedef union {
	uint16_t Val ;
	struct {
		uint8_t L ;
		uint8_t H ;
	} ;
} u16_Type ;

typedef union {
	uint32_t Val ;
	struct {
		u16_Type L ;
		u16_Type H ;
	} ;
} u32_Type ;

typedef union {
	uint64_t Val ;
	struct {
		u32_Type L ;
		u32_Type H ;
	} ;
} u64_Type ;

#define EEP_ADDR_Status				0x0064 //0100
#define EEP_ADDR_Init					0x00C8 //0200
#define EEP_ADDR_Test_Mode		0x012C //0300
#define EEP_ADDR_Config				0x03E4 //996
#define EEP_ADDR_CNT					0x07D0 //2000
#define EEP_ADDR_Password			0x1F40 //8000


typedef struct {
		uint32_t Init;// 4 Byte
		uint32_t Fw_Version;// 4 Byte
		uint32_t Password;// 4 Byte


		uint8_t joinEui[8]; // 8 Byte //Config
		uint8_t devEui[8];	// 8 Byte
		uint32_t devAddr;					// 4 Byte

		uint8_t APP_KEY_[16];							// 16 Byte//Config
		uint8_t NWK_KEY_[16];							// 16 Byte//Config
		uint8_t APP_S_KEY_[16];						// 16 Byte
		uint8_t NWK_S_KEY_[16];						// 16 Byte

		uint32_t UplinkCounter;
		uint16_t Struct_Size;
		uint8_t Firt_init;						// 16 Byte
//		uint8_t Firt_Write;						// 16 Byte

} NFC_Info_Data ;
extern NFC_Info_Data NFC_Info ;



typedef union Alarm_Status {
	uint8_t Value ;
	struct {
			bool_ Module_Remove 	:1 ;//0
			bool_ Reverse_Flow 	:1 ;//1
			bool_ Leakage 				:1 ;//2
			bool_ Low_Battery 		:1 ;//3
			bool_ B4 :1 ;//4
			bool_ B5 :1 ;//5
			bool_ B6 :1 ;//6
			bool_ Counter_OverFlow :1 ;//7
	} Bit ;

} Alarm_Status ;

typedef struct {

	uint8_t Update_Hour ;
	uint8_t Update_Minute ;
	uint8_t Update_Second ;
} NFC_Config_Buff ;
extern NFC_Config_Buff NFC_Buff ;

typedef struct {
	uint32_t	Model;
	uint8_t Name[16] ;
	int32_t CNT_Total ;
	uint32_t Battery_Level ;

	uint32_t CNT_Foward ;
	uint32_t CNT_Reverse ;
	int32_t RSSI ;
	uint32_t Count_Multiply ; // Liter
	uint32_t Count_Divisor ; //Pulse
	uint32_t LoRa_Sampling ;
	uint32_t LoRa_Mode ;
	uint32_t Leak_Value ;
	uint32_t Alarm_Status_ ;
	uint32_t Old_Password ;
	uint32_t New_Password ;
	uint32_t Update_Day ;
	uint32_t Update_Month ;
	uint32_t Update_Year ;
	uint32_t Update_Hour ;
	uint32_t Update_Minute ;
	uint32_t Update_Second ;

	uint8_t devEui[8];	// 8 Byte
	uint32_t devAddr;					// 4 Byte

	uint8_t joinEui[8]; // 8 Byte //Config
	uint8_t APP_KEY_[16];							// 16 Byte//Config
	uint8_t NWK_KEY_[16];							// 16 Byte//Config
	uint8_t APP_S_KEY_[16];						// 16 Byte
	uint8_t NWK_S_KEY_[16];						// 16 Byte
	uint32_t Flag_Count_Update ;

	uint32_t Fw_Version;// 4 Byte
	uint32_t Co2_Multiply;// 4 Byte
	int32_t Co2_CNT ;// 4 Byte

	int32_t LR_Status ;
	int32_t OP_Status ;
	uint32_t Struct_Size ;

} NFC_Config_Data ;
extern NFC_Config_Data NFC_Config ;

typedef struct {

	uint32_t Forward_Raw ;
	uint32_t Reverse_Raw ;
	uint32_t Forward_Grand ;
	uint32_t Reverse_Grand ;

	uint32_t Leak_Index;
	uint32_t Leak_Count[12];
	uint32_t CNT_Foward ;
	uint32_t CNT_Reverse ;
	uint32_t CNT_Last ;
	uint32_t Struct_Size;

} _V23037_CNT ;
extern _V23037_CNT V23037_CNT ;
typedef enum Save_Table{


	Table_NFC_Info,
	Table_V23037_CNT,
	Table_NFC_Config,
}Save_Table;

#define _64Bit_		8
#define _32Bit_		4
#define _16Bit_		2
#define _8Bit_		1
#define NEXT_ENUM(Param, NAME)   NAME,  NAME##_END = NAME - 1 + (sizeof(Param))
#define NEXT_ENUM_lent(Type, NAME)   NAME,  NAME##_END = NAME - 1 + ((Type))

typedef enum NFC_Config_EEP {

	NFC_Info_EEP = EEP_ADDR_Init - 1,
	NEXT_ENUM( NFC_Info. Init, 					EEP_NFC_Info_Init ),
	NEXT_ENUM( NFC_Info. Fw_Version, 		EEP_NFC_Info_Fw_Version ),
	NEXT_ENUM( NFC_Info. Password, 			EEP_NFC_Info_Password ),

	NEXT_ENUM( NFC_Info. joinEui, 			EEP_NFC_Info_joinEui ),
	NEXT_ENUM( NFC_Info. devEui, 				EEP_NFC_Info_devEui ),
	NEXT_ENUM( NFC_Info. devAddr, 			EEP_NFC_Info_devAddr ),

	NEXT_ENUM( NFC_Info. APP_KEY_, 			EEP_NFC_Info_APP_KEY_ ),
	NEXT_ENUM( NFC_Info. NWK_KEY_, 			EEP_NFC_Info_NWK_KEY_ ),
	NEXT_ENUM( NFC_Info. APP_S_KEY_, 		EEP_NFC_Info_APP_S_KEY_ ),
	NEXT_ENUM( NFC_Info. NWK_S_KEY_, 		EEP_NFC_Info_NWK_S_KEY_ ),

	NEXT_ENUM( NFC_Info. UplinkCounter, 	EEP_NFC_Info_UplinkCounter ),
	NEXT_ENUM( NFC_Info. Struct_Size, 	EEP_NFC_Info_Struct_Size ),
	NEXT_ENUM( NFC_Info. Firt_init, 		EEP_NFC_Info_Firt_init ),




	NFC_Config_EEP = EEP_ADDR_Config - 1,
	NEXT_ENUM( NFC_Config. Model, 						EEP_NFC_Config_Model ),
	NEXT_ENUM( NFC_Config. Name, 							EEP_NFC_Config_Name ),
	NEXT_ENUM( NFC_Config. CNT_Total, 				EEP_NFC_Config_CNT_Total ),
	NEXT_ENUM( NFC_Config. Battery_Level, 		EEP_NFC_Config_Battery_Level ),

	NEXT_ENUM( NFC_Config. CNT_Foward, 				EEP_NFC_Config_CNT_Foward ),
	NEXT_ENUM( NFC_Config. CNT_Reverse, 			EEP_NFC_Config_CNT_Reverse ),
	NEXT_ENUM( NFC_Config. RSSI, 							EEP_NFC_Config_RSSI ),
	NEXT_ENUM( NFC_Config. Count_Multiply, 		EEP_NFC_Config_Count_Multiply ), // Liter
	NEXT_ENUM( NFC_Config. Count_Divisor, 		EEP_NFC_Config_Count_Divisor ), //Pulse
	NEXT_ENUM( NFC_Config. LoRa_Sampling, 		EEP_NFC_Config_LoRa_Sampling ),
	NEXT_ENUM( NFC_Config. LoRa_Mode, 				EEP_NFC_Config_LoRa_Mode ),
	NEXT_ENUM( NFC_Config. Leak_Value, 				EEP_NFC_Config_Leak_Value ),
	NEXT_ENUM( NFC_Config. Alarm_Status_, 		EEP_NFC_Config_Alarm_Status_ ),
	NEXT_ENUM( NFC_Config. Old_Password, 			EEP_NFC_Config_Old_Password ),
	NEXT_ENUM( NFC_Config. New_Password, 			EEP_NFC_Config_New_Password ),
	NEXT_ENUM( NFC_Config. Update_Day, 				EEP_NFC_Config_Update_Day ),
	NEXT_ENUM( NFC_Config. Update_Month, 			EEP_NFC_Config_Update_Month ),
	NEXT_ENUM( NFC_Config. Update_Year, 			EEP_NFC_Config_Update_Year ),
	NEXT_ENUM( NFC_Config. Update_Hour, 			EEP_NFC_Config_Update_Hour ),
	NEXT_ENUM( NFC_Config. Update_Minute, 		EEP_NFC_Config_Update_Minute ),
	NEXT_ENUM( NFC_Config. Update_Second, 		EEP_NFC_Config_Update_Second ),

	NEXT_ENUM( NFC_Config. devEui, 						EEP_NFC_Config_devEui ),
	NEXT_ENUM( NFC_Config. devAddr, 					EEP_NFC_Config_devAddr ),

	NEXT_ENUM( NFC_Config. joinEui, 					EEP_NFC_Config_joinEui ),
	NEXT_ENUM( NFC_Config. APP_KEY_, 					EEP_NFC_Config_APP_KEY_ ),
	NEXT_ENUM( NFC_Config. NWK_KEY_, 					EEP_NFC_Config_NWK_KEY_ ),
	NEXT_ENUM( NFC_Config. APP_S_KEY_, 				EEP_NFC_Config_APP_S_KEY_ ),
	NEXT_ENUM( NFC_Config. NWK_S_KEY_, 				EEP_NFC_Config_NWK_S_KEY_ ),
	NEXT_ENUM( NFC_Config. Flag_Count_Update, EEP_NFC_Config_Flag_Count_Update ),

	NEXT_ENUM( NFC_Config. Fw_Version, 				EEP_NFC_Config_Fw_Version ),

	NEXT_ENUM( NFC_Config. Co2_Multiply, 			EEP_NFC_Config_Co2_Multiply ),
	NEXT_ENUM( NFC_Config. Co2_CNT, 					EEP_NFC_Config_Co2_CNT ),

	NEXT_ENUM( NFC_Config. LR_Status, 				EEP_NFC_Config_LR_Status ),
	NEXT_ENUM( NFC_Config. OP_Status, 				EEP_NFC_Config_OP_Status ),

	NEXT_ENUM( NFC_Config. Struct_Size, 			EEP_NFC_Config_Struct_Size ),



	EEP_Log_Check_
} Parameter_EEP ;




extern void Force_Reboot();
extern void Parameter_Init(void) ;
extern void Default_Parameter(void) ;
extern void Parameter_Save(Save_Table Table);
extern void Parameter_Read(Save_Table Table);
extern void UplinkCounter_Save(uint32_t value) ;

#endif /* INC_INTERFACE_PARAMETER_H_ */
