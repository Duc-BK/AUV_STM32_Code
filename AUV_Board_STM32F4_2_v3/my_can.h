/*
 * User_CAN.h
 *
 *  Created on: Apr 9, 2018
 *      Author: Chau Thanh Hai
 */
 #ifndef MY_CAN_H_
#define MY_CAN_H_

#include "misc.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include <stdbool.h>
#include "my_mx28.h"
#include "my_bms24v40ah.h"
#include "my_keller_pa3.h"
#include "my_io.h"
#include "UGV_UART_JOYSTICK.h"
#include "UGV_SBUS.h"
#include "my_delay.h"
#include "my_adc.h"
#include "my_balance.h"
#include "my_uart_esp.h"

typedef enum
{
	JETSON = 0x00,
	JOYSTICK = 0x01,
	BALANCE = 0x02,
	DIVING = 0x03,
}Operation_Mode_Typedef;

extern uint8_t Operation_Mode;

typedef union
{
	float Value;
	
	struct
	{
		unsigned a1:8;
		unsigned a2:8;
		unsigned a3:8;
		unsigned a4:8;
	} byte;
}CAN_UpdateData;

extern CAN_UpdateData UPWM_DutyCycle, MX28_Data, KELLER_Pressure, KELLER_Temperature, Thruster_Speed, ALTIMETER_Range, XSEN_Pitch, MASS_Position, PISTOL_Position;

typedef struct
{
	MX28_Status_Typedef MX28_Status;
	UBMS40_Status_Typedef BMS40_Status;
	uint8_t Leak_Status;
	UKellerPA3_Status_Typedef KellerPA3_Status;
}CAN_DataTypeDef;

typedef enum
{
		//--------FUNCTION--------//
		WRITE_DATA = 0x00,
		READ_DATA = 0x01,
		STATUS_DATA = 0x02,
}Function_CANBUS_TypeDef;

typedef enum
{
		//--------ID BOARD ARM 2--------//
		ARM2_LEAK_SENSOR = 0x01,
		ARM2_STRAIN_GAUGE = 0x02,
		ARM2_BMS24V40AH = 0x03,
		ARM2_RUDDER = 0x04,
		ARM2_BUZZER = 0x05,
		ARM2_PRESSURE = 0x06,
		ARM2_ALL_DATA = 0xFF,
}ID_CANBUS_TypeDef;

typedef enum
{
		//--------REGISTER LEAK-SENSOR--------//
		LEAK_POSITION = 0xFF,
				//--------REGISTER STRAIN-GAUGE--------//
		STATUS_HULL = 0xFF,
		//--------BMS STATUS--------//
		HOURS = 0x01,
		MINUTES = 0x02,
		SECONDS = 0x03,
		BATTERY_TOTAL = 0x04,
		BATTERY_CAPACITY = 0x05,
		BATTERY_USED = 0x06,
		BATTERY_PERCENTAGE = 0x07,
		BATTERY_CURRENT = 0x08, 
		BATTERY_VOLTAGE = 0x09,
		//--------REGISTER RUDDER--------//
		MX28_GOAL_POSITION = 0x00,
		MX28_MOVING_SPEED = 0x01,
		MX28_KP = 0x02,
		MX28_KI = 0x03,
		MX28_KD = 0x04,
		MX28_BAUDRATE = 0x05,
		MX28_PRESENT_POSITION = 0x06,
		MX28_PRESENT_SPEED = 0x07,
		MX28_PRESENT_LOAD = 0x08,
		MX28_PRESENT_VOL = 0x09,
		MX28_PRESENT_TEMP = 0x0A,
		//--------REGISTER PRESSURE--------//
		DEPTH_DATA = 0xFE,
		TEMP_DATA = 0xFF,
		//--------POWER INT--------//
		BUZZER			= 0xFD,
		INT_24V40AH = 0xFE,
		INT_24V10AH = 0xFF,
}REGISTER_CANBUS_TypeDef;

struct Status_flag
{
	bool Emergency;
	bool Joystick_Enable;
	bool Joystick_Disable;
	bool Pre_Joystick_Enable;
	bool Pre_Joystick_Disable;
	bool Joystick_Is_On;
	bool Devo7_Off;
	bool Run_First_Time_After_Devo7_Off;
	bool Send_Data;
	bool End_Frame_Jetson;
	bool Leaking;
	bool Fuzzy_Update;
};
extern struct Status_flag Flag;

void UCAN_Configure(void);
void UCAN_Respond_ALLData(CAN_DataTypeDef *_can_data);
bool UCAN_NeedRespondData(void);
void UCAN_AllowRespondData(FunctionalState NewState);
void UCAN_AllowRespondCheckSystem(FunctionalState NewState);
void UCAN_Convert_Float_to_Bytes(float _data_in, uint8_t* _data_out);
void UCAN_SystemReady(void);
void UCAN_SystemNotReady(void);
bool UCAN_IsSystemReady(void);
bool UCAN_IsNeedCheckSystem(void);
void UCAN_PassSystemReady(FunctionalState NewState);
void UCAN_Transmit(int _IDstd,int _length, uint8_t _data[]);
void UCAN_SystemReady_2(void);
void UCAN_Open_Thruster(void);
void UCAN_Close_Thruster(void);
void UCAN_Open_Pistol(void);
void UCAN_Open_Mass(void);
void UCAN_Run_Thruster(float speed_percent);
void UCAN_Run_Thruster_PID(float speed);
void UCAN_Run_Pistol(float speed_percent);
void UCAN_Run_Pistol_Position(float speed_percent);
void UCAN_Run_Mass(float speed_percent);
void UCAN_Run_Mass_Position(float speed_percent);
void UCAN_Stop_Mass(void);
void UCAN_Send_Data(void);
void UCAN_Send_End_Frame_ARM2(void);
void UCAN_Run_Motor(void);
void UCAN_Run_Motor_Multi_Mode(void);
void Main_Operation(void);

#endif /* USER_CAN_H_ */
