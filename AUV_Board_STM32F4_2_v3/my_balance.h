#ifndef _MY_BALANCE_H_
#define _MY_BALANCE_H_

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
#include "my_can.h"
#include "my_delay.h"
#include "my_adc.h"

typedef enum
{
	NVB = -4,
	NB = -3,
	NM = -2,
	NS = -1,
	ZE = 0,
	PS = 1,
	PM = 2,
	PB = 3,
	PVB = 4
}Fuzzy_DataTypeDef;

typedef enum
{
	MAX_MIN = 0,
	MAX_PROD = 1
}Defuzzy_And_Or_Method;

//PP giai mo: trung binh co trong so

extern float SAMPLE_TIME;
extern float SETPOINT;
extern float ERROR_MAX;	
extern float ERRORDOT_MAX;
extern float OFFSET_MAX;
extern float K1;
extern float K2;
extern float Ku;
extern bool first_data_pass;

void Algorithm_Config(void);
void Membership_Function_Declaration(void);
float x2xdot(float x, float pre_x);
void Fuzzy_Result(float x1, float x2);
float Defuzzy_Result(Defuzzy_And_Or_Method method);
void Balance_Operation(float Xsen_Pitch_Radian);

#endif
