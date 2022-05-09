#include "my_balance.h"

/**
 * @defgroup Module Pin define
 * @{
 */	
			/** 
			* @brief   Timer Module define 
			*/
			#define 	ALGORITHM_TIM  				      				TIM9
			#define 	ALGORITHM_TIM_CLK					 	 				RCC_APB2Periph_TIM9
			#define 	ALGORITHM_TIM_CLK_Cmd    						RCC_APB2PeriphClockCmd
			#define 	ALGORITHM_TIM_IRQn    							TIM1_BRK_TIM9_IRQn				
			#define		ALGORITHM_TIM_PreemptionPriority		0x02
			#define		ALGORITHM_TIM_SubPriority						0x02		
			
			#define 	ALGORITHM_TIM_IRQHandler						TIM1_BRK_TIM9_IRQHandler	

#define pi 3.14159

float SAMPLE_TIME = 20;		 	//(ms)
float SETPOINT = 0;					//(do)
float ERROR_MAX = 120;			//(do)
float ERRORDOT_MAX = 20;			//(do/s)
float OFFSET_MAX = 30;
float K1 = 0;
float K2 = 0;
float Ku = 0;

float theta_in, position_out, offset;
static float error, pre_error, errordot;
static float error_filtered, errordot_filtered;
static uint8_t _sample_count = 1;
bool first_data_pass = true;

static float test = -1;

struct
{
	float NB;
	float NS;
	float ZE;
	float PB;
	float PS;
}muy_theta, muy_thetadot;

struct
{
	float NVB;
	float NB;
	float NM;
	float NS;
	float ZE;
	float PS;
	float PM;
	float PB;
	float PVB;
}velocity, muy_velocity, sum;

struct
{
	float C1;
	float C2;
}theta, thetadot;

void Algorithm_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_InitStructure;
	NVIC_InitTypeDef	NVIC_InitStruct;
	
	ALGORITHM_TIM_CLK_Cmd(ALGORITHM_TIM_CLK,ENABLE);
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_InitStructure.TIM_Prescaler = 72*100-1;
	TIM_InitStructure.TIM_Period = 300;
	TIM_TimeBaseInit(ALGORITHM_TIM, &TIM_InitStructure);
	TIM_Cmd(ALGORITHM_TIM, ENABLE);
	TIM_ITConfig(ALGORITHM_TIM, TIM_IT_Update, ENABLE);
	
	NVIC_InitStruct.NVIC_IRQChannel = ALGORITHM_TIM_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = ALGORITHM_TIM_PreemptionPriority;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = ALGORITHM_TIM_SubPriority;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
}

void Membership_Function_Declaration(void)
{
	//gain
	K1 = 1.0/(float)ERROR_MAX;
	K2 = 1.0/(float)ERRORDOT_MAX;
	Ku = (float)OFFSET_MAX;
	
	//theta
	theta.C1 = 0.33;
	theta.C2 = 0.66;
	
	//thetadot
	thetadot.C1 = 0.33;
	thetadot.C2 = 0.66;
	
	//velocity
	velocity.PVB = 1;
	velocity.PB = 0.75;
	velocity.PM = 0.5;
	velocity.PS = 0.25;
	velocity.ZE = 0;
	velocity.NS = -velocity.PS;
	velocity.NM = -velocity.PM;
	velocity.NB = -velocity.PB;
	velocity.NVB = -velocity.PVB;
}

float x2xdot(float x, float pre_x)
{
	return (x - pre_x)/(SAMPLE_TIME*0.001);
}

static float Fuzzy_Trapzoid(float x, float L, float C1, float C2, float R)
{
	if(x <= L)
	{
		return 0;
	}
	else if(x > L && x < C1)
	{
		return (x - L)/(C1 - L);
	}
	else if(x >= C1 && x <= C2)
	{
		return 1;
	}
	else if(x > C2 && x < R)
	{
		return (R - x)/(R - C2);
	}
	else if(x >= R)
	{
		return 0;
	}
}

void Fuzzy_Result(float x1, float x2)
{
	muy_theta.NB = Fuzzy_Trapzoid(x1,-2,-1,-theta.C2,-theta.C1);
	muy_theta.NS = Fuzzy_Trapzoid(x1,-theta.C2,-theta.C1,-theta.C1,0);
	muy_theta.ZE = Fuzzy_Trapzoid(x1,-theta.C1,0,0,theta.C1);
	muy_theta.PS = Fuzzy_Trapzoid(x1,0,theta.C1,theta.C1,theta.C2);
	muy_theta.PB = Fuzzy_Trapzoid(x1,theta.C1,theta.C2,1,2);
	
	muy_thetadot.NB = Fuzzy_Trapzoid(x2,-2,-1,-theta.C2,-theta.C1);
	muy_thetadot.NS = Fuzzy_Trapzoid(x2,-theta.C2,-theta.C1,-theta.C1,0);
	muy_thetadot.ZE = Fuzzy_Trapzoid(x2,-theta.C1,0,0,theta.C1);
	muy_thetadot.PS = Fuzzy_Trapzoid(x2,0,theta.C1,theta.C1,theta.C2);
	muy_thetadot.PB = Fuzzy_Trapzoid(x2,theta.C1,theta.C2,1,2);
}

static float min(float x, float y)
{
	if(x > y)
		return y;
	else
		return x;
}

static float Fuzzy_Map(float x, float in_min, float in_max, float out_min, float out_max) 
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}	

static void Control_Laws(Defuzzy_And_Or_Method method, float x1, float x2,int8_t y)
{
	float temp = 0;
	if(method == MAX_MIN)
	{
		temp = min(x1,x2);
	}
	else if(method == MAX_PROD)
	{
		temp = x1*x2;
	}
	switch(y)
	{
		case NVB:
			if(temp > muy_velocity.NVB)
				muy_velocity.NVB = temp;
			sum.NVB += temp;
		break;
		case NB:
			if(temp > muy_velocity.NB)
				muy_velocity.NB = temp;
			sum.NB += temp;
		break;
		case NM:
			if(temp > muy_velocity.NM)
				muy_velocity.NM = temp;
			sum.NM += temp;
		break;
		case NS:
			if(temp > muy_velocity.NS)
				muy_velocity.NS = temp;
			sum.NS += temp;
		break;
		case ZE:
			if(temp > muy_velocity.ZE)
				muy_velocity.ZE = temp;
			sum.ZE += temp;
		break;
		case PS:
			if(temp > muy_velocity.PS)
				muy_velocity.PS = temp;
			sum.PS += temp;
		break;
		case PM:
			if(temp > muy_velocity.PM)
				muy_velocity.PM = temp;
			sum.PM += temp;
		break;
		case PB:
			if(temp > muy_velocity.PB)
				muy_velocity.PB = temp;
			sum.PB += temp;
		break;
		case PVB:
			if(temp > muy_velocity.PVB)
				muy_velocity.PVB = temp;
			sum.PVB += temp;
		break;
	}
}

float Defuzzy_Result(Defuzzy_And_Or_Method method)
{
	Control_Laws(method,muy_theta.NB,muy_thetadot.NB,NVB);
	Control_Laws(method,muy_theta.NB,muy_thetadot.NS,NB);
	Control_Laws(method,muy_theta.NB,muy_thetadot.ZE,NM);
	Control_Laws(method,muy_theta.NB,muy_thetadot.PS,NS);
	Control_Laws(method,muy_theta.NB,muy_thetadot.PB,ZE);
	
	Control_Laws(method,muy_theta.NS,muy_thetadot.NB,NB);
	Control_Laws(method,muy_theta.NS,muy_thetadot.NS,NM);
	Control_Laws(method,muy_theta.NS,muy_thetadot.ZE,NS);
	Control_Laws(method,muy_theta.NS,muy_thetadot.PS,ZE);
	Control_Laws(method,muy_theta.NS,muy_thetadot.PB,PS);
	
	Control_Laws(method,muy_theta.ZE,muy_thetadot.NB,NM);
	Control_Laws(method,muy_theta.ZE,muy_thetadot.NS,NS);
	Control_Laws(method,muy_theta.ZE,muy_thetadot.ZE,ZE);
	Control_Laws(method,muy_theta.ZE,muy_thetadot.PS,PS);
	Control_Laws(method,muy_theta.ZE,muy_thetadot.PB,PM);
	
	Control_Laws(method,muy_theta.PS,muy_thetadot.NB,NS);
	Control_Laws(method,muy_theta.PS,muy_thetadot.NS,ZE);
	Control_Laws(method,muy_theta.PS,muy_thetadot.ZE,PS);
	Control_Laws(method,muy_theta.PS,muy_thetadot.PS,PM);
	Control_Laws(method,muy_theta.PS,muy_thetadot.PB,PB);
	
	Control_Laws(method,muy_theta.PB,muy_thetadot.NB,ZE);
	Control_Laws(method,muy_theta.PB,muy_thetadot.NS,PS);
	Control_Laws(method,muy_theta.PB,muy_thetadot.ZE,PM);
	Control_Laws(method,muy_theta.PB,muy_thetadot.PS,PB);
	Control_Laws(method,muy_theta.PB,muy_thetadot.PB,PVB);
	
	float TS = sum.NVB*velocity.NVB + sum.PVB*velocity.PVB +
							sum.NB*velocity.NB + sum.PB*velocity.PB +
							sum.NM*velocity.NM + sum.PM*velocity.PM +
							sum.NS*velocity.NS + sum.PS*velocity.PS +
							sum.ZE*velocity.ZE;
	float MS = sum.NVB + sum.NB + sum.NM + sum.NS + sum.ZE +
							sum.PVB + sum.PB + sum.PM + sum.PS;
	return TS/MS;
}

static void Filter_In(float x1, float x2)
{
	error_filtered = x1*K1;
	errordot_filtered = x2*K2;
	
	if(error_filtered < -1)
		error_filtered = -1;
	else if(error_filtered > 1)
		error_filtered = 1;
	
	if(errordot_filtered < -1)
		errordot_filtered = -1;
	else if(errordot_filtered > 1)
		errordot_filtered = 1;
}

static float rad2do(float x)
{
	return (float)((x*180.0)/pi);
}

static void Filter_Out(float x)
{
	x = x*Ku;
	position_out += x;
	if(position_out > 58)
		position_out = 58;
	else if(position_out < 0)
		position_out = 0;
}

static void Reset_Value(void)
{
	muy_velocity.NVB = 0;
	muy_velocity.NB = 0;
	muy_velocity.NM = 0;
	muy_velocity.NS = 0;
	muy_velocity.ZE = 0;
	muy_velocity.PS = 0;
	muy_velocity.PM = 0;
	muy_velocity.PB = 0;
	muy_velocity.PVB = 0;
	
	sum.NVB = 0;
	sum.NB = 0;
	sum.NM = 0;
	sum.NS = 0;
	sum.ZE = 0;
	sum.PS = 0;
	sum.PM = 0;
	sum.PB = 0;
	sum.PVB = 0;
}

void Balance_Operation(float Xsen_Pitch_Radian)
{
	_sample_count --;
	if(_sample_count <= 0)
	{
		theta_in = rad2do(Xsen_Pitch_Radian);
		error = SETPOINT - theta_in;
		errordot = x2xdot(error, pre_error);
		Filter_In(error, errordot);
		//Fuzzy Logic Controller
		//************************************************
		Fuzzy_Result(error_filtered, errordot_filtered);
		offset = Defuzzy_Result(MAX_PROD);
		//************************************************
		if(first_data_pass)
		{
			position_out = MASS_Position.Value;
			first_data_pass = false;
		}
		else
		{
			Filter_Out(offset);
			UCAN_Run_Mass_Position(position_out);
		}
		pre_error = error;
		Reset_Value();
		_sample_count = SAMPLE_TIME/10.0;
	}
}

void ALGORITHM_TIM_IRQHandler(void)
{
	if(TIM_GetITStatus(ALGORITHM_TIM, TIM_IT_Update) != RESET)
	{
		//Balance_Operation();
		TIM_ClearITPendingBit(ALGORITHM_TIM, TIM_IT_Update);
	}
}
