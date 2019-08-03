#ifndef _17mm_H_
#define _17mm_H_
#include "stm32f4xx_hal.h"
#include "Gimbal_Control.h"
#include "MiniPC.h"
#include "Referee_Decode.h"
#include "Shoot_Control.h"
#include "Remote_Control.h"
#include "CRC.h"
#include "string.h"
typedef struct
{
	float Heat_17mm;
	float Speed_17mm;
	float cooling_rate_17mm;
  float cooling_limit_17mm;
  	
}Heat_Data_t;
typedef __packed struct
{
	float Yaw;
	float Pitch;
}Gimbal_42mm_t;
typedef __packed struct
{
	uint8_t SOF1; //1
	uint8_t SOF2; //1
	MiniPC_t MiniPC_17mm; //12
	uint8_t Aim_State_17mm;
//	Heat_Data_t  Heat_Data;
	Gimbal_42mm_t Gimbal_42mm; //8
	uint8_t Shoot_Enable; //1
	uint16_t CRC16;//2
}Data_17mm_t;
void Data_17mm_Build(void);
extern uint8_t Board_II_Data[26];
extern Data_17mm_t Data_17mm;
#endif



