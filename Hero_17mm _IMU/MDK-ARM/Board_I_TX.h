#ifndef _Board_I_TX_H_
#define _Board_I_TX_H_
#include "stm32f4xx_hal.h"
#include "CRC.h"
typedef __packed struct
{
	float Yaw;
	float Pitch;
}Gimbal_42mm_t;

typedef __packed struct
{
	float Pitch;
	float Yaw;
	float Distance;
}MiniPC_t;

typedef __packed enum
{
	Lost,
	Captured
}Aim_State_e;

typedef __packed struct
{
	uint8_t SOF; //1
	MiniPC_t MiniPC_17mm; //12
	uint8_t Aim_State_17mm;//1
//	Heat_Data_t  Heat_Data;
	Gimbal_42mm_t Gimbal_42mm; //8
	uint8_t Shoot_Enable; //1
	uint16_t CRC16;//2
}Data_17mm_t;
extern Gimbal_42mm_t Gimbal_42mm;
extern MiniPC_t MiniPC_17mm;
extern uint8_t Aim_State;
extern uint8_t Shoot_Enable;
extern uint8_t Board_I_Data[26];
extern uint8_t Board_I_Data_Temp[60];
void Board_I_Decode(uint8_t *pData);
void Mini_PC_Data_Decode(uint8_t *pData);
#endif

