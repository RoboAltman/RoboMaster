#ifndef  _MINIPC_H_
#define _MINIPC_H_
#include "stm32f4xx_hal.h"
#include "Referee_Decode.h"
typedef __packed struct
{
	float Pitch;
	float Yaw;
	float Distance;
}MiniPC_t;
void Mini_PC_Data_Decode(uint8_t *pData);
extern MiniPC_t MiniPC_42mm;
extern MiniPC_t MiniPC_17mm;
typedef __packed enum
{
	Lost,
	Captured
}Aim_State_e;

typedef enum 
{
	Init_Fail,
	Init_OK
}Bullet_Num_Init_e;
extern Aim_State_e Aim_State_42mm;
extern Aim_State_e Aim_State_17mm;
extern volatile int Bullet_Num_Initnum;
extern Aim_State_e Last_Aim_State_42mm;
extern Aim_State_e Last_Aim_State_17mm;
extern volatile Bullet_Num_Init_e Bullet_Num_Init;
extern float Filter1[21];
#endif
