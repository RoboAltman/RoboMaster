#ifndef _SHOOT_CONTROL_H_
#define _SHOOT_CONTROL_H_
#include "stm32f4xx_hal.h"
#include "Motor.h"
typedef enum
{
	Cease,
	Load,
	Series,
	Single
}Shoot_Mode_e;
typedef enum
{
	Motor_OFF,
	Motor_ON
}Motor_State_e;
typedef enum
{
	Ready,
	Not_Ready
}Heat_State_e;
typedef enum
{
	Stuck,
	Not_Stuck
}Stuck_State_e;
extern Shoot_Mode_e Shoot_Mode;
extern Motor_State_e Shoot_Motor_State;
extern Heat_State_e Heat_State;
void Shoot_Control(void);
extern Motor_t BLMotor;
extern Motor_t SLMotor;
extern Motor_t SRMotor;
#endif

