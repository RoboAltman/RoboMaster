#ifndef _SHOOT_CONTROL_H_
#define _SHOOT_CONTROL_H_
#include "stm32f4xx_hal.h"
#include "PID.h"
#include "Motor.h"
#define Load 1u
#define unLoad 0u
#define Stuck 2u
void Shoot_Control(void);
void BulletLoad_Control(void);
extern Motor_t SLMotor;
extern Motor_t SRMotor;
extern Motor_t BLMotor;
extern Motor_t BPMotor;
typedef enum 
{
	Cease,
	Single
}Shoot_Mode_e;

typedef enum 
{
	Cover_Close,
	Cover_Open
}Cover_State_e;

typedef enum 
{
	Motor_OFF,
	Motor_ON
}Motor_State_e;

typedef enum 
{
  Vel_pid,
	Pos_pid
}Motor_Work_State_e;

typedef enum 
{
	Load_OFF,
	Load_ON
}Load_State_e;


typedef enum 
{
	Push_Complete,
	Push_Incomplete
}Push_State_e;

typedef enum 
{
	NOAUTO,
	AUTO
}Aim_Mode_e;

typedef enum 
{
	OFF,
  ON
}Photoelectric_Switch_e;


extern volatile int Bullet_Num;
extern Aim_Mode_e  Aim_Mode;
extern Aim_Mode_e  Last_Aim_Mode;
extern Cover_State_e  Cover_State;
extern Motor_State_e  SMotor_State;
extern Push_State_e  Push_State;
extern Motor_State_e BPMotor_State;
extern Push_State_e  Last_Push_State;
extern Shoot_Mode_e Shoot_Mode;
extern float Speed_42mm;
extern Motor_Work_State_e BLMotor_Work_State;
extern Photoelectric_Switch_e Photoelectric_Switch_State;
extern uint32_t Load_Tick;
extern uint32_t Last_Load_Tick;
void Shoot_Motor_Control(void);
#endif


