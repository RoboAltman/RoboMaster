#ifndef _STRATEGY_H_
#define _STRATEGY_H_
#include "stm32f4xx_hal.h"
#include "Referee_Decode.h"
#define Triple Tap 		  3
#define Shoot_Speed_Limit_42mm 16.5;

void Shoot_Strategy_42mm(uint8_t Fire_Mode);
//void Shoot_Strategy_42mm2(uint8_t Fire_Mode);
enum fire_mode
{
	Fire_At_Will_Mode,
	Fire_Limit_Mode,
	Buff_Mode
};
enum fire_state 
{
	Fire_Negate,
	Fire_Ready
};

enum deceleration_state 
{
	Deceleration_Deny,
	Deceleration_Confirm
};
extern enum fire_mode Fire_Mode;
extern enum fire_state Fire_State;
extern enum fire_state Fire_State_17mm;
extern enum deceleration_state Deceleration_State;
extern float heat_rest ;
extern float last_heat_rest;
//extern float my_heat_rest;
extern float Q;
extern float Q_17mm;
void shoot_calculate(void);
void SMotor_Deceleration_Test(void);
void Shoot_Strategy_17mm(void);
#endif
