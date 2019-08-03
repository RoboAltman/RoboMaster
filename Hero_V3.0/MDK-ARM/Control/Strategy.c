#include "Strategy.h"
#include "Referee_Decode.h"
#include "Motor.h"
#include "Shoot_Control.h"
#include "Remote_Control.h"
int Shoot_Heat_Limit;
int Shoot_cool;
enum fire_mode Fire_Mode;
enum fire_state Fire_State;
enum fire_state Fire_State_17mm;
enum deceleration_state Deceleration_State;
float heat_rest ;
float last_heat_rest;
//float my_heat_rest=0;
float shoot_heat_42mm;
float Q;
float Q_17mm;
float cooling_rate;
void Shoot_Strategy_42mm(uint8_t mode)
{	
	switch(mode)
	{
		case Fire_At_Will_Mode:
		{
			Fire_State=Fire_Ready;
		}break;
		case Fire_Limit_Mode:
		{
			if(RobotState.shooter_heat1_cooling_limit - Q > 100)
			{
					Fire_State = Fire_Ready;
			}
			else
					Fire_State = Fire_Negate;	
		}break;
	}	
}

void Shoot_Strategy_17mm()
{
//	if((RobotState.shooter_heat0_cooling_limit - Q_17mm < RobotState.shooter_heat0_cooling_limit*2/3) || (RobotState.shooter_heat0_cooling_limit - PowerHeatData.shooterHeat0 < RobotState.shooter_heat0_cooling_limit*2/3))
//	{
//		
//	}
	if(RobotState.shooter_heat0_cooling_limit - Q_17mm > 50 && RobotState.shooter_heat0_cooling_limit - PowerHeatData.shooterHeat0 > 50)
	{
		Fire_State_17mm = Fire_Ready;
		Shoot_Enable_STA = Shoot_Enable_STA|0X02;
	}
	else
	{
		Fire_State_17mm = Fire_Negate;
		Shoot_Enable_STA = Shoot_Enable_STA&0XFD;
	}
}


int Deceleration_Flag;
int Speed_Back = 0;
void SMotor_Deceleration_Test()
{
	if(Speed_Back == 1)
	{
		if(5300 - SLMotor.velCtrl.rawvel  > 500 && SLMotor.velCtrl.refvel != 0)
		{
			Deceleration_State = Deceleration_Confirm;
			Deceleration_Flag ++;
			Speed_Back = 0;
		}
		else
		{
			Deceleration_State = Deceleration_Deny;
		}
	}
	if(SLMotor.velCtrl.rawvel > 5100)
	{
		Speed_Back = 1;
	}
}

//void Shoot_Strategy_42mm2(uint8_t mode)
//{	
//	switch(mode)
//	{
//		case Fire_At_Will_Mode:
//		{
//			Fire_State=Fire_Ready;
//		}break;
//		case Fire_Limit_Mode:
//		{
//			if(my_heat_rest >= 100)
//			{
//					Fire_State = Fire_Ready;
//			}
//			else
//					Fire_State = Fire_Negate;	
//		}break;
//	}	
//}
void Shoot_calculate(void)
{
	//42mm
	if(RobotState.shooter_heat1_cooling_rate == 0)
	RobotState.shooter_heat1_cooling_rate = 20;
	Q = Q - 0.1* RobotState.shooter_heat1_cooling_rate;
	if(Q <= 0)
	Q = 0;
	if(Q > RobotState.shooter_heat1_cooling_limit+120)
	{
		Q = RobotState.shooter_heat1_cooling_limit+120;
	}
	
	//17mm
	if(RobotState.shooter_heat0_cooling_rate == 0)
	RobotState.shooter_heat0_cooling_rate = 40;
	Q_17mm = Q_17mm - 0.1 * RobotState.shooter_heat0_cooling_rate;
	if(Q_17mm <= 0)
	Q_17mm = 0;
}




