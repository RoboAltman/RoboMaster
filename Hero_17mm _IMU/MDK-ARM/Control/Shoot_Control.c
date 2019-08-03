#include "Shoot_Control.h"
#include "Gimbal_Control.h"
#include "Board_I_TX.h"
Motor_t BLMotor;
Shoot_Mode_e Shoot_Mode;
Heat_State_e Heat_State;
Motor_State_e Shoot_Motor_State;
Stuck_State_e Stuck_State;
int Stuck_Flag;
int LSM_Speed = 999;
int RSM_Speed = 999;
float Fre_17mm = 3000;
float Fre_17mm_check = 600;
float Load_Speed_17mm = 1500;
float Shoot_Speed = 1300;
//PID_Parameter_t Bullet_Load_PID = {5,0.8,0,10000,10000};
PID_Parameter_t Bullet_Load_PID = {13,0.8,0,10000,10000};
void BLoad_Pos_PID(PosCtrl_t *Pos_t,PID_Parameter_t *pid)
{
		Pos_t->err = Pos_t->refpos - Pos_t->rawpos;
		Pos_t->sum += Pos_t->err;
		
	  if(Pos_t->sum >= pid->integralMax)
		{
			Pos_t->sum = pid->integralMax;
		}
	    if(Pos_t->sum <= -pid->integralMax)
		{
			Pos_t->sum = -pid->integralMax;
		}
		Pos_t->derr = Pos_t->err - Pos_t->lasterr;
		Pos_t->lasterr = Pos_t->err;
		Pos_t->output = pid->kp * Pos_t->err + pid->ki * Pos_t->sum + pid->kd * Pos_t->derr;
	  if(Pos_t->output >  pid->outputMax)
		{
			Pos_t->output =  pid->outputMax;
		}
		if(Pos_t->output < -pid->outputMax)
		{
			Pos_t->output = -pid->outputMax;
		}
}
void BLoad_Vel_PID(VelCtrl_t *vel_t,PID_Parameter_t *pid)
{
		vel_t->err = vel_t->refvel - vel_t->rawvel;
		vel_t->sum += vel_t->err;
	    if(vel_t->sum >= pid->integralMax)
		{
			vel_t->sum = pid->integralMax;
		}
	    if(vel_t->sum <= -pid->integralMax)
		{
			vel_t->sum = -pid->integralMax;
		}
		vel_t->derr = vel_t->err - vel_t->lasterr;
		vel_t->lasterr = vel_t->err;
		vel_t->output = pid->kp * vel_t->err + pid->ki * vel_t->sum + pid->kd * vel_t->derr;
	  if(vel_t->output >  pid->outputMax)
		{
			vel_t->output =  pid->outputMax;
		}
		if(vel_t->output < -pid->outputMax)
		{
			vel_t->output = -pid->outputMax;
		}
	
}

void Shoot_Motor_Control()
{
	if((Shoot_Enable&0x01) == 0x01 || (Shoot_Enable&0x04)==0x04)
	{
		Shoot_Motor_State = Motor_ON;
	}
	else
	{
		Shoot_Motor_State = Motor_OFF;
	}
	switch(Shoot_Motor_State)
	{
		case Motor_OFF:
		{
			LSM_Speed = 1000;
			RSM_Speed = 1000;
			break;
		}
		case Motor_ON:
		{
			LSM_Speed += 5;
			if(LSM_Speed >= Shoot_Speed)
			{
				LSM_Speed = Shoot_Speed;
			}
			if(LSM_Speed == Shoot_Speed)
			{
				RSM_Speed += 5;
				if(RSM_Speed >= Shoot_Speed)
				RSM_Speed =Shoot_Speed;	
			}
			break;
		}
	}
	TIM1->CCR2 = LSM_Speed;
	TIM1->CCR3 = RSM_Speed;
}

void Bullet_Stuck_check()
{
	if(Shoot_Mode != Cease &&( BLMotor.velCtrl.refvel == 2000 || BLMotor.velCtrl.refvel == 1000 )&&BLMotor.encoderCtrl.speed <= 10)
	{
		Stuck_Flag++;
		if(Stuck_Flag >= 30)
		{
			Stuck_State = Stuck;
			Stuck_Flag = 200;
		}
		else
		{
			Stuck_State = Not_Stuck;
		}
	}
	else if(BLMotor.velCtrl.refvel != -1000)
	{
		Stuck_Flag = 0;
		Stuck_State = Not_Stuck;
	}	
  if(BLMotor.velCtrl.refvel < 0&&BLMotor.encoderCtrl.speed <= 0&&Stuck_State == Stuck)
	{
		Stuck_Flag--;
		if(Stuck_Flag == 0)
		{
			Stuck_State = Not_Stuck;
		}
    		
	}
}
void Bullet_Load_Control()
{
	if((Shoot_Enable&0x02) == 0x02)
	{
		Heat_State = Ready;
	}
	else
	{
		Heat_State = Not_Ready;
	}
	Bullet_Stuck_check();
	if((Aim_Mode == Auto && RSM_Speed ==Shoot_Speed && Aim_State == Captured && __fabs(GMotor_Yaw.posCtrl.err)<2.5f && __fabs(GMotor_Pitch.posCtrl.err)<2.5f) || ((Shoot_Enable&0x04)==0x04))
	{
		Shoot_Mode = Series;
	}
	else
	{
		Shoot_Mode = Cease;
	}
	

	switch(Shoot_Mode)
	{
		
		case Cease:
		{
			BLMotor.velCtrl.refvel = 0;
			TIM5->CCR4 = 500;
			break;
		}
		case Load:
		{
			//if(),微动开关
			BLMotor.velCtrl.refvel = Load_Speed_17mm;
			break;
		}
		case Series:
		{
			
			if(Heat_State == Ready)
			{
				TIM5->CCR4 = 1000;
				BLMotor.velCtrl.refvel = Fre_17mm;
				if((Shoot_Enable&0x08) == 0X08)
				{
					BLMotor.velCtrl.refvel = Fre_17mm_check;
				}
			}
			if(Heat_State == Not_Ready)
				BLMotor.velCtrl.refvel = 0;
			break;
		}
//		case single:
//		{
//		}
		default:
		break;
	}
	  if(Stuck_State == Stuck)
		BLMotor.velCtrl.refvel = -1000;
		BLMotor.velCtrl.rawvel = BLMotor.encoderCtrl.speed;
		BLoad_Vel_PID(&BLMotor.velCtrl,&Bullet_Load_PID);
}

void Shoot_Control()
{
	Bullet_Load_Control();
	Shoot_Motor_Control();
}

