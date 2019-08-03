#include "Shoot_Control.h"
#include "Receive.h"
#include "Transmit.h"
#include "math.h"
#include "Motor.h"
#include "strategy.h"
#include "Remote_Control.h"
#include "Referee_Decode.h"
Motor_t SLMotor;    //大摩擦轮左边电机
Motor_t SRMotor;    //大摩擦轮右边电机
Motor_t BLMotor;    //Bullet_Lo
Motor_t BPMotor;    //Bullet_Push
PID_Parameter_t Shoot_VelPID0 = {45,0.5,0,10000,10000}; //PID参数
PID_Parameter_t Shoot_VelPID1 = {30,0.02,0,10000,10000}; //PID参数
PID_Parameter_t BL_Velpid= {50,1,0,10000,30000};
//PID_Parameter_t BP_Velpid = {8,0,10,10000,10000};
PID_Parameter_t BP_Velpid = {1.5,0,0,10000,10000};
PID_Parameter_t BP_Pospid = {0.8,0.1,2,50000,10000};
PID_Parameter_t BL_Pospid = {50,0.001,0,0,20000};
float Speed_42mm = 5800;   //发射速度期望
int Bullet_Load_Vel = 20;
int Last_Shoot_State;
uint32_t Load_Tick = 0;
uint32_t Last_Load_Tick = 0;

Cover_State_e Cover_State;
Motor_State_e SMotor_State;
Shoot_Mode_e Shoot_Mode;
Photoelectric_Switch_e Photoelectric_Switch_State;
Aim_Mode_e Aim_Mode;
Aim_Mode_e Last_Aim_Mode;
Motor_State_e BPMotor_State = Motor_OFF;
Motor_State_e Last_BPMotor_State = Motor_ON;
Push_State_e Push_State;
Push_State_e Last_Push_State;
/***
	*函数名称：Shoot_PID
	*函数功能：摩擦轮PID
	*入口参数：电机名称
	*返回值  ：无
***/



void VelPID(VelCtrl_t *vel_t,PID_Parameter_t *pid)
{
         vel_t->err = vel_t->refvel - vel_t->rawvel;
         vel_t->derr = vel_t->err - vel_t->lasterr;
	       vel_t->lasterr = vel_t->err;
	       vel_t->sum += vel_t->err;
				 if(vel_t->sum >pid->integralMax)
				 {
					  vel_t->sum = pid->integralMax;
				 }
				 if(vel_t->sum <-pid->integralMax)
				 {
					  vel_t->sum =-pid->integralMax;
				 }
         vel_t->output = pid->kp * vel_t->err + pid->ki * vel_t->sum + pid->kd * vel_t->derr;
			   if(vel_t->output > pid->outputMax)
			   {
				    vel_t->output = pid->outputMax;
			   }
			   if(vel_t->output < -pid->outputMax)
			   {
				    vel_t->output = -pid->outputMax;
			   }	  
}
void PosPID(PosCtrl_t *pos_t,PID_Parameter_t *pid)
{
         pos_t->err = pos_t->refpos - pos_t->accpos;
         pos_t->derr = pos_t->err - pos_t->lasterr;
	       pos_t->lasterr = pos_t->err;
	       pos_t->sum += pos_t->err;;
				 if(pos_t->sum >10000)
				 {
					  pos_t->sum = 10000;
				 }
				 if(pos_t->sum <-10000)
				 {
					  pos_t->sum =-10000;
				 }
         pos_t->output = pid->kp * pos_t->err + pid->ki * pos_t->sum + pid->kd * pos_t->derr;
			   if(pos_t->output > pid->outputMax)
			   {
				    pos_t->output = pid->outputMax;
			   }
			   if(pos_t->output < -pid->outputMax)
			   {
				    pos_t->output = -pid->outputMax;
			   }	  
}
/***
	*函数名称：Big_Bullet_Load_PID
    *函数功能: 大拨轮位置PID
	*入口参数：电机名称
	*返回值  ：无
***/


void Cover_Control()
{
	if(Cover_State == 1)
	{
			TIM5->CCR1 = 25;
			TIM5->CCR2 = 5;
	}
	if(Cover_State == 0)
	{
			TIM5->CCR1 = 5;
			TIM5->CCR2 = 19;
	}
}


/***
  *函数名称: BulletLoad_Control
	*函数功能：发射机构控制，装弹，大弹丸位置闭环，小弹丸速度闭环
	*入口参数：电机名称
	*返回值  ：无
***/
Load_State_e Load_State;
Load_State_e Last_Load_State;
Motor_Work_State_e BLMotor_Work_State; 
Motor_Work_State_e Last_BLMotor_Work_State;
volatile int Bullet_Num;
int stuck_flag = 0;
int release_flag = 0;
int stuck_state = 0;
void Bullet_Stuck_Control()
{
	if(BLMotor_Work_State == Vel_pid)
	{
		if(BLMotor.velCtrl.refvel == 20 && BLMotor.encoderCtrl.speed <= 1 )
		{
			stuck_flag++;
			if(stuck_flag > 70)
			{
				stuck_state = 1;
				stuck_flag = 110;
			}	
			else
			{
				stuck_state = 0;
			}
		}
		else if(BLMotor.velCtrl.refvel != -20)
		{
			stuck_flag = 0;
			stuck_state = 0;
		}
		if(BLMotor.velCtrl.refvel < 0 && BLMotor.encoderCtrl.speed <= 0 && stuck_state == 1)
		{
			stuck_flag--;
			if(stuck_flag == 0)
			{
				stuck_state = 0;
			}					
		}
	}	
}
void Shoot_Motor_Control()
{
		switch(SMotor_State)
		{
			case Motor_OFF:
			{
				SLMotor.velCtrl.refvel = 0;
				SRMotor.velCtrl.refvel = 0;
				Last_Shoot_State = Motor_OFF;
			}break;
			case Motor_ON:
			{
				SLMotor.velCtrl.refvel = -Speed_42mm;
				SRMotor.velCtrl.refvel =  Speed_42mm;
				Last_Shoot_State = Motor_ON;
			}break;
		}
		if(SMotor_State == Motor_ON)
		{
	   VelPID(&SLMotor.velCtrl,&Shoot_VelPID0);
	   VelPID(&SRMotor.velCtrl,&Shoot_VelPID0);
		}
		if(SMotor_State == Motor_OFF)
		{
	   VelPID(&SLMotor.velCtrl,&Shoot_VelPID1);
	   VelPID(&SRMotor.velCtrl,&Shoot_VelPID1);
		}
		 SM_CAN_Send_Msg();	 
}
void BulletLoad_Control()
{			
	    Load_State = (int)HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_7);
	    Bullet_Stuck_Control();
			if(Bullet_Num>=3)
			{
				BLMotor_Work_State = Pos_pid;
				BLMotor.velCtrl.sum = 0;
				//Bullet_Num = 3;
			}
			else
				BLMotor_Work_State = Vel_pid;
			if(BLMotor_Work_State == Vel_pid)
			{
				BLMotor.velCtrl.refvel = Bullet_Load_Vel;
				if(stuck_state == 1)
				BLMotor.velCtrl.refvel = -20;	
				VelPID(&BLMotor.velCtrl,&BP_Velpid);
			}
			else if(BLMotor_Work_State == Pos_pid)
			{
				if(Last_BLMotor_Work_State == Vel_pid&&BLMotor_Work_State == Pos_pid)
						BLMotor.posCtrl.refpos = BLMotor.posCtrl.accpos + 140;
						PosPID(&BLMotor.posCtrl,&BL_Pospid);
			}
			Last_BLMotor_Work_State = BLMotor_Work_State;
			Last_Load_State = Load_State;
			VelPID(&BLMotor.velCtrl,&BL_Velpid);
      BL_CAN_Send_Msg();
}

void BulletPush_Control()
{
//		if(Photoelectric_Switch_State == OFF && Deceleration_State == Deceleration_Confirm)
//		{
//			Shoot_Mode = Cease;
//			Deceleration_State = Deceleration_Deny;
//		}
			if(Fire_State == Fire_Ready)
			{
				switch(Shoot_Mode)
				{
					case Cease:
					{
						BPMotor_State = Motor_OFF;
						
						break;
					}
					case Single:
					{
							if(SMotor_State == Motor_ON)
							{
								BPMotor_State = Motor_ON;							
							}
							else
								Shoot_Mode = Cease;
							break;
					}
				}
			}
			else
			{
				BPMotor_State = Motor_OFF;
				Shoot_Mode = Cease;
			}
//			if(BPMotor_State == Motor_OFF)
//			{
//				BPMotor.velCtrl.refvel = 0;
//			}
//			if(BPMotor_State == Motor_ON)
//			{
//					BPMotor.velCtrl.refvel = -2000;
//			}
//			BPMotor.velCtrl.rawvel = BPMotor.encoderCtrl.speed;
//			VelPID(&BPMotor.velCtrl,&BP_Velpid);
			if(BPMotor_State == Motor_OFF && Last_BPMotor_State == Motor_ON)
			{
				BPMotor.posCtrl.refpos = BPMotor.posCtrl.accpos; 
			}
			if(BPMotor_State == Motor_ON && Last_BPMotor_State == Motor_OFF)
			{
				BPMotor.posCtrl.refpos = BPMotor.posCtrl.accpos - 60000;
			}
			PosPID(&BPMotor.posCtrl,&BP_Pospid);
			BPMotor.velCtrl.refvel = BPMotor.posCtrl.output;
			VelPID(&BPMotor.velCtrl,&BP_Velpid);	
      BP_CAN_Send_Msg();
			Last_BPMotor_State = BPMotor_State;
			if(BPMotor_State == Motor_ON && __fabs(BPMotor.posCtrl.refpos - BPMotor.posCtrl.accpos)<= 4000)
			Shoot_Mode = Cease;
			Photoelectric_Switch_State = OFF;
}

/***
	*函数名称：Shoot_Control
	*函数功能：发射机构控制，速度期望赋值，进行PID计算，发送
	*入口参数：电机名称
	*返回值  ：无
*/

void Shoot_Control()
{		
	  Shoot_Strategy_17mm();
	  //SMotor_Deceleration_Test();
	  Shoot_Strategy_42mm(Fire_Limit_Mode);
    //Shoot_Motor_Control();
    BulletPush_Control();
	  BulletLoad_Control();
	  //Cover_Control();
}
