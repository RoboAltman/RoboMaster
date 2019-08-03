#include "Gimbal_Control.h"
#include "Board_I_TX.h"
#include "Motor.h"
#include "IMU.h"
Motor_t GMotor_Pitch;
Motor_t GMotor_Yaw;
Gimbal_Ref_t Gimbal_Ref;
PID_Parameter_t Ptich_PID;
PID_Parameter_t Yaw_Follow_Pos_PID = {10,0.1,0,100,300};
PID_Parameter_t Yaw_Follow_Vel_PID = {80,0,0,0,30000};
PID_Parameter_t Pitch_Follow_Pos_PID = {12,0.4,20,100,600};
PID_Parameter_t Pitch_Follow_Vel_PID = {55,0,0,0,30000};
PID_Parameter_t Yaw_Auto_Pos_PID = {15,1,0,200,100};
PID_Parameter_t Yaw_Auto_Vel_PID = {80,0.8,0,10000,30000};
PID_Parameter_t Pitch_Auto_Pos_PID = {5,0.4,30,100,300};
PID_Parameter_t Pitch_Auto_Vel_PID = {30,0.2,10,500,30000};
Aim_Mode_e Aim_Mode;
float J_Ref;
float J_raw;
float error1;
float avgyaw[40];
float secavgyaw;
float Forecast_Filter(float input,float Filtet_parameters)
{
	static float lastoutput;
	float output;
	output = (1 - Filtet_parameters) * input + Filtet_parameters * lastoutput;
  lastoutput = output;
	return output;	
}
float wYaw_Filter(float input,float Filtet_parameters)
{
	static float lastoutput;
	float output;
	output = (1 - Filtet_parameters) * input + Filtet_parameters * lastoutput;
  lastoutput = output;
	return output;	
}

float Filter_Yaw;
GimbalFilter_t GimbalFilter;
float Yaw_angle,Pitch_angle;
void Gimbal_TraceForecast()//Gimbal_t *gimbal)
{
	//GimbalFilter_t *filter = &(gimbal->gimbalFilter);
	GimbalFilter_t *filter = &GimbalFilter;
	float tmpWYaw = 0, tmpWPitch = 0;
	
	uint8_t i;
	
	Yaw_angle = -IMU_acc;
	Pitch_angle = hi_imu_ang.y;
	if (Aim_State == Captured&& Aim_Mode == Auto)		//??????????
	{
		if (filter->isFirst == 0)				//????????????
		{ 
			filter->tick = HAL_GetTick();		//????
			
			/* ????? */
			for (i = 0; i < 39; i++)
			{
				filter->wYaw[i] = filter->wYaw[i + 1];
				filter->wPitch[i] = filter->wPitch[i + 1];
			}
			
			/* ??????? */
			filter->wYaw[39] = (Yaw_angle + MiniPC_17mm.Yaw - filter->lastAbsYaw) / 
								((float)(filter->tick - filter->lastTick) / 1000);
			filter->wPitch[39] = (Pitch_angle + MiniPC_17mm.Pitch - filter->lastAbsPitch) / 
								((float)(filter->tick - filter->lastTick) / 1000);
			
			/* ???? */
			for (i = 0; i < 40; i++)
			{
				tmpWYaw += filter->wYaw[i];
				tmpWPitch += filter->wPitch[i];
			}
			
			
			filter->avgWYaw = tmpWYaw / 40;
			
//			for (i = 0; i < 40; i++)
//			{
//				tmpWYaw += filter->wYaw[i];
//				tmpWPitch += filter->wPitch[i];
//			}
			
			
			filter->avgWPitch = tmpWPitch / 40;
			Filter_Yaw = wYaw_Filter(filter->avgWYaw,0.9);
//			if(Filter_Yaw > 0)				
//			{
//			 filter->yawFore +=0.2;
//				if(filter->yawFore > 4)
//					filter->yawFore = 4;
//			}
//			else if(Filter_Yaw < 0)
//			{
//				filter->yawFore -=0.2;
//				if(filter->yawFore < -4)
//				filter->yawFore = -4;
//			}
			filter->yawFore = 5.0f/20.0f * Filter_Yaw;
			filter->yawFore = Forecast_Filter(filter->yawFore,0.9);
			/* ???? */
			filter->lastTick = filter->tick;
			filter->lastAbsYaw = Yaw_angle + MiniPC_17mm.Yaw;
			filter->lastAbsPitch = Pitch_angle+ MiniPC_17mm.Pitch;
		}
		else
		{
			filter->isFirst = 0;
			filter->avgWYaw = 0;
			filter->avgWPitch = 0;
			filter->lastAbsYaw = Yaw_angle + MiniPC_17mm.Yaw;
			filter->lastAbsPitch = Pitch_angle + MiniPC_17mm.Pitch;
			filter->lastTick = HAL_GetTick();
			filter->tick = filter->lastTick;
			filter->pitchFore = 0;
			filter->yawFore = 0;
			for (i = 0; i < 40; i++)
			{
				filter->wYaw[i] = 0;
				filter->wPitch[i] = 0;
			}
		}
	}
	else
	{
		filter->isFirst = 1;
		filter->avgWYaw = 0;
		filter->avgWPitch = 0;
		filter->lastAbsYaw =  Yaw_angle;
		filter->lastAbsPitch = Pitch_angle;
		filter->lastTick = HAL_GetTick();
		filter->tick = filter->lastTick;
		filter->pitchFore = 0;
		filter->yawFore = filter->yawFore * 0.95f;
		if(__fabs(filter->yawFore = filter->yawFore) < 0.2)
	  filter->yawFore = filter->yawFore = 0;
		for (i = 0; i < 40; i++)
		{
			filter->wYaw[i] = 0;
			filter->wPitch[i] = 0;
		}
	}
}

void GMotor_PosPID(PosCtrl_t *Pos_t,PID_Parameter_t *pid)
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


void GMotor_VelPID(VelCtrl_t *vel_t,PID_Parameter_t *pid)
{
//		if(vel_t->refvel_soft < (vel_t->refvel - vel_t->acc))
//		vel_t->refvel_soft += vel_t->acc;
//		else if(vel_t->refvel_soft  > (vel_t->refvel + vel_t->dec))
//		vel_t->refvel_soft -= vel_t->dec;
//		else
//		vel_t->refvel_soft = vel_t->refvel;
		
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
uint8_t switch_flag;
void Gimbal_Ref_set()
{
	//Aim_Mode = Shoot_Enable;
	if(Aim_State == Captured &&(Shoot_Enable&0x01) == 0x01)
	{
		Aim_Mode = Auto;
		switch_flag = 0;
	}
  else if(Aim_State == Lost&& Aim_Mode == Auto)
	{
		switch_flag++;
		if(switch_flag >= 200)
		{
			Aim_Mode = Follow;
		}
	}
//	else if((Shoot_Enable&0x04)==0x04)
//	{
//		Aim_Mode = Fix;
//	}
	else 
	Aim_Mode = Follow;	
	switch(Aim_Mode)
	{
		case Fix:
		{
			Gimbal_Ref.Yaw = 0;
			Gimbal_Ref.Pitch = 0;
			break;
		}
		
		case Follow:
		{
			Gimbal_Ref.Yaw = Gimbal_42mm.Yaw;
			Gimbal_Ref.Pitch = Gimbal_42mm.Pitch;
			break;
		}
		
		case Auto:
		{
			if(Aim_State == Captured)
			{
				Gimbal_Ref.Yaw = -IMU_acc + MiniPC_17mm.Yaw;// + GimbalFilter.yawFore;
				Gimbal_Ref.Pitch = (GMotor_Pitch.encoderCtrl.angle - Pitch_Motor_OFFSET)/22.7556f + MiniPC_17mm.Pitch;
        error1 = Gimbal_Ref.Yaw -GMotor_Yaw.posCtrl.rawpos;
			}
				break;
			
		}		
		default:
		break;
	}	
}

void Gimbal_Ref_Limit()
{
	if(Aim_Mode != Auto)
	{
	  if(Gimbal_Ref.Yaw >= 30)
		{
			Gimbal_Ref.Yaw = 30;
		}
		if(Gimbal_Ref.Yaw <= -30)
		{
			Gimbal_Ref.Yaw = -30;
		
		}
	}
		if(Gimbal_Ref.Pitch >= 20)
		{
			Gimbal_Ref.Pitch = 20;
		}
		if(Gimbal_Ref.Pitch<= -15)
		{
			Gimbal_Ref.Pitch = -15;
		}
}
void Gimbal_Control()
{
	  Gimbal_TraceForecast();
	  Gimbal_Ref_set();
	  Gimbal_Ref_Limit();
	//Yaw
		GMotor_Yaw.posCtrl.refpos = Gimbal_Ref.Yaw;
		
	  if(Aim_Mode == Auto)
		{
			GMotor_Yaw.posCtrl.refpos = Gimbal_Ref.Yaw;
			GMotor_Yaw.posCtrl.rawpos = -IMU_acc;
			GMotor_PosPID(&GMotor_Yaw.posCtrl,&Yaw_Auto_Pos_PID);
			GMotor_Yaw.velCtrl.refvel = GMotor_Yaw.posCtrl.output;
			GMotor_Yaw.velCtrl.rawvel = -hi_imu_angvel.z;
			GMotor_VelPID(&GMotor_Yaw.velCtrl,&Yaw_Auto_Vel_PID);
		}
		else
		{
			GMotor_Yaw.posCtrl.rawpos = (GMotor_Yaw.encoderCtrl.angle - Yaw_Motor_OFFSET)/22.7556f;
			GMotor_PosPID(&GMotor_Yaw.posCtrl,&Yaw_Follow_Pos_PID);
			GMotor_Yaw.velCtrl.refvel = GMotor_Yaw.posCtrl.output;
			GMotor_Yaw.velCtrl.rawvel = GMotor_Yaw.encoderCtrl.speed/60;
			GMotor_VelPID(&GMotor_Yaw.velCtrl,&Yaw_Follow_Vel_PID);
		}			

	//Pitch
		GMotor_Pitch.posCtrl.refpos = Gimbal_Ref.Pitch;
		GMotor_Pitch.posCtrl.rawpos = (GMotor_Pitch.encoderCtrl.angle - Pitch_Motor_OFFSET)/22.7556f;
		if(Aim_Mode == Auto)
		{
			GMotor_PosPID(&GMotor_Pitch.posCtrl,&Pitch_Auto_Pos_PID);
			GMotor_Pitch.velCtrl.refvel = GMotor_Pitch.posCtrl.output;
			GMotor_Pitch.velCtrl.rawvel = (float)GMotor_Pitch.encoderCtrl.speed/60.0f;
			GMotor_VelPID(&GMotor_Pitch.velCtrl,&Pitch_Auto_Vel_PID);
		}
		else
		{
			GMotor_PosPID(&GMotor_Pitch.posCtrl,&Pitch_Follow_Pos_PID);
			GMotor_Pitch.velCtrl.refvel = GMotor_Pitch.posCtrl.output;
			GMotor_Pitch.velCtrl.rawvel = GMotor_Pitch.encoderCtrl.speed/60;
			GMotor_VelPID(&GMotor_Pitch.velCtrl,&Pitch_Follow_Vel_PID);
		}
}

