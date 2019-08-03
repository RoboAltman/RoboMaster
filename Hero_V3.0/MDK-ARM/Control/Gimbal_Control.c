#include "Gimbal_Control.h"
#include "Chassis_Control.h"
#include "tim.h"
#include "math.h"
#include "MiniPC.h"
#include "Shoot_Control.h"
Motor_t GMotor_Pitch;
Motor_t GMotor_Yaw;
Motor_t GMotor_FPV;
Follow_Mode_e Follow_Mode;
Follow_Mode_e Last_Follow_Mode;
Forecast_Mode_e Forecast_Mode;
float Yaw_IMU_OFFSET;
GimbalRef_t GimbalRef;
float J_Scope_Yaw;
float J_Scope_Pitch;

PID_Parameter_t Pitch_PosPID = {55,0.6,220,500,1000};
PID_Parameter_t Pitch_VelPID = {50,0,0,10000,30000};
//PID_Parameter_t Yaw_PosPID = {100,0.5,1800,200,500};
//PID_Parameter_t Yaw_VelPID = {20,0,0,10000,30000};
PID_Parameter_t Yaw_PosPID = {30,0.2,220,200,500};
PID_Parameter_t Yaw_VelPID = {50,0,0,10000,30000};

PID_Parameter_t Yaw_IMU_PosPID = {18.5,0.03,20,3000,30000};
PID_Parameter_t Yaw_IMU_VelPID = {100,0,0,3000,30000};

PID_Parameter_t Pitch_IMU_PosPID = {18,0.6,0,200,350};
PID_Parameter_t Pitch_IMU_VelPID = {80,0.1,0,10000,30000};

//PID_Parameter_t Yaw_IMU_PosPID_CCW = {21,0.035,60,3000,30000};//Yaw轴向左转PID
//PID_Parameter_t Yaw_IMU_VelPID_CCW = {120,0,0,3000,30000};

//PID_Parameter_t Yaw_IMU_PosPID_CW = {15,0.01,60,3000,30000};//Yaw轴向右转PID
//PID_Parameter_t Yaw_IMU_VelPID_CW = {90,0,0,3000,30000};

//PID_Parameter_t Yaw_AUTO_PosPID0 = {15,0,50,0,100};
//PID_Parameter_t Yaw_AUTO_VelPID0 = {300,0,0,30000,30000};//0.3
PID_Parameter_t Yaw_AUTO_PosPID0 = {25,0.3,0,200,200};
PID_Parameter_t Yaw_AUTO_VelPID0 = {300,0.3,0,30000,30000};
PID_Parameter_t Yaw_AUTO_PosPID1 = {18,0.7,20,200,200};
PID_Parameter_t Yaw_AUTO_VelPID1 = {150,0,0,30000,30000};
PID_Parameter_t Yaw_AUTO_PosPID2 = {10,0.3,30,3000,200};
PID_Parameter_t Yaw_AUTO_VelPID2 = {100,0,0,10000,30000};
//PID_Parameter_t Yaw_AUTO_PosPID3 = {50,0,300,10000,30000};
//PID_Parameter_t Yaw_AUTO_VelPID3 = {100,0,0,30000,30000};


//CESHI
PID_Parameter_t Pitch_AUTO_PosPID0 = {18,0.6,0,200,350};
PID_Parameter_t Pitch_AUTO_VelPID0 = {80,0.1,0,10000,30000};
PID_Parameter_t Pitch_AUTO_PosPID1 = {18,0.6,0,200,350};
PID_Parameter_t Pitch_AUTO_VelPID1 = {80,0.1,0,10000,30000};
PID_Parameter_t Pitch_AUTO_PosPID2 = {20,0,0,200,350};
PID_Parameter_t Pitch_AUTO_VelPID2 = {50,0,0,10000,30000};

void Gimbal_Limit()
{
		if(GimbalRef.pitch_angle_ref >= 30)
       GimbalRef.pitch_angle_ref  = 30;
		if(GimbalRef.pitch_angle_ref <= -17)
		   GimbalRef.pitch_angle_ref  = -17;
}

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
	
	Yaw_angle = Yaw_IMU_OFFSET - IMU_acc;
	Pitch_angle = hi_imu_ang.y;
	if (Aim_State_42mm == Captured&& Aim_Mode == AUTO)		//??????????
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
			filter->wYaw[39] = (Yaw_angle + MiniPC_42mm.Yaw - filter->lastAbsYaw) / 
								((float)(filter->tick - filter->lastTick) / 1000);
			filter->wPitch[39] = (Pitch_angle + MiniPC_42mm.Pitch - filter->lastAbsPitch) / 
								((float)(filter->tick - filter->lastTick) / 1000);
			
			/* ???? */
			for (i = 0; i < 40; i++)
			{
				tmpWYaw += filter->wYaw[i];
				tmpWPitch += filter->wPitch[i];
			}
			filter->avgWYaw = tmpWYaw / 40;
			filter->avgWPitch = tmpWPitch / 40;
			Filter_Yaw = wYaw_Filter(filter->avgWYaw,0.8);
			if(Forecast_Mode == NO_Forecast)
			{
				filter->yawFore = 0;
			}
			if(Forecast_Mode == Sentry_Mode)
			{
				if(__fabs(Filter_Yaw)>8.0f)
				{	
					filter->yawFore = (Filter_Yaw/__fabs(Filter_Yaw)*5);
					if(filter->yawFore > 5)
						filter->yawFore = 5;
					if(filter->yawFore < -5)
						filter->yawFore = -5;				
				}
				else
				{
					if(filter->yawFore>=0)
					{
						if(filter->yawFore - 0.1f < 0)
						{
							filter->yawFore = 0;				
						}
						else
						{
							filter->yawFore -= 0.1f;
						}
					}
					if(filter->yawFore<= 0)
					{
						if(filter->yawFore + 0.1f > 0)
						{
							filter->yawFore = 0;				
						}
						else
						{
							filter->yawFore += 0.1f;
						}
					}
				}
				
			}
			if(Forecast_Mode == Infantry_Mode)
			{
				filter->yawFore = MiniPC_42mm.Distance/14 * Filter_Yaw;
			}
			
			
			filter->yawFore = Forecast_Filter(filter->yawFore,0.9);
			/* ???? */
			filter->lastTick = filter->tick;
			filter->lastAbsYaw = Yaw_angle + MiniPC_42mm.Yaw;
			filter->lastAbsPitch = Pitch_angle+ MiniPC_42mm.Pitch;
		}
		else
		{
			filter->isFirst = 0;
			filter->avgWYaw = 0;
			filter->avgWPitch = 0;
			filter->lastAbsYaw = Yaw_angle + MiniPC_42mm.Yaw;
			filter->lastAbsPitch = Pitch_angle + MiniPC_42mm.Pitch;
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


//跟踪微分器


static float sign(float x)
{
	if(x > 0)
		return (1.0f);
	if(x < 0)
		return (-1.0f);
	return 0;
}
static float fhan(float x1, float x2, float r, float h)
{
	float d, d0, y, a0, a, fh;
    d = r*h;
    d0 = h*d;
    y = x1 + h*x2;
    a0 = sqrt(d*d +8*r*fabs(y));
     
     if(fabs(y)>d0)
         a = x2 + (a0 - d)*sign(y)/2;
     else 
         a = x2 + y/h;
     
     if (fabs(a)>d)
         fh = -r*sign(a);
     else
         fh = -r*a/d;
	 
     return fh;
}

//跟踪微分器初始化
TD td;
TD td1;
TD td2;
void TD_Init(TD *td, float r, float h, float n0)
{
	td->r = r;
	td->h = h;
	td->N0 = n0;
}

//跟踪微分器计算
void TD_Calculate(TD *td, float expert)
{
	td->v1 += td->h * td->v2;
	td->v2 += td->h * fhan(td->v1 - expert, td->v2, td->r, td->h * td->N0);
}



/***
	*函数名称：GMotor_VelPID
    *函数功能: 云台电机速度PID
	*入口参数：云台电机速度结构体，pid参数
	*返回值  ：无
***/
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
void PID_Clear(Motor_t* motor)
{
	motor->posCtrl.derr = 0;
	motor->posCtrl.lasterr = 0;
	motor->posCtrl.sum = 0;
	motor->velCtrl.derr = 0;
	motor->velCtrl.lasterr = 0;
	motor->velCtrl.sum = 0;
}
/***
	*函数名称：GMotor_PosPID
    *函数功能: 云台电机位置PID
	*入口参数：云台电机位置结构体，pid参数
	*返回值  ：无
***/
void GMotor_PosPID(PosCtrl_t *Pos_t,PID_Parameter_t *pid)
{
//		if(Pos_t->refpos_soft < (Pos_t->refpos - Pos_t->acc))
//		Pos_t->refpos_soft += Pos_t->acc;
//		else if(Pos_t->refpos_soft > (Pos_t->refpos + Pos_t->dec))
//		Pos_t->refpos_soft -= Pos_t->dec;
//		else
//		Pos_t->refpos_soft = Pos_t->refpos;
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
//不完全微分PID
void Pos_half_PID(PosCtrl_t *Pos_t,PID_Parameter_t *pid,float alpha)
{
		Pos_t->err = Pos_t->refpos - Pos_t->rawpos;
	  //if(Pos_t->err<=5)
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
		Pos_t->derr = pid->kd * (1-alpha) * Pos_t->derr + alpha * Pos_t->lastderr;
		Pos_t->lasterr = Pos_t->err;
		Pos_t->lastderr = Pos_t->derr;
		Pos_t->output = pid->kp * Pos_t->err + pid->ki * Pos_t->sum + Pos_t->derr;
	  if(Pos_t->output >  pid->outputMax)
		{
			Pos_t->output =  pid->outputMax;
		}
		if(Pos_t->output < -pid->outputMax)
		{
			Pos_t->output = -pid->outputMax;
		}
	
}

void Vel_half_PID(VelCtrl_t *Vel_t,PID_Parameter_t *pid,float alpha)
{
		Vel_t->err = Vel_t->refvel - Vel_t->rawvel;
		Vel_t->sum += Vel_t->err;
	    if(Vel_t->sum >= pid->integralMax)
		{
			Vel_t->sum = pid->integralMax;
		}
	    if(Vel_t->sum <= -pid->integralMax)
		{
			Vel_t->sum = -pid->integralMax;
		}
		Vel_t->derr = Vel_t->err - Vel_t->lasterr;
		Vel_t->derr = pid->kd * (1-alpha) * Vel_t->derr + alpha * Vel_t->lastderr;
		Vel_t->lasterr = Vel_t->err;
		Vel_t->lastderr = Vel_t->derr;
		Vel_t->output = pid->kp * Vel_t->err + pid->ki * Vel_t->sum + Vel_t->derr;
	  if(Vel_t->output >  pid->outputMax)
		{
			Vel_t->output =  pid->outputMax;
		}
		if(Vel_t->output < -pid->outputMax)
		{
			Vel_t->output = -pid->outputMax;
		}
	
}

/***
	*函数名称：Pitch_Control
    *函数功能: Pitch轴控制
	*入口参数：无
	*返回值  ：无
***/
//float Filtet_parameters;
float Ptich_Filter(float input,float Filtet_parameters)
{
	static float lastoutput;
	float output;
	output = (1 - Filtet_parameters) * input + Filtet_parameters * lastoutput;
  lastoutput = output;
	return output;	
}

float Yaw_Filter(float input,float Filtet_parameters)
{
	static float lastoutput;
	float output;
	output = (1 - Filtet_parameters) * input + Filtet_parameters * lastoutput;
  lastoutput = output;
	return output;	
}





void Pitch_Control()
{
	if(Control_Mode == Remote)
	{
			if(Last_Control_Mode == Mouse_Board)
			{ 
				PID_Clear(&GMotor_Pitch);
				GimbalRef.pitch_angle_ref  = GMotor_Pitch.posCtrl.accpos/22.7556f;
			}
			  Gimbal_Limit();
				GMotor_Pitch.posCtrl.refpos = GimbalRef.pitch_angle_ref ;
				GMotor_Pitch.posCtrl.rawpos = GMotor_Pitch.posCtrl.accpos/22.7556f;
				GMotor_PosPID(&GMotor_Pitch.posCtrl,&Pitch_PosPID);
				GMotor_Pitch.velCtrl.refvel = GMotor_Pitch.posCtrl.output;
				GMotor_Pitch.velCtrl.rawvel = -GMotor_Pitch.encoderCtrl.speed/60;
				GMotor_VelPID(&GMotor_Pitch.velCtrl,&Pitch_VelPID);
				Last_Control_Mode = Remote;
	}
	else if(Control_Mode == Mouse_Board)
	{
		if(Aim_Mode == NOAUTO)
		{
			if(Last_Aim_Mode == AUTO)
			{ 
				PID_Clear(&GMotor_Pitch);
				GimbalRef.pitch_angle_ref  = GMotor_Pitch.posCtrl.accpos/22.7556f;
			}
			  Gimbal_Limit();
				GMotor_Pitch.posCtrl.refpos = GimbalRef.pitch_angle_ref ;
				GMotor_Pitch.posCtrl.rawpos = GMotor_Pitch.posCtrl.accpos/22.7556f;
				GMotor_PosPID(&GMotor_Pitch.posCtrl,&Pitch_PosPID);
				GMotor_Pitch.velCtrl.refvel = GMotor_Pitch.posCtrl.output;
				GMotor_Pitch.velCtrl.rawvel = -GMotor_Pitch.encoderCtrl.speed/60;
				GMotor_VelPID(&GMotor_Pitch.velCtrl,&Pitch_VelPID);
				//Filter(GMotor_Pitch.velCtrl.output,0.9);
				//Last_Aim_Mode = NOAUTO;	
		}
		else if(Aim_Mode == AUTO)
		{
//				 if(Last_Aim_Mode == NOAUTO)
//					{
//             GimbalRef.pitch_angle_ref = hi_imu_ang.y;
//					}
				if((Aim_State_42mm == Lost && Last_Aim_State_42mm == Captured )|| (Last_Aim_Mode == NOAUTO&&Aim_State_42mm == Lost))
				{
					//PID_Clear(&GMotor_Pitch);
					GimbalRef.pitch_angle_ref = hi_imu_ang.y;
				}
				if(Aim_State_42mm == Captured)
					GimbalRef.pitch_angle_ref = hi_imu_ang.y + Ptich_Filter(MiniPC_42mm.Pitch,0.9f);
				  Gimbal_Limit();
					GMotor_Pitch.posCtrl.refpos = GimbalRef.pitch_angle_ref;
					GMotor_Pitch.posCtrl.rawpos = hi_imu_ang.y;
					GMotor_Pitch.posCtrl.err = GMotor_Pitch.posCtrl.refpos - GMotor_Pitch.posCtrl.rawpos;
				  GMotor_Pitch.posCtrl.derr = GMotor_Pitch.posCtrl.err - GMotor_Pitch.posCtrl.lasterr;
          if(__fabs(GMotor_Pitch.posCtrl.err) > 8)
					{
						GMotor_Pitch.posCtrl.sum = 0;
						Pitch_AUTO_PosPID0 = 	Pitch_AUTO_PosPID2;
            Pitch_AUTO_VelPID0 = 	Pitch_AUTO_VelPID2;
					}
          else
					{
						Pitch_AUTO_PosPID0 = 	Pitch_AUTO_PosPID1;
            Pitch_AUTO_VelPID0 = 	Pitch_AUTO_VelPID1;
					}						
					GMotor_PosPID(&GMotor_Pitch.posCtrl,&Pitch_AUTO_PosPID0);
					GMotor_Pitch.velCtrl.refvel = GMotor_Pitch.posCtrl.output;
					GMotor_Pitch.velCtrl.rawvel = hi_imu_angvel.x;
					J_Scope_Pitch = -hi_imu_angvel.x;
					GMotor_VelPID(&GMotor_Pitch.velCtrl,&Pitch_AUTO_VelPID0);		
			}
				Last_Control_Mode = Mouse_Board;
		//Last_Aim_Mode = AUTO;
		}

}
/***
	*函数名称：Yaw_Control
    *函数功能: Yaw轴控制
	*入口参数：无
	*返回值  ：无
***/
void Yaw_Control()
{
	if(Control_Mode == Remote)//遥控器模式，只控Pitch轴，Yaw轴跟随底盘
	{
			GMotor_Yaw.posCtrl.refpos = 0;
			GMotor_Yaw.posCtrl.rawpos = (float)(GMotor_Yaw.posCtrl.accpos-Yaw_Motor_OFFSET)/22.7556f;
			Pos_half_PID(&GMotor_Yaw.posCtrl,&Yaw_PosPID,0.9);
			GMotor_Yaw.velCtrl.refvel = GMotor_Yaw.posCtrl.output;
			GMotor_Yaw.velCtrl.rawvel = (float)GMotor_Yaw.encoderCtrl.speed/60;
			Vel_half_PID(&GMotor_Yaw.velCtrl,&Yaw_VelPID,0.9);
			Last_Follow_Mode = Gimbal_Follow_Chassis;
	}
	else if(Control_Mode == Mouse_Board)//键鼠模式
	{
		if(Aim_Mode == NOAUTO)//不是自瞄
			{
				if(Follow_Mode == Chassis_Follow_Gimbal)//底盘跟随云台，反馈为IMU
				{
					if(Last_Follow_Mode == Gimbal_Follow_Chassis || Last_Aim_Mode == AUTO )
					{
            PID_Clear(&GMotor_Yaw);
						GimbalRef.yaw_angle_ref = Yaw_IMU_OFFSET - IMU_acc;
					}
					GMotor_Yaw.posCtrl.refpos = GimbalRef.yaw_angle_ref;
					GMotor_Yaw.posCtrl.rawpos = Yaw_IMU_OFFSET - IMU_acc;
					GMotor_PosPID(&GMotor_Yaw.posCtrl,&Yaw_IMU_PosPID);
					GMotor_Yaw.velCtrl.refvel = GMotor_Yaw.posCtrl.output;
					GMotor_Yaw.velCtrl.rawvel = -hi_imu_angvel.z;
					GMotor_VelPID(&GMotor_Yaw.velCtrl,&Yaw_IMU_VelPID);
				  Last_Follow_Mode = Chassis_Follow_Gimbal;
				}	
				else if(Control_Mode == Gimbal_Follow_Chassis)//底盘跟随云台\，反馈为Yaw轴电机
				{
					GMotor_Yaw.posCtrl.refpos = 0;//-GimbalRef.yaw_angle_ref;
					GMotor_Yaw.posCtrl.rawpos = (float)(GMotor_Yaw.posCtrl.accpos-Yaw_Motor_OFFSET)/22.7556f;
					GMotor_PosPID(&GMotor_Yaw.posCtrl,&Yaw_PosPID);
					GMotor_Yaw.velCtrl.refvel = GMotor_Yaw.posCtrl.output;
					GMotor_Yaw.velCtrl.rawvel = (float)GMotor_Yaw.encoderCtrl.speed/60;
					GMotor_VelPID(&GMotor_Yaw.velCtrl,&Yaw_VelPID);
					Last_Follow_Mode = Gimbal_Follow_Chassis;	
				}
				Last_Aim_Mode = NOAUTO;
			}			
			else if(Aim_Mode == AUTO)//自瞄模式
			{
				  if(Last_Aim_Mode == NOAUTO)
					{
						GMotor_Yaw.posCtrl.sum = 0;
					}
					if(Last_Aim_State_42mm == Captured && Aim_State_42mm == Lost)
					{
             //PID_Clear(&GMotor_Yaw);
						 GimbalRef.yaw_angle_ref = Yaw_IMU_OFFSET-IMU_acc;
					}
//					GMotor_Yaw.posCtrl.refpos = GimbalRef.yaw_angle_ref;
//					GMotor_Yaw.posCtrl.rawpos = Yaw_IMU_OFFSET - IMU_acc;
//					GMotor_PosPID(&GMotor_Yaw.posCtrl,&Yaw_IMU_PosPID);
//					GMotor_Yaw.velCtrl.refvel = GMotor_Yaw.posCtrl.output;
//					GMotor_Yaw.velCtrl.rawvel = -hi_imu_angvel.z;
//					GMotor_VelPID(&GMotor_Yaw.velCtrl,&Yaw_IMU_VelPID);
//				
//				  if(Last_Aim_State_42mm == Lost && Last_Aim_Mode == NOAUTO)
//					{
//							PID_Clear(&GMotor_Yaw);
//					}
					if(Aim_State_42mm == Captured)
					GimbalRef.yaw_angle_ref = Yaw_IMU_OFFSET - IMU_acc + MiniPC_42mm.Yaw+ GimbalFilter.yawFore;
					GMotor_Yaw.posCtrl.refpos = GimbalRef.yaw_angle_ref;// + GimbalFilter.yawFore;//GimbalFilter.yawFore;//MiniPC.Yaw*0.05f  ;
					GMotor_Yaw.posCtrl.rawpos = Yaw_IMU_OFFSET - IMU_acc;
				  GMotor_Yaw.posCtrl.err = GMotor_Yaw.posCtrl.refpos - GMotor_Yaw.posCtrl.rawpos;
					GMotor_Yaw.posCtrl.derr = GMotor_Yaw.posCtrl.err - GMotor_Yaw.posCtrl.lastderr;
//					if(__fabs(GMotor_Yaw.posCtrl.derr) >= 10.0f )
//					{
          if(__fabs(GMotor_Yaw.posCtrl.derr) >= 1.0f)
					{
						Yaw_AUTO_PosPID0 = Yaw_AUTO_PosPID2;
						Yaw_AUTO_VelPID0 = Yaw_AUTO_VelPID2;
					}
//					}
					else
					{
						Yaw_AUTO_PosPID0 = Yaw_AUTO_PosPID1;
						Yaw_AUTO_VelPID0 = Yaw_AUTO_VelPID1;
					}
					J_Scope_Yaw = -(IMU_acc - Yaw_IMU_OFFSET);
// 					GMotor_Yaw.posCtrl.err = GMotor_Yaw.posCtrl.refpos - GMotor_Yaw.posCtrl.rawpos;
//      		if(__fabs(GMotor_Yaw.posCtrl.err) >= 3)
//					{
//						Yaw_AUTO_PosPID0 = Yaw_AUTO_PosPID1;
//						Yaw_AUTO_VelPID0 = Yaw_AUTO_VelPID1;
//					}
//					else
//					{
//						Yaw_AUTO_PosPID0 = Yaw_AUTO_PosPID2;
//            Yaw_AUTO_VelPID0 = Yaw_AUTO_VelPID2;
//					}					
//					else if(__fabs(GMotor_Yaw.posCtrl.err)>12)
//					{
//						Yaw_AUTO_PosPID0 = Yaw_AUTO_PosPID3;
//						Yaw_AUTO_VelPID0 = Yaw_AUTO_VelPID3;
//					}
					GMotor_PosPID(&GMotor_Yaw.posCtrl,&Yaw_AUTO_PosPID0);
					GMotor_Yaw.velCtrl.refvel =  GMotor_Yaw.posCtrl.output;
					GMotor_Yaw.velCtrl.rawvel =  -hi_imu_angvel.z;		
					GMotor_VelPID(&GMotor_Yaw.velCtrl,&Yaw_AUTO_VelPID0);
					Last_Aim_Mode = AUTO;
     }

	}
}


/***
	*函数名称：Gimbal_Control
  *函数功能: 云台控制
	*入口参数：无
	*返回值  ：无
***/
float a1=200;
float	b1=0.085;
float	c1=0.005;

void Gimbal_Control()
{ 
	//TD_Init(&td2,a1,b1,c1);
	Gimbal_TraceForecast();
	Pitch_Control();
  Yaw_Control();
  GM_CAN_Send_Msg();
}


