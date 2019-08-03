#include "Chassis_Control.h"
#include "Receive.h"
#include "Transmit.h"
#include "Power_Control.h"
#include "Referee_Decode.h"
#include "Gimbal_Control.h"
#include "Shoot_Control.h"
#include "math.h"
Motor_t CMotor1;
Motor_t CMotor2;
Motor_t CMotor3;
Motor_t CMotor4;
Motor_t Rotate;
PID_Parameter_t velPara={10,0,0.1,0,13000};
PID_Parameter_t rotate_Pos_PID0 = {0,0,0,0,0};
PID_Parameter_t rotate_Pos_PID1 = {0.8,0,3,0,800};
PID_Parameter_t rotate_Pos_PID2 = {5,0,5,0,100};
PID_Parameter_t rotate_Vel_PID1 = {1,0,2,0,700};
PID_Parameter_t rotate_Vel_PID2 = {7,0,0,10,1500};
PID_Parameter_t rotate_Vel_PID0 = {0,0,0,0,0};
float REMOTE_CHASIS_SPEED_TO_REF_FB = 12.0f; 
float REMOTE_CHASIS_SPEED_TO_REF_LR = -6.0f; 
float outputsum;
float error;
float power_max=80.0;

enum Chassis_Work_Mode Work_Mode,Ctrl_Chassis_Mode;                        //���̹���ģʽ

/***
	*函数名称：Motor_VelPID
    *函数功能: 底盘电机速度PID
	*入口参数：电机速度结构体
	*返回值  ：无
***/
int chassis_buffer_out=0;
void Motor_VelPID(VelCtrl_t *vel_t)
{
	vel_t->err = vel_t->refvel - vel_t->rawvel;
	vel_t->sum += vel_t->err;
	vel_t->derr = vel_t->err - vel_t->lasterr;
	if(vel_t->sum >= 10000)
		vel_t->sum = 10000;
	if(vel_t->sum <= -10000)
		vel_t->sum =-10000;
	vel_t->output = vel_t->err * velPara.kp + vel_t->sum * velPara.ki + vel_t->derr * velPara.kd;
	vel_t->lasterr = vel_t->err;
	if(vel_t->output >= velPara.outputMax)
		vel_t->output = velPara.outputMax;
	if(vel_t->output <= -velPara.outputMax)
		vel_t->output = -velPara.outputMax;	
}


void Set_Chassis_Movement_Sta(){
    static int count=0;
    if(ChassisRef.forward_back_ref==0 
    && ChassisRef.left_right_ref == 0 
    && ChassisRef.rotate_ref == 0 ){
            if(Movement_Status != CAR_STOP)    count++;
            else count = 0;
            if(count > 50){
                Movement_Status = CAR_STOP;
                count = 0;
            }
            //Movement_Status = CAR_STOP;
    }else if(ChassisRef.rotate_ref == 0){
            if(Movement_Status != CAR_MOVE)    count++;
            else count = 0;
            if(count > 50){
                Movement_Status = CAR_MOVE;
                count = 0;
            }
            //Movement_Status = CAR_MOVE;
    }else if(ChassisRef.forward_back_ref==0 
                && ChassisRef.left_right_ref == 0){
            if(Movement_Status != CAR_ROTATE)    count++;
            else count = 0;
            if(count > 50){
                Movement_Status = CAR_ROTATE;
                count = 0;
            }
            //Movement_Status = CAR_ROTATE;
    }else{
            if(Movement_Status != CAR_MIXMOVING)    count++;
            else count = 0;
            if(count > 50){
                Movement_Status = CAR_MIXMOVING;
                count = 0;
            }
            //Movement_Status = CAR_MIXMOVING;
    }
}

void Set_Zero_Ref(){
	  CMotor1.velCtrl.refvel = 0;
		CMotor2.velCtrl.refvel = 0;
		CMotor3.velCtrl.refvel = 0;
		CMotor4.velCtrl.refvel = 0;
//    CMspeed1.pid.ref=0;
//    CMspeed2.pid.ref=0;
//    CMspeed3.pid.ref=0;
//    CMspeed4.pid.ref=0;
}




void Rotate_Pos_PID(PosCtrl_t *Pos_t,PID_Parameter_t *pid)
{
	Pos_t->err  = Pos_t->refpos - Pos_t->rawpos;
	Pos_t->sum += Pos_t->err;
	Pos_t->derr =Pos_t->err - Pos_t->lasterr;
	if(Pos_t->sum > 300)
	{
		Pos_t->sum = 300;
	}
	if(Pos_t->sum < -300)
	{
		Pos_t->sum = -300;
	}
	Pos_t->output = Pos_t->err * pid->kp + Pos_t->sum * pid->ki + Pos_t->derr * pid->kd;
		    
	if(Pos_t->output >  pid->outputMax)
	{
		Pos_t->output =  pid->outputMax;
	}
	if(Pos_t->output < -pid->outputMax)
	{
		Pos_t->output = -pid->outputMax;
	}	
}

void Rotate_Vel_PID(VelCtrl_t *Vel_t,PID_Parameter_t *pid)
{
	Vel_t->err  = Vel_t->refvel - Vel_t->rawvel;
	Vel_t->sum += Vel_t->err;
	Vel_t->derr =Vel_t->err - Vel_t->lasterr;
	if(Vel_t->sum > 300)
	{
		Vel_t->sum = 300;
	}
	if(Vel_t->sum < -300)
	{
		Vel_t->sum = -300;
	}
	Vel_t->output = Vel_t->err * pid->kp + Vel_t->sum * pid->ki + Vel_t->derr * pid->kd;
		    
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
	*函数名称：Chassis_ref_Set
    *函数功能: 底盘期望值设定
	*入口参数：无
	*返回值  ：无
***/
void Chassis_ref_Set()
{
		CMotor1.velCtrl.refvel =  ChassisRef.forward_back_ref * REMOTE_CHASIS_SPEED_TO_REF_FB
	                             +ChassisRef.left_right_ref   * REMOTE_CHASIS_SPEED_TO_REF_LR
	                             +ChassisRef.rotate_ref       * ROTATE_TO_REF; 
		CMotor2.velCtrl.refvel = -ChassisRef.forward_back_ref * REMOTE_CHASIS_SPEED_TO_REF_FB 
	                             +ChassisRef.left_right_ref   * REMOTE_CHASIS_SPEED_TO_REF_LR
	                             +ChassisRef.rotate_ref       * ROTATE_TO_REF;
		CMotor3.velCtrl.refvel = -ChassisRef.forward_back_ref * REMOTE_CHASIS_SPEED_TO_REF_FB 
	                             -ChassisRef.left_right_ref   * REMOTE_CHASIS_SPEED_TO_REF_LR	
	                             +ChassisRef.rotate_ref       * ROTATE_TO_REF;
		CMotor4.velCtrl.refvel =  ChassisRef.forward_back_ref * REMOTE_CHASIS_SPEED_TO_REF_FB 
	                             -ChassisRef.left_right_ref   * REMOTE_CHASIS_SPEED_TO_REF_LR      
	                             +ChassisRef.rotate_ref       * ROTATE_TO_REF;	
}


/***
	*函数名称：Chassis_Control
    *函数功能: 底盘控制，Can发送
	*入口参数：无
	*返回值  ：无
***/
void Chassis_Control()
{
	if(Rotate_State == Zero)
	{
		Rotate.posCtrl.rawpos = ((float)(Yaw_Motor_OFFSET-GMotor_Yaw.posCtrl.accpos))/22.7556f;
		Rotate_Pos_PID(&Rotate.posCtrl,&rotate_Pos_PID1);
		Rotate.velCtrl.refvel = Rotate.posCtrl.output;
//		if(__fabs((float)(GMotor_Yaw.posCtrl.accpos-Yaw_Motor_OFFSET))/22.7556f <=5 && Rotate.posCtrl.refpos == 0 && (RC_Ctrl_Data.key.v & 0x0020) != 0x0020)
//		{
//			Rotate.velCtrl.refvel = 0;
//		}
		if(NPG_dir == 0)
		Rotate.velCtrl.refvel = 150;
		if(NPG_dir == 1)
    Rotate.velCtrl.refvel = -150;
    if(Aim_Mode == AUTO)
		{
			if(Rotate.posCtrl.err > 20)
			{
				Rotate.velCtrl.refvel = 15;
			}
			else if(Rotate.posCtrl.err < -20)
			{
				Rotate.velCtrl.refvel = -15;
			}
			else
			{
				Rotate.velCtrl.refvel = 0;
			}
		}			
		Rotate.velCtrl.rawvel = -(float)GMotor_Yaw.encoderCtrl.speed/60.0f ;
		Rotate_Vel_PID(&Rotate.velCtrl,&rotate_Vel_PID1);
	}
	
		Chassis_ref_Set();	  
  	Set_Chassis_Movement_Sta();

	  Power_Supply_Ctrl();
	
//	if(exstate == _BATTERY)
//	{
//		if(PowerHeatData.chassisPowerBuffer>40)
//		{
//			if(RC_Ctrl_Data.remote.s2 == REMOTE_INPUT){
//					power_max = 80;
//				}else {
//					power_max = (PowerHeatData.chassisPowerBuffer-40)*7+73;
//				}
//		}
//		else
//		{
//			power_max=80;
//		}	
//	}
//	else if(exstate == _CAP)
//		{
//			/* 萇衪肮鼎萇ㄩ癹秶菁攫郔湮髡薹峈250W(忳汔揤耀輸癹秶) */
//			power_max = (cap_volt*10);					//棒撰DC怀郔湮萇霜峈10A
//			power_max = power_max>250 ? 250:(power_max<120 ? 120:power_max);
//														//癹秶Power_Max腔毓峓峈120~250W
//			/* 髡薹豻講假癹秶 */
//			//if(PowerHeatData.chassisPowerBuffer <25)	power_max=80;
//		}
//				Motor_VelPID(&CMotor1.velCtrl);
//				Motor_VelPID(&CMotor2.velCtrl);
//				Motor_VelPID(&CMotor3.velCtrl);
//				Motor_VelPID(&CMotor4.velCtrl);

//			//Power_Control_test();

//		//SuperCapCtrl();//libianyou Power_Ctrl;
//		Power_Ctrl(80);


    if(Ctrl_Chassis_Mode == C_JianLu_MODE){
        /* 80W���ʿ��� ��¼ģʽ */ 
        Power_Ctrl(80);
        
    }else if(Ctrl_Chassis_Mode == C_RACE_MODE){
        /* ����ģʽʹ�õ��� */
        if(exstate == _BATTERY)
					{
            if( Charge_Switch == SWITCH_OFF
            && ( (PowerHeatData.chassisPowerBuffer == 60 
            && PowerHeatData.chassisPower <= 20.0f )
            || Work_Mode == C_LIMIT_MODE ) )
						{
                Work_Mode = C_LIMIT_MODE;
                if(PowerHeatData.chassisPowerBuffer > 40){    //MAX_Power= 213W
                    if(RC_Ctrl_Data.remote.s2 == Check_Mode){
                        power_max = 80;
                    }else {
                        power_max = 80;
                   }
                }else{
                    power_max = 80;
								}
                /* ���ʿ��� + ���PID���� */ 
                //Power_Ctrl(power_max);
            }
						else{
                Set_Zero_Ref();
                Work_Mode = C_STOP_MODE;
                Motor_VelPID(&CMotor1.velCtrl);                 
                Motor_VelPID(&CMotor2.velCtrl);
                Motor_VelPID(&CMotor3.velCtrl);
                Motor_VelPID(&CMotor4.velCtrl);
             }
						Power_Ctrl(power_max);
					 }
				else if(exstate == _CAP){
                Motor_VelPID(&CMotor1.velCtrl);                 
                Motor_VelPID(&CMotor2.velCtrl);
                Motor_VelPID(&CMotor3.velCtrl);
                Motor_VelPID(&CMotor4.velCtrl);
                Work_Mode = C_INFINITE_MODE;

   
    }
     
	}
		CM_CAN_Send_Msg();		
}



