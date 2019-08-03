#include "Remote_Control.h"
#include "Shoot_Control.h"
#include "Power_Control.h"
#include "Gimbal_Control.h"
#include "Receive.h"
#include "Power_Control.h"
#include "Chassis_Control.h"
#include "Referee_Decode.h"
#include "IMU.h"
#include "math.h"
#include "MiniPC.h"
#include "tim.h"
ChassisRef_t ChassisRef;
RC_Ctrl_t RC_Ctrl_Data;
Control_Mode_e Control_Mode;
Control_Mode_e Last_Control_Mode;
Power_Mode_e Power_Mode;
Rotate_State_e Rotate_State;
uint8_t Shoot_Enable_STA;
int press_l_flag;
int press_r_flag;
int last_press_l;
int last_press_r;
int last_key_state;
int last_s1_state;
int last_s2_state;
int Rotate_State_Flag;
int Shoot_Flag;
int strategy_flag;
int NPG_dir;
int Shoot_Request_Flag;
int Rotate_Limit = 5;
int clear_flag;


/****************************************************************************

****************************************************************************/
void RC_DataHandle(uint8_t *pData)
{
	if(pData == NULL)
    {
        return;
    }
		/*pData[0]Ϊch0�ĵ�8λ��Data[1]�ĵ�3λch0�ĸ�3λ*/
	    RC_Ctrl_Data.remote.ch0 = ((uint16_t)pData[0] | (uint16_t)pData[1] << 8) & 0x07FF;
		
		/*pData[1]�ĸ�5λΪch1�ĵ�5λ��pData[2]�ĵ�6λΪch1�ĸ�6λ*/
		RC_Ctrl_Data.remote.ch1 = ((uint16_t)pData[1] >> 3 | (uint16_t)pData[2] << 5) & 0x07FF;
		
		/*pData[2]�ĸ�2λΪch2�ĵ�2λ, pData[3]Ϊch2����8λ��pData[4]�ĵ�1λΪch2�ĸ�1λ*/
		RC_Ctrl_Data.remote.ch2 = ((uint16_t)pData[2] >> 6 | (uint16_t)pData[3] << 2 | (uint16_t)pData[4] << 10) & 0x07FF;
		
		/*pData[4]�ĸ�7λΪch3�ĵ�7λ��pData[5]�ĵ�4λΪch3�ĸ�4λ*/
		RC_Ctrl_Data.remote.ch3 = ((uint16_t)pData[4] >> 1 | (uint16_t)pData[5] << 7) & 0x07FF;

		/*pData[5]�ĸ�2λΪs1*/
		RC_Ctrl_Data.remote.s1  = ((pData[5] >> 6) & 0x03);
		
		/*pData[6]��6��7λΪs2*/
		RC_Ctrl_Data.remote.s2  = ((pData[5] >> 4) & 0x03);
		
		/*pData[6],pData[7]Ϊx*/
		RC_Ctrl_Data.mouse.x    = ((int16_t)pData[6] | (int16_t)pData[7] << 8);
		
		/*pData[8],pData[9]Ϊy*/
		RC_Ctrl_Data.mouse.y    = ((int16_t)pData[8] | (int16_t)pData[9] << 8);
		
		/*pData[10],pData[11]Ϊz*/
		RC_Ctrl_Data.mouse.z    = ((int16_t)pData[10] | (int16_t)pData[11] << 8);
		
		/*pData[12]Ϊ���*/
		RC_Ctrl_Data.mouse.press_l = pData[12];
		
		/*pData[13]Ϊ�Ҽ�*/
		RC_Ctrl_Data.mouse.press_r = pData[13];
		
		/*pData[14],pData[15]Ϊ����ֵ*/
		RC_Ctrl_Data.key.v      = ((int16_t)pData[14]) | ((int16_t)pData[15] << 8);

	    /*�ж��������Ͳ�����ת��Ϊ������*/
		Remode_Input_Judge();
}



void Remote_Process()
{
		ChassisRef.forward_back_ref = (RC_Ctrl_Data.remote.ch1 - (int16_t)REMOTE_STICK_OFFSET);
		ChassisRef.left_right_ref   = -(RC_Ctrl_Data.remote.ch0 - (int16_t)REMOTE_STICK_OFFSET);	
	  ChassisRef.rotate_ref       = (RC_Ctrl_Data.remote.ch2 - (int16_t)REMOTE_STICK_OFFSET);
		if(RC_Ctrl_Data.remote.s2 == Check_Mode)
		{
			ChassisRef.forward_back_ref = ChassisRef.forward_back_ref * 0.4f;
			ChassisRef.left_right_ref   = ChassisRef.left_right_ref * 0.4f;	
			ChassisRef.rotate_ref       = ChassisRef.rotate_ref * 0.4f;
		}
		GimbalRef.pitch_angle_ref   += (RC_Ctrl_Data.remote.ch3 - (int16_t)REMOTE_STICK_OFFSET) * REMOTE_PITCH_ANGLE_TO_REF;
	  //GimbalRef.yaw_angle_ref     += (RC_Ctrl_Data.remote.ch2 - (int16_t)REMOTE_STICK_OFFSET) * REMOTE_YAW_ANGLE_TO_REF;
		Bullet_Num_Init = Init_OK;
    if(RC_Ctrl_Data.remote.s1 == 1 && last_s1_state == 3 && Bullet_Num >= 0)
		{
			Shoot_Mode = Single;
			Shoot_Request_Flag = 1;
		}
		if(RC_Ctrl_Data.remote.s1 != 2 && RC_Ctrl_Data.remote.s2 == REMOTE_INPUT)
		{
			SMotor_State = Motor_ON;
		}
		else
		{
			SMotor_State = Motor_OFF;
		}
		
		//小发射机构控制
		if(RC_Ctrl_Data.remote.s2 == Check_Mode)
		{
			if( RC_Ctrl_Data.remote.s1 == 1)
			{
				Shoot_Enable_STA = Shoot_Enable_STA|0x04;
				Shoot_Enable_STA = Shoot_Enable_STA&0xF7;
			}
			else if( RC_Ctrl_Data.remote.s1 == 3)
			{
				Shoot_Enable_STA = Shoot_Enable_STA|0x04;
				Shoot_Enable_STA = Shoot_Enable_STA|0x08;
			}
			else
			{
				Shoot_Enable_STA = Shoot_Enable_STA&0xF3;//11110011
			}
		}
		else
		{
			Shoot_Enable_STA = Shoot_Enable_STA&0xF3;//11110011
		}
		
		
		
		last_s1_state = RC_Ctrl_Data.remote.s1;
		last_s2_state = RC_Ctrl_Data.remote.s2;
}

int rotate_flag;
int NPG_flag = 0;
void Key_Mouse_Process()
{
//		if((RC_Ctrl_Data.key.v & 0x8000) == 0x8000 && (last_key_state & 0x8000) != 0x8000)//B
//		{
//			if(Follow_Mode == Chassis_Follow_Gimbal)
//			{
//				Follow_Mode = Gimbal_Follow_Chassis;
//			}
//			else if(Follow_Mode == Gimbal_Follow_Chassis)
//			{
//				Follow_Mode = Chassis_Follow_Gimbal;
//			}
//		}
	  int cap_flag;
//		if((RC_Ctrl_Data.key.v&0x0010)==0x0010)//shift���£�����ģʽ
//		{
//			REMOTE_CHASIS_SPEED_TO_REF_FB = 22.0f;
//			REMOTE_CHASIS_SPEED_TO_REF_LR = 22.0f;
//			
//		}
//		else
//		{
			REMOTE_CHASIS_SPEED_TO_REF_FB = 12.0f;
			REMOTE_CHASIS_SPEED_TO_REF_LR = 12.0f;
//		}

		if((RC_Ctrl_Data.key.v&1)==1&&((RC_Ctrl_Data.key.v&0x0010)!=0x0010))//W
		{	
				ChassisRef.forward_back_ref += 7.5;
				if( ChassisRef.forward_back_ref >= 350)
				ChassisRef.forward_back_ref =350;
		}
		else if((RC_Ctrl_Data.key.v&2)==2&&((RC_Ctrl_Data.key.v&0x0010)!=0x0010))//S
		{
				ChassisRef.forward_back_ref -= 7.5;
				if( ChassisRef.forward_back_ref <= -350)
				ChassisRef.forward_back_ref = -350; 
		}
		else if((RC_Ctrl_Data.key.v&1)==1&&((RC_Ctrl_Data.key.v&0x0010)==0x0010))
		{
			ChassisRef.forward_back_ref+=10;	
			if(ChassisRef.forward_back_ref>=700)
			ChassisRef.forward_back_ref = 700;
		}
		else if((RC_Ctrl_Data.key.v&2)==2&&((RC_Ctrl_Data.key.v&0x0010)==0x0010))
		{
			ChassisRef.forward_back_ref-=10;
 			if(ChassisRef.forward_back_ref<=-660)
			ChassisRef.forward_back_ref = -660;			
		}		
		else
		{		  
				ChassisRef.forward_back_ref = 0;
		}
		
		if(Follow_Mode == Chassis_Follow_Gimbal)
		{
			if((RC_Ctrl_Data.key.v&4)==4&&((RC_Ctrl_Data.key.v&0x0010)!=0x0010))//A
			{ 
				ChassisRef.left_right_ref -= 6;
				if( ChassisRef.left_right_ref <= -300)
				ChassisRef.left_right_ref = -300;				
			}
			else if((RC_Ctrl_Data.key.v&8)==8&&((RC_Ctrl_Data.key.v&0x0010)!=0x0010))//D
			{
				ChassisRef.left_right_ref += 6;
				if( ChassisRef.left_right_ref >= 300)
				ChassisRef.left_right_ref = 300;						
			}
			else if((RC_Ctrl_Data.key.v&4)==4&&((RC_Ctrl_Data.key.v&0x0010)==0x0010))//A
			{
				ChassisRef.left_right_ref  -= 7.5;	
				if(ChassisRef.left_right_ref<=-350)
				ChassisRef.left_right_ref = -350;   										
			}
			else if((RC_Ctrl_Data.key.v&8)==8&&((RC_Ctrl_Data.key.v&0x0010)==0x0010))//D
			{
        ChassisRef.left_right_ref  += 7.5;
				if(ChassisRef.left_right_ref>= 350)
				ChassisRef.left_right_ref = 350;
			}
			else 
			{
				if(ChassisRef.left_right_ref > 0)
				{
					ChassisRef.left_right_ref -= 20;
					if(ChassisRef.left_right_ref <= 0)
					ChassisRef.left_right_ref = 0;	
				}
				else if(ChassisRef.left_right_ref < 0)
				{
					ChassisRef.left_right_ref += 20;
					if(ChassisRef.left_right_ref >= 0)
					ChassisRef.left_right_ref = 0; 
				}
				else
				{
					ChassisRef.left_right_ref = 0;
				}
			}
		
	}
		
		GimbalRef.pitch_angle_ref += -0.05f * RC_Ctrl_Data.mouse.y;
		GimbalRef.yaw_angle_ref   += 0.1f * RC_Ctrl_Data.mouse.x;

//预测控制
	if((RC_Ctrl_Data.key.v&0x0200) == 0x0200&&(last_key_state&0x0200) != 0x0200)//F
	{
		Forecast_Mode = Infantry_Mode;
	}
	if((RC_Ctrl_Data.key.v&0x0100) == 0x0100&&(last_key_state&0x0100) != 0x0100)//R
	{
		Forecast_Mode = Sentry_Mode;
	}
	if((RC_Ctrl_Data.key.v&0x4000) == 0x4000&&(last_key_state&0x4000) != 0x4000)//V
	{
		Forecast_Mode = NO_Forecast;
	}
//小发射机构控制
	if((RC_Ctrl_Data.key.v&0x0080) == 0x0080&&(last_key_state&0x0080) != 0x0080)//E
	{
		Shoot_Enable_STA = Shoot_Enable_STA|0X01;
	}
	if((RC_Ctrl_Data.key.v&0x2000) == 0x2000&&(last_key_state&0x2000) != 0x2000)//C
	{
		Shoot_Enable_STA = Shoot_Enable_STA&0xFE;
	}

//       if((Shoot_Enable_STA&0x01) == 0x01)
//			 {
//				 Shoot_Enable_STA = Shoot_Enable_STA&0xFE;
//			 }
//			 else if((Shoot_Enable_STA&0x01) != 0x01)
//			 {
//				 Shoot_Enable_STA = Shoot_Enable_STA|0X01;
//			 }
	
//子弹数清零
	if((RC_Ctrl_Data.key.v&0x8000) == 0x8000)//B Bullet_Num = 0�F
	{
		clear_flag++;
		if(clear_flag > 200)
		{
			Bullet_Num = 0;
			clear_flag = 0;
		}
	}
	else
	{
		clear_flag = 0;
	}
	
	if((RC_Ctrl_Data.key.v&0x0800) == 0x0800)//Z
	{
		Armor_Chose = 1;
	}
	if((RC_Ctrl_Data.key.v&0x1000) == 0x1000)//X
	{
		Armor_Chose = 0;
	}
		
//摩擦轮控制		
	if((RC_Ctrl_Data.key.v&0x0400) == 0x0400 && (last_key_state&0x0400) != 0x0400)//G
	{
		if(SMotor_State == Motor_OFF)
	  {
			SMotor_State = Motor_ON;
		}
		else if(SMotor_State == Motor_ON)
		{ 
		  SMotor_State = Motor_OFF;
		}
	}	

	if(RC_Ctrl_Data.mouse.press_l == 1 && last_press_l == 0 )//击发
	{
		Shoot_Mode = Single;
 		Shoot_Request_Flag = 1;
	}


//	if((RC_Ctrl_Data.key.v & 0x2000) == 0x2000&&(RC_Ctrl_Data.key.v &0x0020) == 0x0020&&((last_key_state& 0x2000)!=0x2000||(RC_Ctrl_Data.key.v &0x0020) != 0x0020))//c ctrl
//	{
//		GimbalRef.yaw_angle_ref += 90;
//	}
		

	if(RC_Ctrl_Data.mouse.press_r == 1)
	{
		Aim_Mode = AUTO;
		Rotate_Limit = 30;
	}
	else
	{
		Aim_Mode = NOAUTO;
		Rotate_Limit = 5;
	}

	if((RC_Ctrl_Data.key.v & 0x0020) == 0x0020)//Ctrl
		{
			if(NPG_flag == 0)
			{
				NPG_dir = 0;
				NPG_flag = 1;
			}
			if(NPG_flag == 1)
			{
				if(((float)(Yaw_Motor_OFFSET-GMotor_Yaw.posCtrl.accpos))/22.7556f < -42)
        NPG_dir = 0;
				if(((float)(Yaw_Motor_OFFSET-GMotor_Yaw.posCtrl.accpos))/22.7556f >  42)
				NPG_dir = 1;
			}
			
		}
    else
		{
			NPG_flag = 0;
			Rotate.posCtrl.refpos = 0;
			NPG_dir = 2;
		}
		if((Follow_Mode == Chassis_Follow_Gimbal&&(__fabs((float)(Yaw_Motor_OFFSET - GMotor_Yaw.posCtrl.accpos))/22.7556f > Rotate_Limit))|| NPG_flag==1||Aim_Mode==AUTO)
		{
			ChassisRef.rotate_ref = Rotate.velCtrl.output;
		}
		else 
		{
			ChassisRef.rotate_ref = 0;
		}
	 
		if((RC_Ctrl_Data.key.v&0x4000) == 0x4000&&(last_key_state&0x4000)!=0x4000)//V
		{

		}
		
		
		last_press_l = RC_Ctrl_Data.mouse.press_l;
		last_press_r = RC_Ctrl_Data.mouse.press_r;
	  last_key_state = RC_Ctrl_Data.key.v;

}

void Stop_Process()
{
	ChassisRef.left_right_ref = 0;
	ChassisRef.forward_back_ref = 0;
	ChassisRef.rotate_ref=0;
	
}

void Control_Init()
{
	Stop_Process();
}


/***
	*�������ƣ�Remode_Input_Judge
	*�������ܣ��жϿ����ź���������
	*��ڲ�������
	*����ֵ  ����
***/
void Remode_Input_Judge(void)
{
	 switch(RC_Ctrl_Data.remote.s2)
	 {
		 case REMOTE_INPUT:      
		 {
			 Remote_Process();
			 //Gimbal_Limit();
			 //CAP_AUTO_CTRL=CAP_MANUAL;
			 //Power_Mode = Cap;
			 //if((RC_Ctrl_Data.key.v & 0x4000) != 0x4000)
			 Control_Mode = Remote;
       Ctrl_Chassis_Mode = C_RACE_MODE;

			 break;
		 }
		 case KEY_MOUSE_INPUT:   
		 {
			 Key_Mouse_Process();
			 //Gimbal_Limit();
		   //CAP_AUTO_CTRL = CAP_MANUAL;
			 Control_Mode = Mouse_Board;
			 Ctrl_Chassis_Mode = C_RACE_MODE;

			 break;
		 }		
		 case Check_Mode:              
		 {
			 //Gimbal_Limit();
			 Remote_Process();
			 Control_Mode = Remote;
			 //CAP_AUTO_CTRL = CAP_MANUAL;
			 //Power_Mode = Battery;
			 break;
       Ctrl_Chassis_Mode = C_JianLu_MODE;  
		 }
	 }
}
