#include "17mm.h"
#include "Strategy.h"
Data_17mm_t Data_17mm;
uint8_t Board_II_Data[26];
void Data_17mm_Set()
{ 
	Data_17mm.SOF1 = 0xA5;
	Data_17mm.SOF2 = 0X5A;
	Data_17mm.MiniPC_17mm.Distance = MiniPC_17mm.Distance;
	Data_17mm.MiniPC_17mm.Pitch = MiniPC_17mm.Pitch;
	Data_17mm.MiniPC_17mm.Yaw = MiniPC_17mm.Yaw;
	Data_17mm.Aim_State_17mm = (uint8_t)Aim_State_17mm;
	Data_17mm.Gimbal_42mm.Yaw = (GMotor_Yaw.posCtrl.accpos-Yaw_Motor_OFFSET)/22.7556f;
	Data_17mm.Gimbal_42mm.Pitch =  GMotor_Pitch.posCtrl.accpos/22.7556f;
	Data_17mm.Shoot_Enable = Shoot_Enable_STA;	
}
void Data_17mm_Build()
{
	Data_17mm_Set();
	memcpy(Board_II_Data,&Data_17mm,24);
	Data_17mm.CRC16 = Get_CRC16_Check_Sum(Board_II_Data ,24, 0xFFFF);
	memcpy(Board_II_Data,&Data_17mm,26);
//  int i;
//  for(i=1 ; i<=25 ;i++)
//  {	
//		Board_II_Data[i] = i;
//	}
}

