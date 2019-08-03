#include "Transmit.h"
#include "DataScope_DP.h"
#include "Shoot_Control.h"
#include "Chassis_Control.h"
#include "Gimbal_Control.h"
#include "can.h"
#include "usart.h"
#include "Referee_Decode.h"
#include "Power_Control.h"
#include "adc.h"
#include "MiniPC.h"
#include "strategy.h"	
#include "17mm.h"
int DataScope_Count = 0;
uint8_t Armor_Chose;
/***
	*函数名称：SM_Can_Send_Msg()
	*函数功能：通过CAN2给四个摩擦轮电机发送数据
	*入口参数：无
	*返回值  ：无
***/
void SM_CAN_Send_Msg(void)
{   	 
	  hcan1.pTxMsg->StdId = 0x200;
		hcan1.pTxMsg->IDE = CAN_ID_STD;
		hcan1.pTxMsg->RTR = CAN_RTR_DATA;
		hcan1.pTxMsg->DLC = 0x08;
		
		hcan1.pTxMsg->Data[0] = (uint8_t)((int16_t)SLMotor.velCtrl.output >> 8);
		hcan1.pTxMsg->Data[1] = (uint8_t)SLMotor.velCtrl.output;
		hcan1.pTxMsg->Data[2] = (uint8_t)((int16_t)SRMotor.velCtrl.output >> 8);
		hcan1.pTxMsg->Data[3] = (uint8_t)SRMotor.velCtrl.output;
		HAL_CAN_Transmit(&hcan1, 10);
}
/***
	*函数名称：BL_Can_Send_Msg
	*函数功能：通过CAN2给拨轮电机发送数据
	*入口参数：无
	*返回值  ：无
***/
void BL_CAN_Send_Msg(void)
{
	  hcan2.pTxMsg->StdId = 0x1FF;
		hcan2.pTxMsg->IDE = CAN_ID_STD;
		hcan2.pTxMsg->RTR = CAN_RTR_DATA;
		hcan2.pTxMsg->DLC = 0x08;
	  if(BLMotor_Work_State == Vel_pid)
		{
			hcan2.pTxMsg->Data[4] = (uint8_t)((int16_t)BLMotor.velCtrl.output >> 8);
			hcan2.pTxMsg->Data[5] = (uint8_t)BLMotor.velCtrl.output;
		}
		else if(BLMotor_Work_State == Pos_pid)
		{
			hcan2.pTxMsg->Data[4] = (uint8_t)((int16_t)BLMotor.posCtrl.output >> 8);
			hcan2.pTxMsg->Data[5] = (uint8_t)BLMotor.posCtrl.output;
		}
		HAL_CAN_Transmit(&hcan2, 10);
		
}

/***
	*函数名称：BP_Can_Send_Msg
	*函数功能：通过CAN2给拨轮电机发送数据
	*入口参数：无
	*返回值  ：无
***/
void BP_CAN_Send_Msg(void)
{
	  hcan2.pTxMsg->StdId = 0x1FF;
		hcan2.pTxMsg->IDE = CAN_ID_STD;
		hcan2.pTxMsg->RTR = CAN_RTR_DATA;
		hcan2.pTxMsg->DLC = 0x08;
		hcan2.pTxMsg->Data[6] = (uint8_t)((int16_t)BPMotor.velCtrl.output >> 8);
		hcan2.pTxMsg->Data[7] = (uint8_t)BPMotor.velCtrl.output;
		HAL_CAN_Transmit(&hcan2, 10);
}

/***
	*函数名称：CM_Can_Send_Msg
	*函数功能：通过CAN1给四个底盘电机发送
	*入口参数：无
	*返回值  ：无
***/
void CM_CAN_Send_Msg(void)
{
	  hcan1.pTxMsg->StdId = 0x1FF;
		hcan1.pTxMsg->IDE = CAN_ID_STD;
		hcan1.pTxMsg->RTR = CAN_RTR_DATA;
		hcan1.pTxMsg->DLC = 0x08;
		
		hcan1.pTxMsg->Data[0] = (uint8_t)((int16_t)CMotor1.velCtrl.output >> 8);
		hcan1.pTxMsg->Data[1] = (uint8_t)CMotor1.velCtrl.output;
		hcan1.pTxMsg->Data[2] = (uint8_t)((int16_t)CMotor2.velCtrl.output >> 8);
		hcan1.pTxMsg->Data[3] = (uint8_t)CMotor2.velCtrl.output;
		hcan1.pTxMsg->Data[4] = (uint8_t)((int16_t)CMotor3.velCtrl.output >> 8);
		hcan1.pTxMsg->Data[5] = (uint8_t)CMotor3.velCtrl.output;
		hcan1.pTxMsg->Data[6] = (uint8_t)((int16_t)CMotor4.velCtrl.output >> 8);
		hcan1.pTxMsg->Data[7] = (uint8_t)CMotor4.velCtrl.output;
	
		HAL_CAN_Transmit(&hcan1, 10);
}

/***
	*函数名称：GM_Can_Send_Msg
    *函数功能: 给云台电机发送数据
	*入口参数：无
	*返回值  ：无
***/
void GM_CAN_Send_Msg(void)
{

	  hcan2.pTxMsg->StdId = 0x1FF;
		hcan2.pTxMsg->IDE = CAN_ID_STD;
		hcan2.pTxMsg->RTR = CAN_RTR_DATA;
		hcan2.pTxMsg->DLC = 0x08;
		
//		hcan2.pTxMsg->Data[0] = 0;
//		hcan2.pTxMsg->Data[1] = 0;
		hcan2.pTxMsg->Data[0] = (uint8_t)((int16_t)GMotor_Yaw.velCtrl.output >> 8);
		hcan2.pTxMsg->Data[1] = (uint8_t)GMotor_Yaw.velCtrl.output;
		hcan2.pTxMsg->Data[2] = -(uint8_t)((int16_t)GMotor_Pitch.velCtrl.output >> 8);
		hcan2.pTxMsg->Data[3] = -(uint8_t)GMotor_Pitch.velCtrl.output;
	
	  HAL_CAN_Transmit(&hcan2, 10);

}


/***
	*函数名称：DataScope_Transmit
	*函数功能: 虚拟示波器发送
	*入口参数：无
	*返回值  ：无
***/
void DataScope_Transmit()
{	   
		float i;
	   DataScope_Count++;
	    if(DataScope_Count==4)
		{
			i = DataScope_Data_Generate(5);
			DataScope_Get_Channel_Data(BPMotor.velCtrl.rawvel,1);		
			DataScope_Get_Channel_Data(PowerHeatData.shooter_heat1,2);
//			DataScope_Get_Channel_Data(my_heat_rest,3);
			DataScope_Get_Channel_Data(heat_rest,4);
			DataScope_Get_Channel_Data(last_heat_rest,5);			
			//DataScope_Get_Channel_Data(total_power[0],6);
	
			HAL_UART_Transmit(&huart2,DataScope_OutPut_Buffer,i,100);
			DataScope_Count=0;
		}
}

void Data_17mm_Transmit()
{
	Data_17mm_Build();
	HAL_UART_Transmit(&huart8,Board_II_Data,26,50);
}

void Client_Transmit()
{	

  data_head_init();

	Client_Data_Build();
	
//	for (i = 0; i < 16; i++)
//	{
		HAL_UART_Transmit(&huart6, Client_Data , 28, 10);
//	}
}
uint8_t Mini_PC_Data[12];
FormatTrans16 Mini_PC_16Trans;
FormatTrans32 Mini_PC_32Trans;
void Mini_PC_Transmit()
{
	Mini_PC_Data[0] = 0xA5;
	Mini_PC_Data[1] = 0x5A;
	Mini_PC_16Trans.u16_temp = Bullet_Num;
	Mini_PC_Data[2] = Mini_PC_16Trans.u8_temp[0];
  Mini_PC_Data[3] = Mini_PC_16Trans.u8_temp[1];
	Mini_PC_Data[4] = Armor_Chose;
//	Mini_PC_32Trans.float_temp = IMU_acc - Yaw_IMU_OFFSET;
//	Mini_PC_Data[4] = Mini_PC_32Trans.u8_temp[0];
//	Mini_PC_Data[5] = Mini_PC_32Trans.u8_temp[1];
//	Mini_PC_Data[6] = Mini_PC_32Trans.u8_temp[2];
//	Mini_PC_Data[7] = Mini_PC_32Trans.u8_temp[3];
//	Mini_PC_32Trans.float_temp = Pitch_Motor_OFFSET-(float)GMotor_Pitch.posCtrl.accpos;
//	Mini_PC_Data[8] = Mini_PC_32Trans.u8_temp[0];
//	Mini_PC_Data[9] = Mini_PC_32Trans.u8_temp[1];
//	Mini_PC_Data[10] = Mini_PC_32Trans.u8_temp[2];
//	Mini_PC_Data[11] = Mini_PC_32Trans.u8_temp[3];
	HAL_UART_Transmit(&huart3,Mini_PC_Data,5,20);
}
