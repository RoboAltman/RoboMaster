#include "Receive.h"
#include "IMU.h"
#include "usart.h"
#include "Referee_Decode.h"
#include "Chassis_Control.h"
#include "Shoot_Control.h"
#include "Gimbal_Control.h"
#include "Power_Control.h"
#include "MiniPC.h"
#include "adc.h"
int16_t Mem;
uint8_t USART1_DMA_RX_BUF[BSP_USART1_DMA_RX_BUF_LEN];  //定义一个数组用于存放从DMA接收到的遥控器数据
uint8_t USART6_DMA_RX_BUF[BSP_USART6_DMA_RX_BUF_LEN];  //定义一个数组用于存放从DMA接收到的裁判系统数据
uint8_t USART8_DMA_RX_BUF[BSP_USART8_DMA_RX_BUF_LEN];
uint8_t USART3_DMA_RX_BUF[BSP_USART3_DMA_RX_BUF_LEN];
uint8_t USART7_DMA_RX_BUF[BSP_USART7_DMA_RX_BUF_LEN];
/***
	*函数名称：CMotor_Decode
	*函数功能：发射机构数据解码
	*入口参数：CAN接收到的摩擦轮电机反馈，电机名称
	*返回值  ：无
***/
void CMotor_Decode(CanRxMsgTypeDef* msg,Motor_t* motor)
{
	    motor->encoderCtrl.speed = (uint16_t)msg->Data[2] << 8 |(uint16_t)msg->Data[3];
	    motor->velCtrl.rawvel = (float)motor->encoderCtrl.speed; 
      motor->encoderCtrl.current = (uint16_t)msg->Data[4]<<8 |(uint16_t)msg->Data[5];	
}
/***
	*函数名称：SM_Decode
	*函数功能：发射机构数据解码
	*入口参数：CAN接收到的摩擦轮电机反馈，电机名称
	*返回值  ：无
***/
float gama=0.2;
float Shoot_Filter_Left(float input,float Filtet_parameters)
{
	static float lastoutput;
	float output;
	output = (1 - Filtet_parameters) * input + Filtet_parameters * lastoutput;
  lastoutput = output;
	return output;	
}

float Shoot_Filter_Right(float input,float Filtet_parameters)
{
	static float lastoutput;
	float output;
	output = (1 - Filtet_parameters) * input + Filtet_parameters * lastoutput;
  lastoutput = output;
	return output;	
}

void SM_Decode(CanRxMsgTypeDef* msg,Motor_t* motor)
{     
      motor->velCtrl.lastrawvel = motor->velCtrl.rawvel;
			motor->encoderCtrl.angle = (uint16_t)msg->Data[0] << 8 |(uint16_t) msg->Data[1];
	    motor->posCtrl.rawpos = (float)motor->encoderCtrl.angle;
		  motor->encoderCtrl.speed = (uint16_t)msg->Data[2] << 8 |(uint16_t) msg->Data[3];
	    if(motor == &SLMotor)
			motor->velCtrl.rawvel = Shoot_Filter_Left(motor->encoderCtrl.speed,0.92);
			else if(motor == &SRMotor)
			motor->velCtrl.rawvel = Shoot_Filter_Right(motor->encoderCtrl.speed,0.92);
			if(motor->velCtrl.rawvel > 0)
	    motor->posCtrl.accpos = motor->velCtrl.rawvel - Speed_42mm;
			else if(motor->velCtrl.rawvel < 0)
			motor->posCtrl.accpos = motor->velCtrl.rawvel + Speed_42mm;
}

float last_accpos;
void BP_Decode(CanRxMsgTypeDef* msg,Motor_t* motor)
{
			float diff;
			motor->posCtrl.lastrawpos = motor->encoderCtrl.angle;
		  motor->encoderCtrl.angle = (uint16_t) msg->Data[0] << 8 |(uint16_t) msg->Data[1];
		  motor->encoderCtrl.speed = (uint16_t) msg->Data[2] << 8 |(uint16_t) msg->Data[3];
	    motor->velCtrl.rawvel = motor->encoderCtrl.speed;
			diff = motor->encoderCtrl.angle - motor-> posCtrl.lastrawpos;
	    if(diff < -6000)
			{
					motor->posCtrl.round++;
			}
			else if(diff > 6000)
			{
					motor->posCtrl.round--;
			}
			motor->posCtrl.accpos = motor->posCtrl.round * 8192 + motor->encoderCtrl.angle;
}

/***
	*函数名称：GMotor_Decode
    *函数功能: 云台数据解码
	*入口参数：CAN接收到的摩擦轮电机反馈，电机名称
	*返回值  ：无
***/
//void GMotor_Pitch_Decode(CanRxMsgTypeDef* msg,Motor_t *motor)
//{
//			float diff;
//	    motor->posCtrl.lastrawpos = motor->encoderCtrl.angle;
//     	motor->encoderCtrl.angle = (int16_t) msg->Data[0] << 8 |(int16_t) msg->Data[1];
//	    motor->encoderCtrl.current = (int16_t) msg->Data[2] << 8 |(int16_t) msg->Data[3];
//	    diff = motor->encoderCtrl.angle - motor-> posCtrl.lastrawpos;
//      if(diff < -6000)
//			{
//					motor->posCtrl.round++;
//			}
//			else if(diff > 6000)
//			{
//					motor->posCtrl.round--;
//			}
//			motor->posCtrl.accpos = (motor->posCtrl.round * 8192 + motor->encoderCtrl.angle)/22.7556f;
//}



/***
	*函数名称：GMotor_Decode
    *函数功能: 云台数据解码
	*入口参数：CAN接收到的摩擦轮电机反馈，电机名称
	*返回值  ：无
***/
void GMotor_Decode(CanRxMsgTypeDef* msg,Motor_t *motor)
{
			float diff;
	    motor->encoderCtrl.angle   = (uint16_t) msg->Data[0] << 8 |(uint16_t) msg->Data[1];
	    motor->encoderCtrl.speed   = (uint16_t) msg->Data[2] << 8 |(uint16_t) msg->Data[3];
	    motor->encoderCtrl.current = (int16_t) msg->Data[4] << 8 |(int16_t) msg->Data[5];
	    //motor->posCtrl.rawpos      = (float)motor->encoderCtrl.angle/22.7556f;
	    //motor->velCtrl.rawvel      = (float)motor->encoderCtrl.speed/360.0f/60.0f;
	    diff = motor->encoderCtrl.angle - motor->encoderCtrl.lastangle;
	    if(diff<-6000)
		  {
					motor->posCtrl.round++;
		  }
		  else if(diff>7500)
			{
					motor->posCtrl.round--;
			}
			motor->posCtrl.accpos =(motor->posCtrl.round  * 8192 + motor->encoderCtrl.angle);
		  motor->encoderCtrl.lastangle  = motor->encoderCtrl.angle;
//	  motor->velCtrl.lastrawvel  = motor->velCtrl.rawvel;
			if(motor == &GMotor_Pitch)
			{
				if(motor->encoderCtrl.angle >= 4096)
				motor->posCtrl.accpos = 8192 - motor->encoderCtrl.angle;
				else
				motor->posCtrl.accpos = -motor->encoderCtrl.angle;
			}
}
/***
	*函数名称：BL_Decode
	*函数功能：拨弹数据解码
	*入口参数：CAN接收到的摩擦轮电机反馈，电机名称
	*返回值  ：无
***/
void BL_Decode(CanRxMsgTypeDef* msg,Motor_t* motor)
{       
			float diff;
	    motor->posCtrl.lastrawpos  = motor->posCtrl.rawpos;
	    motor->encoderCtrl.angle   = (uint16_t) msg->Data[0] << 8 |(uint16_t) msg->Data[1];
	    motor->encoderCtrl.speed   = (uint16_t) msg->Data[2] << 8 |(uint16_t) msg->Data[3];
	    motor->encoderCtrl.current = (int16_t) msg->Data[4] << 8 |(int16_t) msg->Data[5];
	    motor->posCtrl.rawpos      = motor->encoderCtrl.angle;
	    motor->velCtrl.rawvel      = motor->encoderCtrl.speed;
	    diff = motor->posCtrl.rawpos - motor-> posCtrl.lastrawpos;
	    if(diff<-6000)
		  {
					motor->posCtrl.round++;
		  }
		  else if(diff>6000)
			{
					motor->posCtrl.round--;
			}
			motor->posCtrl.accpos = (motor->posCtrl.round * 8192 + motor->encoderCtrl.angle);
}
/***
	*函数名称：CAN_Receive_Msg
	*函数功能：CAN接收
	*入口参数：CAN句柄
	*返回值  ：无
***/
void CAN_Receive_Msg(CAN_HandleTypeDef* hcan)
{
		if(hcan==&hcan1)
		{
				switch(hcan1.pRxMsg->StdId)
				{
					case  0x201:
					{
						SM_Decode(hcan1.pRxMsg,&SLMotor);
						break;
					}
					case  0x202:
					{
						SM_Decode(hcan1.pRxMsg,&SRMotor);
						break;
					}
					case  0x205:
					{
						CMotor_Decode(hcan1.pRxMsg,&CMotor1);
						break;
					}
					case  0x206:
					{
						CMotor_Decode(hcan1.pRxMsg,&CMotor2);
						break;
					}
					case  0x207:
					{
						CMotor_Decode(hcan1.pRxMsg,&CMotor3);
						break;
					}
					case  0x208:
					{
						CMotor_Decode(hcan1.pRxMsg,&CMotor4);
						break;
					}

			  }
		}
		else if(hcan==&hcan2)
		{
			switch(hcan2.pRxMsg->StdId)
			{
				case  0x205:
				{
					GMotor_Decode(hcan2.pRxMsg,&GMotor_Yaw);
					break;
				}
				case  0x206:
				{
					GMotor_Decode(hcan2.pRxMsg,&GMotor_Pitch);
					break;
				}
				case  0x208:
				{
					BP_Decode(hcan2.pRxMsg,&BPMotor);		
					break;
				}
				case  0x207:
				{
					BL_Decode(hcan2.pRxMsg,&BLMotor);
					break;
				}
			}
				
		}
}
/***
	*函数名称：Referee_Data_Receive
	*函数功能：裁判系统接收
	*入口参数：无
	*返回值  ：无
***/

void Referee_Data_Receive(void)
{
		if((__HAL_UART_GET_FLAG(&huart6,UART_FLAG_IDLE)!=RESET)) 
		{
			__HAL_UART_CLEAR_IDLEFLAG(&huart6);
			__HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_FLAG_TCIF1_5);
			HAL_UART_DMAStop(&huart6);
			HAL_UART_Receive_DMA(&huart6, USART6_DMA_RX_BUF, BSP_USART6_DMA_RX_BUF_LEN);
			RefereeDecode(USART6_DMA_RX_BUF);
		}
}

/***
	*函数名称：RemotreCtl_Data_Receive
	*函数功能：接收遥控器数据，位于USART1的中断函数中
	*入口参数：无
	*返回值  ：无
***/
void RemotreCtl_Data_Receive(void)
{
		uint32_t rx_data_len = 0;
	                                                          //本次接收长度
		if((__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE)!=RESET)) 
	   {
				__HAL_UART_CLEAR_IDLEFLAG(&huart1);                                           //清除空闲中断的标
				__HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx,DMA_FLAG_TCIF2_6);                       //清除 DMA2_Steam2传输完成标志
			  //__HAL_UART_CLEAR_FLAG(&huart1,UART_FLAG_FE);
				HAL_UART_DMAStop(&huart1);                                                    //传输完成以后关闭串口DMA
				rx_data_len = BSP_USART1_DMA_RX_BUF_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx); //获取这一次数据量大小（总长度-保留的长度）
				HAL_UART_Receive_DMA(&huart1, USART1_DMA_RX_BUF, BSP_USART1_DMA_RX_BUF_LEN);  //接受数据
         if(rx_data_len == 18)                                                        //判断数据是否为正确的数据长度
			 {
					RC_DataHandle(USART1_DMA_RX_BUF);                                           //进入数据解码函数
			 }
		 }
}

/***
	*函数名称：RemotreCtl_Data_Receive
	*函数功能：接收遥控器数据，位于USART1的中断函数中
	*入口参数：无
	*返回值  ：无
***/


void HI_IMU_Data_Receive(void)
{                                                          //本次接收长度
		if((__HAL_UART_GET_FLAG(&huart7,UART_FLAG_IDLE)!=RESET)) 
	   {
				__HAL_UART_CLEAR_IDLEFLAG(&huart7);                                           //清除空闲中断的标志
				__HAL_DMA_CLEAR_FLAG(&hdma_uart7_rx,DMA_FLAG_TCIF3_7);                        //清除 DMA2_Steam2传输完成标志
				HAL_UART_DMAStop(&huart7);                                                    //
				HAL_UART_Receive_DMA(&huart7, USART7_DMA_RX_BUF, BSP_USART7_DMA_RX_BUF_LEN);  //接受数据
        HI_IMU_Data_Decode(USART7_DMA_RX_BUF); 
			  //Mini_PC_Transmit();  			 //进入数据解码函
		 }
}

void Mini_PC_Data_Receive(void)
{                                                          //本次接收长度
		if((__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE)!=RESET)) 
	   {
				__HAL_UART_CLEAR_IDLEFLAG(&huart3);                                           //清除空闲中断的标志
				__HAL_DMA_CLEAR_FLAG(&hdma_usart3_rx,DMA_FLAG_TCIF1_5);                        //清除 DMA2_Steam2传输完成标志
				HAL_UART_DMAStop(&huart3);                                                    //
				HAL_UART_Receive_DMA(&huart3, USART3_DMA_RX_BUF, BSP_USART3_DMA_RX_BUF_LEN);  //接受数据
        Mini_PC_Data_Decode(USART3_DMA_RX_BUF);                                        //进入数据解码函
		 }
}
/***
	*函数名称：RemotreCtl_Data_Receive_Start
	*函数功能：接收遥控器数据，位于USART1的中断函数中
	*入口参数：无
	*返回值  ：无
***/
void RemotreCtl_Data_Receive_Start(void)
{
	  __HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);                                       //开启不定长中断
		HAL_UART_Receive_DMA(&huart1,USART1_DMA_RX_BUF, BSP_USART1_DMA_RX_BUF_LEN);
}
/***
	*函数名称：Referee_Data_Receive_Start
	*函数功能: 开启裁判系统数据接收中断
	*入口参数：无
	*返回值  ：无
***/
void Referee_Data_Receive_Start(void)
{
	  __HAL_UART_ENABLE_IT(&huart6,UART_IT_IDLE);                                       //开启不定长中断
		HAL_UART_Receive_DMA(&huart6,USART6_DMA_RX_BUF, BSP_USART6_DMA_RX_BUF_LEN);
}
/***
	*函数名称：IMU_Data_Receive_Start
	*函数功能: 开启IMU数据接收中断
	*入口参数：无
	*返回值  ：无
***/
void IMU_Data_Receive_Start(void)
{
	  __HAL_UART_ENABLE_IT(&huart8,UART_IT_IDLE);                                       //开启不定长中断
		HAL_UART_Receive_DMA(&huart8,USART8_DMA_RX_BUF, BSP_USART8_DMA_RX_BUF_LEN);
		__HAL_UART_ENABLE_IT(&huart7,UART_IT_IDLE);                                       //开启不定长中断
		HAL_UART_Receive_DMA(&huart7,USART7_DMA_RX_BUF, BSP_USART7_DMA_RX_BUF_LEN);
}

void Mini_PC_Receive_Start(void)
{
	  __HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart3,USART3_DMA_RX_BUF, BSP_USART3_DMA_RX_BUF_LEN);	
}

