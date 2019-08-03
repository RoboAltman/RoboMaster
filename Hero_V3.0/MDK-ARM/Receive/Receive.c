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
uint8_t USART1_DMA_RX_BUF[BSP_USART1_DMA_RX_BUF_LEN];  //����һ���������ڴ�Ŵ�DMA���յ���ң��������
uint8_t USART6_DMA_RX_BUF[BSP_USART6_DMA_RX_BUF_LEN];  //����һ���������ڴ�Ŵ�DMA���յ��Ĳ���ϵͳ����
uint8_t USART8_DMA_RX_BUF[BSP_USART8_DMA_RX_BUF_LEN];
uint8_t USART3_DMA_RX_BUF[BSP_USART3_DMA_RX_BUF_LEN];
uint8_t USART7_DMA_RX_BUF[BSP_USART7_DMA_RX_BUF_LEN];
/***
	*�������ƣ�CMotor_Decode
	*�������ܣ�����������ݽ���
	*��ڲ�����CAN���յ���Ħ���ֵ���������������
	*����ֵ  ����
***/
void CMotor_Decode(CanRxMsgTypeDef* msg,Motor_t* motor)
{
	    motor->encoderCtrl.speed = (uint16_t)msg->Data[2] << 8 |(uint16_t)msg->Data[3];
	    motor->velCtrl.rawvel = (float)motor->encoderCtrl.speed; 
      motor->encoderCtrl.current = (uint16_t)msg->Data[4]<<8 |(uint16_t)msg->Data[5];	
}
/***
	*�������ƣ�SM_Decode
	*�������ܣ�����������ݽ���
	*��ڲ�����CAN���յ���Ħ���ֵ���������������
	*����ֵ  ����
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
	*�������ƣ�GMotor_Decode
    *��������: ��̨���ݽ���
	*��ڲ�����CAN���յ���Ħ���ֵ���������������
	*����ֵ  ����
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
	*�������ƣ�GMotor_Decode
    *��������: ��̨���ݽ���
	*��ڲ�����CAN���յ���Ħ���ֵ���������������
	*����ֵ  ����
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
	*�������ƣ�BL_Decode
	*�������ܣ��������ݽ���
	*��ڲ�����CAN���յ���Ħ���ֵ���������������
	*����ֵ  ����
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
	*�������ƣ�CAN_Receive_Msg
	*�������ܣ�CAN����
	*��ڲ�����CAN���
	*����ֵ  ����
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
	*�������ƣ�Referee_Data_Receive
	*�������ܣ�����ϵͳ����
	*��ڲ�������
	*����ֵ  ����
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
	*�������ƣ�RemotreCtl_Data_Receive
	*�������ܣ�����ң�������ݣ�λ��USART1���жϺ�����
	*��ڲ�������
	*����ֵ  ����
***/
void RemotreCtl_Data_Receive(void)
{
		uint32_t rx_data_len = 0;
	                                                          //���ν��ճ���
		if((__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE)!=RESET)) 
	   {
				__HAL_UART_CLEAR_IDLEFLAG(&huart1);                                           //��������жϵı�
				__HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx,DMA_FLAG_TCIF2_6);                       //��� DMA2_Steam2������ɱ�־
			  //__HAL_UART_CLEAR_FLAG(&huart1,UART_FLAG_FE);
				HAL_UART_DMAStop(&huart1);                                                    //��������Ժ�رմ���DMA
				rx_data_len = BSP_USART1_DMA_RX_BUF_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx); //��ȡ��һ����������С���ܳ���-�����ĳ��ȣ�
				HAL_UART_Receive_DMA(&huart1, USART1_DMA_RX_BUF, BSP_USART1_DMA_RX_BUF_LEN);  //��������
         if(rx_data_len == 18)                                                        //�ж������Ƿ�Ϊ��ȷ�����ݳ���
			 {
					RC_DataHandle(USART1_DMA_RX_BUF);                                           //�������ݽ��뺯��
			 }
		 }
}

/***
	*�������ƣ�RemotreCtl_Data_Receive
	*�������ܣ�����ң�������ݣ�λ��USART1���жϺ�����
	*��ڲ�������
	*����ֵ  ����
***/


void HI_IMU_Data_Receive(void)
{                                                          //���ν��ճ���
		if((__HAL_UART_GET_FLAG(&huart7,UART_FLAG_IDLE)!=RESET)) 
	   {
				__HAL_UART_CLEAR_IDLEFLAG(&huart7);                                           //��������жϵı�־
				__HAL_DMA_CLEAR_FLAG(&hdma_uart7_rx,DMA_FLAG_TCIF3_7);                        //��� DMA2_Steam2������ɱ�־
				HAL_UART_DMAStop(&huart7);                                                    //
				HAL_UART_Receive_DMA(&huart7, USART7_DMA_RX_BUF, BSP_USART7_DMA_RX_BUF_LEN);  //��������
        HI_IMU_Data_Decode(USART7_DMA_RX_BUF); 
			  //Mini_PC_Transmit();  			 //�������ݽ��뺯
		 }
}

void Mini_PC_Data_Receive(void)
{                                                          //���ν��ճ���
		if((__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE)!=RESET)) 
	   {
				__HAL_UART_CLEAR_IDLEFLAG(&huart3);                                           //��������жϵı�־
				__HAL_DMA_CLEAR_FLAG(&hdma_usart3_rx,DMA_FLAG_TCIF1_5);                        //��� DMA2_Steam2������ɱ�־
				HAL_UART_DMAStop(&huart3);                                                    //
				HAL_UART_Receive_DMA(&huart3, USART3_DMA_RX_BUF, BSP_USART3_DMA_RX_BUF_LEN);  //��������
        Mini_PC_Data_Decode(USART3_DMA_RX_BUF);                                        //�������ݽ��뺯
		 }
}
/***
	*�������ƣ�RemotreCtl_Data_Receive_Start
	*�������ܣ�����ң�������ݣ�λ��USART1���жϺ�����
	*��ڲ�������
	*����ֵ  ����
***/
void RemotreCtl_Data_Receive_Start(void)
{
	  __HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);                                       //�����������ж�
		HAL_UART_Receive_DMA(&huart1,USART1_DMA_RX_BUF, BSP_USART1_DMA_RX_BUF_LEN);
}
/***
	*�������ƣ�Referee_Data_Receive_Start
	*��������: ��������ϵͳ���ݽ����ж�
	*��ڲ�������
	*����ֵ  ����
***/
void Referee_Data_Receive_Start(void)
{
	  __HAL_UART_ENABLE_IT(&huart6,UART_IT_IDLE);                                       //�����������ж�
		HAL_UART_Receive_DMA(&huart6,USART6_DMA_RX_BUF, BSP_USART6_DMA_RX_BUF_LEN);
}
/***
	*�������ƣ�IMU_Data_Receive_Start
	*��������: ����IMU���ݽ����ж�
	*��ڲ�������
	*����ֵ  ����
***/
void IMU_Data_Receive_Start(void)
{
	  __HAL_UART_ENABLE_IT(&huart8,UART_IT_IDLE);                                       //�����������ж�
		HAL_UART_Receive_DMA(&huart8,USART8_DMA_RX_BUF, BSP_USART8_DMA_RX_BUF_LEN);
		__HAL_UART_ENABLE_IT(&huart7,UART_IT_IDLE);                                       //�����������ж�
		HAL_UART_Receive_DMA(&huart7,USART7_DMA_RX_BUF, BSP_USART7_DMA_RX_BUF_LEN);
}

void Mini_PC_Receive_Start(void)
{
	  __HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart3,USART3_DMA_RX_BUF, BSP_USART3_DMA_RX_BUF_LEN);	
}

