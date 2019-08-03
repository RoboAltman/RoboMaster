#include "Receive.h"
#include "Shoot_Control.h"
#include "Gimbal_Control.h"
#include "usart.h"
#include "Board_I_TX.h"
#include "string.h"
#include "IMU.h"
uint8_t USART1_DMA_RX_BUF[BSP_USART1_DMA_RX_BUF_LEN];

void GMotor_Decode(CanRxMsgTypeDef* msg,Motor_t *motor)
{
			float diff;
	    motor->encoderCtrl.angle   = (uint16_t) msg->Data[0] << 8 |(uint16_t) msg->Data[1];
	    motor->encoderCtrl.speed   = (uint16_t) msg->Data[2] << 8 |(uint16_t) msg->Data[3];
	    motor->encoderCtrl.current = (int16_t) msg->Data[4] << 8 |(int16_t) msg->Data[5];
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
	    motor->velCtrl.lastrawvel  = motor->velCtrl.rawvel;
}

void BLMotor_Decode(CanRxMsgTypeDef* msg,Motor_t *motor)
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

void CAN_Receive_Msg(CAN_HandleTypeDef* hcan)
{
	  switch(hcan->pRxMsg->StdId)
		{
      case  0x205:
			{
				GMotor_Decode(hcan->pRxMsg,&GMotor_Yaw);
				break;
			}
			case  0x206:
			{
				GMotor_Decode(hcan->pRxMsg,&GMotor_Pitch);
				break;
			}
			case  0x207:
			{
				BLMotor_Decode(hcan->pRxMsg,&BLMotor);
				break;
			}
		}	
}
int test_num = 0,test_num1 = 0,num_ptr = 0;
uint16_t USART_RX_STA;
uint8_t Res;
void Board_I_Receive(void)
{
//	if((__HAL_UART_GET_FLAG(&huart6,UART_FLAG_RXNE)!=RESET))
//	{
//		
//		HAL_UART_Receive(&huart6, &Res,1,1000);
//		if((USART_RX_STA&0x8000) == 0x8000 && (USART_RX_STA&0x4000) == 0x4000)
//		{
//			Board_I_Data_Temp[USART_RX_STA&0x3FFF]= Res;
//			USART_RX_STA++;
//			if((USART_RX_STA&0x3FFF)>=24)
//			{
//				USART_RX_STA = 0;
//				memcpy(Board_I_Data+2,Board_I_Data_Temp,24);
//				Board_I_Decode();
//			}
//		}
//		else if((USART_RX_STA&0x8000) == 0x8000 && (USART_RX_STA&0x4000) != 0x4000)
//		{
//			if(Res == 0x5A)
//			{
//				USART_RX_STA|=0x4000;
//			}
//			else
//			{
//				USART_RX_STA = 0;
//			}
//		}
//		else if(Res == 0xA5&&(USART_RX_STA&0x8000) != 0x8000)//
//		{
//			USART_RX_STA|=0x8000;			
//		}
//		else
//		{
//			USART_RX_STA = 0;
//		}
//		}
//	else
//	{
//		test_num1 = USART6->SR;
//		test_num1 = USART6->DR;
//	}
//}
	
	//if(Res == 0xA5)
		
		if((__HAL_UART_GET_FLAG(&huart6,UART_FLAG_IDLE)!=RESET)) 
		{
			__HAL_UART_CLEAR_IDLEFLAG(&huart6);
			__HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_FLAG_TCIF1_5);
      //(void)USART6->SR;  
      //(void)USART6->DR;
			HAL_UART_DMAStop(&huart6);
			HAL_UART_Receive_DMA(&huart6, Board_I_Data_Temp, 50);			
		  Board_I_Decode(Board_I_Data_Temp);
		}
	}

void Board_I_Data_Receive_Start(void)
{
	  __HAL_UART_ENABLE_IT(&huart6,UART_IT_IDLE);                                       //ߪǴһ֨Ӥא׏
		//HAL_UART_Receive_DMA(&huart6,Board_I_Data, 25);
}


void Mini_PC_Data_Receive(void)
{                                                          //ѾՎޓ˕Ӥ׈
		if((__HAL_UART_GET_FLAG(&huart7,UART_FLAG_IDLE)!=RESET)) 
	   {
				__HAL_UART_CLEAR_IDLEFLAG(&huart7);                                           //ȥԽࠕАא׏քҪ־
				__HAL_DMA_CLEAR_FLAG(&hdma_uart7_rx,DMA_FLAG_TCIF3_7);                        //ȥԽ DMA2_Steam2ԫˤΪԉҪ־
				HAL_UART_DMAStop(&huart7);                                                    //
				HAL_UART_Receive_DMA(&huart7, UART7_DMA_RX_BUF, BSP_UART7_DMA_RX_BUF_LEN);  //ޓ˜˽ߝ
        Mini_PC_Data_Decode(UART7_DMA_RX_BUF);                                        //޸ɫ˽ߝޢëگ
		 }
}

void Mini_PC_Data_Receive_Start(void)
{
	  __HAL_UART_ENABLE_IT(&huart7,UART_IT_IDLE);                                       //ߪǴһ֨Ӥא׏
		HAL_UART_Receive_DMA(&huart7,UART7_DMA_RX_BUF, BSP_UART7_DMA_RX_BUF_LEN);
}

void HI_IMU_Data_Receive(void)
{                                                          //ѾՎޓ˕Ӥ׈
		if((__HAL_UART_GET_FLAG(&huart8,UART_FLAG_IDLE)!=RESET)) 
	   {
				__HAL_UART_CLEAR_IDLEFLAG(&huart8);                                           //ȥԽࠕАא׏քҪ־
				__HAL_DMA_CLEAR_FLAG(&hdma_uart8_rx,DMA_FLAG_TCIF2_6);                        //ȥԽ DMA2_Steam2ԫˤΪԉҪ־
				HAL_UART_DMAStop(&huart8);                                                    //
				HAL_UART_Receive_DMA(&huart8, UART8_DMA_RX_BUF, BSP_UART8_DMA_RX_BUF_LEN);  //ޓ˜˽ߝ
        HI_IMU_Data_Decode(UART8_DMA_RX_BUF); 
			  //Mini_PC_Transmit();  			 //޸ɫ˽ߝޢëگ
		 }
}

void IMU_Data_Receive_Start(void)
{
	  __HAL_UART_ENABLE_IT(&huart8,UART_IT_IDLE);                                       //ߪǴһ֨Ӥא׏
		HAL_UART_Receive_DMA(&huart8,UART8_DMA_RX_BUF, BSP_UART8_DMA_RX_BUF_LEN);
}
