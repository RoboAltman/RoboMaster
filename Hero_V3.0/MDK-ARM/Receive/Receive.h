#ifndef _RECEIVE_H_
#define _RECEIVE_H_
#include "stm32f4xx_hal.h"
#include "can.h"

#define BSP_USART1_DMA_RX_BUF_LEN 30u                            //ң�������ݽ���DMA�洢����
#define BSP_USART6_DMA_RX_BUF_LEN 128u                           //����ϵͳ���ݽ���DMA�洢����
#define BSP_USART8_DMA_RX_BUF_LEN 40u 
#define BSP_USART7_DMA_RX_BUF_LEN 40u 
#define BSP_USART3_DMA_RX_BUF_LEN 31u 

extern DMA_HandleTypeDef hdma_uart8_rx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;

void CAN_Receive_Msg(CAN_HandleTypeDef* hcan);
void RemotreCtl_Data_Receive_Start(void);
void Referee_Data_Receive_Start(void);
void IMU_Data_Receive_Start(void);
void Mini_PC_Receive_Start(void);
void RemotreCtl_Data_Receive(void);
void Referee_Data_Receive(void);
void Mini_PC_Data_Receive(void);
void JY_IMU_Data_Receive(void);
void HI_IMU_Data_Receive(void);
#endif




