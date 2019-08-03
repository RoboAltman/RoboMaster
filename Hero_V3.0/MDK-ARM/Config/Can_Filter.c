#include "Can_Filter.h"
#include "can.h"
CanTxMsgTypeDef  CAN1_TxMessage;
CanRxMsgTypeDef  CAN1_RxMessage;
CanTxMsgTypeDef  CAN2_TxMessage;
CanRxMsgTypeDef  CAN2_RxMessage;

/***
	*函数名称：Can_Filter_Init
	*函数功能：CAN滤波器配置
	*入口参数：无
	*返回值  ：无
***/

void Can2_Filter_Init(void)
{
  CAN_FilterConfTypeDef canfilter;
  
  //create memory to save the message, if not will raise error
  
  canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilter.FilterScale = CAN_FILTERSCALE_32BIT;
  
  //filtrate any ID you want here
  canfilter.FilterIdHigh = 0x0000;
  canfilter.FilterIdLow = 0x0000;
  canfilter.FilterMaskIdHigh = 0x0000;
  canfilter.FilterMaskIdLow = 0x0000;
  canfilter.FilterNumber = 14;
  canfilter.FilterFIFOAssignment = CAN_FilterFIFO0;
  canfilter.FilterActivation = ENABLE;
  canfilter.BankNumber = 14; 
 // hcan1.pTxMsg = &CAN1_TxMessage;
 // hcan1.pRxMsg = &CAN1_RxMessage;
  hcan2.pTxMsg = &CAN2_TxMessage;
  hcan2.pRxMsg = &CAN2_RxMessage;
  //HAL_CAN_ConfigFilter(&hcan1, &canfilter); 
  HAL_CAN_ConfigFilter(&hcan2, &canfilter);
}
void Can1_Filter_Init(void)
{
  CAN_FilterConfTypeDef canfilter;
  
  //create memory to save the message, if not will raise error
  
  canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilter.FilterScale = CAN_FILTERSCALE_32BIT;
  
  //filtrate any ID you want here
  canfilter.FilterIdHigh = 0x0000;
  canfilter.FilterIdLow = 0x0000;
  canfilter.FilterMaskIdHigh = 0x0000;
  canfilter.FilterMaskIdLow = 0x0000;
  canfilter.FilterNumber = 0;
  canfilter.FilterFIFOAssignment = CAN_FilterFIFO0;
  canfilter.FilterActivation = ENABLE;
  canfilter.BankNumber = 14; 
  hcan1.pTxMsg = &CAN1_TxMessage;
  hcan1.pRxMsg = &CAN1_RxMessage;
 // hcan2.pTxMsg = &CAN2_TxMessage;
 // hcan2.pRxMsg = &CAN2_RxMessage;
  HAL_CAN_ConfigFilter(&hcan1, &canfilter); 
  //HAL_CAN_ConfigFilter(&hcan2, &canfilter);
}


