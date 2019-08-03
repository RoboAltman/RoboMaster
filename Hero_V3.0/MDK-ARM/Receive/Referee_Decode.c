/**
  ******************************************************************************
  * @file       Refree_Decode.c
  * @brief      ²ÃÅÐÏµÍ³Êý¾Ý½âÂë      
  ****************************************************************************
  */
#include "Referee_Decode.h"
#include "CRC.h"
#include "Transmit.h"
#include "Receive.h"
#include "string.h"
#include "usart.h"
#include "stm32f4xx_it.h"
#include "Power_Control.h"
#include "Shoot_Control.h"
#include "Remote_Control.h"
#include "Gimbal_Control.h"
#include "strategy.h"
#include "tim.h"
uint8_t seq=0;
uint8_t Client_Data[28];
float last_17mm_speed;
float last_42mm_speed;
client_custom_data_t	 Client_Custom_Data;
ext_buff_musk_t BuffMusk;

ext_shoot_data_t      ShootData;             //·¢Éä×´Ì¬,°üÀ¨ÉäËÙÉäÆµ
ext_power_heat_data_t  PowerHeatData;         //¹¦ÂÊºÍÈÈÁ¿ÓÃÕâ¸ö±äÁ¿¾Í¿ÉÒÔ

ext_game_robot_state_t RobotState;

ext_referee_data_t RefereeData_t;


float speed_store[100];
int store_cnt;


//these are used to test
uint32_t test_last_time = 0,test_now_time = 0,test_delta_time = 0,max_delta_time;

//extGameRobotState_t RobotState;
//extRobotHurt_t      RobotHurt;
//extShootData_t      ShootData;
//extPowerHeatData_t  PowerHeatData;
//extRfidDetect_t     RfidDetect;
//extGameResult_t     GameResult;
//extBuffMusk_t       BuffMusk;
//extGameRobotPos_t   GameRobotPos;
//extShowData_t       ShowData;

//extPowerHeatData_t  DataGameinfo;

FormatTrans32       datatrans32;
FormatTrans16       datatrans16;
int Datalength;//Êý¾Ý¶ÎµÄ×Ö½Ú³¤¶È
int DWlength;//Êý¾Ý°ü£¨²»°üº¬CRC116£©µÄ³¤¶È
int i=0,j=0,k=0;//ÓÃÀ´¼ÆÊý
uint8_t DWData[30];//ÓÃÀ´×··¢ËÍµÄÊý¾Ý£¬ÓÃÓÚCRCÐ£Ñé
client_custom_data_t Data1,Data2,Data3;//×Ô¶¨ÒåÊý¾Ý
uint8_t lightcontrol=0;//²Ù×÷½çÃæµÆ¿ØÖÆ
float Shoot_Speed_Store[500] = {0};
int Number = 0;
int heat_first_flag=0;
send_data_t send_data;
/**********************************************************************************

***********************************************************************************/
void RefereeDecode(uint8_t *pData)
{
	
		uint8_t frameLoc = 0;
	uint8_t *frameHeadLoc;					//ÔÝ´æµ±Ç°Ö¡µÄÖ¡Í·µØÖ·
	uint16_t dataLength, cmdID;
	
	while (frameLoc < BSP_USART6_DMA_RX_BUF_LEN)		//»º´æÇøÖ»ÄÜ±£´æ128¸ö×Ö½Ú£¬Ñ­»·¼ì²éÊÇ²»ÊÇÓÐÊý¾Ý°üÔÚÄÚ
	{
		/* µ±Ç°Ö¡µÄÖ¡Í·Ê×µØÖ·ÎªpData + frameLoc */
		if (pData[frameLoc] == FRAME_HEADER_SOF)
		{
			if (Verify_CRC8_Check_Sum(pData + frameLoc, FRAME_HEADER_LEN) == 1)		//Ö¡Í·CRC8Ð£Ñé³É¹¦
			{
				frameHeadLoc = pData + frameLoc;	//µ¥´¿ÔÝ´æ£¬¼ò»¯ºóÃæµÄ´úÂë
				memcpy(&dataLength, frameHeadLoc + 1, 2);	//»ñÈ¡Êý¾Ý°ü³¤¶ÈºóÐ£ÑéÕû°ü
				if (Verify_CRC16_Check_Sum(pData + frameLoc, FRAME_HEADER_LEN + CMD_ID_LEN + dataLength + CRC16_LEN) == 1)
				{
					memcpy(&cmdID, frameHeadLoc + FRAME_HEADER_LEN, 2);
					switch (cmdID)
					{
						case GAME_STATE_CMD_ID:		//1Hz
							memcpy(&RefereeData_t.GameState_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, GAME_STATE_LEN);
							break;
						case GAME_RESULT_CMD_ID:	//±ÈÈü½áÊøºó·¢ËÍ
							memcpy(&RefereeData_t.GameResult_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, GAME_RESULT_LEN);
							break;
						case GAME_ROBOTSURVIVORS_CMD_ID:	//1Hz·¢ËÍ
							memcpy(&RefereeData_t.GameRobotSurvivors_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, GAME_ROBOTSURVIVORS_LEN);
							break;
						case EVENT_DATA_CMD_ID:		//ÊÂ¼þ¸Ä±äºó·¢ËÍ
							memcpy(&RefereeData_t.EventData_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, EVENT_DATA_LEN);
							break;
//						case SUPPLY_PROJECTILE_ACTION_CMD_ID:	//¶¯×÷¸Ä±äºó·¢ËÍ
//							memcpy(&RefereeData_t.SupplyProjectileAction_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, SUPPLY_PROJECTILE_ACTION_LEN);
//							break;
						case GAME_ROBOT_STATE_CMD_ID:	//10Hz
							memcpy(&RobotState, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, GAME_ROBOT_STATE_LEN);
							break;
						case POWER_HEAT_DATA_CMD_ID:	//50Hz
						{
							test_last_time = test_now_time;
							test_now_time = system_time;
							test_delta_time = test_now_time - test_last_time;
							if(test_delta_time>max_delta_time) max_delta_time = test_delta_time;

							memcpy(&PowerHeatData, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, POWER_HEAT_DATA_LEN);
//							if(((RobotState.shooter_heat1_cooling_limit - PowerHeatData.shooter_heat1) - last_heat_rest >80) && (heat_first_flag!=0)) return;
//						  last_heat_rest = heat_rest;
							heat_rest = RobotState.shooter_heat1_cooling_limit - PowerHeatData.shooter_heat1;
							
//							if(heat_first_flag==0)
//						  {	
//								my_heat_rest = heat_rest;
//								heat_first_flag = 1;
//							}
//						  if(my_heat_rest<last_heat_rest &&last_heat_rest>heat_rest)
//							{
//								my_heat_rest = heat_rest;
//							}
//							else if(my_heat_rest == last_heat_rest)
//							{
//								my_heat_rest =heat_rest;
//							}
						}
							break;
						case GAME_ROBOT_POS_CMD_ID:		//10Hz
							memcpy(&RefereeData_t.GameRobotPos_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, GAME_ROBOT_POS_LEN);
							break;
						case BUFF_MUSK_CMD_ID:			//ÔöÒæ×´Ì¬¸Ä±äºó·¢ËÍ
							memcpy(&BuffMusk, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, BUFF_MUSK_LEN);
							break;
						case ROBOT_HURT_CMD_ID:			//ÉËº¦·¢Éúºó·¢ËÍ
							memcpy(&RefereeData_t.RobotHurt_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, ROBOT_HURT_LEN);
							break;
						case SHOOT_DATA_CMD_ID:			//×Óµ¯·¢Éäºó·¢ËÍ
						{
							memcpy(&ShootData, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, SHOOT_DATA_LEN);
							if(ShootData.bullet_type == 2)
							{
								if(last_42mm_speed != ShootData.bullet_speed)
								{
									if(Shoot_Request_Flag == 1)
									{
										Q += 100;
										Shoot_Request_Flag = 0;
										Bullet_Num--;
										if(Bullet_Num <= 0)
										Bullet_Num = 0;
									}
								}
//								else
//								{
//									Shoot_Request_Flag = 0;
//								}
								speed_store[store_cnt] = ShootData.bullet_speed;
								store_cnt ++ ;
								if(store_cnt>=99)
								store_cnt = 0;
								last_42mm_speed = ShootData.bullet_speed;
							}
							if(ShootData.bullet_type == 1)
							{
								if(last_17mm_speed != ShootData.bullet_speed)
								{
                   Q_17mm = Q_17mm + ShootData.bullet_speed;
								}
								last_17mm_speed = ShootData.bullet_speed;
							}
							break;
						}
						default:
							break;
					}
					frameLoc += FRAME_HEADER_LEN + CMD_ID_LEN + dataLength + CRC16_LEN;		//Õû°ü´¦ÀíÍê±Ï£¬ÏòºóÒ»¸öÕû°ü³¤¶È
				}
				else
					frameLoc += FRAME_HEADER_LEN;		//Ö¡Í·ÕýÈ·ÇÒ°üÍ·Ð£ÕýÕýÈ·£¬µ«ÊÇÕû°ü³ö´í£¬ÏòºóÒ»¸ö°üÍ·³¤¶È
			}
			else
				frameLoc++;			//Ö¡Í·ÕýÈ·£¬µ«ÊÇ°üÍ·Ð£Ñé³ö´í£¬ÏòºóÒ»¸ö×Ö½Ú
		}
		else
			frameLoc++;			//Î´Ê¶±ðÖ¡Í·£¬ÏòºóÒ»¸ö×Ö½Ú
	}
}
	
void FloatToUint8(uint8_t * char_array,float data)
{
	uint8_t i;
	for(i=0;i<4;i++)
	{
		char_array[i] = ((uint8_t*)(&data))[i];
	}
}

uint8_t bitclr(uint8_t num,uint8_t bit) /*æ¸…é™¤æŸä¸€ä½*/
{
uint8_t bit_value[]={1,2,4,8,16,32,64,128};
return num&~bit_value[bit];
}
uint8_t bitset(uint8_t num,uint8_t bit) /*è®¾ç½®æŸä¸€ä½*/
{
uint8_t bit_value[]={1,2,4,8,16,32,64,128};
return num|bit_value[bit];
}
uint8_t bitcpl(uint8_t num,uint8_t bit) /*å–åæŸä¸€ä½*/
{
uint8_t bit_value[]={1,2,4,8,16,32,64,128};
if((num>>bit&0x01)==1)
return num&~bit_value[bit];
else
return num|bit_value[bit];
}

void Client_Custom_Data_Set()
{
	send_data.client_data.data1 = cap_volt;
	send_data.client_data.data2 = Bullet_Num;
	send_data.client_data.data3 = Q;	
	//ç¯1ï¼Œæ‘©æ“¦è½®å¼€å¯æ˜¾ç¤º
	if(SMotor_State==Motor_ON)
	{
		send_data.client_data.masks = bitset(send_data.client_data.masks,0);
	}
	else
	{
		send_data.client_data.masks = bitclr(send_data.client_data.masks,0);
	}
	//ç¯2ï¼Œå°å‘å°„æœºæž„Enableæ˜¾ç¤º
	if((Shoot_Enable_STA&0x01) == 0x01)
	{
		send_data.client_data.masks = bitset(send_data.client_data.masks,1);
	}
	else
	{
		send_data.client_data.masks = bitclr(send_data.client_data.masks,1);
	}	
  //ç¯3ï¼Œé¢„æµ‹å¼€å¯
	if(Forecast_Mode != NO_Forecast)
	{
		send_data.client_data.masks = bitset(send_data.client_data.masks,2);
	}
	else
	{
		send_data.client_data.masks = bitclr(send_data.client_data.masks,2);
	}	
	//ç¯4ï¼Œæ‹¨ç›˜çŠ¶æ€ï¼Œæ—‹è½¬ç»¿ï¼Œä¸è½¬çº¢
	if(BLMotor_Work_State == Vel_pid)
	{
		send_data.client_data.masks = bitset(send_data.client_data.masks,3);
	}
	else
	{
		send_data.client_data.masks = bitclr(send_data.client_data.masks,3);
	}
	//ç”µå®¹OKåˆ™ç»¿ï¼Œæ²¡ç”µçº¢
	if(CAP_STA != CAP_USEDOUT)
	{
		send_data.client_data.masks = bitset(send_data.client_data.masks,4);
	}
	else
	{
		send_data.client_data.masks = bitclr(send_data.client_data.masks,4);
	}
	
	
//	if(Aim_Mode == AUTO)
//	{
//		send_data.client_data.masks = bitset(send_data.client_data.masks,2);
//	}
//	else
//	{
//		send_data.client_data.masks = bitclr(send_data.client_data.masks,2);
//	}	

		
}

void data_head_init()
{
	//????
	send_data.frame_header.sof = FRAME_HEADER_SOF;
	send_data.frame_header.dataLength = 19;		//2 + 2 + 2 + 1
	send_data.frame_header.seq = 0;
	send_data.cmd_id = 0x301;
	//??????
	send_data.client_header.data_cmd_id = 0xD180;     		 	  //????????ID
	if(RobotState.robot_id == 1)              //????
	{
		send_data.client_header.receiver_ID = 0x101;     		  //????????
		send_data.client_header.send_ID = 1;              		  //????ID2
	}
	
	else if(RobotState.robot_id == 11)  	  //????
	{
		send_data.client_header.receiver_ID = 0x0111;     		  //????????
		send_data.client_header.send_ID = 11;              		  //????ID2
	}
	
}


void Client_Data_Build()
{
  //send_data.frame_header.seq++;

	Client_Custom_Data_Set();
	memcpy(Client_Data,&send_data, 4);
	//??CRC8??
	send_data.frame_header.crc8 = Get_CRC8_Check_Sum(Client_Data,4,0xff);	
	memcpy(Client_Data,&send_data, 26);
	//??CRC16??
	send_data.CRC16 = Get_CRC16_Check_Sum(Client_Data ,26, 0xffff);
	memcpy(Client_Data,&send_data, 28);
}
//	Client_Data[0] = 0xA5;
//	Client_Data[1] = 0x13;
//	Client_Data[2] = 0x00;
//	Client_Data[3] = seq++;
//	Client_Data[4] = Get_CRC8_Check_Sum(Client_Data,4,0xFF);
//	Client_Data[5] = 0x01;
//	Client_Data[6] = 0x03;
//	Client_Data[7] = 0x80;
//	Client_Data[8] = 0xD1;
//	Client_Data[9] = (uint8_t)(RobotState.robot_id);
//	Client_Data[10] = (uint8_t)(RobotState.robot_id>>8);
//	switch(RobotState.robot_id)
//	{
//		case 0x01:Client_Data[11] = 0x01;break;
//		case 0x0B:Client_Data[11] = 0X11;break;
//	}
//	Client_Data[12] = 0x01;
//	Client_Custom_Data_Set();
//	datatrans32.float_temp = Client_Custom_Data.data1;
//	Client_Data[13] = datatrans32.u8_temp[0];
//	Client_Data[14] = datatrans32.u8_temp[1];
//	Client_Data[15] = datatrans32.u8_temp[2];
//	Client_Data[16] = datatrans32.u8_temp[3];
//	datatrans32.float_temp = Client_Custom_Data.data2;
//	Client_Data[17] = datatrans32.u8_temp[0];
//	Client_Data[18] = datatrans32.u8_temp[1];
//	Client_Data[19] = datatrans32.u8_temp[2];
//	Client_Data[20] = datatrans32.u8_temp[3];
//	datatrans32.float_temp = Client_Custom_Data.data3;
//	Client_Data[21] = datatrans32.u8_temp[0];
//	Client_Data[22] = datatrans32.u8_temp[1];
//	Client_Data[23] = datatrans32.u8_temp[2];
//	Client_Data[24] = datatrans32.u8_temp[3];
//	Client_Data[25] = Client_Custom_Data.masks;
//	CRC16 = Get_CRC16_Check_Sum(Client_Data,26,0xFFFF);
//	Client_Data[26] = (uint8_t)CRC16;
//	Client_Data[27] = (uint8_t)(CRC16>>8);


