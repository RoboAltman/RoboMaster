#include "IMU.h"
#include "Gimbal_Control.h"
#include "Referee_Decode.h"
#define Id_LEN 1u

IMU_acc_t hi_imu_acc;
IMU_angVel_t hi_imu_angvel;
IMU_ang_t hi_imu_ang;

IMU_State_e IMU_State;
float temper;
int16_t temp;
uint16_t HI_CRCreceive;
uint16_t HI_Data_Len;
uint16_t HI_LEN;
FormatTrans16 imutrans16;
FormatTrans32 imutrans32;//32位数据的转换
float IMU_Store[35];
float Pitch_Store[35];
float last_IMU_z;
int IMU_round;
float IMU_acc;
float IMU_Delay;
float Pitch_Delay;

void HI_IMU_Data_Decode(uint8_t *pData)
{
  int i,j;
	for(i = 0;i < 40;i++)
	{
		if(pData[i] == 0x5A && pData[i+1] == 0xA5)
		{
      HI_LEN = (uint16_t)pData[i+2] | (uint16_t)(pData[i+3]<<8);
			HI_CRCreceive = (uint16_t)pData[i+4] | (uint16_t)(pData[i+5]<<8);
			for(j = 0; i+6+HI_Data_Len+j <= (HI_LEN - 5) ;j++)
			{
				switch(pData[i+6+HI_Data_Len+j])
				{
					case 0x90:
					  HI_Data_Len += 1;
						break;
					case 0xA0:
					  HI_Data_Len += 6;
						break;
					case 0xA5:
					  HI_Data_Len += 6;
						break;
					case 0xB0:
					{
						imutrans16.u8_temp[0] = pData[i+ 7+HI_Data_Len+j];
						imutrans16.u8_temp[1] = pData[i+ 8+HI_Data_Len+j];
						hi_imu_angvel.x = (float)(int16_t)(imutrans16.u16_temp)*0.1f;
						imutrans16.u8_temp[0] = pData[i+ 9+HI_Data_Len+j];
						imutrans16.u8_temp[1] = pData[i+10+HI_Data_Len+j];
						hi_imu_angvel.y = (float)(int16_t)(imutrans16.u16_temp)*0.1f;
						imutrans16.u8_temp[0] = pData[i+11+HI_Data_Len+j];
						imutrans16.u8_temp[1] = pData[i+12+HI_Data_Len+j];
						hi_imu_angvel.z = (float)(int16_t)(imutrans16.u16_temp)*0.1f;
						HI_Data_Len += 6;
					}
						break;
					case 0xC0:
						HI_Data_Len += 6;
						break;
					case 0xD0:
					{
					  imutrans16.u8_temp[0] = pData[i+ 7+HI_Data_Len+j];
						imutrans16.u8_temp[1] = pData[i+ 8+HI_Data_Len+j];
						hi_imu_ang.x = (float)(int16_t)(imutrans16.u16_temp)*0.01f;
						imutrans16.u8_temp[0] = pData[i+ 9+HI_Data_Len+j];
						imutrans16.u8_temp[1] = pData[i+10+HI_Data_Len+j];
						hi_imu_ang.y = (float)(int16_t)(imutrans16.u16_temp)*0.01f;
						imutrans16.u8_temp[0] = pData[i+11+HI_Data_Len+j];
						imutrans16.u8_temp[1] = pData[i+12+HI_Data_Len+j];
						hi_imu_ang.z = (float)(int16_t)(imutrans16.u16_temp)*0.1f;
						HI_Data_Len += 6;
            hi_imu_ang.z = 180 + hi_imu_ang.z;
						if(hi_imu_ang.z - last_IMU_z < -300)
						{
							IMU_round++;
						}
						else if(hi_imu_ang.z - last_IMU_z > 300)
						{
							IMU_round--;
						}
						IMU_acc = (IMU_round) * 360.0f + hi_imu_ang.z;	
            last_IMU_z = hi_imu_ang.z;
						
//						for(i = 34;i > 0;i--)
//						{
//							IMU_Store[i] = IMU_Store[i-1];
//						}
//						IMU_Delay = IMU_Store[34] - Yaw_IMU_OFFSET;
//            IMU_Store[0] = 	IMU_acc;	

//						for(i = 34;i > 0;i--)
//						{
//							Pitch_Store[i] = Pitch_Store[i-1];
//						}
//						Pitch_Delay = Pitch_Store[34];
//            Pitch_Store[0] = 	hi_imu_ang.y;	
						
					}
						break;
					case 0xD9:
						HI_Data_Len += 12;
						break;
					case 0xD1:
						HI_Data_Len += 16;
						break;
					case 0xF0:
						HI_Data_Len += 4;
						break;
					default:
						HI_Data_Len += 0;
						break;
				}
			}
		HI_Data_Len = 0;
		}
		
	
}
}
int imu_flag;
void IMU_Calibration()
{
	imu_flag = 0;
	IMU_State = Incomplete;
	while(IMU_State == Incomplete)
	{
		if(__fabs((float)(GMotor_Yaw.posCtrl.accpos-Yaw_Motor_OFFSET)/22.7556f) < 0.2)
			imu_flag++;
		else
			imu_flag = 0;
		if(imu_flag >= 3000)
		{
			Yaw_IMU_OFFSET = hi_imu_ang.z;
			IMU_State = Complete;
		}
	}
}
