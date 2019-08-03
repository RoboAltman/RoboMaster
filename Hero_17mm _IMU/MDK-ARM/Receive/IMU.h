#ifndef _IMU_H_
#define _IMU_H_
#include "stm32f4xx_hal.h"
typedef struct
{
	float x;
	float y;
	float z;	
}IMU_acc_t;

typedef struct
{
	float x;
	float y;
	float z;	
}IMU_angVel_t;

typedef struct
{
	float x;
	float y;
	float z;	
}IMU_ang_t;

typedef enum 
{
	Incomplete,
	Complete
}IMU_State_e;

typedef union
{
        uint8_t     u8_temp[4];
        uint32_t    u32_temp;
	      float       float_temp;
}FormatTrans32;//32位数据的转换

typedef union
{
        uint8_t     u8_temp[2];
        uint16_t    u16_temp;
}FormatTrans16;//16位数据的转换

void JY_IMU_Data_Decode(uint8_t *pData);
void HI_IMU_Data_Decode(uint8_t *pData);
void IMU_Init(void);

extern IMU_angVel_t jy_imu_angvel;
extern IMU_ang_t jy_imu_ang;
extern int IMU_round;
extern IMU_acc_t hi_imu_acc;
extern IMU_angVel_t hi_imu_angvel;
extern IMU_ang_t hi_imu_ang;
extern float IMU_acc;
extern float temper;
extern float IMU_Store[35];
extern float IMU_Delay;
extern float Pitch_Delay;
extern float Yaw_IMU_OFFSET;
#endif
