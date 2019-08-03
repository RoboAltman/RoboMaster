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
#endif
