#ifndef _CHASSIS_CONTROL_H_
#define _CHASSIS_CONTROL_H_
#include "stm32f4xx_hal.h"
#include "PID.h"
#include "Remote_Control.h"
#include "Motor.h"

enum Chassis_Work_Mode{C_JianLu_MODE=0,C_RACE_MODE=1,C_STOP_MODE=2,C_LIMIT_MODE=3,C_INFINITE_MODE=4};    
void Set_Chassis_Movement_Sta(void);
void Set_Zero_Ref(void);

extern enum Chassis_Work_Mode Work_Mode,Ctrl_Chassis_Mode;    
extern Motor_t CMotor1;
extern Motor_t CMotor2;
extern Motor_t CMotor3;
extern Motor_t CMotor4;
extern Motor_t Rotate;
extern ChassisRef_t ChassisRef;
//extern float power_max;
void Chassis_Control(void);
void Motor_VelPID(VelCtrl_t *vel_t);

extern float REMOTE_CHASIS_SPEED_TO_REF_FB; 
extern float REMOTE_CHASIS_SPEED_TO_REF_LR; 
extern PID_Parameter_t rotate_Vel_PID1;
extern PID_Parameter_t rotate_Vel_PID2;
extern PID_Parameter_t rotate_Vel_PID0; 
extern PID_Parameter_t rotate_Pos_PID0;
extern PID_Parameter_t rotate_Pos_PID1;
extern PID_Parameter_t rotate_Pos_PID2;
extern PID_Parameter_t  velPara;

#endif



