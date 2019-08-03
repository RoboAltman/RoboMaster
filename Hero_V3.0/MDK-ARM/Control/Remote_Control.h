#ifndef _REMOTE_CONTROL_H_
#define _REMOTE_CONTROL_H_
#include "stm32f4xx_hal.h"
#include "Remote_Control.h"

/**********************ң��������********************/
#define REMOTE_INPUT                           1u
#define KEY_MOUSE_INPUT                        3u
#define Check_Mode                             2u
#define REMOTE_STICK_OFFSET                 1024u
//#define REMOTE_CHASIS_SPEED_TO_REF_FB        12.0f//7.0f          //1219*0.5/660f
//#define REMOTE_CHASIS_SPEED_TO_REF_LR        12.0f //6.9f
#define REMOTE_PITCH_ANGLE_TO_REF           0.001f
#define REMOTE_YAW_ANGLE_TO_REF             0.004f
#define ROTATE_TO_REF                       7.0f

/***********************�����������*****************/
#define MOUSE_PITCH_ANGLE_TO_FACT 		     0.075f
#define MOUSE_YAW_ANGLE_TO_FACT 		       0.075f

/************************����flag*********************/

typedef struct
{
struct
	{
		uint16_t ch0;//ͨ��0
		uint16_t ch1;//ͨ��1
		uint16_t ch2;//ͨ��2
		uint16_t ch3;//ͨ��3
		uint8_t  s1;//����1
		uint8_t  s2;//����2
	}remote;
	
struct
	{
		int16_t  x;//���x
		int16_t  y;//���y
		int16_t  z;//���z
		uint8_t  press_l;//������
		uint8_t  press_r;//����Ҽ�
	}mouse;
	
struct
	{
		uint16_t v;//����
	}key;
}RC_Ctrl_t;


typedef struct
{
	int16_t forward_back_ref;
	int16_t left_right_ref;
	int16_t rotate_ref;	
}ChassisRef_t;

typedef struct
{
    float pitch_angle_ref;
    float yaw_angle_ref;
	  float yaw_last_angle_ref;
}GimbalRef_t;

typedef struct
{
    float Frequency_42mm;
    float Frequency_17mm;
}ShootRef_t;

typedef struct
{
	float SpeedtoRef_FB;//ǰ���ƶ��ٶ�ƥ��
	float SpeedtoRef_LR;//�����ƶ��ٶ�ƥ��
	float Acc;//���ٶ�
	float Dec;//���ٶ�
}SpeedMatch_t;
typedef enum
{
	Remote,
	Mouse_Board
}Control_Mode_e;

//typedef enum
//{
//	Disable,
//  Enable
//}Shoot_Enable_e;

typedef enum
{
	Zero,
	Positive45,
	Negetive45
}Rotate_State_e;

typedef enum
{
	Battery,
	Cap
}Power_Mode_e;
void Remode_Input_Judge(void);
void Remote_Process(void);
void Control_Init(void);
void RC_DataHandle(uint8_t *pData);
extern RC_Ctrl_t RC_Ctrl_Data;
extern Power_Mode_e Power_Mode;
extern uint8_t Shoot_Enable_STA;
extern Control_Mode_e Control_Mode;
extern Control_Mode_e Last_Control_Mode;
extern Rotate_State_e Rotate_State;
extern int strategy_flag;
extern int NPG_dir;
extern int Shoot_Request_Flag;
#endif







