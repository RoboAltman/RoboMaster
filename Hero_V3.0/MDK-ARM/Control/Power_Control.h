#ifndef _POWER_CONTROL_H_
#define _POWER_CONTROL_H_
#include "stm32f4xx_hal.h"
#include "Motor.h"
#include "PID.h"
#define CAP_I_OFFSET 0.5f//��ѹ�������ʴ���Ч��
#define TIM_MAX 500
//新
/*******/
void SuperCapCtrl(void);
void Auto_Charge(void);

/* 陔唳髡薹諷秶妏蚚腔滲杅 */
void Power_Ctrl(float Power_Max);				
float Power_PID_Cal(Motor_t *CMotor,PID_Parameter_t CMpid,float ref);
float Motor_Theory_Power(float avg_speed,float output_i);
float Power_to_ref(float m_power,float avg_speed,float max);

/* 萇耀輸袨怓曹講 */
enum EX_STATE{_CAP=0,_BATTERY=1};												//菁攫鼎萇袨怓
enum CAP_Ctrl{BATTERY=0,CHARGE=1,DISCHARGE=2,CAP=3,TO_CAP=4,TO_BATTERY=5};		//萇諷秶袨怓
enum CAP_Auto{CAP_MANUAL=0,CAP_AUTO=1};											//萇諷秶耀宒ㄩ赻雄迵忒雄
enum CAP_Status{CAP_USEDOUT=0,CAP_OK=1,CAP_FULL=2};								//萇赻旯袨怓ㄩ蚚俇 | 夔劂妏蚚 | 雛萇
enum DC_State{DC_CLOSE=0,DC_OPEN=1};
enum Chassis_Movement_Sta{CAR_STOP=0,CAR_MOVE=1,CAR_ROTATE=2,CAR_MIXMOVING=3};//�����˶�״̬�ж�                                                    //�����ƶ�ת̬
enum Auto_Charge_Switch{SWITCH_ON=1,SWITCH_OFF=0};                                                         //�����Զ���ŵ翪��
enum CHASSIS_STA{JUDGE_ON=0,JUDGE_OFF=1};									//裁判系偷着坛供电状态指示：底盘开启供电 | 底盘关闭供电

extern enum Auto_Charge_Switch Charge_Switch,ex_Charge_Switch;        //���ڿ��Ƴ�翪��


extern enum EX_STATE exstate;													//菁攫鼎萇袨怓							
extern enum CAP_Ctrl EX_CTRL;													//萇奻棒腔鼎萇袨怓
extern enum CAP_Ctrl CAP_CTRL;													//萇掛棒腔鼎萇袨怓
extern enum CAP_Auto CAP_AUTO_CTRL;												//萇耀輸忒赻遙袨怓弇
extern enum CAP_Status CAP_STA;													//萇赻旯袨怓
extern enum DC_State Primary_DC_State,Secondary_DC_State;						//DC-DC馱釬袨怓

extern enum Chassis_Movement_Sta Movement_Status;
extern enum CHASSIS_STA Chassis_State;

/* NEW:萇杅擂賤呾滲杅 */
float GetDCCurrent(void);									//鳳場撰DC怀堤萇霜
float GetChassisCurrent(void);								//鳳菁攫怀萇霜
void GetCapVoltage(void);									//黍萇萇揤硉 
void Cal_Power(void);										//萇耀輸ADC杅擂賤呾
			
/* NEW:萇耀輸諷秶滲杅 */
void SetPrimaryDC(enum DC_State State);						//扢离場撰DC-PC4腔羲壽
void SetScondaryDC(enum DC_State State);					//扢离棒撰DC-PC5腔羲壽
void SetChassisPowerSupply(enum EX_STATE SetState);			//离樟萇萇埭鼎跤
void SetCAPCurrent(float CAP_RI);							//扢隅萇喃萇萇霜 
void CAP_State_Config(void);								//袨怓瓚剿
void Power_Supply_Ctrl(void);								//萇埭奪燴諷秶
void Set_Chassis_Judge_State(void);					//设置底盘功率检测
/* 萇耀輸笭猁換覜杅擂 */ 
extern float cap_volt;										//萇萇揤
extern float current_i[2];									//萇霜杅擂ㄩ場撰DC怀堤萇霜PC0 

/******/
//void CapUsedOut_Ctrl(void);
//void ToBattery(void);
//void ToCap(void);//����ǰ��Ҫ������״̬��exstate�� 
//void OpenDcDc(void);
//void SetCAPCurrent(float CAP_RI);
//void GetCapVoltage(void);
//void SuperCapCtrl(void);
//void Cal_Power(void);
//void Auto_Charge(void);
////void StateJudge(void);
///*
//�°湦�ʿ���ʹ�õĺ���
//*/
//void Power_Ctrl(float Power_Max);
//void Power_Control_test(void);
//float Motor_Theory_Power(float avg_speed,float output_i);
//float Power_to_ref(float m_power,float avg_speed,float max);
//extern uint32_t cap_temp;
//extern float cap_volt;
//extern uint16_t cap_current_value;
//extern float current_i[2];
//extern float power_max;
//enum EX_STATE{_CAP=0,_BATTERY=1};
//enum CAP_Ctrl{BATTERY=0,CHARGE=1,DISCHARGE=2,CAP=3,TO_CAP=4,TO_BATTERY=5};
//enum CAP_Auto{CAP_MANUAL=0,CAP_AUTO=1};
//enum CAP_Status{CAP_USEDOUT=0,CAP_OK=1};

//extern enum EX_STATE exstate;
//extern enum CAP_Ctrl EX_CTRL;
//extern enum CAP_Ctrl CAP_CTRL;
//extern enum CAP_Auto CAP_AUTO_CTRL;
//extern enum CAP_Status CAP_STA;


#endif
