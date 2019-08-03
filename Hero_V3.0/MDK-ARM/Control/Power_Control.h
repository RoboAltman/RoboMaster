#ifndef _POWER_CONTROL_H_
#define _POWER_CONTROL_H_
#include "stm32f4xx_hal.h"
#include "Motor.h"
#include "PID.h"
#define CAP_I_OFFSET 0.5f//恒压恒流功率传输效率
#define TIM_MAX 500
//鏂�
/*******/
void SuperCapCtrl(void);
void Auto_Charge(void);

/* 闄斿敵楂¤柟璜风Ф濡忚殮鑵旀徊鏉� */
void Power_Ctrl(float Power_Max);				
float Power_PID_Cal(Motor_t *CMotor,PID_Parameter_t CMpid,float ref);
float Motor_Theory_Power(float avg_speed,float output_i);
float Power_to_ref(float m_power,float avg_speed,float max);

/* 钀囷牕鑰�杓歌ⅷ鎬撴浌璎� */
enum EX_STATE{_CAP=0,_BATTERY=1};												//鑿佹敨榧庤悋琚ㄦ��
enum CAP_Ctrl{BATTERY=0,CHARGE=1,DISCHARGE=2,CAP=3,TO_CAP=4,TO_BATTERY=5};		//钀囷牕璜风Ф琚ㄦ��
enum CAP_Auto{CAP_MANUAL=0,CAP_AUTO=1};											//钀囷牕璜风Ф鑰�瀹掋劑璧婚泟杩靛繏闆�
enum CAP_Status{CAP_USEDOUT=0,CAP_OK=1,CAP_FULL=2};								//钀囷牕璧绘棷琚ㄦ�撱劑铓氫繃 | 澶斿妭濡忚殮 | 闆涜悋
enum DC_State{DC_CLOSE=0,DC_OPEN=1};
enum Chassis_Movement_Sta{CAR_STOP=0,CAR_MOVE=1,CAR_ROTATE=2,CAR_MIXMOVING=3};//锟斤拷锟斤拷锟剿讹拷状态锟叫讹拷                                                    //锟斤拷锟斤拷锟狡讹拷转态
enum Auto_Charge_Switch{SWITCH_ON=1,SWITCH_OFF=0};                                                         //锟斤拷锟斤拷锟皆讹拷锟斤拷诺缈拷锟�
enum CHASSIS_STA{JUDGE_ON=0,JUDGE_OFF=1};									//瑁佸垽绯诲伔鐫�鍧涗緵鐢电姸鎬佹寚绀猴細搴曠洏寮�鍚緵鐢� | 搴曠洏鍏抽棴渚涚數

extern enum Auto_Charge_Switch Charge_Switch,ex_Charge_Switch;        //锟斤拷锟节匡拷锟狡筹拷缈拷锟�


extern enum EX_STATE exstate;													//鑿佹敨榧庤悋琚ㄦ��							
extern enum CAP_Ctrl EX_CTRL;													//钀囷牕濂绘鑵旈紟钀囪ⅷ鎬�
extern enum CAP_Ctrl CAP_CTRL;													//钀囷牕鎺涙鑵旈紟钀囪ⅷ鎬�
extern enum CAP_Auto CAP_AUTO_CTRL;												//钀囷牕鑰�杓稿繏璧伙澖閬欒ⅷ鎬撳紘
extern enum CAP_Status CAP_STA;													//钀囷牕璧绘棷琚ㄦ��
extern enum DC_State Primary_DC_State,Secondary_DC_State;						//DC-DC棣遍嚞琚ㄦ��

extern enum Chassis_Movement_Sta Movement_Status;
extern enum CHASSIS_STA Chassis_State;

/* NEW:钀囷牕鏉呮搨璩ゅ懢婊叉潊 */
float GetDCCurrent(void);									//槌筹煫鍫存挵DC鎬�鍫よ悋闇�
float GetChassisCurrent(void);								//槌筹煫鑿佹敨鎬�餇佃悋闇�
void GetCapVoltage(void);									//榛嶏煫钀囷牕钀囨彜纭� 
void Cal_Power(void);										//钀囷牕鑰�杓窤DC鏉呮搨璩ゅ懢
			
/* NEW:钀囷牕鑰�杓歌绉舵徊鏉� */
void SetPrimaryDC(enum DC_State State);						//鎵㈢鍫存挵DC-PC4鑵旂静澹�
void SetScondaryDC(enum DC_State State);					//鎵㈢妫掓挵DC-PC5鑵旂静澹�
void SetChassisPowerSupply(enum EX_STATE SetState);			//绂绘钀囷渿钀囧煭榧庤筏
void SetCAPCurrent(float CAP_RI);							//鎵㈤殔钀囷牕鍠冭悋钀囬湝 
void CAP_State_Config(void);								//餇цⅷ鎬撶摎鍓�
void Power_Supply_Ctrl(void);								//钀囧煭濂嚧璜风Ф
void Set_Chassis_Judge_State(void);					//璁剧疆搴曠洏鍔熺巼妫�娴�
/* 钀囷牕鑰�杓哥鐚佹彌瑕滐渿鏉呮搨 */ 
extern float cap_volt;										//钀囷牕钀囨彜
extern float current_i[2];									//钀囬湝鏉呮搨銊╁牬鎾癉C鎬�鍫よ悋闇淧C0 

/******/
//void CapUsedOut_Ctrl(void);
//void ToBattery(void);
//void ToCap(void);//调用前需要先声明状态（exstate） 
//void OpenDcDc(void);
//void SetCAPCurrent(float CAP_RI);
//void GetCapVoltage(void);
//void SuperCapCtrl(void);
//void Cal_Power(void);
//void Auto_Charge(void);
////void StateJudge(void);
///*
//新版功率控制使用的函数
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
