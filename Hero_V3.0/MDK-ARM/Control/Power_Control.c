#include "Power_Control.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_dac.h"
#include "stm32f4xx_hal.h"
#include "dac.h"
#include "gpio.h"
#include "adc.h"
#include "Referee_Decode.h"
#include "Chassis_Control.h"
#include "math.h"

enum Chassis_Movement_Sta Movement_Status;//���ڵ���״̬ʶ��
enum Auto_Charge_Switch Charge_Switch = SWITCH_OFF,ex_Charge_Switch = SWITCH_OFF;        //���ڿ��Ƴ�翪��

float Buffer_to_Current=4.5f;
float theory_to_fact=0.6f;//mÛֵսʵֵ݊քתۻѶÊ

float theory_power=0,origin_theory_power=0;//mÛ؜٦Ê
float Motor_power[4]={0};//ÿٶ֧ܺքmÛ٦Ê
float Origin_Motor_power[4]={0};
//float dcrease_ratio=0.8f;
float total_output=0;
float temp_i=0;
//新
/********/
//float theory_to_fact=1.3f;								//NEW:燴蹦硉善妗暱硉腔蛌遙炵杅(迵換雄虴薹眈壽)
//float theory_power=0;											//NEW:燴蹦賤呾髡薹
//float origin_theory_power=0;							//燴蹦軞髡薹
//float Motor_power[4]={0};									//藩跺萇儂腔燴蹦髡薹
//float Origin_Motor_power[4]={0};					//帤迉熬藩跺萇儂腔燴蹦髡薹
float dcrease_ratio=1.0f;									//脹掀瞰迉熬炵杅ㄩ場宎峈1.0
float Motor_r=0.194f;											//理论内阻


PID_Parameter_t Pri_DC_PID={10,0,0,15};		//蚚衾場撰DC喃萇髡薹諷秶
float Pri_DC_Power_Ref=80.0f;							//初级DC充电功率Ref

enum EX_STATE exstate=_BATTERY;             //底盘供电状态
enum CAP_Auto CAP_AUTO_CTRL = CAP_AUTO;   //电容自身状态1
enum CAP_Status CAP_STA=CAP_USEDOUT;      
enum DC_State Primary_DC_State = DC_CLOSE,Secondary_DC_State = DC_CLOSE;
enum CHASSIS_STA Chassis_State;

float cap_volt=0;													//萇萇揤
//float Buffer_to_Current=4.5f;						//OLDㄩ蚚衾髡薹癹秶腔渠囥
float i_DC_OUT=0;
float K_DC_I=1.05f;												//喃萇萇霜掀瞰炵杅
//float temp_i = 0;													//隅砱DC喃萇萇霜曹講
float delt_DC_i=0;												//崝講宒敕遠諷秶ㄩDC萇霜
float Set_DC_I=0;
/********/


float Motor_Theory_Power(float avg_speed,float output_i)
{
	return (float)(avg_speed*(output_i/76.0f)/5215.19f);//  /16384.0f*3.14f);
}

float Power_PID_Cal(Motor_t *CMotor,PID_Parameter_t CMpid,float ref)
{
	/*PID Cotrol*/
		float dError,Error,output,Lasterror;
	  static float SUM;
		Error = ref - CMotor->velCtrl.rawvel;
		dError = Error - Lasterror;
	  SUM += Error;
		output = CMpid.kp * Error + CMpid.ki * SUM  + CMpid.kd * dError;  
	
	    //���Ƶ�������ֵ
		if (output > CMpid.outputMax)
		{
			output = CMpid.outputMax;
		}
		else if (output < -CMpid.outputMax)
		{
			output = -CMpid.outputMax;
		}
		return output;
}


void Power_Ctrl(float Power_Max)
	{
	static int stop_mode=0;
	float output_max=0;
	//ͨ����������PID.ref���������۹���
	for(dcrease_ratio=1.0f;dcrease_ratio>=0.20f;dcrease_ratio-=0.06f)
		{
		Motor_power[0]=fabsf(Motor_Theory_Power( CMotor1.encoderCtrl.speed , Power_PID_Cal(&CMotor1,velPara,CMotor1.velCtrl.refvel*dcrease_ratio) ));
		Motor_power[1]=fabsf(Motor_Theory_Power( CMotor2.encoderCtrl.speed , Power_PID_Cal(&CMotor2,velPara,CMotor2.velCtrl.refvel*dcrease_ratio) ));
		Motor_power[2]=fabsf(Motor_Theory_Power( CMotor3.encoderCtrl.speed , Power_PID_Cal(&CMotor3,velPara,CMotor3.velCtrl.refvel*dcrease_ratio) ));
		Motor_power[3]=fabsf(Motor_Theory_Power( CMotor4.encoderCtrl.speed , Power_PID_Cal(&CMotor4,velPara,CMotor4.velCtrl.refvel*dcrease_ratio) ));
		
		if(dcrease_ratio==1.0f)
		{//��¼δ˥��ԭʼrefʱ��������۹���
			for(int i=0;i<4;i++)
			{
				Origin_Motor_power[i]=Motor_power[i];
				origin_theory_power+=Origin_Motor_power[i];
				
			}
		}
		//theory_power=Motor_power[0]+Motor_power[1]+Motor_power[2]+Motor_power[3];
		theory_power=theory_to_fact*(Motor_power[0]+Motor_power[1]+Motor_power[2]+Motor_power[3]);
		
		if(theory_power<Power_Max){//������ֵС�����ֵ�����ô���Ϊ���
		CMotor1.velCtrl.refvel = CMotor1.velCtrl.refvel*dcrease_ratio;
		CMotor2.velCtrl.refvel = CMotor2.velCtrl.refvel*dcrease_ratio;
		CMotor3.velCtrl.refvel = CMotor3.velCtrl.refvel*dcrease_ratio;
		CMotor4.velCtrl.refvel = CMotor4.velCtrl.refvel*dcrease_ratio;
			stop_mode=0;
			break;
		}
		//�ж��Ƿ�Ϊɲ��ģʽ
		if(theory_power>Power_Max && fabsf(CMotor1.velCtrl.refvel)<=30.0f && fabsf(CMotor2.velCtrl.refvel)<=30.0f && fabsf(CMotor3.velCtrl.refvel)<=30.0f && fabsf(CMotor4.velCtrl.refvel)<=30.0f)
			{
				stop_mode=1;
				dcrease_ratio=1.0f;
				break;
			}
			else if(ChassisRef.forward_back_ref == 0&& ChassisRef.left_right_ref == 0)
			{
				stop_mode=1;
				dcrease_ratio=1.0f;
				break;
			}
			else
			{
				stop_mode = 0;
			}
		}
		if(theory_power>Power_Max && !stop_mode){
			for(dcrease_ratio=0.9f;dcrease_ratio>=0.4f;dcrease_ratio-=0.05f){	
				if(theory_power<=Power_Max){
					dcrease_ratio+=0.05f;
					break;
				}
				Motor_power[0]=fabsf(Motor_Theory_Power( CMotor1.encoderCtrl.speed , Power_PID_Cal(&CMotor1,velPara,CMotor1.velCtrl.refvel*dcrease_ratio+(1-dcrease_ratio)*CMotor1.velCtrl.rawvel) ));
				Motor_power[1]=fabsf(Motor_Theory_Power( CMotor2.encoderCtrl.speed , Power_PID_Cal(&CMotor2,velPara,CMotor2.velCtrl.refvel*dcrease_ratio+(1-dcrease_ratio)*CMotor2.velCtrl.rawvel) ));
				Motor_power[2]=fabsf(Motor_Theory_Power( CMotor3.encoderCtrl.speed , Power_PID_Cal(&CMotor3,velPara,CMotor3.velCtrl.refvel*dcrease_ratio+(1-dcrease_ratio)*CMotor3.velCtrl.rawvel) ));
				Motor_power[3]=fabsf(Motor_Theory_Power( CMotor4.encoderCtrl.speed , Power_PID_Cal(&CMotor4,velPara,CMotor4.velCtrl.refvel*dcrease_ratio+(1-dcrease_ratio)*CMotor4.velCtrl.rawvel) ));
				//theory_power=Motor_power[0]+Motor_power[1]+Motor_power[2]+Motor_power[3];
				theory_power=(Motor_power[0]+Motor_power[1]+Motor_power[2]+Motor_power[3])*theory_to_fact;		
				
			}		
			CMotor1.velCtrl.refvel = CMotor1.velCtrl.refvel*dcrease_ratio+(1-dcrease_ratio)*CMotor1.velCtrl.rawvel;//CMspeed1.pid.ref*dcrease_ratio+(1-dcrease_ratio)*CMspeed1.pid.fdb;
			CMotor2.velCtrl.refvel = CMotor2.velCtrl.refvel*dcrease_ratio+(1-dcrease_ratio)*CMotor2.velCtrl.rawvel;//CMspeed2.pid.ref*dcrease_ratio+(1-dcrease_ratio)*CMspeed2.pid.fdb;
			CMotor3.velCtrl.refvel = CMotor3.velCtrl.refvel*dcrease_ratio+(1-dcrease_ratio)*CMotor3.velCtrl.rawvel;//CMspeed3.pid.ref*dcrease_ratio+(1-dcrease_ratio)*CMspeed3.pid.fdb;
			CMotor4.velCtrl.refvel = CMotor4.velCtrl.refvel*dcrease_ratio+(1-dcrease_ratio)*CMotor4.velCtrl.rawvel;//CMspeed4.pid.ref*dcrease_ratio+(1-dcrease_ratio)*CMspeed4.pid.fdb;
		}

		Motor_VelPID(&CMotor1.velCtrl);                 
		Motor_VelPID(&CMotor2.velCtrl);
		Motor_VelPID(&CMotor3.velCtrl);
		Motor_VelPID(&CMotor4.velCtrl);
	//����������ʧ���Ҳ��Ǽ�ͣģʽ ��������������ȫ��Χ ��ʹ��80W���ʷ������
	if( (theory_power>Power_Max && !stop_mode) || PowerHeatData.chassisPowerBuffer<25 ||(ChassisRef.forward_back_ref>=300&&PowerHeatData.chassisPower<=15))
		{//4800ʱ80W�µ����OUTPUT_SUM
		float K=( (origin_theory_power*theory_to_fact)>=80.0f ? 80.0f : (origin_theory_power*theory_to_fact) ) /80.0f;
		output_max=3000*K;
		CMotor1.velCtrl.output=(CMotor1.velCtrl.output>=0?1:(CMotor1.velCtrl.output<0))*Origin_Motor_power[0]/origin_theory_power*output_max;
							//*Power_to_ref(Power_Max*Origin_Motor_power[0]/origin_theory_power,CMspeed1.encoder.avg_speed,CM1pid.outputMax);
		CMotor2.velCtrl.output=(CMotor2.velCtrl.output>=0?1:(CMotor2.velCtrl.output<0))*Origin_Motor_power[1]/origin_theory_power*output_max;
							//*Power_to_ref(Power_Max*Origin_Motor_power[1]/origin_theory_power,CMspeed2.encoder.avg_speed,CM2pid.outputMax);
		CMotor3.velCtrl.output=(CMotor3.velCtrl.output>=0?1:(CMotor3.velCtrl.output<0))*Origin_Motor_power[2]/origin_theory_power*output_max;
							//*Power_to_ref(Power_Max*Origin_Motor_power[2]/origin_theory_power,CMspeed3.encoder.avg_speed,CM3pid.outputMax);
		CMotor4.velCtrl.output=(CMotor4.velCtrl.output>=0?1:(CMotor4.velCtrl.output<0))*Origin_Motor_power[3]/origin_theory_power*output_max;
							//*Power_to_ref(Power_Max*Origin_Motor_power[3]/origin_theory_power,CMspeed4.encoder.avg_speed,CM4pid.outputMax);
		if(CMotor4.velCtrl.output >= 1750)
		{
			CMotor4.velCtrl.output = 1750;
		}
		if(CMotor3.velCtrl.output >= 1750)
		{
			CMotor3.velCtrl.output = 1750;
		}
		if(CMotor2.velCtrl.output >= 1750)
		{
			CMotor2.velCtrl.output = 1750;
		}
		if(CMotor1.velCtrl.output >= 1750)
		{
			CMotor1.velCtrl.output >= 1750;
		}
				if(CMotor4.velCtrl.output <= -1750)
		{
			CMotor4.velCtrl.output = -1750;
		}
		if(CMotor3.velCtrl.output <= -1750)
		{
			CMotor3.velCtrl.output = -1750;
		}
		if(CMotor2.velCtrl.output <= -1750)
		{
			CMotor2.velCtrl.output = -1750;
		}
		if(CMotor1.velCtrl.output <= -1750)
		{
			CMotor1.velCtrl.output = -1750;
		}
	}  
}
/***
	*�������ƣ�CapUsedOut_Ctrl
	*�������ܣ�����û��ʱ��������������80Wһ�£��ָ���������
	*��ڲ���: ��
	*����ֵ  ����
***/
//void CapUsedOut_Ctrl()
//{
//	if(PowerHeatData.chassisPowerBuffer<50 )// && total_power[0] > 60)
//		{
//				//if (total_power[0] > 60)
//			{
//				float Current_Sum = fabs(CMotor1.velCtrl.output) + fabs(CMotor2.velCtrl.output) + fabs(CMotor3.velCtrl.output) + fabs(CMotor4.velCtrl.output);
//				float Current_Adjust = Buffer_to_Current * PowerHeatData.chassisPowerBuffer * PowerHeatData.chassisPowerBuffer;
//				
//				CMotor1.velCtrl.output = Current_Adjust * CMotor1.velCtrl.output / Current_Sum;
//				CMotor2.velCtrl.output = Current_Adjust * CMotor2.velCtrl.output / Current_Sum;
//				CMotor3.velCtrl.output = Current_Adjust * CMotor3.velCtrl.output / Current_Sum;
//				CMotor4.velCtrl.output = Current_Adjust * CMotor4.velCtrl.output / Current_Sum;
//			  
//				CMotor1.velCtrl.sum = 0;
//				CMotor2.velCtrl.sum = 0;
//				CMotor3.velCtrl.sum = 0;
//				CMotor4.velCtrl.sum = 0;
//			}
//		}
//}

/***
	*�������ƣ�OpenDcDc
	*�������ܣ�����DC-DC
	*��ڲ���: ��  
	*����ֵ  ����
***/
//void OpenDcDc(){
//  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
//  HAL_Delay(80);
//  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
//  SetCAPCurrent(0);//���ú�ѹ�������Ϊ0
//}
/***
	*�������ƣ�Battery
	*�������ܣ��л�����ع���״̬ 
	*��ڲ���: ��
	*����ֵ  ����
***/
//void ToBattery(void)
//{
//	if(exstate == _CAP)
//	{
////		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
////		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
//		//HAL_Delay(8);
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
//		exstate = _BATTERY;
//		HAL_Delay(8);
//	}
//	else
//	{
//		//do nothing
//	}
//}

/***
	*�������ƣ�ToCap
	*�������ܣ��л������ݹ���״̬ 
	*��ڲ���: ��
	*����ֵ  ����
***/
//void ToCap(void) 
//{
//	if(exstate == _BATTERY)
//	{
////		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
////		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
//		//HAL_Delay(8);
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
//		exstate = _CAP;
//		HAL_Delay(8);
//	}
//	else
//	{
//		//do nothing
//	}
//}

/***
	*滲杅靡備ㄩSetCAPCurrent
	*滲杅髡夔ㄩNEW:扢隅萇喃萇萇霜 
	*諳統杅: CAP_RI 場撰DC喃萇萇霜
	*殿隙硉  ㄩ拸
***/
void SetCAPCurrent(float CAP_RI)
{
	//float current = 1280 * CAP_RI + 60;
	/* NEW:0~2V 勤茼0~20A */
	uint32_t current=0;
	if(CAP_RI<=0.05f)	current=0;					//喃萇萇霜扢隅侚
	else if(CAP_RI>=15.0f)	current=1861;			//扢隅喃萇萇霜奻癹祥閉徹15A: 15/20*4096*2/3.3f;
	else	current = (uint32_t)(CAP_RI*124.12f);	// CAP_RI*4096*2/3.3f/20;
	
	HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,current); 
}
/***
	*滲杅靡備ㄩGetCapVoltage
	*滲杅髡夔ㄩNEW:萇萇揤硉賤呾
	*諳統杅: 拸
	*殿隙硉  ㄩ拸 
***/
void GetCapVoltage(void)
{
	uint32_t cap_temp = value_res[0];
	//cap_volt = (float)cap_temp*26/2720.0f*0.82f-0.65f;//EX:
	if(cap_temp<2100){
			cap_volt=(float)(cap_temp*0.01f+3.22f);
	}else{
			cap_volt=(float)(cap_temp*cap_temp*1.1994f/100000-0.0333f*cap_temp+40.1631f);
	}
	//cap_volt=(float)(cap_temp*0.0133f-6.1167f);//110*3.3f/4096.0f);勤坋豻綺謹ADC腔呴儂曹
	if(cap_volt<=0 | cap_volt>32.4f)	cap_volt=0;//萇祑都杅擂賤呾﹝
}
/***
	*�������ƣ�SuperCapCtrl
	*�������ܣ����������Զ����Ʋ���
	*��ڲ���: ��
	*����ֵ  ���� 
***/
void Cal_Power(){
	/* 髡薹摯髡薹船煦數呾*/
	if(1){//current_base[0] && current_base[1]
		/*鳳萇萇揤*/
		GetCapVoltage();		
		if(cap_volt<0)	cap_volt=0;
		
		/* NEW:萇霜杅擂賤呾 */
		i_DC_OUT=GetDCCurrent();				//場撰DC怀堤萇霜
		float i_Chassis_OUT=GetChassisCurrent();	//菁攫怀萇霜PC1
		/* NEW:髡薹杅擂賤呾 */
		if(cap_volt>0){
			DC_DC_Power=(cap_volt- 1.4f)*i_DC_OUT;//current_i[0];
		}else{
			DC_DC_Power=0.0f;
		}
		/* total_power [0]絞軞髡薹硉 | [1]珨棒髡薹硉 */
		total_power[1]=total_power[0];
		total_power[0]=(float)(PowerHeatData.chassisVolt*i_Chassis_OUT/1000+C_P_OFFSET+DC_DC_Power);//current_i[1]
		/* 髡薹船煦數呾 */
		if(total_power[1]<0){
			diff_t_p=0;
		}else{
			diff_t_p=total_power[0]-total_power[1];
		}
	}else{
		//DC_DC_Power=0;
		total_power[1]=total_power[0];
		total_power[0]=0;
		diff_t_p=0;
		cap_volt=0;
	}
}

void Auto_Charge(){
	/* 瓚剿絞腔鼎萇袨怓扢离喃萇萇霜 */
	/* 喃萇萇霜諷秶 */
	
	/* 萇喀鼎萇ㄩ菁攫腴髡薹奀瞳蚚呁豻髡薹喃萇 */
	if(exstate==_BATTERY){
	/* 羲遠諷秶 */	
		/* 扢离DC-DC馱釬袨怓 */
		SetScondaryDC(DC_CLOSE);	//壽敕棒撰DC-DC
		SetPrimaryDC(DC_OPEN);		//羲場撰DC-DC
		//if((PowerHeatData.chassisPower-DC_DC_Power) < 15.0f){
									//彆菁攫髡薹腴衾15Wㄗ假毓峓ㄘ -◎蚚蜓啥髡薹跤萇喃萇ㄗ箝髡薹ㄘ
			if(cap_volt >= 23.0f){
					temp_i=2.8f;
			}else if(cap_volt >= 8.0f){
					temp_i = (80.0f-9.0f)/cap_volt;//(-PowerHeatData.chassisPower+DC_DC_Power)
									//箝髡薹喃萇
			}else{
				temp_i = 8.0f;		//腴萇揤厒喃萇
			}
			//temp_i = temp_i>8 ? 8:temp_i;//15A癹盟
		//}else{
		//	temp_i=0;				//蜓啥髡薹腴衾假硉ㄛ祥喃萇
		//}
			
	/* 敕遠諷秶 */
		//	/* 扢离DC-DC馱釬袨怓 */
		//	SetPrimaryDC(DC_OPEN);		//羲場撰DC-DC
		//	/* 喃萇髡薹敕遠諷秶 */
		//	float Pri_DC_Power_Fdb = DC_DC_Power;//
		//	static float Pri_Dc_Output=0;
		//	static float Power_Error[2]={0};
		//	Power_Error[1]=Power_Error[0];
		//	Power_Error[0]=Pri_DC_Power_Ref-Pri_DC_Power_Fdb;
		//	Pri_Dc_Output += Pri_DC_PID.kp*(Power_Error[0]-Power_Error[1]) + Pri_DC_PID.ki*Power_Error[0];
		//	/* 癹盟揭燴 */
		//	if(Pri_Dc_Output<=0){
		//		Pri_Dc_Output=0;
		//		Power_Error[0]=0;
		//	}else if(Pri_Dc_Output>=15.0f){
		//		Pri_Dc_Output=15.0f;
		//	}
		//	/* 笛瓚炵苀髡薹豻講悵誘 */
		//	if(PowerHeatData.chassisPowerBuffer<50){
		//		Pri_Dc_Output=2.5f;
		//	}
		//	
		//	SetCAPCurrent(Pri_Dc_Output);
	}else if(exstate==_CAP){
		/* 萇鼎萇耀宒ㄛ森奀剒猁場撰DC眕80W箝髡薹怀堤 */
		/* 扢离DC袨怓 */
		SetPrimaryDC(DC_OPEN);
		SetScondaryDC(DC_OPEN);
		//SetChassisPowerSupply(_CAP);
		/* 扢离場撰DC怀堤萇霜 */
//敕遠諷秶
//		switch(CAP_STA){
//			case CAP_FULL:
//				temp_i=4.75f;
//				break;
//			case CAP_OK:
//					temp_i = K_DC_I*(75.0f)/(cap_volt+1);//箝髡薹喃萇(-PowerHeatData.chassisPower+DC_DC_Power)
//				break;
//			case CAP_USEDOUT:
//				break;
//		}
//羲遠諷秶	
			if(cap_volt >= 24.5f){
						temp_i=2.8f;
			}else if(cap_volt >= 8.0f){
				temp_i = K_DC_I*(70.0f)/(cap_volt+1);
				//temp_i = (80.0f-6.5f)/cap_volt;//箝髡薹喃萇(-PowerHeatData.chassisPower+DC_DC_Power)
			}else{
				temp_i = 8.0f;//腴萇揤厒喃萇
			}
			
			//temp_i=Set_DC_I;		//聆彸蚚ㄩDEBUG董硉聆彸
	}
	
	
	temp_i = temp_i>8.0f ? 8.0f:temp_i;//8A癹盟
	/* 笛瓚炵苀髡薹豻講悵誘 */
	static int buffer_sta=0;
	if(PowerHeatData.chassisPowerBuffer==60){
		buffer_sta=0;
	}
	
	if(PowerHeatData.chassisPowerBuffer<20){
		temp_i=2.6f;
		buffer_sta=1;
	}else if(buffer_sta == 0 & PowerHeatData.chassisPowerBuffer<60 ){
		temp_i=2.6f+((temp_i-2.6f)>0? (temp_i-2.6f):0)*(PowerHeatData.chassisPowerBuffer-30.0f)/30.0f;
	}
	
	SetCAPCurrent(temp_i);
}
	
float GetDCCurrent(void){
	return (float)(value_filter[1]*110.0f/4096.0f*0.29f);//Vi_out=0.03*I_out;//*3.3f/4096.0f/0.03f
}
	
float GetChassisCurrent(void){
	if(current_base[1]>0){//場宎趙俇傖ㄛ價袧硉隅
		return (float)(((float)value_filter[2]-(float)current_base[1])*33.0f/4096.0f);
	}else{
		return 0;
	}
}

void SetPrimaryDC(enum DC_State State){
	if(State==DC_OPEN){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);		//場撰DC-DC諷秶場宎趙ㄩ羲
	}else if(State==DC_CLOSE){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);	//場撰DC-DC諷秶場宎趙ㄩ壽敕
	}
	Primary_DC_State=State;
}

void SetScondaryDC(enum DC_State State){
	if(State==DC_OPEN){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);		//棒撰DC-DC諷秶場宎趙ㄩ羲
	}else if(State==DC_CLOSE){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);	//棒撰DC-DC諷秶場宎趙ㄩ壽敕
	}
	Secondary_DC_State=State;
}

void SetChassisPowerSupply(enum EX_STATE SetState){
	if(SetState==_BATTERY){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);	//樟萇萇埭遙諷秶場宎趙ㄩ笛瓚炵苀鼎萇
		SetScondaryDC(DC_CLOSE);								//壽敕萇(棒撰DC)怀堤
	}else if(SetState==_CAP){
		SetScondaryDC(DC_OPEN);									//羲萇(棒撰DC)怀堤
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);		//樟萇萇埭遙諷秶場宎趙ㄩ萇衪肮鼎萇
	}
	exstate=SetState;
}

void CAP_State_Config(){	
	static int count = 0;
	if(cap_volt > 24.5f){
		if(CAP_STA != CAP_FULL)	count++;
		else count = 0;
		if(count > 50){
			CAP_STA = CAP_FULL;
			count = 0;
		}
	}else if(cap_volt > 18.0f){
		if(CAP_STA != CAP_OK)	count++;
		else count = 0;
		if(count > 50){
			CAP_STA = CAP_OK;
			count = 0;
		}
	}else{
		if(CAP_STA != CAP_USEDOUT)	count++;
		else count = 0;
		if(count > 50){
			CAP_STA = CAP_USEDOUT;
			count = 0;
		}
	}
}

void Power_Supply_Ctrl()
	{
	/* 底盘供电状态监测 */
	Set_Chassis_Judge_State();
	/* 換覜杅擂賤呾 */
	Cal_Power();
	/* 萇袨怓扢离 */
	CAP_State_Config();
/* 跦擂萇諷秶袨怓樵隅萇埭遙習謹 */
	if(CAP_AUTO_CTRL == CAP_AUTO)
	{
			/* 检测裁判系统地盘宫殿状态 */
		if(Chassis_State==JUDGE_OFF&&RC_Ctrl_Data.remote.s2 == Check_Mode){
			SetChassisPowerSupply(_BATTERY);
		}else{
		/* 赻雄諷秶 */
			if(exstate == _BATTERY){
			/* 萇喀鼎萇袨怓 */
				if(cap_volt > 22.0f){
				/* 萇雛萇ㄩ遙峈萇鼎萇 */
					SetChassisPowerSupply(_CAP);
				}else{
				/* 萇帤雛ㄩ赻雄喃萇+峎厥萇喀鼎萇 */
											/* ����û��ʹ�õ�ع��� */
							SetChassisPowerSupply(_BATTERY);        
							//ex_Charge_Switch = Charge_Switch;
							/* ������������ʽϵ�ʱ��ʹ��80W��������ݳ�� */
							if((Movement_Status == CAR_STOP 
							&& PowerHeatData.chassisPowerBuffer == 60 
							&& PowerHeatData.chassisPower <= 20.0f ) 
							|| Charge_Switch == SWITCH_ON ){
									Auto_Charge();
									Charge_Switch = SWITCH_ON;
									if(Movement_Status != CAR_STOP ){
											Charge_Switch = SWITCH_OFF;
									}
							}else{
									/* ����DC������ */
									SetCAPCurrent(0.0f);                            //ֹͣ���
									Charge_Switch = SWITCH_OFF;
							}    

	//				Auto_Charge();
	//				SetChassisPowerSupply(_BATTERY);
				}
			/* 萇鼎萇袨怓 */	
			}else if(exstate == _CAP){
				/* 瓚剿萇袨怓 */
				if(CAP_STA==CAP_USEDOUT){
				/* 萇羶萇ㄩ遙萇喀鼎萇 */
					SetChassisPowerSupply(_BATTERY);
				}else if(CAP_STA==CAP_OK || CAP_STA==CAP_FULL){
				/* 箝髡薹怀堤 */
					Auto_Charge();
				}
				
			}
		}
	}else if(CAP_AUTO_CTRL == CAP_MANUAL)
	{
	/* 忒雄遙 */
		if(exstate == _BATTERY){
		/* 萇喀鼎萇袨怓 */
			SetChassisPowerSupply(_BATTERY);
		}else if(exstate == _CAP){
		/* 萇鼎萇袨怓 */
			SetChassisPowerSupply(_CAP);
			Auto_Charge();
		}
	}
}

//void SuperCapCtrl(void)
//{
//	
//	Cal_Power();//���㵱ǰ���̹��� | DC/DC���� �� ���ʲ��
//	//���õ��ݵĵ�ǰ״̬���ͻ����ߣ�
//	if(cap_volt>22.5f){
//		CAP_STA=CAP_OK;
//	}else if(cap_volt<21.0f){
//		CAP_STA=CAP_USEDOUT;
//	}
//	if(CAP_AUTO_CTRL==CAP_AUTO)
//	{//�Զ�����-start
//		if(exstate==_BATTERY)
//		{//�����ǵ�ع���
//			if(( total_power[0]>=70.0f) || (PowerHeatData.chassisPower-DC_DC_Power >=78.0f) )
//			{	//����ϵͳ���ʸ���80W -���л��������ݹ��� PowerHeatData.chassisPowerBuffer<45&PowerHeatData.chassisPowerBuffer>=5
//				SetCAPCurrent(0);	//�ر�DC-DCЭͬ����
//				if(CAP_STA==CAP_OK)
//				{	//�жϵ���״̬�ܷ�ʹ��
//					ToCap();//�л����ݹ���
//					//SetCAPCurrent(2.5);//����DC-DCЭͬ����
//				}
//				else 
//				{
//					ToBattery();//�������û�磬��ά�ֵ�ع���
//				}
//			}else //if(PowerHeatData.chassisPower<=80)
//			{//����ϵͳ���ʵ���80W -�� ��ع���
//				ToBattery();	//���ֵ�ع���
//				Auto_Charge();//�Զ����ó�����
//			}
//			
//		}
//		else if(exstate==_CAP)
//		{
//			static int tim_count;
//			if(tim_count<TIM_MAX)	
//			{
//				tim_count++;//�������ݹ���֮�󣬿�����ʱ����
//			}
//			else	
//			{
//				tim_count=TIM_MAX;//TIM_MAX ms��������������
//			}
//			SetCAPCurrent(0);//SetCAPCurrent(0.0f*tim_count/TIM_MAX); //���ó���������ǰЭͬ���������ȶ�	
//			if((CAP_STA==CAP_USEDOUT) || (total_power[0]<=70.0f && tim_count>20) )
//			{//���ݵ�ѹ���ڰ�ȫֵ �� �����������С��2.5A(�������ֵ��Ҫ���ݲο��������ٵ���)  | value_res[2]<3250
//				SetCAPCurrent(0);//�ر�DC-DC����
//				ToBattery();//����Ϊ��ع���
//				tim_count=0;
//			}
//		}
//	}//�Զ�����-end
//	else if(CAP_AUTO_CTRL == CAP_MANUAL)
//	{
//		if(Power_Mode == Cap)
//		{
//			//if( ((diff_t_p>0?diff_t_p:0)>6.0f && total_power[0]>=70.0f) )//|| (Power_Heat_Data.chassis_power+(diff_t_p>0?diff_t_p:0) >=80.0f))
//			//{	//����ϵͳ���ʸ���80W -���л��������ݹ��� PowerHeatData.chassisPowerBuffer<45&PowerHeatData.chassisPowerBuffer>=5
//				SetCAPCurrent(0);	//�ر�DC-DCЭͬ����
//				//if(cap_volt>19.5f&&CAP_STA==CAP_OK)
//				//{	//�жϵ����Ƿ���ڰ�ȫֵ ����һ�η��������ѹΪ����
//					ToCap();//�л����ݹ���
//					Motor_VelPID(&CMotor1.velCtrl);
//        	Motor_VelPID(&CMotor2.velCtrl);
//        	Motor_VelPID(&CMotor3.velCtrl);
//         	Motor_VelPID(&CMotor4.velCtrl);
//				//}
////				else
////				{
////					Power_Mode = Battery;
////				}
//					
//		}
//		
//		else if(Power_Mode == Battery)
//		{
//          	ToBattery();	//���ֵ�ع���
//				   Auto_Charge();
//					Power_Ctrl(80);
//		}
//	}
//}





//int output_sum;
//float decay_ratio;
//void Power_Control_test()
//{
//	Motor_VelPID(&CMotor1.velCtrl);
//  Motor_VelPID(&CMotor2.velCtrl);
//  Motor_VelPID(&CMotor3.velCtrl);
//  Motor_VelPID(&CMotor4.velCtrl);
//	output_sum =__fabs(CMotor1.velCtrl.output) + __fabs(CMotor2.velCtrl.output) + __fabs(CMotor3.velCtrl.output) + __fabs(CMotor4.velCtrl.output);
//	if(output_sum > 2730 && PowerHeatData.chassisPower > 80)
//	{
//		for(decay_ratio = 1.0 ; decay_ratio > 0 ;decay_ratio -= 0.1)
//		{
//			ChassisRef.forward_back_ref = decay_ratio * ChassisRef.forward_back_ref;
//			ChassisRef.left_right_ref   = decay_ratio * ChassisRef.left_right_ref;
//		  ChassisRef.rotate_ref       = decay_ratio * ChassisRef.rotate_ref;
//			
//			CMotor1.velCtrl.refvel =  ChassisRef.forward_back_ref * REMOTE_CHASIS_SPEED_TO_REF_FB
//	                             +ChassisRef.left_right_ref   * REMOTE_CHASIS_SPEED_TO_REF_LR
//	                             +ChassisRef.rotate_ref       * ROTATE_TO_REF; 
//		  CMotor2.velCtrl.refvel = -ChassisRef.forward_back_ref * REMOTE_CHASIS_SPEED_TO_REF_FB 
//	                             +ChassisRef.left_right_ref   * REMOTE_CHASIS_SPEED_TO_REF_LR
//	                             +ChassisRef.rotate_ref       * ROTATE_TO_REF;
//		  CMotor3.velCtrl.refvel = -ChassisRef.forward_back_ref * REMOTE_CHASIS_SPEED_TO_REF_FB 
//	                             -ChassisRef.left_right_ref   * REMOTE_CHASIS_SPEED_TO_REF_LR	
//	                             +ChassisRef.rotate_ref       * ROTATE_TO_REF;
//		  CMotor4.velCtrl.refvel =  ChassisRef.forward_back_ref * REMOTE_CHASIS_SPEED_TO_REF_FB 
//	                             -ChassisRef.left_right_ref   * REMOTE_CHASIS_SPEED_TO_REF_LR      
//	                             +ChassisRef.rotate_ref       * ROTATE_TO_REF;
//			Motor_VelPID(&CMotor1.velCtrl);
//			Motor_VelPID(&CMotor2.velCtrl);
//			Motor_VelPID(&CMotor3.velCtrl);
//			Motor_VelPID(&CMotor4.velCtrl);
//			output_sum =__fabs(CMotor1.velCtrl.output) + __fabs(CMotor2.velCtrl.output) + __fabs(CMotor3.velCtrl.output) + __fabs(CMotor4.velCtrl.output);
//			if(output_sum <= 2730 || PowerHeatData.chassisPower <= 80)
//			{
//				break;
//			}
//		}			
//	}
//	else
//	{
//		CM_CAN_Send_Msg();
//	}		
//}
	

void Set_Chassis_Judge_State(){
	static int count = 0;

	 	if(PowerHeatData.chassisPower <= 4.0f){
			if(Chassis_State != JUDGE_OFF)	count++;
			else count = 0;
			if(count > 10){
				Chassis_State = JUDGE_OFF;
				count = 0;
			}
	}else if(PowerHeatData.chassisPower >=8.0f){
		if(Chassis_State != JUDGE_ON)	count++;
		else count = 0;
		if(count > 10){
			Chassis_State = JUDGE_ON;
			count = 0;
			}
		}
	
}
