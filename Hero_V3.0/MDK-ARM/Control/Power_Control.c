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

enum Chassis_Movement_Sta Movement_Status;//ï¿½ï¿½ï¿½Úµï¿½ï¿½ï¿½×´Ì¬Ê¶ï¿½ï¿½
enum Auto_Charge_Switch Charge_Switch = SWITCH_OFF,ex_Charge_Switch = SWITCH_OFF;        //ï¿½ï¿½ï¿½Ú¿ï¿½ï¿½Æ³ï¿½ç¿ªï¿½ï¿½

float Buffer_to_Current=4.5f;
float theory_to_fact=0.6f;//mÃ›ÖµÕ½ÊµİŠÖµÖ„×ªÛ»Ñ¶ÃŠ

float theory_power=0,origin_theory_power=0;//mÃ›ØœÙ¦ÃŠ
float Motor_power[4]={0};//Ã¿Ù¶Ö§ÜºÖ„mÃ›Ù¦ÃŠ
float Origin_Motor_power[4]={0};
//float dcrease_ratio=0.8f;
float total_output=0;
float temp_i=0;
//æ–°
/********/
//float theory_to_fact=1.3f;								//NEW:ç‡´è¹¦ç¡‰å–„å¦—æš±ç¡‰è…”è›Œé™ç‚µæ…(è¿µæ›é›„è™´è–¹çœˆå£½)
//float theory_power=0;											//NEW:ç‡´è¹¦è³¤å‘¾é«¡è–¹
//float origin_theory_power=0;							//ç‡´è¹¦è»é«¡è–¹
//float Motor_power[4]={0};									//è—©è·ºè‡å„‚è…”ç‡´è¹¦é«¡è–¹
//float Origin_Motor_power[4]={0};					//å¸¤è¿‰ç†¬ïè—©è·ºè‡å„‚è…”ç‡´è¹¦é«¡è–¹
float dcrease_ratio=1.0f;									//è„¹æ€ç°è¿‰ç†¬ç‚µæ…ã„©å ´å®å³ˆ1.0
float Motor_r=0.194f;											//ç†è®ºå†…é˜»


PID_Parameter_t Pri_DC_PID={10,0,0,15};		//èššè¡¾å ´æ’°DCå–ƒè‡é«¡è–¹è«·ç§¶
float Pri_DC_Power_Ref=80.0f;							//åˆçº§DCå……ç”µåŠŸç‡Ref

enum EX_STATE exstate=_BATTERY;             //åº•ç›˜ä¾›ç”µçŠ¶æ€
enum CAP_Auto CAP_AUTO_CTRL = CAP_AUTO;   //ç”µå®¹è‡ªèº«çŠ¶æ€1
enum CAP_Status CAP_STA=CAP_USEDOUT;      
enum DC_State Primary_DC_State = DC_CLOSE,Secondary_DC_State = DC_CLOSE;
enum CHASSIS_STA Chassis_State;

float cap_volt=0;													//è‡ï §è‡æ¤
//float Buffer_to_Current=4.5f;						//OLDã„©èššè¡¾é«¡è–¹ç™¹ç§¶è…”æ¸ å›¥
float i_DC_OUT=0;
float K_DC_I=1.05f;												//å–ƒè‡è‡éœœæ€ç°ç‚µæ…
//float temp_i = 0;													//éš…ç ±DCå–ƒè‡è‡éœœæ›¹è¬›
float delt_DC_i=0;												//å´è¬›å®’æ••é è«·ç§¶ã„©DCè‡éœœ
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
	
	    //ÏŞÖÆµç»úÊä³ö·ùÖµ
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
	//Í¨¹ı±ÈÀıËõ¼õPID.refÀ´½µµÍÀíÂÛ¹¦ÂÊ
	for(dcrease_ratio=1.0f;dcrease_ratio>=0.20f;dcrease_ratio-=0.06f)
		{
		Motor_power[0]=fabsf(Motor_Theory_Power( CMotor1.encoderCtrl.speed , Power_PID_Cal(&CMotor1,velPara,CMotor1.velCtrl.refvel*dcrease_ratio) ));
		Motor_power[1]=fabsf(Motor_Theory_Power( CMotor2.encoderCtrl.speed , Power_PID_Cal(&CMotor2,velPara,CMotor2.velCtrl.refvel*dcrease_ratio) ));
		Motor_power[2]=fabsf(Motor_Theory_Power( CMotor3.encoderCtrl.speed , Power_PID_Cal(&CMotor3,velPara,CMotor3.velCtrl.refvel*dcrease_ratio) ));
		Motor_power[3]=fabsf(Motor_Theory_Power( CMotor4.encoderCtrl.speed , Power_PID_Cal(&CMotor4,velPara,CMotor4.velCtrl.refvel*dcrease_ratio) ));
		
		if(dcrease_ratio==1.0f)
		{//¼ÇÂ¼Î´Ë¥¼õÔ­Ê¼refÊ±µÄÊä³öÀíÂÛ¹¦ÂÊ
			for(int i=0;i<4;i++)
			{
				Origin_Motor_power[i]=Motor_power[i];
				origin_theory_power+=Origin_Motor_power[i];
				
			}
		}
		//theory_power=Motor_power[0]+Motor_power[1]+Motor_power[2]+Motor_power[3];
		theory_power=theory_to_fact*(Motor_power[0]+Motor_power[1]+Motor_power[2]+Motor_power[3]);
		
		if(theory_power<Power_Max){//Èô¼ÆËãÖµĞ¡ÓÚ×î´óÖµ£¬ÕâÓÃ´Ë×÷ÎªÊä³ö
		CMotor1.velCtrl.refvel = CMotor1.velCtrl.refvel*dcrease_ratio;
		CMotor2.velCtrl.refvel = CMotor2.velCtrl.refvel*dcrease_ratio;
		CMotor3.velCtrl.refvel = CMotor3.velCtrl.refvel*dcrease_ratio;
		CMotor4.velCtrl.refvel = CMotor4.velCtrl.refvel*dcrease_ratio;
			stop_mode=0;
			break;
		}
		//ÅĞ¶ÏÊÇ·ñÎªÉ²³µÄ£Ê½
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
	//ÈôÉÏÊöËõ¼õÊ§°ÜÇÒ²»ÊÇ¼±Í£Ä£Ê½ »ò¹¦ÂÊÓàÁ¿³¬¹ı°²È«·¶Î§ ÔòÊ¹ÓÃ80W¹¦ÂÊ·ÖÅäÊä³ö
	if( (theory_power>Power_Max && !stop_mode) || PowerHeatData.chassisPowerBuffer<25 ||(ChassisRef.forward_back_ref>=300&&PowerHeatData.chassisPower<=15))
		{//4800Ê±80WÏÂµÄ×î´óOUTPUT_SUM
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
	*º¯ÊıÃû³Æ£ºCapUsedOut_Ctrl
	*º¯Êı¹¦ÄÜ£ºµçÈİÃ»µçÊ±£¬½«¹¦ÂÊÏŞÖÆÔÚ80WÒ»ÏÂ£¬»Ö¸´¹¦ÂÊÓàÁ¿
	*Èë¿Ú²ÎÊı: ÎŞ
	*·µ»ØÖµ  £ºÎŞ
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
	*º¯ÊıÃû³Æ£ºOpenDcDc
	*º¯Êı¹¦ÄÜ£º¿ªÆôDC-DC
	*Èë¿Ú²ÎÊı: ÎŞ  
	*·µ»ØÖµ  £ºÎŞ
***/
//void OpenDcDc(){
//  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
//  HAL_Delay(80);
//  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
//  SetCAPCurrent(0);//ÉèÖÃºãÑ¹ºãÁ÷Êä³öÎª0
//}
/***
	*º¯ÊıÃû³Æ£ºBattery
	*º¯Êı¹¦ÄÜ£ºÇĞ»»µ½µç³Ø¹©µç×´Ì¬ 
	*Èë¿Ú²ÎÊı: ÎŞ
	*·µ»ØÖµ  £ºÎŞ
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
	*º¯ÊıÃû³Æ£ºToCap
	*º¯Êı¹¦ÄÜ£ºÇĞ»»µ½µçÈİ¹©µç×´Ì¬ 
	*Èë¿Ú²ÎÊı: ÎŞ
	*·µ»ØÖµ  £ºÎŞ
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
	*æ»²æ…é¡å‚™ã„©SetCAPCurrent
	*æ»²æ…é«¡å¤”ã„©NEW:æ‰¢éš…è‡ï §å–ƒè‡è‡éœœ 
	*ï µè«³çµ±æ…: CAP_RI å ´æ’°DCå–ƒè‡è‡éœœ
	*æ®¿éš™ç¡‰  ã„©æ‹¸
***/
void SetCAPCurrent(float CAP_RI)
{
	//float current = 1280 * CAP_RI + 60;
	/* NEW:0~2V å‹¤èŒ¼0~20A */
	uint32_t current=0;
	if(CAP_RI<=0.05f)	current=0;					//å–ƒè‡è‡éœœæ‰¢éš…ä¾šï¥
	else if(CAP_RI>=15.0f)	current=1861;			//æ‰¢éš…å–ƒè‡è‡éœœå¥»ç™¹ç¥¥é–‰å¾¹15A: 15/20*4096*2/3.3f;
	else	current = (uint32_t)(CAP_RI*124.12f);	// CAP_RI*4096*2/3.3f/20;
	
	HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,current); 
}
/***
	*æ»²æ…é¡å‚™ã„©GetCapVoltage
	*æ»²æ…é«¡å¤”ã„©NEW:è‡ï §è‡æ¤ç¡‰è³¤å‘¾
	*ï µè«³çµ±æ…: æ‹¸
	*æ®¿éš™ç¡‰  ã„©æ‹¸ 
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
	//cap_volt=(float)(cap_temp*0.0133f-6.1167f);//110*3.3f/4096.0f);å‹¤å‹ïŸ«è±»ç¶ºè¬¹ADCè…”å‘´å„‚ïš¾æ›¹
	if(cap_volt<=0 | cap_volt>32.4f)	cap_volt=0;//è‡ï §ç¥‘éƒ½æ…æ“‚è³¤å‘¾ï¹
}
/***
	*º¯ÊıÃû³Æ£ºSuperCapCtrl
	*º¯Êı¹¦ÄÜ£º³¬¼¶µçÈİ×Ô¶¯¿ØÖÆ²ßÂÔ
	*Èë¿Ú²ÎÊı: ÎŞ
	*·µ»ØÖµ  £ºÎŞ 
***/
void Cal_Power(){
	/* é«¡è–¹æ‘¯é«¡è–¹èˆ¹ç…¦æ•¸å‘¾*/
	if(1){//current_base[0] && current_base[1]
		/*é³³ïŸ«è‡ï §è‡æ¤*/
		GetCapVoltage();		
		if(cap_volt<0)	cap_volt=0;
		
		/* NEW:è‡éœœæ…æ“‚è³¤å‘¾ */
		i_DC_OUT=GetDCCurrent();				//å ´æ’°DCæ€€å ¤è‡éœœ
		float i_Chassis_OUT=GetChassisCurrent();	//èæ”«æ€€ï µè‡éœœPC1
		/* NEW:é«¡è–¹æ…æ“‚è³¤å‘¾ */
		if(cap_volt>0){
			DC_DC_Power=(cap_volt- 1.4f)*i_DC_OUT;//current_i[0];
		}else{
			DC_DC_Power=0.0f;
		}
		/* total_power [0]çµïè»é«¡è–¹ç¡‰ | [1]ïç¨æ£’é«¡è–¹ç¡‰ */
		total_power[1]=total_power[0];
		total_power[0]=(float)(PowerHeatData.chassisVolt*i_Chassis_OUT/1000+C_P_OFFSET+DC_DC_Power);//current_i[1]
		/* é«¡è–¹èˆ¹ç…¦æ•¸å‘¾ */
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
	/* ç“šå‰¿çµïè…”é¼è‡è¢¨æ€“æ‰¢ç¦»å–ƒè‡è‡éœœ */
	/* å–ƒè‡è‡éœœè«·ç§¶ */
	
	/* è‡å–€é¼è‡ã„©èæ”«è…´é«¡è–¹å¥€ç³èššå‘è±»é«¡è–¹å–ƒè‡ */
	if(exstate==_BATTERY){
	/* ç¾²é è«·ç§¶ */	
		/* æ‰¢ç¦»DC-DCé¦±é‡¬è¢¨æ€“ */
		SetScondaryDC(DC_CLOSE);	//å£½æ••æ£’æ’°DC-DC
		SetPrimaryDC(DC_OPEN);		//ç¾²ïœ„å ´æ’°DC-DC
		//if((PowerHeatData.chassisPower-DC_DC_Power) < 15.0f){
									//ï ±å½†èæ”«é«¡è–¹è…´è¡¾15Wã„—å‡ïŸµæ¯“å³“ã„˜ -â—èššèœ“å•¥é«¡è–¹è·¤è‡ï §å–ƒè‡ã„—ç®é«¡è–¹ã„˜
			if(cap_volt >= 23.0f){
					temp_i=2.8f;
			}else if(cap_volt >= 8.0f){
					temp_i = (80.0f-9.0f)/cap_volt;//(-PowerHeatData.chassisPower+DC_DC_Power)
									//ç®é«¡è–¹å–ƒè‡
			}else{
				temp_i = 8.0f;		//è…´è‡æ¤ïŸµå’å–ƒè‡
			}
			//temp_i = temp_i>8 ? 8:temp_i;//15Aç™¹ç›Ÿ
		//}else{
		//	temp_i=0;				//èœ“å•¥é«¡è–¹è…´è¡¾å‡ïŸµç¡‰ã„›ç¥¥å–ƒè‡
		//}
			
	/* æ••é è«·ç§¶ */
		//	/* æ‰¢ç¦»DC-DCé¦±é‡¬è¢¨æ€“ */
		//	SetPrimaryDC(DC_OPEN);		//ç¾²ïœ„å ´æ’°DC-DC
		//	/* å–ƒè‡é«¡è–¹æ••é è«·ç§¶ */
		//	float Pri_DC_Power_Fdb = DC_DC_Power;//
		//	static float Pri_Dc_Output=0;
		//	static float Power_Error[2]={0};
		//	Power_Error[1]=Power_Error[0];
		//	Power_Error[0]=Pri_DC_Power_Ref-Pri_DC_Power_Fdb;
		//	Pri_Dc_Output += Pri_DC_PID.kp*(Power_Error[0]-Power_Error[1]) + Pri_DC_PID.ki*Power_Error[0];
		//	/* ç™¹ç›Ÿæ­ç‡´ */
		//	if(Pri_Dc_Output<=0){
		//		Pri_Dc_Output=0;
		//		Power_Error[0]=0;
		//	}else if(Pri_Dc_Output>=15.0f){
		//		Pri_Dc_Output=15.0f;
		//	}
		//	/* ç¬›ç“šç‚µè‹€é«¡è–¹è±»è¬›æ‚µèª˜ */
		//	if(PowerHeatData.chassisPowerBuffer<50){
		//		Pri_Dc_Output=2.5f;
		//	}
		//	
		//	SetCAPCurrent(Pri_Dc_Output);
	}else if(exstate==_CAP){
		/* è‡ï §é¼è‡è€€å®’ã„›æ£®å¥€å‰’çŒå ´æ’°DCçœ•80Wç®é«¡è–¹æ€€å ¤ */
		/* æ‰¢ç¦»DCè¢¨æ€“ */
		SetPrimaryDC(DC_OPEN);
		SetScondaryDC(DC_OPEN);
		//SetChassisPowerSupply(_CAP);
		/* æ‰¢ç¦»å ´æ’°DCæ€€å ¤è‡éœœ */
//æ••é è«·ç§¶
//		switch(CAP_STA){
//			case CAP_FULL:
//				temp_i=4.75f;
//				break;
//			case CAP_OK:
//					temp_i = K_DC_I*(75.0f)/(cap_volt+1);//ç®é«¡è–¹å–ƒè‡(-PowerHeatData.chassisPower+DC_DC_Power)
//				break;
//			case CAP_USEDOUT:
//				break;
//		}
//ç¾²é è«·ç§¶	
			if(cap_volt >= 24.5f){
						temp_i=2.8f;
			}else if(cap_volt >= 8.0f){
				temp_i = K_DC_I*(70.0f)/(cap_volt+1);
				//temp_i = (80.0f-6.5f)/cap_volt;//ç®é«¡è–¹å–ƒè‡(-PowerHeatData.chassisPower+DC_DC_Power)
			}else{
				temp_i = 8.0f;//è…´è‡æ¤ïŸµå’å–ƒè‡
			}
			
			//temp_i=Set_DC_I;		//è†å½¸èššã„©DEBUGè‘£ç¡‰è†å½¸
	}
	
	
	temp_i = temp_i>8.0f ? 8.0f:temp_i;//8Aç™¹ç›Ÿ
	/* ç¬›ç“šç‚µè‹€é«¡è–¹è±»è¬›æ‚µèª˜ */
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
	if(current_base[1]>0){//å ´å®è¶™ä¿‡å‚–ã„›åƒ¹è¢§ç¡‰ï éš…
		return (float)(((float)value_filter[2]-(float)current_base[1])*33.0f/4096.0f);
	}else{
		return 0;
	}
}

void SetPrimaryDC(enum DC_State State){
	if(State==DC_OPEN){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);		//å ´æ’°DC-DCè«·ç§¶å ´å®è¶™ã„©ç¾²ïœ„
	}else if(State==DC_CLOSE){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);	//å ´æ’°DC-DCè«·ç§¶å ´å®è¶™ã„©å£½æ••
	}
	Primary_DC_State=State;
}

void SetScondaryDC(enum DC_State State){
	if(State==DC_OPEN){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);		//æ£’æ’°DC-DCè«·ç§¶å ´å®è¶™ã„©ç¾²ïœ„
	}else if(State==DC_CLOSE){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);	//æ£’æ’°DC-DCè«·ç§¶å ´å®è¶™ã„©å£½æ••
	}
	Secondary_DC_State=State;
}

void SetChassisPowerSupply(enum EX_STATE SetState){
	if(SetState==_BATTERY){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);	//æ¨Ÿè‡ïœ‡è‡åŸ­ï½é™è«·ç§¶å ´å®è¶™ã„©ç¬›ç“šç‚µè‹€é¼è‡
		SetScondaryDC(DC_CLOSE);								//å£½æ••è‡ï §(æ£’æ’°DC)æ€€å ¤
	}else if(SetState==_CAP){
		SetScondaryDC(DC_OPEN);									//ç¾²ïœ„è‡ï §(æ£’æ’°DC)æ€€å ¤
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);		//æ¨Ÿè‡ïœ‡è‡åŸ­ï½é™è«·ç§¶å ´å®è¶™ã„©è‡ï §è¡ªè‚®é¼è‡
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
	/* åº•ç›˜ä¾›ç”µçŠ¶æ€ç›‘æµ‹ */
	Set_Chassis_Judge_State();
	/* æ›è¦œïœ‡æ…æ“‚è³¤å‘¾ */
	Cal_Power();
	/* è‡ï §è¢¨æ€“æ‰¢ç¦» */
	CAP_State_Config();
/* è·¦æ“‚è‡ï §è«·ç§¶è¢¨æ€“æ¨µéš…è‡åŸ­ï½é™ç¿’è¬¹ */
	if(CAP_AUTO_CTRL == CAP_AUTO)
	{
			/* æ£€æµ‹è£åˆ¤ç³»ç»Ÿåœ°ç›˜å®«æ®¿çŠ¶æ€ */
		if(Chassis_State==JUDGE_OFF&&RC_Ctrl_Data.remote.s2 == Check_Mode){
			SetChassisPowerSupply(_BATTERY);
		}else{
		/* èµ»é›„è«·ç§¶ */
			if(exstate == _BATTERY){
			/* è‡å–€é¼è‡è¢¨æ€“ */
				if(cap_volt > 22.0f){
				/* è‡ï §é››è‡ã„©ï½é™å³ˆè‡ï §é¼è‡ */
					SetChassisPowerSupply(_CAP);
				}else{
				/* è‡ï §å¸¤é››ã„©èµ»é›„å–ƒè‡+å³å¥è‡å–€é¼è‡ */
											/* ï¿½ï¿½ï¿½ï¿½Ã»ï¿½ï¿½Ê¹ï¿½Ãµï¿½Ø¹ï¿½ï¿½ï¿½ */
							SetChassisPowerSupply(_BATTERY);        
							//ex_Charge_Switch = Charge_Switch;
							/* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê½Ïµï¿½Ê±ï¿½ï¿½Ê¹ï¿½ï¿½80Wï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½İ³ï¿½ï¿½ */
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
									/* ï¿½ï¿½ï¿½ï¿½DCï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ */
									SetCAPCurrent(0.0f);                            //Í£Ö¹ï¿½ï¿½ï¿½
									Charge_Switch = SWITCH_OFF;
							}    

	//				Auto_Charge();
	//				SetChassisPowerSupply(_BATTERY);
				}
			/* è‡ï §é¼è‡è¢¨æ€“ */	
			}else if(exstate == _CAP){
				/* ç“šå‰¿è‡ï §è¢¨æ€“ */
				if(CAP_STA==CAP_USEDOUT){
				/* è‡ï §ç¾¶è‡ã„©ï½é™è‡å–€é¼è‡ */
					SetChassisPowerSupply(_BATTERY);
				}else if(CAP_STA==CAP_OK || CAP_STA==CAP_FULL){
				/* ç®é«¡è–¹æ€€å ¤ */
					Auto_Charge();
				}
				
			}
		}
	}else if(CAP_AUTO_CTRL == CAP_MANUAL)
	{
	/* å¿’é›„ï½é™ */
		if(exstate == _BATTERY){
		/* è‡å–€é¼è‡è¢¨æ€“ */
			SetChassisPowerSupply(_BATTERY);
		}else if(exstate == _CAP){
		/* è‡ï §é¼è‡è¢¨æ€“ */
			SetChassisPowerSupply(_CAP);
			Auto_Charge();
		}
	}
}

//void SuperCapCtrl(void)
//{
//	
//	Cal_Power();//¼ÆËãµ±Ç°µ×ÅÌ¹¦ÂÊ | DC/DC¹¦ÂÊ ºÍ ¹¦ÂÊ²î·Ö
//	//ÉèÖÃµçÈİµÄµ±Ç°×´Ì¬£¨ÖÍ»ØÇúÏß£©
//	if(cap_volt>22.5f){
//		CAP_STA=CAP_OK;
//	}else if(cap_volt<21.0f){
//		CAP_STA=CAP_USEDOUT;
//	}
//	if(CAP_AUTO_CTRL==CAP_AUTO)
//	{//×Ô¶¯¿ØÖÆ-start
//		if(exstate==_BATTERY)
//		{//ÏÖÔÚÊÇµç³Ø¹©µç
//			if(( total_power[0]>=70.0f) || (PowerHeatData.chassisPower-DC_DC_Power >=78.0f) )
//			{	//²ÃÅĞÏµÍ³¹¦ÂÊ¸ßÓÚ80W -¡·ÇĞ»»³¬¼¶µçÈİ¹©µç PowerHeatData.chassisPowerBuffer<45&PowerHeatData.chassisPowerBuffer>=5
//				SetCAPCurrent(0);	//¹Ø±ÕDC-DCĞ­Í¬¹©µç
//				if(CAP_STA==CAP_OK)
//				{	//ÅĞ¶ÏµçÈİ×´Ì¬ÄÜ·ñÊ¹ÓÃ
//					ToCap();//ÇĞ»»µçÈİ¹©µç
//					//SetCAPCurrent(2.5);//¿ªÆôDC-DCĞ­Í¬¹©µç
//				}
//				else 
//				{
//					ToBattery();//Èç¹ûµçÈİÃ»µç£¬ÔòÎ¬³Öµç³Ø¹©µç
//				}
//			}else //if(PowerHeatData.chassisPower<=80)
//			{//²ÃÅĞÏµÍ³¹¦ÂÊµÍÓÚ80W -¡· µç³Ø¹©µç
//				ToBattery();	//±£³Öµç³Ø¹©µç
//				Auto_Charge();//×Ô¶¯ÉèÖÃ³äµçµçÁ÷
//			}
//			
//		}
//		else if(exstate==_CAP)
//		{
//			static int tim_count;
//			if(tim_count<TIM_MAX)	
//			{
//				tim_count++;//¿ªÆôµçÈİ¹©µçÖ®ºó£¬¿ªÆô¶¨Ê±¼ÆÊı
//			}
//			else	
//			{
//				tim_count=TIM_MAX;//TIM_MAX msºóµçÁ÷ÌáÉıÖÁ×î´ó
//			}
//			SetCAPCurrent(0);//SetCAPCurrent(0.0f*tim_count/TIM_MAX); //ÉèÖÃ³äµçµçÁ÷£ºµ±Ç°Ğ­Í¬¹¤×÷²»¹»ÎÈ¶¨	
//			if((CAP_STA==CAP_USEDOUT) || (total_power[0]<=70.0f && tim_count>20) )
//			{//µçÈİµçÑ¹µÍÓÚ°²È«Öµ »ò µ×ÅÌÊä³öµçÁ÷Ğ¡ÓÚ2.5A(¾ßÌåµçÁ÷ÖµĞèÒª¸ù¾İ²Î¿¼±øÖÖÔÈËÙµçÁ÷)  | value_res[2]<3250
//				SetCAPCurrent(0);//¹Ø±ÕDC-DC¹©µç
//				ToBattery();//¸ü¸ÄÎªµç³Ø¹©µç
//				tim_count=0;
//			}
//		}
//	}//×Ô¶¯¿ØÖÆ-end
//	else if(CAP_AUTO_CTRL == CAP_MANUAL)
//	{
//		if(Power_Mode == Cap)
//		{
//			//if( ((diff_t_p>0?diff_t_p:0)>6.0f && total_power[0]>=70.0f) )//|| (Power_Heat_Data.chassis_power+(diff_t_p>0?diff_t_p:0) >=80.0f))
//			//{	//²ÃÅĞÏµÍ³¹¦ÂÊ¸ßÓÚ80W -¡·ÇĞ»»³¬¼¶µçÈİ¹©µç PowerHeatData.chassisPowerBuffer<45&PowerHeatData.chassisPowerBuffer>=5
//				SetCAPCurrent(0);	//¹Ø±ÕDC-DCĞ­Í¬¹©µç
//				//if(cap_volt>19.5f&&CAP_STA==CAP_OK)
//				//{	//ÅĞ¶ÏµçÈİÊÇ·ñ¸ßÓÚ°²È«Öµ £ºÒÔÒ»´Î·ÉÆÂËùĞèµçÑ¹ÎªÒÀ¾İ
//					ToCap();//ÇĞ»»µçÈİ¹©µç
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
//          	ToBattery();	//±£³Öµç³Ø¹©µç
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
