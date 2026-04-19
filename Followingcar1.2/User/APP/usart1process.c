/*! ----------------------------------------------------------------------------
 * @file	Usart1Process.c
 * @brief	Following car Each function realization process
 * 
 * @attention
 * Copyright 2025 (c) Suzhou Ailanxin electronic Technology Co., LTD.
 * All rights reserved.
 *
 * @author Yang
 ----------------------------------------------------------------------------*/

#include "main.h"


void DoCmd(u16 cmd);
void Usart1Receive(void);
void display_to_lcd_twr(void);
void KeyProcess(void);
void GetAdcprocess(void);
void DriveProcess(void);
/*
 *******************************************************************************
 *
 *******************************************************************************
 */
uint32 IntTime32_incr;
TAGTRANS TagTrans;
MOTORDATA MotorData;
KEYBUS KeyB1,KeyB2;

/*
 *******************************************************************************
 *
 *******************************************************************************
 */
void DriveProcess(void)
{
	if(TagTrans.Azimuth<0)
	{		
		MotorData.PWML=(MotorData.Velocity-abs(TagTrans.Azimuth)*MotorData.Dis);
		MotorData.PWMR=(MotorData.Velocity);
	}
    else
	{
		MotorData.PWMR=(MotorData.Velocity-abs(TagTrans.Azimuth)*MotorData.Dis);
		MotorData.PWML=(MotorData.Velocity);
	}
	
	if(TagTrans.Distance>FOLLOWDIS)
	{
		STBY_on();
	}
	else
	{
		STBY_off();
	}
	
	AMOT_D();
	BMOT_D();
	CMOT_D();
	DMOT_D();
	
	//开环控制-根据PWM值驱动电机
	Set_PWM(MotorData.PWML,MotorData.PWML,MotorData.PWMR,MotorData.PWMR);
}

void display_to_lcd_twr(void)
{
	static int display_idx = 0;

	if(Usart1.cls==0)
	{
		Usart1.cls=1;
		OLED_CLS();
	}
			
	if(++display_idx%5 == 0)
	{
		char buf_dist[64];
		memset(buf_dist, 0, sizeof(buf_dist));
		sprintf(buf_dist, " D: %3d  A: %3d ",TagTrans.Distance,TagTrans.Azimuth);
		OLED_ShowStr(0, 0, buf_dist, 2);	

		char buf_v[64];
		memset(buf_v, 0, sizeof(buf_v));
		sprintf(buf_v,  " V : %d   ",MotorData.Velocity);
		OLED_ShowStr(0, 5, buf_v, 2);	
		
		char buf_vcc[64];
		memset(buf_vcc, 0, sizeof(buf_vcc));
		sprintf(buf_vcc,  " Vcc : %2.1f v ",MotorData.Vcc);
		OLED_ShowStr(0, 3, buf_vcc, 2);	
	}
}

void GetAdcprocess(void)
{
	u16 adcx;
	adcx=Get_adc_Average(ADC_Channel_5,10);  //获取adc的值
	MotorData.Vcc=(float)adcx*(3.3*11/4096); //求当前电压
}

void KeyProcess(void)
{
	if(KeyB1.KeyFlag)
	{
		KeyB1.KeyFlag=0;
		MotorData.Velocity+=1000;
		if(MotorData.Velocity>7200)
		{
			MotorData.Velocity=7200;
			
		}
	    SystemPar.item.Velocity=MotorData.Velocity;
	    FM24CxxRandomContinuesWriteCh(SLAVE_Address,0,SYSTEMSIZE,SystemPar.ch);


	}
	
	if(KeyB2.KeyFlag)
	{
		KeyB2.KeyFlag=0;
		MotorData.Velocity-=1000;
		if(MotorData.Velocity<4500)
		{
			MotorData.Velocity=4500;
		}
	    SystemPar.item.Velocity=MotorData.Velocity;
		FM24CxxRandomContinuesWriteCh(SLAVE_Address,0,SYSTEMSIZE,SystemPar.ch);
	}
}

/*
 *******************************************************************************
 *
 *******************************************************************************
 */
void InitProcess(void)
{

	 if((portGetTickCnt() - IntTime32_incr) > 10000)
	 {
		 IntTime32_incr = portGetTickCnt();
		 USART1_Configuration(115200);
	 }
	 if((portGetTickCnt() - TagTrans.GetTime) > 200)
	 {
		 TagTrans.Distance=0;
	 }

}

/*
 *******************************************************************************
 *
 *******************************************************************************
 */
void Usart1Process(void)
{
	DriveProcess();
	KeyProcess();
	GetAdcprocess();
	Usart1Receive();
	InitProcess();
}


/*
 *******************************************************************************
 *   接收数据
 *******************************************************************************
 */
void Usart1Receive(void)
{
	if(Usart1.rxstatus!=UART_RECED)
		return ;

	DoCmd(U16HighLowByteSwap(Usart1.rxbuf.item.RequestCommand));

	Usart1RxStatusClr();
}
/*
 *******************************************************************************
 *
 *******************************************************************************
 */
void DoCmd(u16 cmd)
{
	Usart1.end_flag = 0;
	
	switch(cmd)
	{

		case 0x2001:
			TagTrans.Distance=U32HighLowByteSwap(Usart1.rxbuf.item.Distance);
			TagTrans.Azimuth=U16HighLowByteSwap(Usart1.rxbuf.item.Azimuth);
					
			display_to_lcd_twr();
		    TagTrans.GetTime=portGetTickCnt();

			break;

		default:
			break;
	}
}

