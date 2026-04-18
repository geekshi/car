/*
 *******************************************************************************
 *
 *******************************************************************************
 */
#ifndef _USER_USART1PROCESS_H
#define _USER_USART1PROCESS_H
#include "main.h"

#define FOLLOWDIS  (150)   //¸úËæ¾àÀë£¨ÀåÃ×£©

typedef struct
{
	u32 Distance;
	s16 Azimuth;//·½Î»
	u32 GetTime;
}TAGTRANS;
extern TAGTRANS TagTrans;

typedef struct
{
	u16 PWML;
	u16 PWMR;
	u16 Velocity;
	u16 Dis;
	float Vcc;
}MOTORDATA;
extern MOTORDATA MotorData;

typedef struct
{
	u32 KeyTime;
	u8  KeyFlag;		
}KEYBUS;

extern KEYBUS KeyB1,KeyB2,KeyB3;

void Usart1Process(void);
void InitProcess(void);

#endif	//_USER_USART1PROCESS_H
