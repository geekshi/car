/*! ----------------------------------------------------------------------------
 * @file	port.h
 * @brief	HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */


#ifndef PORT_H_
#define PORT_H_

#include "main.h"

/***********************************************************************/


#define SYSTEMSIZE 128

 typedef union
 {
   u8 ch[512];
   u16 word[256];
   struct
   {
     UNION_LONG addr;
	   u16 Velocity;
   }item;
 }SYSTEMpar;
 extern SYSTEMpar SystemPar;

 //LED
#define led1_GPIO					GPIOB
#define led1_PIN					GPIO_Pin_10
#define led1_on()		            GPIO_ResetBits(led1_GPIO, led1_PIN)
#define led1_off()		            GPIO_SetBits(led1_GPIO, led1_PIN)

#define led2_GPIO					GPIOB
#define led2_PIN					GPIO_Pin_11
#define led2_on()		            GPIO_ResetBits(led2_GPIO, led2_PIN)
#define led2_off()		            GPIO_SetBits(led2_GPIO, led2_PIN)

//Motor Control 
//AIN
#define AIN1_GPIO					GPIOC
#define AIN1_PIN					GPIO_Pin_13
#define AIN2_GPIO					GPIOC
#define AIN2_PIN					GPIO_Pin_14
#define AMOT_D()		            GPIO_ResetBits(AIN1_GPIO, AIN1_PIN);GPIO_SetBits(AIN2_GPIO, AIN2_PIN)
#define AMOT_R()		            GPIO_SetBits(AIN1_GPIO, AIN1_PIN);GPIO_ResetBits(AIN2_GPIO, AIN2_PIN)
#define AMOT_P()                    GPIO_ResetBits(AIN1_GPIO, AIN1_PIN);GPIO_ResetBits(AIN2_GPIO, AIN2_PIN)

//BIN
#define BIN1_GPIO					GPIOB
#define BIN1_PIN					GPIO_Pin_12
#define BIN2_GPIO					GPIOB
#define BIN2_PIN					GPIO_Pin_13
#define BMOT_D()		            GPIO_ResetBits(BIN1_GPIO, BIN1_PIN);GPIO_SetBits(BIN2_GPIO, BIN2_PIN)
#define BMOT_R()		            GPIO_SetBits(BIN1_GPIO, BIN1_PIN);GPIO_ResetBits(BIN2_GPIO, BIN2_PIN)
#define BMOT_P()                    GPIO_ResetBits(BIN1_GPIO, BIN1_PIN);GPIO_ResetBits(BIN2_GPIO, BIN2_PIN)

//CIN
#define CIN1_GPIO					GPIOB
#define CIN1_PIN					GPIO_Pin_0
#define CIN2_GPIO					GPIOB
#define CIN2_PIN					GPIO_Pin_1
#define CMOT_D()		            GPIO_SetBits(CIN1_GPIO, CIN1_PIN);GPIO_ResetBits(CIN2_GPIO, CIN2_PIN)
#define CMOT_R()		            GPIO_ResetBits(CIN1_GPIO, CIN1_PIN);GPIO_SetBits(CIN2_GPIO, CIN2_PIN)
#define CMOT_P()                    GPIO_ResetBits(CIN1_GPIO, CIN1_PIN);GPIO_ResetBits(CIN2_GPIO, CIN2_PIN)

//DIN
#define DIN1_GPIO					GPIOC
#define DIN1_PIN					GPIO_Pin_1
#define DIN2_GPIO					GPIOC
#define DIN2_PIN					GPIO_Pin_2
#define DMOT_D()		            GPIO_SetBits(DIN1_GPIO, DIN1_PIN);GPIO_ResetBits(DIN2_GPIO, DIN2_PIN)
#define DMOT_R()		            GPIO_ResetBits(DIN1_GPIO, DIN1_PIN);GPIO_SetBits(DIN2_GPIO, DIN2_PIN)
#define DMOT_P()                    GPIO_ResetBits(DIN1_GPIO, DIN1_PIN);GPIO_ResetBits(DIN2_GPIO, DIN2_PIN)

//STBY
#define STBY_GPIO					GPIOC
#define STBY_PIN					GPIO_Pin_11
#define STBY_on()		            GPIO_SetBits(STBY_GPIO, STBY_PIN)
#define STBY_off()		            GPIO_ResetBits(STBY_GPIO, STBY_PIN)

//User button
#define KEY1_PIN               	GPIO_Pin_8
#define KEY1_GPIO           		GPIOB

#define KEY2_PIN               	GPIO_Pin_9
#define KEY2_GPIO            		GPIOB

int parameter_init(void);
int RCC_Configuration(void);
int SysTick_Configuration(void);
int peripherals_init(void);
void CheckVelocity(void);
unsigned long portGetTickCnt(void);
#define portGetTickCount() 			portGetTickCnt()
void Sleep(u32 x);


#endif /* PORT_H_ */
