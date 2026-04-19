/*! ----------------------------------------------------------------------------
 *  @file 	deca_types.h
 *  @brief 	DecaWave general type definitions
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */

#ifndef _TYPES_H_
#define _TYPES_H_

#include "stm32f10x.h"

#ifndef uint8
#ifndef _DECA_UINT8_
#define _DECA_UINT8_
typedef unsigned char uint8;
#endif
#endif

#ifndef uint16
#ifndef _DECA_UINT16_
#define _DECA_UINT16_
typedef unsigned short uint16;
#endif
#endif

#ifndef uint32
#ifndef _DECA_UINT32_
#define _DECA_UINT32_
typedef unsigned long uint32;
#endif
#endif

#ifndef int8
#ifndef _DECA_INT8_
#define _DECA_INT8_
typedef signed char int8;
#endif
#endif

#ifndef int16
#ifndef _DECA_INT16_
#define _DECA_INT16_
typedef signed short int16;
#endif
#endif

#ifndef int32
#ifndef _DECA_INT32_
#define _DECA_INT32_
typedef signed long int32;
#endif
#endif

typedef uint64_t        uint64 ;

typedef int64_t         int64 ;


#ifndef FALSE
#define FALSE               0
#endif

#ifndef TRUE
#define TRUE                1
#endif

///////////////////////////////typedef
typedef union
{
	u32 lint;
    u16 uint[2];
	u8 ch[4];
}UNION_LONG;


typedef union
{
	u16 uint;
	u8 ch[2];
}UNION_INT;


typedef union
{
	float lint;
	char ch[4];
}FLOAT;

//注:此处没有启用时钟
#define    GPIO_CONFIG(GPIOx,PINx,mode)      GPIO_InitStructure.GPIO_Pin = PINx;\
														GPIO_InitStructure.GPIO_Mode = mode;\
														GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;\
														GPIO_Init(GPIOx, &GPIO_InitStructure)



#endif /* TYPES_H_ */


