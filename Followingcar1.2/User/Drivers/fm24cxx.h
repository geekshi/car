/*
 *******************************************************************************
 *
 *******************************************************************************
 */
#ifndef _USER_FM24Cxx_H
#define _USER_FM24Cxx_H

#include "stm32f10x.h"

extern u8 FM24_Count;


#define SLAVE_Address 0xA4

#define FM24_BASE       0x0A
#define FM24_ADDR_FLAG  0x00
#define FM24_MY_ADDR    0x01
#define FM24_BAUD_LV    0x02
#define FM24_ATT_VALUE1 0x03
#define FM24_ATT_VALUE2 0x04

#define FM24_OVER 20    //The most read or write opration

void IICStart(void);
void IICStop(void);
void IICAck(void);
void IICNoAck(void);
u8 IICGetAck(void);
void FM24CxxByteWrite(u8 nByte);
void FM24CxxRandomContinuesWriteCh(u8 SlaveAddress,u16 nMemAddr,u16 nLen,u8 *nBuf);
u8 FM24CxxByteRead(void);
void FM24CxxRandomContinuesReadCh(u8 SlaveAddress,u16 nMemAddr,u16 nLen,u8 *nBuf);


#endif  //__FM24Cxx_H
