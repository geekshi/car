/*
 *******************************************************************************
 *                           Includes
 *******************************************************************************
 */


#include  "fm24cxx.h"
#include  "delay.h"

/*
 *******************************************************************************
 *     PA11--SDA  PA12--SCL  0xa4 
 *     PB10--SDA  PB11--SCL  0xa4
 *******************************************************************************
 */
#define SCLF GPIO_Pin_4
#define SDAF GPIO_Pin_5

#define FM24_DELAY() Delay(1)

#define SDAF_IN()   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);\
                    GPIO_InitStructure.GPIO_Pin = SDAF;\
                    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;\
                    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;\
                    GPIO_Init(GPIOB, &GPIO_InitStructure)

#define GET_SDAF()  GPIO_ReadInputDataBit(GPIOB, SDAF)

#define SDAF_OUT()  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);\
                    GPIO_InitStructure.GPIO_Pin = SDAF;\
                    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;\
                    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;\
                    GPIO_Init(GPIOB, &GPIO_InitStructure)

#define SDAF_HIGH()  GPIO_SetBits(GPIOB, SDAF);FM24_DELAY()
#define SDAF_LOW()   GPIO_ResetBits(GPIOB, SDAF);FM24_DELAY()

#define SCLF_OUT()  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);\
                    GPIO_InitStructure.GPIO_Pin = SCLF;\
                    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;\
                    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;\
                    GPIO_Init(GPIOB, &GPIO_InitStructure)

#define SCLF_HIGH()  GPIO_SetBits(GPIOB, SCLF);FM24_DELAY()
#define SCLF_LOW()   GPIO_ResetBits(GPIOB, SCLF);FM24_DELAY()
/*
 *******************************************************************************
 *
 *******************************************************************************
 */
u8 FM24_Count=0;           //Read or Write FM24Cxx opration
/*
 *******************************************************************************
 *
 *******************************************************************************
 */
void IICStart(void)
{  
  GPIO_InitTypeDef GPIO_InitStructure;
  
  SCLF_OUT();
  SDAF_OUT();
  
  SDAF_HIGH();
  SCLF_HIGH();
  SDAF_LOW();
  SCLF_LOW();
}

void IICStop(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  SDAF_OUT();
  SDAF_LOW();
  
  SCLF_HIGH();
  SDAF_HIGH();
}

void IICAck(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  SCLF_LOW();
  SDAF_OUT();
  SDAF_LOW();
  SCLF_HIGH();
  SCLF_LOW();
}

void IICNoAck(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  SCLF_LOW();
  SDAF_OUT();
  SDAF_HIGH();
  SCLF_HIGH();
  SCLF_LOW();
}

u8 IICGetAck(void)
{
  u8 ackflag;
  GPIO_InitTypeDef GPIO_InitStructure;
  
  SDAF_HIGH();
  SDAF_IN();
  SCLF_HIGH();
  if(GET_SDAF())
  {
    ackflag = 0;
  }
  else
  {
    ackflag = 1;
  }
  SCLF_LOW();
  SDAF_OUT();
  
  return ackflag;
}

//Write a byte to FM24Cxx
void FM24CxxByteWrite(u8 nByte)
{
  u8 i;
  
  for(i=0x80;i;i>>=1)
  {
    if(nByte&i)
    {
      SDAF_HIGH();
    }
    else
    {
      SDAF_LOW();
    }
    SCLF_HIGH();
    SCLF_LOW();
  }
}

void FM24CxxRandomContinuesWriteCh(u8 SlaveAddress,u16 nMemAddr,u16 nLen,u8 *nBuf)
{
  FM24_Count = 0;                       //FM24xx opration time count
  
  while(1)
  {
    if(FM24_Count++>FM24_OVER)          //If the read or write opration is over 20 times
    {                                   //Set the useless flag and return
      return;
    }
    IICStart();                         //start
    FM24CxxByteWrite(SlaveAddress);     //Write the slave address
    if(!IICGetAck())
      continue;                         //slaver ack
    FM24CxxByteWrite(nMemAddr>>8);      //Write the high address
    if(!IICGetAck())                    //slaver ack
      continue;
    FM24CxxByteWrite(nMemAddr);         //Write the low address
    if(!IICGetAck())                    //slaver ack
      continue;
    break;
  }
  
  for(;nLen;)
  {
    FM24CxxByteWrite(*nBuf);            //Write data
    if(!IICGetAck())                    //slaver ack
      continue;
    nBuf++;
    nLen--;
  }
  IICStop();                            //stop
}

//Read a byte from FM24Cxx
u8 FM24CxxByteRead(void)
{
  u8 i;
  u8 byte;
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  SDAF_HIGH();
  SDAF_IN();
  for(i=0,byte=0;i<8;i++)
  {
    byte <<= 1;
    if(GET_SDAF())
    {
      byte |= 0x01;
    }
    SCLF_HIGH();
    SCLF_LOW();
  }
  return byte;
}

//Random continues read from FM24Cxx
void FM24CxxRandomContinuesReadCh(u8 SlaveAddress,u16 nMemAddr,u16 nLen,u8 *nBuf)
{
  u8 *p;
  
  FM24_Count = 0;                       //FM24xx repet opration count
  
  while(1)
  {
    if(FM24_Count++>FM24_OVER)          //If the read or write opration is over 20 times
    {                                   //Set the useless flag and return      
      return;
    }
    IICStart();                         //start
    FM24CxxByteWrite(SlaveAddress);     //Write the slave address
    if(!IICGetAck())                    //slaver ack
      continue;
    FM24CxxByteWrite(nMemAddr>>8);      //Write the high address
    if(!IICGetAck())                    //slaver ack
      continue;
    FM24CxxByteWrite(nMemAddr);         //Write the low address
    if(!IICGetAck())                    //slaver ack
      continue;
    
    IICStart();                         //start
    FM24CxxByteWrite(SlaveAddress+1);   //Slave address plus 1 means to read
    if(!IICGetAck())                    //slaver ack
      continue;
    break;
  }
  p = nBuf;
  for(;nLen;nLen--,p++)
  {
    *p = FM24CxxByteRead();
    if(nLen==1)
    {
      IICNoAck();                       //noack
    }
    else
    {
      IICAck();                         //ack
    }
  }
  IICStop();                            //stop
}

/*
 *******************************************************************************
 *
 *******************************************************************************
 */
u8 TestReadDataBufXXXX[50];
u8 TestWriteDataBufXXXX[50];
u8 TestFlag;   //
u8 ChipAddr=0xA4;

void MemTest(void)
{
	u16 i;

	while(1)
	{
		if(TestFlag)
		{
		    //for(i=0;i<100;i++)
			{
				FM24CxxRandomContinuesReadCh(ChipAddr,0,50,TestReadDataBufXXXX);
			}
			for(i=0;i<50;i++)
			{
					TestReadDataBufXXXX[i]=0;
					TestWriteDataBufXXXX[i]=TestFlag+i;//0x55;//
			}
			//for(i=0;i<100;i++)
			{
				FM24CxxRandomContinuesWriteCh(ChipAddr,0,50,TestWriteDataBufXXXX);
			}
		}
	}
}
