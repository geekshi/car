/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <math.h>
#include <ctype.h>
#include "stm32f10x.h"
#include "delay.h"
#include "timer.h"

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/
void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

/*****************************************************************************/
void SysTick_Handler(void)
{
    TimingDelay_Decrement();
}

void TIM1_UP_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM1,TIM_IT_Update)!=RESET)
    {
        TIM_ClearITPendingBit(TIM1,TIM_FLAG_Update);
        TIM_ClearFlag(TIM1, TIM_FLAG_Update);
        HalIwdgFeed();
    }
}


/**************************************************************************UART4 扫码枪数据接收中断**************************************************************************/
#include "oled_i2c.h"
#include <string.h>

#define BARCODE_MAX_LEN 64
char g_barcode_buffer[BARCODE_MAX_LEN];
static uint8_t g_barcode_index = 0;
volatile uint8_t g_barcode_ready = 0;

void UART4_IRQHandler(void)
{
    uint8_t ch;
    if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
    {
        ch = USART_ReceiveData(UART4);

			  /*
        if(ch == '\r' || ch == '\n')
        {
            if(g_barcode_index > 0)
            {
                g_barcode_buffer[g_barcode_index] = '\0';
                g_barcode_ready = 1;
                g_barcode_index = 0;
            }
        }
        else if(g_barcode_index < (BARCODE_MAX_LEN - 1))
        {
            g_barcode_buffer[g_barcode_index++] = ch;
        }
			  */
			  if(g_barcode_index < 63)
        {
            // 扫码枪通常以回车(0x0D)或换行(0x0A)结束
            if(ch == 0x0D || ch == 0x0A) 
            {
                if(g_barcode_index > 0) // 确保有数据
                {
                    g_barcode_buffer[g_barcode_index] = '\0'; // 添加字符串结束符
                    g_barcode_ready = 1; // 置位标志，通知主循环处理
                    g_barcode_index = 0; // 重置索引
                }
            }
            else
            {
                g_barcode_buffer[g_barcode_index++] = ch; // 存入缓冲区
            }
        }
        else
        {
            // 缓冲区满，重置防止溢出
            g_barcode_index = 0;
        }

        //USART_ClearITPendingBit(UART4, USART_IT_RXNE);
    }
}

// 在主循环中调用的条码处理和回发函数
void Barcode_ProcessAndEcho(void)
{
    if(g_barcode_ready)
    {
			
			  Delay_ms(10);
			
        // 显示在 OLED 上
        OLED_CLS();
			  OLED_ShowStr(0, 2, (char*)g_barcode_buffer, 2);
			  Delay_ms(50);


        // 回发数据给扫码枪
			  /*
        for(uint8_t i = 0; i < strlen(g_barcode_buffer); i++)
        {
            USART_SendData(UART4, g_barcode_buffer[i]);
            while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
        }
        // 发送换行符
        USART_SendData(UART4, '\r');
        while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
        USART_SendData(UART4, '\n');
        while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
        while(USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET);
				*/

        g_barcode_ready = 0;
    }
}