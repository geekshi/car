/**********************************************************************************
 * @file        main.c
 * @author
 * @phone       13566273308(-)
 * @version V3.0.0
 * @date        2023.06.02
 * @brief       ٸĲԴ(ʹٷ04.00.06) ƽ̨:UWB-S1(οַ)
 * @store       https://item.taobao.com/item.htm?spm=a1z10.5-c.w4002-23565193320.10.6e6c3f96tF7wds&id=572212584700
**********************************************************************************/

#include "stm32f10x.h"
#include "delay.h"
#include "Periph_init.h"
#include "timer.h"
#include "hal_usart.h"
#include "control.h"
#include "uwb.h"
#include "hal_iic.h"
#include "oled_i2c.h"
#include "motor.h"
#include "usb_host.h"
#include "barcode_scanner.h"
/*******************************************************************************
*******************************************************************************/
void RCC_Configuration_part(void)
{
        ErrorStatus HSEStartUpStatus;
        RCC_ClocksTypeDef RCC_ClockFreq;

        /* RCCĴΪĬֵ */
        RCC_DeInit();

        /* ⲿHSE */
        RCC_HSEConfig(RCC_HSE_ON);

        /* ȴ */
        HSEStartUpStatus = RCC_WaitForHSEStartUp();

        if(HSEStartUpStatus != ERROR)
        {
                /* FlashԤ幦,ʱ */
                FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

                /* 48~72MhzLatencyΪ2 */
                FLASH_SetLatency(FLASH_Latency_2);

                /* AHBʱ72MHz HCLK = SYSCLK */
                RCC_HCLKConfig(RCC_SYSCLK_Div1);
                /* APB2ʱ172MHz PCLK2 = HCLK */
                RCC_PCLK2Config(RCC_HCLK_Div1);
                /* APB1ʱ236MHz PCLK1 = HCLK/2 */
                RCC_PCLK1Config(RCC_HCLK_Div2);
                /*  ADCʱ ADCCLK = PCLK2/4 */
                RCC_ADCCLKConfig(RCC_PCLK2_Div6);

                //PLLʱԴƵϵ RCC_PLLSource_HSE_Div1 9ƵRCC_PLLMul_9
                RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
                /* PLL */
                RCC_PLLCmd(ENABLE);
                /* ȴPLL */
                while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){}

                /* ѡPLLʱΪʱԴ */
                RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

                /* ȴʱԴ״̬ */
                while (RCC_GetSYSCLKSource() != 0x08){}
        }

        RCC_GetClocksFreq(&RCC_ClockFreq);
}

/*******************************************************************************
*   : init
*     : ʼ
*     :
*     :
*   :
*******************************************************************************/
void init(void)
{
        //ʼ
        SystemInit();
        RCC_Configuration_part();
        SysTick_Init();      //ʱ
        Periph_init();       //ָʾ+ʼ
        Timer_Init();        //ʱʼ
        Uart_Queue_Init();   //PDOAݶгʼ
        HalUARTInit();       //1+2
        OLED_Configuration();//Ļʼʼ
        EXTI_ALL_Init();     //IOʼ
        Motor_Gpio_init();   //IOڳʼ

        // 条码扫描系统初始化
        Barcode_System_Init();
}


/*******************************************************************************
*   : main
*     :
*     :
*     :
*   :
********************************************************************************/
int main(void)
{
        init();
        while(1)
        {
                Read_AoA_Control();

                // 条码扫描任务处理
                Barcode_Scanner_Task();
        }
}