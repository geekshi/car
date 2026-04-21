/*! ----------------------------------------------------------------------------
 * @file	port.c
 * @brief	HW specific definitions and functions for portability
 *
 * @attention
 * Copyright 2025 (c) Suzhou Ailanxin electronic Technology Co., LTD.
 * All rights reserved.
 *
 * @author Yang
 ----------------------------------------------------------------------------*/

#include "main.h"

SYSTEMpar SystemPar;

/* System tick 32 bit variable defined by the platform */
extern __IO unsigned long time32_incr;
unsigned long portGetTickCnt(void)
{
    return time32_incr;
}


int SysTick_Configuration(void)
{
    if(SysTick_Config(SystemCoreClock / CLOCKS_PER_SEC))
    {
        /* Capture error */
        while(1);
    }
    NVIC_SetPriority(SysTick_IRQn, 5);

    return 0;
}

int RCC_Configuration(void)
{
    ErrorStatus HSEStartUpStatus;
    RCC_ClocksTypeDef RCC_ClockFreq;

    /* RCC system reset(for debug purpose) */
    RCC_DeInit();

    /* Enable HSE */
    RCC_HSEConfig(RCC_HSE_ON);

    /* Wait till HSE is ready */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();

    if(HSEStartUpStatus != ERROR)
    {
        /* Enable Prefetch Buffer */
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

        /****************************************************************/
        /* HSE= up to 25MHz (on EVB1000 is 12MHz),
         * HCLK=72MHz, PCLK2=72MHz, PCLK1=36MHz 						*/
        /****************************************************************/
        /* Flash 2 wait state */
        FLASH_SetLatency(FLASH_Latency_2);
        /* HCLK = SYSCLK */
        RCC_HCLKConfig(RCC_SYSCLK_Div1);
        /* PCLK2 = HCLK */
        RCC_PCLK2Config(RCC_HCLK_Div1);
        /* PCLK1 = HCLK/2 */
        RCC_PCLK1Config(RCC_HCLK_Div2);
        /*  ADCCLK = PCLK2/4 */
        //RCC_ADCCLKConfig(RCC_PCLK2_Div6);

        /* Configure PLLs *********************************************************/
        /* PLL1 configuration: PLLCLK = (HCLK / 2) * 9 = 72 MHz */
        RCC_PLLConfig(RCC_PLLSource_HSE_Div2, RCC_PLLMul_9);

        /* Enable PLL */
        RCC_PLLCmd(ENABLE);

        /* Wait till PLL is ready */
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {}

        /* Select PLL as system clock source */
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

        /* Wait till PLL is used as system clock source */
        while(RCC_GetSYSCLKSource() != 0x08) {}
    }

    RCC_GetClocksFreq(&RCC_ClockFreq);

    return 0;
}


int GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Configure all unused GPIO port pins in Analog Input mode (floating input
    * trigger OFF), this will reduce the power consumption and increase the device
    * immunity against EMI/EMC */

    RCC_APB2PeriphClockCmd(
        RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
        RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
        RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO,
        ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); 
    // Set all GPIO pins as analog inputs
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    // Enable GPIO used for LEDs
    GPIO_CONFIG(led1_GPIO, led1_PIN, GPIO_Mode_Out_PP);
    GPIO_CONFIG(led2_GPIO, led2_PIN, GPIO_Mode_Out_PP);
	// Enable GPIO used for Motor Control 
    GPIO_CONFIG(AIN1_GPIO, AIN1_PIN, GPIO_Mode_Out_PP);
    GPIO_CONFIG(AIN2_GPIO, AIN2_PIN, GPIO_Mode_Out_PP);
    GPIO_CONFIG(BIN1_GPIO, BIN1_PIN, GPIO_Mode_Out_PP);
    GPIO_CONFIG(BIN2_GPIO, BIN2_PIN, GPIO_Mode_Out_PP);
    GPIO_CONFIG(CIN1_GPIO, CIN1_PIN, GPIO_Mode_Out_PP);
	GPIO_CONFIG(CIN2_GPIO, CIN2_PIN, GPIO_Mode_Out_PP);
	GPIO_CONFIG(DIN1_GPIO, DIN1_PIN, GPIO_Mode_Out_PP);
	GPIO_CONFIG(DIN2_GPIO, DIN2_PIN, GPIO_Mode_Out_PP);
    GPIO_CONFIG(STBY_GPIO, STBY_PIN, GPIO_Mode_Out_PP);
	
	// Enable GPIO used as KEY IRQ for interrupt
	GPIO_InitStructure.GPIO_Pin = KEY1_PIN;
	GPIO_InitStructure.GPIO_Mode = 	GPIO_Mode_IPU;	//IRQ pin should be Pull Down to prevent unnecessary EXT IRQ while DW1000 goes to sleep mode
	GPIO_Init(KEY1_GPIO, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = KEY2_PIN;
	GPIO_InitStructure.GPIO_Mode = 	GPIO_Mode_IPU;	//IRQ pin should be Pull Down to prevent unnecessary EXT IRQ while DW1000 goes to sleep mode
	GPIO_Init(KEY2_GPIO, &GPIO_InitStructure);
    return 0;
}


void Hal_I2C_LCD_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_15;	 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; 	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);	
 	GPIO_SetBits(GPIOB,GPIO_Pin_14|GPIO_Pin_15);
	OLED_Init();
	OLED_Fill(0xFF);
}

void HalI2cInit( void )
{
	Hal_I2C_LCD_Init();
	OLED_CLS();
	OLED_ShowStr(50, 1, "ALX", 2);
	OLED_ShowStr(15, 3, "FollowingCar", 2);
}


void IWDG_Configuration(void)
{
    /* IWDG timeout equal to 280 ms (the timeout may varies due to LSI frequency dispersion) */
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);// Enable write access to IWDG_PR and IWDG_RLR registers
    IWDG_SetPrescaler(IWDG_Prescaler_256);       // IWDG counter clock: 40KHz(LSI) / 256 = 156.25Hz
    IWDG_SetReload(312);                         // 256 / 40KHz(LSI) * 312 = 1996.8 ms
    IWDG_ReloadCounter();                        // Reload IWDG counter */
    IWDG_Enable();                               // Enable IWDG (the LSI oscillator will be enabled by hardware) */
}

void Sleep(u32 x)
{
    u32 y = portGetTickCount();
    while((portGetTickCount() - y) < x);
}

/*
 *******************************************************************************
 *
 *******************************************************************************
 */

void OutputInit(void)
{
	STBY_off();	
    led1_off();
    led2_off();
}

int parameter_init(void)
{
    Sleep(1000);
    FM24CxxRandomContinuesReadCh(SLAVE_Address, 0, SYSTEMSIZE, SystemPar.ch);
	CheckVelocity();
	MotorData.Velocity=SystemPar.item.Velocity;
	MotorData.Dis=50;
    IWDG_Configuration();   
    return 0;
}

int peripherals_init(void)
{
    RCC_Configuration();
    GPIO_Configuration();
    SysTick_Configuration();
    USART1_Configuration(115200);
    adc_Init();	
    PWM_Int(7199,0); 
	HalI2cInit();
	OutputInit();
	
    return 0;
}

/*
 *******************************************************************************
 *   1-65535
 *******************************************************************************
 */
void CheckVelocity(void)
{
    if((SystemPar.item.Velocity < 4500) || (SystemPar.item.Velocity >7200))
    {
        SystemPar.item.Velocity = 7200;
    }
}


