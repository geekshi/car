/**********************************************************************************
 * @file        main.c
 * @author      久凌电子
 * @phone       13566273308(久凌电子 - 唐工)
 * @version V3.0.0
 * @date        2023.06.02
 * @brief       官改测距源码 (使用官方库 04.00.06) 运行平台:UWB-S1(参考下列网址)
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
#include <string.h>

extern volatile uint8_t g_barcode_ready;
extern char g_barcode_buffer[];
void Barcode_Echo_String(char *str);
void ShowBarcode(void);

// 商品库结构定义
#define MAX_ITEM_TYPES 3
typedef struct
{
    char Item[32];  // 商品条码
    uint8_t Qty;    // 数量
} ItemInfo;

// 商品库数组（RAM 中）
static ItemInfo g_itemDB[MAX_ITEM_TYPES] = {0};
static uint8_t g_itemCount = 0;  // 当前存储的商品种类数

// 查找商品在库中的索引，不存在则返回 -1
static int FindItemIndex(const char *item)
{
    for (uint8_t i = 0; i < g_itemCount; i++)
    {
        if (strcmp(g_itemDB[i].Item, item) == 0)
        {
            return i;
        }
    }
    return -1;
}

// 添加商品到库中
static void AddItemToDB(const char *item)
{
    int index = FindItemIndex(item);
    if (index >= 0)
    {
        // 商品已存在，数量 +1
        g_itemDB[index].Qty++;
    }
    else
    {
        // 商品不存在，添加新商品（最多 3 类）
        if (g_itemCount < MAX_ITEM_TYPES)
        {
            strncpy(g_itemDB[g_itemCount].Item, item, 31);
            g_itemDB[g_itemCount].Item[31] = '\0';
            g_itemDB[g_itemCount].Qty = 1;
            g_itemCount++;
        }
    }
}

// 显示商品库信息
static void DisplayItemDB(void)
{
    OLED_CLS();
    OLED_ShowStr(0, 0, (unsigned char*)"Item         Qty", 2);

    for (uint8_t i = 0; i < g_itemCount; i++)
    {
        char line[32];
        //sprintf(line, "%d%s %d", i + 1, g_itemDB[i].Item, g_itemDB[i].Qty);
			  sprintf(line, "%s   %d", g_itemDB[i].Item, g_itemDB[i].Qty);
        OLED_ShowStr(0, (i + 1) * 2, (unsigned char*)line, 2);
    }
}

/*******************************************************************************
*******************************************************************************/
void RCC_Configuration_part(void)
{
        ErrorStatus HSEStartUpStatus;
        RCC_ClocksTypeDef RCC_ClockFreq;

        /* 将 RCC 寄存器重新设置为默认值 */
        RCC_DeInit();

        /* 打开外部高速时钟晶振 HSE */
        RCC_HSEConfig(RCC_HSE_ON);

        /* 等待外部高速时钟晶振工作 */
        HSEStartUpStatus = RCC_WaitForHSEStartUp();

        if(HSEStartUpStatus != ERROR)
        {
                /* 开启 Flash 预读缓冲功能，时钟起振后使用 */
                FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

                /* 48~72Mhz 推荐 Latency 为 2 */
                FLASH_SetLatency(FLASH_Latency_2);

                /* 设置 AHB 时钟，72MHz HCLK = SYSCLK */
                RCC_HCLKConfig(RCC_SYSCLK_Div1);
                /* 设置告诉 APB2 时钟，1 分频 72MHz PCLK2 = HCLK */
                RCC_PCLK2Config(RCC_HCLK_Div1);
                /* 设置低速 APB1 时钟，2 分频 36MHz PCLK1 = HCLK/2 */
                RCC_PCLK1Config(RCC_HCLK_Div2);
                /*  设置 ADC 时钟 ADCCLK = PCLK2/4 */
                RCC_ADCCLKConfig(RCC_PCLK2_Div6);

                //设置 PLL 时钟源及倍频系数 不分频：RCC_PLLSource_HSE_Div1 9 倍频：RCC_PLLMul_9
                RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
                /* 打开 PLL */
                RCC_PLLCmd(ENABLE);
                /* 等待 PLL 稳定工作 */
                while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){}

                /* 选择 PLL 时钟作为时钟源 */
                RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

                /* 等待时钟源切换，进入稳定状态 */
                while (RCC_GetSYSCLKSource() != 0x08){}
        }

        RCC_GetClocksFreq(&RCC_ClockFreq);
}

/*******************************************************************************
* 函数名  : init
* 描述    : 初始化函数
* 输入    : 无
* 输出    : 无
* 返回值  : 无
*******************************************************************************/
void init(void)
{
        //初始化设置
        SystemInit();
        RCC_Configuration_part();
        SysTick_Init();      //嘀嗒定时
        Periph_init();       //指示灯 + 蜂鸣器初始化
        Timer_Init();        //定时器初始化设置
        Uart_Queue_Init();   //PDOA 数据队列初始化
        HalUARTInit();       //串口 1+ 串口 2 设置
        OLED_Configuration();//屏幕初始化与初始打印
        EXTI_ALL_Init();     //编码器 IO 初始化
        Motor_Gpio_init();   //马达电机 IO 口初始化
}


/*******************************************************************************
* 函数名  : main
* 描述    : 主函数
* 输入    : 无
* 输出    : 无
* 返回值  : 无
********************************************************************************/
int main(void)
{
        init();
        while(1)
        {
                Read_AoA_Control();
                if (g_barcode_ready == 1)
								{
										g_barcode_ready = 0;
										ShowBarcode();
								}
        }
}

void ShowBarcode(void)
{
		I2C_GenerateSTOP(I2C2, ENABLE);
		I2C_DeInit(I2C2);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,DISABLE);
		IIC_Init();
		OLED_CLS();
		OLED_ShowStr(0, 3, (char*)g_barcode_buffer, 2);
		Delay_ms(2000);
		OLED_display(3);

		// 将条码信息添加至商品库（最多存储 3 类商品）
		AddItemToDB(g_barcode_buffer);
		// 刷屏并显示商品库中所有商品信息，保留 3 秒
		DisplayItemDB();
		Delay_ms(3000);  // 保留 3 秒
		OLED_display(3);
}

void Barcode_Echo_String(char *str)
{
    char *p = str;
    while (*p)
    {
        USART_SendData(UART4, *p);
        while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
        p++;
    }
    USART_SendData(UART4, 0x0D);
    while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
    USART_SendData(UART4, 0x0A);
    while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);

    while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET);
}