#include "usb_host.h"
#include "delay.h"
#include "hal_usart.h"

/**************************************************************************************************
 * LOCAL VARIABLES
 **************************************************************************************************/
USB_HostStatus usb_host_status = {0};
static uint8_t usb_rx_buffer[USB_HOST_BUFFER_SIZE] = {0};
static uint8_t barcode_buffer[USB_BARCODE_MAX_LENGTH] = {0};

/**************************************************************************************************
 * USB Host Low Level Functions
 **************************************************************************************************/

/**************************************************************************
函数名  : USB_Port_Enable
函数功能：使能/禁用USB端口
入口参数：state - ENABLE: 使能, DISABLE: 禁用
返回值  : 无
**************************************************************************/
void USB_Port_Enable(FunctionalState state)
{
    if (state == ENABLE)
    {
        // PA12 配置为下拉输入（模拟断开连接）
        GPIO_InitTypeDef GPIO_InitStructure;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;  // USB DP
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;  // 下拉输入
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

        Delay_ms(200);  // 延时等待

        // 重新配置为上拉输入（模拟连接）
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  // 浮空输入
        GPIO_Init(GPIOA, &GPIO_InitStructure);

        Delay_ms(100);
    }
    else
    {
        // 禁用USB端口
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, DISABLE);
    }
}

/**************************************************************************
函数名  : USB_Host_Init
函数功能：USB主机初始化
入口参数：无
返回值  : 无
**************************************************************************/
void USB_Host_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    // 使能USB时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);

    // 配置USB中断优先级
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 初始化USB主机状态
    usb_host_status.status = USB_HOST_IDLE;
    usb_host_status.device_connected = 0;
    usb_host_status.device_configured = 0;
    usb_host_status.current_address = 0;

    // 初始化条码数据
    usb_host_status.barcode.length = 0;
    usb_host_status.barcode.state = BARCODE_STATE_IDLE;

    // USB端口上电序列
    USB_Port_Enable(ENABLE);

    // 延时等待设备连接
    Delay_ms(500);
}

/**************************************************************************
函数名  : USB_Host_Check_Device
函数功能：检查USB设备是否连接
入口参数：无
返回值  : 1-设备已连接，0-设备未连接
**************************************************************************/
uint8_t USB_Host_Check_Device(void)
{
    // 检查USB_DP引脚状态（PA12）
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12) == Bit_SET)
    {
        return 1;
    }
    return 0;
}

/**************************************************************************
函数名  : USB_Host_Reset
函数功能：USB总线复位
入口参数：无
返回值  : 无
**************************************************************************/
void USB_Host_Reset(void)
{
    // 通过控制DP线的下拉电阻实现复位
    USB_Port_Enable(DISABLE);
    Delay_ms(50);
    USB_Port_Enable(ENABLE);
    Delay_ms(100);
}

/**************************************************************************
函数名  : USB_Host_Set_Address
函数功能：设置USB设备地址
入口参数：addr - 设备地址
返回值  : 无
**************************************************************************/
void USB_Host_Set_Address(uint8_t addr)
{
    // STM32F103 USB主机模式下设置地址
    // 这里简化处理，实际需要通过控制传输发送SET_ADDRESS请求
    usb_host_status.current_address = addr;
}

/**************************************************************************
函数名  : USB_Host_Get_Descriptor
函数功能：获取USB描述符
入口参数：type - 描述符类型，index - 索引，buffer - 缓冲区，length - 长度
返回值  : 无
**************************************************************************/
void USB_Host_Get_Descriptor(uint8_t type, uint8_t index, uint8_t* buffer, uint16_t length)
{
    // 简化实现，实际需要通过控制端点传输
    // 这里仅做框架展示
}

/**************************************************************************
函数名  : USB_Host_Set_Configuration
函数功能：设置USB配置
入口参数：config_value - 配置值
返回值  : 无
**************************************************************************/
void USB_Host_Set_Configuration(uint8_t config_value)
{
    // 简化实现，实际需要通过控制端点传输
    usb_host_status.device_configured = 1;
}

/**************************************************************************
函数名  : USB_Host_Read_Interrupt
函数功能：读取中断端点数据
入口参数：ep - 端点描述符，buffer - 缓冲区，length - 长度
返回值  : 无
**************************************************************************/
void USB_Host_Read_Interrupt(USB_EndpointDescriptor* ep, uint8_t* buffer, uint16_t length)
{
    // 简化实现，实际需要从USB FIFO读取数据
}

/**************************************************************************
函数名  : USB_Host_Clear_Interrupt
函数功能：清除USB中断标志
入口参数：无
返回值  : 无
**************************************************************************/
void USB_Host_Clear_Interrupt(void)
{
    // 清除USB中断标志
}

/**************************************************************************************************
 * Barcode Scanner Functions
 **************************************************************************************************/

/**************************************************************************
函数名  : Barcode_Scanner_Init
函数功能：条码扫描器初始化
入口参数：无
返回值  : 无
**************************************************************************/
void Barcode_Scanner_Init(void)
{
    USB_Host_Init();
    usb_host_status.barcode.state = BARCODE_STATE_IDLE;
    usb_host_status.barcode.length = 0;
}

/**************************************************************************
函数名  : Barcode_Scanner_Process
函数功能：条码扫描器处理函数（需要在主循环中调用）
入口参数：无
返回值  : 无
**************************************************************************/
void Barcode_Scanner_Process(void)
{
    static uint32_t last_check_time = 0;

    // 定期检查设备连接状态
    if (portGetTickCnt() - last_check_time > 100)
    {
        last_check_time = portGetTickCnt();

        if (USB_Host_Check_Device())
        {
            if (!usb_host_status.device_connected)
            {
                usb_host_status.device_connected = 1;
                // 设备连接，执行枚举过程
                USB_Host_Reset();
                Delay_ms(200);

                // 简化枚举过程
                USB_Host_Set_Address(1);
                Delay_ms(50);
                USB_Host_Set_Configuration(1);
            }

            // 尝试读取数据（模拟从USB接收）
            // 实际应用中需要真正的USB主机驱动
        }
        else
        {
            usb_host_status.device_connected = 0;
            usb_host_status.device_configured = 0;
            usb_host_status.barcode.state = BARCODE_STATE_IDLE;
        }
    }
}

/**************************************************************************
函数名  : Barcode_Scanner_Get_Data
函数功能：获取条码数据
入口参数：buffer - 数据缓冲区，max_length - 最大长度
返回值  : 实际读取的数据长度
**************************************************************************/
uint8_t Barcode_Scanner_Get_Data(uint8_t* buffer, uint8_t max_length)
{
    uint8_t len = 0;

    if (usb_host_status.barcode.state == BARCODE_STATE_COMPLETE)
    {
        len = (usb_host_status.barcode.length < max_length) ?
              usb_host_status.barcode.length : max_length;

        for (uint8_t i = 0; i < len; i++)
        {
            buffer[i] = usb_host_status.barcode.data[i];
        }

        // 清除状态
        usb_host_status.barcode.state = BARCODE_STATE_IDLE;
        usb_host_status.barcode.length = 0;
    }

    return len;
}

/**************************************************************************
函数名  : Barcode_Scanner_Clear_Data
函数功能：清除条码数据
入口参数：无
返回值  : 无
**************************************************************************/
void Barcode_Scanner_Clear_Data(void)
{
    usb_host_status.barcode.state = BARCODE_STATE_IDLE;
    usb_host_status.barcode.length = 0;
}

/**************************************************************************
函数名  : Barcode_Scanner_Is_Ready
函数功能：检查是否有新条码数据
入口参数：无
返回值  : 1-有数据，0-无数据
**************************************************************************/
uint8_t Barcode_Scanner_Is_Ready(void)
{
    return (usb_host_status.barcode.state == BARCODE_STATE_COMPLETE) ? 1 : 0;
}

/**************************************************************************************************
 * USB Interrupt Handlers
 **************************************************************************************************/

/**************************************************************************
函数名  : USB_HP_CAN1_TX_IRQHandler
函数功能：USB高优先级中断处理
入口参数：无
返回值  : 无
**************************************************************************/
void USB_HP_CAN1_TX_IRQHandler(void)
{
    // USB高优先级中断处理
}

/**************************************************************************
函数名  : USB_LP_CAN1_RX0_IRQHandler
函数功能：USB低优先级中断处理
入口参数：无
返回值  : 无
**************************************************************************/
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    // USB低优先级中断处理
    // 检查接收到的数据
}

/**************************************************************************************************
 * UART-based Barcode Scanner Support (Alternative Method)
 * 注意：由于STM32F103的USB主机功能有限，很多USB扫码枪实际上是通过串口模拟的
 * 以下函数用于支持串口方式的扫码枪
 **************************************************************************************************/

// 如果需要支持串口扫码枪，可以使用USART2或其他空闲串口
// 请参考 hal_usart.c 中的串口初始化代码