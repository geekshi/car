#include "barcode_scanner.h"
#include "oled_i2c.h"
#include "delay.h"
#include "hal_usart.h"
#include "string.h"
#include "control.h"

/**************************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************************/
ProductDatabase g_product_db = {0};
ScannerState g_scanner_state = SCANNER_IDLE;
char g_current_barcode[USB_BARCODE_MAX_LENGTH] = {0};
uint8_t g_barcode_length = 0;

// 当前模式状态（用于恢复显示）
static uint8_t g_saved_mode = 0;
static uint32_t g_state_start_time = 0;

/**************************************************************************************************
 * LOCAL FUNCTIONS
 **************************************************************************************************/
static void UART_Scanner_Init(void);
static uint8_t UART_Scanner_Get_Data(char* buffer, uint8_t max_length);
static void Clear_OLED_Line(uint8_t line);

/**************************************************************************************************
 * 初始化函数
 **************************************************************************************************/

/**************************************************************************
函数名  : Product_Database_Init
函数功能：初始化商品数据库
入口参数：无
返回值  : 无
**************************************************************************/
void Product_Database_Init(void)
{
    // 初始化三种商品信息
    strcpy(g_product_db.products[0].name, BARCODE_COKE);
    g_product_db.products[0].quantity = 1;

    strcpy(g_product_db.products[1].name, BARCODE_FANTA);
    g_product_db.products[1].quantity = 2;

    strcpy(g_product_db.products[2].name, BARCODE_SPRITE);
    g_product_db.products[2].quantity = 3;

    g_product_db.product_count = MAX_PRODUCTS;
}

/**************************************************************************
函数名  : UART_Scanner_Init
函数功能：初始化串口扫码枪（USART2）
入口参数：无
返回值  : 无
**************************************************************************/
static void UART_Scanner_Init(void)
{
    // USART2 已经在 HalUARTInit() 中初始化
    // 这里可以重新配置波特率等参数（如果需要）
    // 大多数 USB 扫码枪模拟串口时默认波特率为 9600 或 115200
}

/**************************************************************************
函数名  : Barcode_System_Init
函数功能：条码扫描系统初始化
入口参数：无
返回值  : 无
**************************************************************************/
void Barcode_System_Init(void)
{
    // 初始化商品数据库
    Product_Database_Init();

    // 初始化扫码器接口
#if (DEFAULT_SCANNER_INTERFACE == SCANNER_INTERFACE_UART)
    UART_Scanner_Init();
#else
    Barcode_Scanner_Init();
#endif

    // 初始化状态
    g_scanner_state = SCANNER_IDLE;
    g_saved_mode = 4;  // 默认遥控模式
}

/**************************************************************************************************
 * 条码数据处理函数
 **************************************************************************************************/

/**************************************************************************
函数名  : UART_Scanner_Get_Data
函数功能：从串口获取扫码数据
入口参数：buffer - 数据缓冲区，max_length - 最大长度
返回值  : 实际读取的数据长度
**************************************************************************/
static uint8_t UART_Scanner_Get_Data(char* buffer, uint8_t max_length)
{
    static char rx_buffer[USB_BARCODE_MAX_LENGTH] = {0};
    static uint8_t rx_index = 0;
    uint8_t data = 0;

    // 检查是否有数据可读
    if (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != RESET)
    {
        data = USART_ReceiveData(USART2);

        // 检查是否为回车符（扫码结束标志）
        if (data == '\r' || data == '\n')
        {
            if (rx_index > 0)
            {
                // 复制数据到输出缓冲区
                uint8_t len = (rx_index < max_length) ? rx_index : max_length;
                memcpy(buffer, rx_buffer, len);
                buffer[len] = '\0';

                // 重置接收索引
                rx_index = 0;
                memset(rx_buffer, 0, sizeof(rx_buffer));

                return len;
            }
        }
        else if (rx_index < (USB_BARCODE_MAX_LENGTH - 1))
        {
            // 存储接收到的字符
            rx_buffer[rx_index++] = data;
        }
    }

    return 0;
}

/**************************************************************************
函数名  : Find_Product_By_Barcode
函数功能：根据条码查找商品
入口参数：barcode - 条码字符串
返回值  : 商品索引（0-2），未找到返回 0xFF
**************************************************************************/
uint8_t Find_Product_By_Barcode(char* barcode)
{
    for (uint8_t i = 0; i < g_product_db.product_count; i++)
    {
        if (strcmp(barcode, g_product_db.products[i].name) == 0)
        {
            return i;
        }
    }
    return 0xFF;  // 未找到
}

/**************************************************************************
函数名  : Update_Product_Quantity
函数功能：更新商品数量
入口参数：index - 商品索引
返回值  : 无
**************************************************************************/
void Update_Product_Quantity(uint8_t index)
{
    if (index < g_product_db.product_count)
    {
        g_product_db.products[index].quantity++;
    }
}

/**************************************************************************************************
 * OLED 显示函数
 **************************************************************************************************/

/**************************************************************************
函数名  : Clear_OLED_Line
函数功能：清除 OLED 指定行
入口参数：line - 行号（0-7）
返回值  : 无
**************************************************************************/
static void Clear_OLED_Line(uint8_t line)
{
    OLED_ShowStr(0, line * 2, "                    ", 16);
}

/**************************************************************************
函数名  : OLED_Show_Barcode_Product
函数功能：在 OLED 上显示扫码商品名称
入口参数：product_name - 商品名称
返回值  : 无
**************************************************************************/
void OLED_Show_Barcode_Product(char* product_name)
{
    OLED_CLS();  // 清屏
    OLED_ShowStr(0, 0, "Scanned:", 16);
    OLED_ShowStr(0, 2, product_name, 16);
}

/**************************************************************************
函数名  : OLED_Show_Product_Database
函数功能：在 OLED 上显示所有商品信息
入口参数：无
返回值  : 无
**************************************************************************/
void OLED_Show_Product_Database(void)
{
    char buf[32];

    OLED_CLS();  // 清屏
    OLED_ShowStr(0, 0, "Product Database:", 16);

    // 显示每个商品的信息
    for (uint8_t i = 0; i < g_product_db.product_count; i++)
    {
        sprintf(buf, "%s, %d",
                g_product_db.products[i].name,
                g_product_db.products[i].quantity);
        OLED_ShowStr(0, (i + 1) * 2, buf, 16);
    }
}

/**************************************************************************
函数名  : OLED_Clear_And_Show_Mode
函数功能：清除屏幕并恢复遥控/跟随模式显示
入口参数：无
返回值  : 无
**************************************************************************/
void OLED_Clear_And_Show_Mode(void)
{
    OLED_ALL_Display(g_saved_mode);
}

/**************************************************************************************************
 * 主处理任务
 **************************************************************************************************/

/**************************************************************************
函数名  : Process_Barcode_Data
函数功能：处理接收到的条码数据
入口参数：barcode - 条码字符串，length - 长度
返回值  : 无
**************************************************************************/
void Process_Barcode_Data(char* barcode, uint8_t length)
{
    uint8_t product_index;

    // 保存当前模式
    g_saved_mode = AVG.Car_mode;

    // 查找商品
    product_index = Find_Product_By_Barcode(barcode);

    if (product_index != 0xFF)
    {
        // 找到商品，更新数量
        Update_Product_Quantity(product_index);

        // 保存条码信息
        strncpy(g_current_barcode, barcode, USB_BARCODE_MAX_LENGTH - 1);
        g_current_barcode[USB_BARCODE_MAX_LENGTH - 1] = '\0';
        g_barcode_length = length;

        // 进入显示商品状态
        g_scanner_state = SCANNER_DISPLAY_PRODUCT;
        g_state_start_time = portGetTickCnt();
    }
    else
    {
        // 未找到商品，显示错误信息
        OLED_CLS();
        OLED_ShowStr(0, 0, "Unknown Product:", 16);
        OLED_ShowStr(0, 2, barcode, 16);

        // 保存状态和时间，用于非阻塞延时
        g_scanner_state = SCANNER_RETURN_MODE;
        g_state_start_time = portGetTickCnt();
    }
}

/**************************************************************************
函数名  : Barcode_Scanner_Task
函数功能：条码扫描任务（需要在主循环中调用）
入口参数：无
返回值  : 无
**************************************************************************/
void Barcode_Scanner_Task(void)
{
    char barcode_buf[USB_BARCODE_MAX_LENGTH];
    uint8_t barcode_len;
    uint32_t current_time;

    // 状态机处理
    switch (g_scanner_state)
    {
        case SCANNER_IDLE:
            // 等待扫码数据
#if (DEFAULT_SCANNER_INTERFACE == SCANNER_INTERFACE_UART)
            barcode_len = UART_Scanner_Get_Data(barcode_buf, USB_BARCODE_MAX_LENGTH);
#else
            barcode_len = Barcode_Scanner_Get_Data((uint8_t*)barcode_buf, USB_BARCODE_MAX_LENGTH);
#endif

            if (barcode_len > 0)
            {
                // 接收到条码数据，开始处理
                Process_Barcode_Data(barcode_buf, barcode_len);
            }
            break;

        case SCANNER_DISPLAY_PRODUCT:
            // 获取当前时间
            current_time = portGetTickCnt();

            // 首次进入此状态时显示商品信息（前100ms内执行一次）
            if (current_time - g_state_start_time < 100)
            {
                OLED_Show_Barcode_Product(g_current_barcode);
            }

            // 显示商品信息 2 秒
            if (current_time - g_state_start_time >= DISPLAY_TIME_1)
            {
                // 显示商品数据库
                OLED_Show_Product_Database();
                g_scanner_state = SCANNER_DISPLAY_DATABASE;
                g_state_start_time = current_time;
            }
            break;

        case SCANNER_DISPLAY_DATABASE:
            // 显示数据库信息 3 秒
            current_time = portGetTickCnt();
            if (current_time - g_state_start_time >= DISPLAY_TIME_2)
            {
                // 恢复遥控/跟随模式显示
                OLED_Clear_And_Show_Mode();
                g_scanner_state = SCANNER_IDLE;
            }
            break;


        case SCANNER_RETURN_MODE:
            // 未知商品错误信息显示 2 秒后返回
            current_time = portGetTickCnt();
            if (current_time - g_state_start_time >= DISPLAY_TIME_1)
            {
                // 恢复遥控/跟随模式显示
                OLED_Clear_And_Show_Mode();
                g_scanner_state = SCANNER_IDLE;
            }
            break;
        default:
            g_scanner_state = SCANNER_IDLE;
            break;
    }
}