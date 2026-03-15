#ifndef __BARCODE_SCANNER_H
#define __BARCODE_SCANNER_H

#include "stm32f10x.h"
#include "OSAL_Comdef.h"
#include "usb_host.h"

/**************************************************************************************************
 * CONSTANTS
 **************************************************************************************************/
// 商品条码定义
#define BARCODE_COKE      "Coke 330ml"
#define BARCODE_FANTA     "Fanta 330ml"
#define BARCODE_SPRITE    "Sprite 330ml"

// 商品数量
#define MAX_PRODUCTS      3
#define MAX_NAME_LENGTH   32

// 显示时间（毫秒）
#define DISPLAY_TIME_1    2000    // 显示商品信息2秒
#define DISPLAY_TIME_2    3000    // 显示数据库信息3秒

// 扫码枪接口类型
#define SCANNER_INTERFACE_USB     0
#define SCANNER_INTERFACE_UART    1

// 默认使用串口方式（更可靠）
#define DEFAULT_SCANNER_INTERFACE SCANNER_INTERFACE_UART

/**************************************************************************************************
 * TYPEDEF
 **************************************************************************************************/
// 商品信息结构体
typedef struct {
    char name[MAX_NAME_LENGTH];
    uint8_t quantity;
} ProductInfo;

// 商品数据库结构体
typedef struct {
    ProductInfo products[MAX_PRODUCTS];
    uint8_t product_count;
} ProductDatabase;

// 扫码器状态
typedef enum {
    SCANNER_IDLE = 0,
    SCANNER_RECEIVING,
    SCANNER_PROCESSING,
    SCANNER_DISPLAY_PRODUCT,
    SCANNER_DISPLAY_DATABASE,
    SCANNER_RETURN_MODE
} ScannerState;

/**************************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************************/
extern ProductDatabase g_product_db;
extern ScannerState g_scanner_state;
extern char g_current_barcode[USB_BARCODE_MAX_LENGTH];
extern uint8_t g_barcode_length;

/**************************************************************************************************
 * FUNCTIONS - API
 **************************************************************************************************/
// 初始化函数
void Barcode_System_Init(void);
void Product_Database_Init(void);

// 处理函数
void Barcode_Scanner_Task(void);
void Process_Barcode_Data(char* barcode, uint8_t length);

// 商品数据库操作
uint8_t Find_Product_By_Barcode(char* barcode);
void Update_Product_Quantity(uint8_t index);
void Display_Product_Info(uint8_t index);
void Display_All_Products(void);

// OLED显示相关
void OLED_Show_Barcode_Product(char* product_name);
void OLED_Show_Product_Database(void);
void OLED_Clear_And_Show_Mode(void);

#endif /* __BARCODE_SCANNER_H */