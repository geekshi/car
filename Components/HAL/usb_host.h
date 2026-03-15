#ifndef __USB_HOST_H
#define __USB_HOST_H

#include "stm32f10x.h"
#include "OSAL_Comdef.h"

/**************************************************************************************************
 * CONSTANTS
 **************************************************************************************************/
// USB Host Configuration
#define USB_HOST_MAX_DEVICES        1
#define USB_HOST_BUFFER_SIZE        64
#define USB_BARCODE_MAX_LENGTH      32

// USB HID Constants
#define USB_HID_CLASS               0x03
#define USB_HID_SUBCLASS            0x00
#define USB_HID_PROTOCOL            0x01

// USB Standard Requests
#define USB_REQ_GET_DESCRIPTOR      0x06
#define USB_REQ_SET_CONFIGURATION   0x09
#define USB_REQ_GET_REPORT          0x01
#define USB_REQ_SET_REPORT          0x09

// USB Descriptor Types
#define USB_DESC_TYPE_DEVICE        0x01
#define USB_DESC_TYPE_CONFIG        0x02
#define USB_DESC_TYPE_STRING        0x03
#define USB_DESC_TYPE_INTERFACE     0x04
#define USB_DESC_TYPE_HID           0x21
#define USB_DESC_TYPE_REPORT        0x22

// USB Endpoint Types
#define USB_EP_TYPE_CONTROL         0x00
#define USB_EP_TYPE_INTERRUPT       0x03

// USB Host Status
#define USB_HOST_IDLE               0
#define USB_HOST_WAIT_RESET         1
#define USB_HOST_WAIT_SOF           2
#define USB_HOST_WAIT_RCV_STATUS    3
#define USB_HOST_ERROR              4
#define USB_HOST_CONNECTED          5

// Barcode Scanner State
#define BARCODE_STATE_IDLE          0
#define BARCODE_STATE_RECEIVING     1
#define BARCODE_STATE_COMPLETE      2

/**************************************************************************************************
 * TYPEDEF
 **************************************************************************************************/
// USB Device Descriptor
typedef struct {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t bcdUSB;
    uint8_t  bDeviceClass;
    uint8_t  bDeviceSubClass;
    uint8_t  bDeviceProtocol;
    uint8_t  bMaxPacketSize0;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t  iManufacturer;
    uint8_t  iProduct;
    uint8_t  iSerialNumber;
    uint8_t  bNumConfigurations;
} USB_DeviceDescriptor;

// USB Configuration Descriptor
typedef struct {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t wTotalLength;
    uint8_t  bNumInterfaces;
    uint8_t  bConfigurationValue;
    uint8_t  iConfiguration;
    uint8_t  bmAttributes;
    uint8_t  bMaxPower;
} USB_ConfigDescriptor;

// USB Interface Descriptor
typedef struct {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bInterfaceNumber;
    uint8_t  bAlternateSetting;
    uint8_t  bNumEndpoints;
    uint8_t  bInterfaceClass;
    uint8_t  bInterfaceSubClass;
    uint8_t  bInterfaceProtocol;
    uint8_t  iInterface;
} USB_InterfaceDescriptor;

// USB Endpoint Descriptor
typedef struct {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bEndpointAddress;
    uint8_t  bmAttributes;
    uint16_t wMaxPacketSize;
    uint8_t  bInterval;
} USB_EndpointDescriptor;

// USB HID Descriptor
typedef struct {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t bcdHID;
    uint8_t  bCountryCode;
    uint8_t  bNumDescriptors;
    uint8_t  bReportDescriptorType;
    uint16_t wReportDescriptorLength;
} USB_HIDDescriptor;

// Barcode Data Structure
typedef struct {
    uint8_t data[USB_BARCODE_MAX_LENGTH];
    uint8_t length;
    uint8_t state;
} BarcodeData;

// USB Host Status Structure
typedef struct {
    uint8_t status;
    uint8_t device_connected;
    uint8_t device_configured;
    uint8_t current_address;
    USB_DeviceDescriptor device_desc;
    USB_ConfigDescriptor config_desc;
    USB_EndpointDescriptor int_endpoint;
    BarcodeData barcode;
} USB_HostStatus;

/**************************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************************/
extern USB_HostStatus usb_host_status;

/**************************************************************************************************
 * FUNCTIONS - API
 **************************************************************************************************/
void USB_Host_Init(void);
void USB_Host_Process(void);
uint8_t USB_Host_Check_Device(void);
void USB_Host_Reset(void);
void USB_Host_Set_Address(uint8_t addr);
void USB_Host_Get_Descriptor(uint8_t type, uint8_t index, uint8_t* buffer, uint16_t length);
void USB_Host_Set_Configuration(uint8_t config_value);
void USB_Host_Read_Interrupt(USB_EndpointDescriptor* ep, uint8_t* buffer, uint16_t length);
void USB_Host_Clear_Interrupt(void);

// Barcode Scanner Functions
void Barcode_Scanner_Init(void);
void Barcode_Scanner_Process(void);
uint8_t Barcode_Scanner_Get_Data(uint8_t* buffer, uint8_t max_length);
void Barcode_Scanner_Clear_Data(void);
uint8_t Barcode_Scanner_Is_Ready(void);

// USB Low Level Functions
void USB_HP_CAN1_TX_IRQHandler(void);
void USB_LP_CAN1_RX0_IRQHandler(void);
void USB_Port_Enable(FunctionalState state);

#endif /* __USB_HOST_H */