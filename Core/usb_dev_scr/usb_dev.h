#ifndef __CUSTOM_USB_DRIVERS_SCRATCH
#define __CUSTOM_USB_DRIVERS_SCRATCH

#include "stm32f407xx.h"
#include "string.h"

#define __IO volatile
//#define USB_OTG_FS_BASE_ADDRESS 0x50000000
//#define USB_OTG_FS_BASE_DEVICE_ADDRESS 0x50000800
//#define USB_OTG_FIFO_BASE                    0x1000UL
//#define USB_OTG_FIFO_SIZE                    0x1000UL

#define GET_BYTE(w, n)   ((uint8_t)((w) >> (8*(n))))
#define GET_WORD(w, n)   ((uint16_t)((w) >> (16*(n))))

#define USB_OTG_FS_D ((USB_OTG_DeviceTypeDef*) (USB_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE))

#define USB_OTG_FS_OE0 ((USB_OTG_OUTEndpointTypeDef*) (USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE))
#define USB_OTG_FS_IE0 ((USB_OTG_INEndpointTypeDef*) (USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE))

#define USB_OTG_FS_OE1 ((USB_OTG_OUTEndpointTypeDef*) (USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + (0x20 * 1)))
#define USB_OTG_FS_IE1 ((USB_OTG_INEndpointTypeDef*) (USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + (0x20 * 1)))

#define USB_OTG_FS_OE2 ((USB_OTG_OUTEndpointTypeDef*) (USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + (0x20 * 2)))
#define USB_OTG_FS_IE2 ((USB_OTG_INEndpointTypeDef*) (USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + (0x20 * 2)))

#define USB_OTG_FS_OE3 ((USB_OTG_OUTEndpointTypeDef*) (USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + (0x20 * 3)))
#define USB_OTG_FS_IE3 ((USB_OTG_INEndpointTypeDef*) (USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + (0x20 * 3)))

#define USB_OTG_FS_RX_FIFO ((volatile uint32_t*) (USB_OTG_FS_PERIPH_BASE + USB_OTG_FIFO_BASE))
//#define USB_OTG_FS_TX0_FIFO ((volatile uint32_t*) (USB_OTG_FS_PERIPH_BASE + USB_OTG_FIFO_BASE + 64*4))
//#define USB_OTG_FS_TX1_FIFO ((volatile uint32_t*) (USB_OTG_FS_PERIPH_BASE + USB_OTG_FIFO_BASE + 64*4 + 64*4))

#define USB_OTG_FS_TX0_FIFO ((volatile uint32_t*) (USB_OTG_FS_PERIPH_BASE + 0x1000UL))
#define USB_OTG_FS_TX1_FIFO ((volatile uint32_t*) (USB_OTG_FS_PERIPH_BASE + 0x2000UL))

#define SETUP_DATA_BUFFER_ARRAY_SIZE 10

extern uint32_t enum_speed; // enumeration speed
extern uint32_t setup_data_buffer[SETUP_DATA_BUFFER_ARRAY_SIZE];
extern volatile uint8_t usb_is_configured;
extern uint32_t test_data_buffer[50]; // OUT data buffer for sending to HOST
extern uint32_t tb_ind;
extern uint32_t loop_testing;

typedef struct __attribute__((packed)) {
	uint8_t len; // byte size
	uint8_t desc_type; // device descriptor type
	uint16_t usb_spec_bcd; // usb specification release number
	uint8_t device_class; // class code assigned by USB-IF
	uint8_t device_subclass; // subclass code assigned by USB-IF
	uint8_t device_protocol; // protocol code assigned by USB-IF
	uint8_t max_packet_size;
	uint16_t vendor_id; // STM 0x0483
	uint16_t product_id;
	uint16_t device_rel_bcd;
	uint8_t manufacturer_ind_string; // index of string descriptor describing the manufacturer
	uint8_t product_ind_string; // index of string descriptor describing the product
	uint8_t serial_no_ind_string; // index of string descriptor describing serial number
	uint8_t num_of_configs; // number of possible configurations
} Device_Descriptor ;

typedef struct __attribute__((packed)) {
	uint8_t len;  // byte size
	uint8_t desc_type; // configuration descriptor type
	uint16_t total_len; // total length of the data returned for this configuration including combined length of all descriptors
	uint8_t num_of_interfaces; // number of supported interfaces
	uint8_t config_value; // value to use for SetConfiguration()
	uint8_t config_ind_string; // index of string descriptor describing this configuration
	uint8_t bm_attributes; // bitmap attributes related to power configuration
	uint8_t max_power; // maximum power consumption of this device
} Configuration_Descriptor;

typedef struct __attribute__((packed)) {
	uint8_t len;  // byte size
	uint8_t desc_type; // interface descriptor type
	uint8_t interface_num; // Number of this interface
	uint8_t alternate_setting; // Alternate setting value
	uint8_t num_of_endpoints; // number of end points used by this interface
	uint8_t class; // class code assigned by USB-IF
	uint8_t subclass; // subclass code assigned by USB-IF
	uint8_t protocol; // Protocol code assigned by USB-IF
	uint8_t interface_ind_string; // index of string descriptor describing this interface
} Interface_Descriptor;

typedef struct __attribute__((packed)) {
	uint8_t len; // byte size
	uint8_t desc_type; // endpoint descriptor type
	uint8_t endpoint_address; // (bit 3..0 : end point number) (bit 7 : 0 for OUT and 1 for IN)
	uint8_t bm_attributes; // (bit 1..0: transfer type) (bit 3..2: synchronization type) (bit 5..4: Usage type)
	uint16_t max_packet_size; // maximum packet size this end point is capable of receiving or sending
	uint8_t interval; // Interval for polling data transfers expressed in frames
} Endpoint_Descriptor;

typedef struct __attribute__((packed)) {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint16_t bcdHID;
	uint8_t bCountryCode;
	uint8_t bNumDescriptors;
	uint8_t bDescriptorType_Report;
	uint16_t wDescriptorLength;
} HID_Descriptor;

// 2. Create a "Super Struct" that guarantees order and contiguous memory
typedef struct __attribute__((packed)) {
	Configuration_Descriptor config;
	Interface_Descriptor     interface;
	HID_Descriptor           hid;
	Endpoint_Descriptor      endpoint;
} HID_Configuration_Set;

#define USB_REQ_GET_STATUS        0x00
#define USB_REQ_SET_ADDRESS       0x05
#define USB_REQ_GET_DESCRIPTOR    0x06
#define USB_REQ_SET_DESCRIPTOR    0x07
#define USB_REQ_GET_CONFIGURATION 0x08
#define USB_REQ_SET_CONFIGURATION 0x09

void USB_Core_Init(void); // Initialize USB core according to programming model
void USB_Device_Init(void); // Initialize USB device according to programming model
void USB_Post_Reset_Pre_Enum(void);
void USB_Setup_Process(void);

void EP0_SendP(uint8_t pckt_cnt, uint16_t bytes_trans);
void EP0_RecP(uint8_t pckt_cnt, uint16_t bytes_trans);

void USB_HID_Send_Consumer_Control(uint8_t cmd);

#endif
