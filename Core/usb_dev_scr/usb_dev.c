#include "usb_dev.h"
#include "../lcd_utilities/lcd_util.h"

uint32_t enum_speed = 0;
uint32_t setup_data_buffer[SETUP_DATA_BUFFER_ARRAY_SIZE];
volatile uint8_t usb_is_configured = 0;

uint32_t test_data_buffer[50]; // OUT data buffer for sending to HOST
uint32_t tb_ind = 0;
uint32_t loop_testing = 0;

void EP0_SendP(uint8_t pckt_cnt, uint16_t bytes_trans) {
    // Send 0-length packet on EP0 IN
    USB_OTG_FS_IE0->DIEPTSIZ = 0;
    USB_OTG_FS_IE0->DIEPTSIZ = (pckt_cnt << 19) | bytes_trans; // 1 Packet, 0 Bytes
    USB_OTG_FS_IE0->DIEPCTL |= (1 << 31) | (1 << 26); // EP Enable, CNAK
}

void EP0_RecP(uint8_t pckt_cnt, uint16_t bytes_trans) {
	USB_OTG_FS_OE0->DOEPTSIZ = (3 << 29) | (pckt_cnt << 19) | bytes_trans;
	USB_OTG_FS_OE0->DOEPCTL |= (1 << 31) | (1 << 26); //  Enable the Endpoint (Bit 31) and Clear NAK (Bit 26)
}

const uint8_t hid_report_desc[] = {
    0x05, 0x0C,        // Usage Page (Consumer)
    0x09, 0x01,        // Usage (Consumer Control)
    0xA1, 0x01,        // Collection (Application)
    0x09, 0xE9,        //   Usage (Volume Increment)
    0x09, 0xEA,        //   Usage (Volume Decrement)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1 bit)
    0x95, 0x02,        //   Report Count (2 buttons)
    0x81, 0x02,        //   Input (Data, Var, Abs)
    0x75, 0x06,        //   Report Size (6 bits padding)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x03,        //   Input (Cnst, Var, Abs)
    0xC0               // End Collection
};

Device_Descriptor device_descriptor = {
	.len = 18,                       // Device Descriptor size (always 18)
	.desc_type = 1,                  // USB_DESC_TYPE_DEVICE
	.usb_spec_bcd = 0x0200,          // USB 2.0
	.device_class = 0x00,            // Class defined at interface level
	.device_subclass = 0x00,
	.device_protocol = 0x00,
	.max_packet_size = 32,           // EP0 max packet size for FS

	.vendor_id = 0x0483,             // Example: STMicroelectronics
	.product_id = 0x5751,            // Your custom HID product ID
	.device_rel_bcd = 0x0100,        // Device release number: 1.00

	.manufacturer_ind_string = 0,    // String index 1 = "Manufacturer"
	.product_ind_string = 0,         // String index 2 = "Product"
	.serial_no_ind_string = 0,       // String index 3 = "SN1234"

	.num_of_configs = 1              // HID devices typically have 1 configuration
};

const HID_Configuration_Set hid_config_set = {

	.config = {
		.len = 9,
		.desc_type = 0x02,
		.total_len = sizeof(HID_Configuration_Set), // Auto-calculates 34 bytes
		.num_of_interfaces = 1,
		.config_value = 1,
		.config_ind_string = 0,
		.bm_attributes = 0b10000000,
		.max_power = 50        // 100mA
	},

	.interface = {
		.len = 9,
		.desc_type = 0x04,
		.interface_num = 0,
		.alternate_setting = 0,
		.num_of_endpoints = 1,
		.class = 0x03,    // HID Class
		.subclass = 0x00, // No Boot
		.protocol = 0x00, // No Protocol
		.interface_ind_string = 0
	},

	.hid = {
		.bLength = 9,
		.bDescriptorType = 0x21, // HID
		.bcdHID = 0x0111,        // v1.11
		.bCountryCode = 0,
		.bNumDescriptors = 1,
		.bDescriptorType_Report = 0x22, // Report Descriptor type
		.wDescriptorLength = sizeof(hid_report_desc) // Size of array above
	},

	.endpoint = {
		.len = 7,
		.desc_type = 0x05,
		.endpoint_address = 0x81, // EP1 IN
		.bm_attributes = 0x03,    // Interrupt
		.max_packet_size = 4,
		.interval = 10            // 10ms
	}
};

void USB_Core_Init(void) {

	// CORE INITIALIZATION --

	RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN; // OTG_FS clock enabled
	volatile uint32_t tmp = RCC->AHB2ENR;  // read back to ensure the clock is enabled

	HAL_NVIC_SetPriority(OTG_FS_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(OTG_FS_IRQn);

	// B plug connected so OTG_FS_GINTSTS CMOD will reflect the current mode

	USB_OTG_FS -> GAHBCFG |= (1); // Global interrupt mask is unmasked
	USB_OTG_FS -> GAHBCFG |= (1 << 7); // TxFIFO empty level is completely empty
	USB_OTG_FS -> GAHBCFG |= (1 << 8); // Periodic TxFIFO empty level is completely empty

	// USB_CORE_R -> OTG_FS_GINTSTS (RXFLVL) is for checking pending packet in RxFIFO

	USB_OTG_FS -> GUSBCFG &= ~(1 << 9); // HNP capabilities
	USB_OTG_FS -> GUSBCFG &= ~(1 << 8); // SRP capabilities
	USB_OTG_FS -> GUSBCFG |= (5 << 10); // Turn around time 5
	USB_OTG_FS -> GUSBCFG |= (3); // FS timeout calibration

	USB_OTG_FS -> GINTMSK |= (1 << 2); // OTG interrupt unmask
	USB_OTG_FS -> GINTMSK |= (1 << 1); // Mode mismatch interrupt unmask

	USB_OTG_FS -> GUSBCFG |= (1 << 30); // Force device mode

	HAL_Delay(25);

//	USB_OTG_FS->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;
//	    while (USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_CSRST);

}

void USB_Device_Init(void) {
	// DEVICE INITIALIZATION --
	USB_OTG_FS -> GINTMSK |= (1 << 3); // SOF unmask
	USB_OTG_FS -> GINTMSK |= (1 << 10); // Early Suspend unmask
	USB_OTG_FS -> GINTMSK |= (1 << 13); // Enumeration done unmask
	USB_OTG_FS -> GINTMSK |= (1 << 12); // USB Reset unmask
	USB_OTG_FS -> GINTMSK |= (1 << 11); // USB Suspend unmask

	USB_OTG_FS -> GINTMSK |= (1 << 7); // Global OUT NAK unmasked
	USB_OTG_FS -> GINTMSK |= (1 << 6); // Global non-periodic IN NAK unmasked

	USB_OTG_FS_D -> DCFG |= (1 << 2); // STALL on data packet in handshake control stage
	USB_OTG_FS_D -> DCFG |= (3); // FS mode

	USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_PWRDWN; // power down
	USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBUSASEN; // A Bus sensing off
	USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_VBUSBSEN; // B device Bus sensing on

}

void USB_Post_Reset_Pre_Enum(void) {

	USB_OTG_FS_OE0 -> DOEPCTL |= (1 << 27); // SNAK = 1 for all
	USB_OTG_FS_OE1 -> DOEPCTL |= (1 << 27);
	USB_OTG_FS_OE2 -> DOEPCTL |= (1 << 27);
	USB_OTG_FS_OE3 -> DOEPCTL |= (1 << 27);

	USB_OTG_FS_D -> DAINTMSK |= (1); // IN End0 Interrupt unmasked
	USB_OTG_FS_D -> DAINTMSK |= (1 << 16); // OUT End0 Interrupt unmasked

	USB_OTG_FS_D -> DOEPMSK |= (1); // Transfer completed Interrupt mask
	USB_OTG_FS_D -> DOEPMSK |= (1 << 3); // Setup phase done interrupt mask

	USB_OTG_FS_D -> DIEPMSK |= (1); // Transfer completed Interrupt mask
	USB_OTG_FS_D -> DIEPMSK |= (1 << 3); // Timeout condition done interrupt mask

	USB_OTG_FS -> GRXFSIZ = (64); // Rx FIFO depth
	USB_OTG_FS -> DIEPTXF0_HNPTXFSIZ = (64 << 16) | (64); // TX0 FIFO depth for endpoint 0
	USB_OTG_FS->DIEPTXF[0] = (64 << 16) | (128);  // TX1 FIFO for endpoint 1 (for HID)

	USB_OTG_FS_OE0 -> DOEPTSIZ |= (3 << 29); // Receive 3 back-to-back setup packets

//	USB_OTG_FS_D->DCTL &= ~USB_OTG_DCTL_SDIS; // Soft disconnect by removing DP pull-up resisitor
}

void USB_WriteFIFO(const uint8_t *data, uint16_t len) {
    uint32_t count32b = (len + 3) / 4;
    volatile uint32_t *fifo = USB_OTG_FS_TX0_FIFO;

    for (uint32_t i = 0; i < count32b; i++) {
        uint32_t word_to_send = 0;

        // Manual byte-packing to handle unaligned memory addresses safely
        for (uint8_t j = 0; j < 4; j++) {
            if ((i * 4 + j) < len) {
                word_to_send |= ((uint32_t)data[i * 4 + j]) << (8 * j);
            }
        }
        *fifo = word_to_send;
    }
}

void USB_Setup_Process(void) {
	uint32_t w0 = setup_data_buffer[0];
	uint32_t w1 = setup_data_buffer[1];

	uint8_t  bmRequestType = (uint8_t)(w0 & 0xFF);
	uint8_t  bRequest      = (uint8_t)((w0 >> 8) & 0xFF);
	uint16_t wValue        = (uint16_t)((w0 >> 16) & 0xFFFF);
	uint16_t wIndex        = (uint16_t)(w1 & 0xFFFF);
	uint16_t wLength       = (uint16_t)((w1 >> 16) & 0xFFFF);

	// 1. HANDLE STANDARD DEVICE REQUESTS
	if ((bmRequestType & 0x60) == 0x00) { // Standard Request
		switch (bRequest) {

			case USB_REQ_SET_ADDRESS: // Host sends address in wValue
				uint8_t dev_addr_pending = (uint8_t)(wValue & 0x7F);

				USB_OTG_FS_D->DCFG &= ~(0x7F << 4);     // Clear old address
				USB_OTG_FS_D->DCFG |= (dev_addr_pending << 4); // Set new address

				LCD_PrintString16(200, 320, "add_set", 7);

				EP0_SendP(1,0); // Send Status IN (ACK)
				// Note: We do NOT write to DCFG here. We wait for the ZLP to finish.
				break;

			case USB_REQ_SET_CONFIGURATION: // Host selects configuration (wValue)
				USB_OTG_FS_D -> DAINTMSK |= (1 << 1); // IN End0 Interrupt unmasked

				USB_OTG_FS_IE1->DIEPCTL = 0; // Clear
				USB_OTG_FS_IE1->DIEPCTL |= (1 << 28); // Data0 PID
				USB_OTG_FS_IE1->DIEPCTL |= (3 << 18); // Interrupt Type
				USB_OTG_FS_IE1->DIEPCTL |= (1 << 22); // FIFO #1
				USB_OTG_FS_IE1->DIEPCTL |= (1 << 15); // Active
				USB_OTG_FS_IE1->DIEPCTL |= 4;        // MPS

				// ADD THIS: Flush TxFIFO #1 to ensure it's clean
				USB_OTG_FS->GRSTCTL = (1 << 5) | (1 << 6); // Bit 5: Flush, Bit 6-10: FIFO Num (1)
				while (USB_OTG_FS->GRSTCTL & (1 << 5));    // Wait for flush

				usb_is_configured = 1;

				EP0_SendP(1,0); // Send Status IN (ACK)
				break;

			case USB_REQ_GET_DESCRIPTOR: // Host requests for descriptor
				uint8_t desc_type = (wValue >> 8) & 0xFF;
				const void *pData = 0;
				uint16_t len = 0;

				if (desc_type == 1) { // DEVICE
					pData = &device_descriptor;
					len = sizeof(device_descriptor);
				}
				else if (desc_type == 2) { // CONFIGURATION
					pData = &hid_config_set;
					len = sizeof(hid_config_set);
				}
				else if (desc_type == 0x22) { // HID REPORT
					pData = hid_report_desc;
					len = sizeof(hid_report_desc);
				}

				if (pData != 0) {
					if (len > wLength) len = wLength; // Send MIN

					uint8_t pkt_cnt = (len + device_descriptor.max_packet_size - 1) / device_descriptor.max_packet_size;

					EP0_SendP(pkt_cnt, len); // Send correct packet count
					USB_WriteFIFO((const uint8_t*)pData, len);

					EP0_RecP(1,32); // Preparing for the acknowledge
				}
				break;

			default:
				// Unsupported request? Stall.
				// USB_OTG_FS_OE0->DOEPCTL |= (1 << 21);
				break;
		}
	}

	test_data_buffer[tb_ind] = w0;
	test_data_buffer[tb_ind + 1] = w1;
	tb_ind += 2;

	setup_data_buffer[0] = 0; // clearing buffer
	setup_data_buffer[1] = 0; // clearing buffers

	return;
}

void USB_HID_Send_Consumer_Control(uint8_t cmd) {
	if (USB_OTG_FS_IE1->DIEPCTL & (1 << 31)) {
		return;
	}
    // Write cmd to FIFO 1 (Consumer Control)
    USB_OTG_FS_IE1->DIEPTSIZ = (1 << 19) | 1; // 1 packet, 1 byte
    USB_OTG_FS_IE1->DIEPCTL |= (1 << 31) | (1 << 26); // Enable, CNAK
    USB_OTG_FS_TX1_FIFO[0] = (uint32_t)cmd;
}

