/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../usb_dev_scr/usb_dev.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(IR_IN_Pin);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void OTG_FS_IRQHandler(void) {

    uint32_t status = USB_OTG_FS->GINTSTS;

    if (status & USB_OTG_GINTSTS_USBSUSP) {
		USB_OTG_FS->GINTSTS = USB_OTG_GINTSTS_USBSUSP; // Clear flag

		usb_is_configured = 0; // <--- STOP SENDING

		// Optional: Put MCU to low power mode here
		return;
	}

    if (status & USB_OTG_GINTSTS_USBRST) {
    	USB_OTG_FS->GINTSTS |= USB_OTG_GINTSTS_USBRST; // clear interrupt

    	usb_is_configured = 0;

    	USB_Post_Reset_Pre_Enum();

    	return;
    }

    if (status & USB_OTG_GINTSTS_ENUMDNE) {
    	USB_OTG_FS->GINTSTS |= USB_OTG_GINTSTS_ENUMDNE; // clear interrupt
    	enum_speed = (USB_OTG_FS_D ->DSTS >> 1) & 0x3;

    	// ENDPOINT 0 PRE-ACTIVATION --

    	USB_OTG_FS -> GINTMSK |= USB_OTG_GINTMSK_OEPINT; // Unmasked Out endpoints interrupt (OEPINT)
    	USB_OTG_FS -> GINTMSK |= USB_OTG_GINTMSK_IEPINT; // Unmasked In endpoints interrupt (IEPINT)
    	USB_OTG_FS_D -> DOEPMSK |= (1 << 1); // Endpoint disabled interrupt mask unmasked
    	USB_OTG_FS_D -> DIEPMSK |= (1 << 1); //  Endpoint disabled interrupt mask unmasked

    	USB_OTG_FS_OE0-> DOEPCTL |= (0b01); // Maximum Packet size
    	USB_OTG_FS_IE0-> DIEPCTL |= (0b01); // Maximum Packet size

    	return;
    }

    if (status & USB_OTG_GINTSTS_RXFLVL) {
    	USB_OTG_FS -> GINTMSK &= ~(1 << 4);

    	uint32_t rx = USB_OTG_FS->GRXSTSP;
		uint32_t epnum  =  rx        & 0x0F;
		uint32_t bcnt   = (rx >> 4)  & 0x7FF;
		uint32_t dpid   = (rx >> 15) & 0x03;
		uint32_t pktsts = (rx >> 17) & 0x0F;

		if (pktsts == 6) { // 0 <4:1 <8:2

			USB_OTG_FS->GRSTCTL = (1 << 5) | (0 << 6); // Bit 5: TxFIFO Flush, Bits 10:6: FIFO Num
			while (USB_OTG_FS->GRSTCTL & (1 << 5));      // Wait for flush to clear

			// SETUP packet received with data
			for (int i = 0; i < (bcnt + 3) / 4; i++) {
				if (i < 2) { // We only expect 8 bytes (2 words) for Setup
					setup_data_buffer[i] = USB_OTG_FS_RX_FIFO[0];
//					LCD_PrintUnsigned32Hex(i*20 + 30,320,setup_data_buffer[i]);
				} else {
					volatile uint32_t trash = USB_OTG_FS_RX_FIFO[0]; // Flush rest
					(void)trash;
				}
			}

			LCD_PrintString16(0, 320, "PKT 6 ", 6);
		} else if (pktsts == 4) {
			// SETUP transaction completed
			USB_Setup_Process();

			LCD_PrintString16(0, 200, "PKT 4 ", 6);

		} else if (pktsts == 2) {
			// OUT data packet received
			for(int i=0; i < (bcnt+3)/4; i++) { volatile uint32_t t = USB_OTG_FS_RX_FIFO[0]; (void)t; } // Flush OUT data

		} else if (pktsts == 3) {
			// OUT transfer completed
			for(int i=0; i < (bcnt+3)/4; i++) { volatile uint32_t t = USB_OTG_FS_RX_FIFO[0]; (void)t; } // Flush OUT data
			LCD_PrintString16(0, 100, "PKT 3 ", 6);

		} else if (pktsts == 1) {
			// Global NAK
			for(int i=0; i < (bcnt+3)/4; i++) { volatile uint32_t t = USB_OTG_FS_RX_FIFO[0]; (void)t; } // Flush OUT data
		}

//		LCD_PrintUnsigned32Hex(0,320,rx_fifo_pop);
    	USB_OTG_FS -> GINTMSK |= (1 << 4);

    	return;
    }

    if (status & USB_OTG_GINTSTS_OEPINT) {
    	uint32_t ep_int = USB_OTG_FS_D -> DAINT;
    	if (ep_int & (1 << 16)) {
    		// Out endpoint 0
    		if (USB_OTG_FS_OE0 -> DOEPINT & (1 << 3)) {
    			// SETUP phase done
    			uint32_t setup_packet_count = (USB_OTG_FS_OE0 -> DOEPTSIZ & (0b11 << 29)) >> 29;

    			USB_OTG_FS_OE0 -> DOEPINT |= (1 << 3); // clearing bit

    		} else if (USB_OTG_FS_OE0 -> DOEPINT & (1 << 0)) {
    			// Out Transfer completed
    			USB_OTG_FS_OE0 -> DOEPINT |= (1 << 0); // clearing bit
    		}
    	}

    	return;
    }

    if (status & USB_OTG_GINTSTS_IEPINT) {
		// Read the DAINT register to see which IN endpoint triggered
		uint32_t ep_int = USB_OTG_FS_D->DAINT;

		if (ep_int & (1 << 0)) { // If Endpoint 0 IN triggered
			// Read DIEPINT to see source (XFRC - Transfer Complete)
			if (USB_OTG_FS_IE0->DIEPINT & (1 << 0)) {
				USB_OTG_FS_IE0->DIEPINT |= (1 << 0);    // Clear the XFRC interrupt bit
			}

			if (USB_OTG_FS_IE0->DIEPINT & (1 << 3)) { // Timeout
				 USB_OTG_FS_IE0->DIEPINT = (1 << 3);
			}
		}

		if (ep_int & (1 << 1)) { // Check Bit 1 for EP1
			// Read EP1 Interrupt Status
			if (USB_OTG_FS_IE1->DIEPINT & (1 << 0)) { // XFRC: Transfer Completed
				// Clear the interrupt bit so the CPU can return to main()
				loop_testing++;
				USB_OTG_FS_IE1->DIEPINT = (1 << 0);
			}

			// Also clear Timeout or other errors if they occur
			if (USB_OTG_FS_IE1->DIEPINT & (1 << 3)) { // Timeout
				 USB_OTG_FS_IE1->DIEPINT = (1 << 3);
			}
		}

		return;
	}

    if (status & USB_OTG_GINTSTS_BOUTNAKEFF) {
    	return;
    }

    if (status & USB_OTG_GINTSTS_SOF) {
    	USB_OTG_FS->GINTSTS |= USB_OTG_GINTSTS_SOF;
    	return;
    }

    if (status & USB_OTG_GINTSTS_PTXFE ) { // PTXFE: Periodic TxFIFO Empty
		USB_OTG_FS->GINTMSK &= ~(1 << 26); // Disable the mask
    	return;
	}

	if (status & USB_OTG_GINTSTS_NPTXFE) { // NPTXFE: Non-Periodic TxFIFO Empty
		USB_OTG_FS->GINTMSK &= ~(1 << 5); // Disable the mask
    	return;
	}

    if (status & USB_OTG_GINTSTS_EOPF) {
    	return;
    }
}

/* USER CODE END 1 */
