/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../usb_dev_scr/usb_dev.h"
#include "../lcd_utilities/lcd_util.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern UART_HandleTypeDef huart2;
static uint32_t last_falling_edge_time = 0;
static uint8_t bit_index = 0;
static uint32_t ir_data = 0;
static uint8_t receiving = 0;
uint8_t last_cmd = 0;
uint8_t cmd_proc = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//int _write(int file, char *ptr, int len)
//{
//	CDC_Transmit_FS((uint8_t*)ptr, len);
//    return len;
//}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_FSMC_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim2);

  LCD_Init();
  LCD_SetFont(&Font16x24); // Choose font - big or small (&Font8x8)

  USB_Core_Init();
  USB_Device_Init();

  if (USB_OTG_FS -> GINTSTS & 1) {
	  LCD_PrintString16(220,320,"HOS",3);
  } else {
	  LCD_PrintString16(220,320,"DEV",3);
  }


  uint32_t* temp = test_data_buffer;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if ( !HAL_GPIO_ReadPin(IR_IN_GPIO_Port, IR_IN_Pin) ) {
//		  HAL_Delay(100);
//		  printf("0\n");
	  } else {
//		  HAL_Delay(100);
//		  printf("1\n");
	  }

	  if (!cmd_proc) {
		  if (usb_is_configured == 1) {
			  if (last_cmd == 0x0C) {
				  USB_HID_Send_Consumer_Control(1, 0x01); // Volume up control
				  HAL_Delay(50); // Short press
				  USB_HID_Send_Consumer_Control(1, 0x00); // Release

				  char chars[] = "Volume up";
				  LCD_PrintString16(0,320,chars, sizeof(chars)-1);

			  } else if (last_cmd == 0x18) {
				  USB_HID_Send_Consumer_Control(1, 0x02); // Volume down control
				  HAL_Delay(50); // Short press
				  USB_HID_Send_Consumer_Control(1, 0x00); // Release

				  char chars[] = "Volume down";
				  LCD_PrintString16(100,320,chars,sizeof(chars)-1);

			  } else if (last_cmd == 0x5e) {
				  USB_HID_Send_Consumer_Control(1, 0x04); // Mute
				  HAL_Delay(50); // Short press
				  USB_HID_Send_Consumer_Control(1, 0x00); // Release

				  char chars[] = "Mute";
				  LCD_PrintString16(100,320,chars,sizeof(chars)-1);

			  } else if (last_cmd == 0x16) {
				  USB_HID_Send_Consumer_Control(1, 0x08); // Brightness up
				  HAL_Delay(50); // Short press
				  USB_HID_Send_Consumer_Control(1, 0x00); // Release

				  char chars[] = "Bright down";
				  LCD_PrintString16(100,320,chars,sizeof(chars)-1);

			  } else if (last_cmd == 0x19) {
				  USB_HID_Send_Consumer_Control(1, 0x10); // Brightness down
				  HAL_Delay(50); // Short press
				  USB_HID_Send_Consumer_Control(1, 0x00); // Release

				  char chars[] = "Bright up";
				  LCD_PrintString16(100,320,chars,sizeof(chars)-1);

			  } else if (last_cmd == 0x46) {
				  USB_HID_Send_Consumer_Control(2, 0x02); // Sleep
				  HAL_Delay(50); // Short press
				  USB_HID_Send_Consumer_Control(2, 0x00); // Release

				  char chars[] = "Sleep";
				  LCD_PrintString16(100,320,chars,sizeof(chars)-1);

			  } else if (last_cmd == 0x45) {
				  USB_HID_Send_Consumer_Control(2, 0x04); // Wakeup
				  HAL_Delay(50); // Short press
				  USB_HID_Send_Consumer_Control(2, 0x00); // Release

				  char chars[] = "Wake up";
				  LCD_PrintString16(100,320,chars,sizeof(chars)-1);

			  } else if (last_cmd == 0x47) {
				  USB_HID_Send_Consumer_Control(2, 0x01); // Shutdown
				  HAL_Delay(50); // Short press
				  USB_HID_Send_Consumer_Control(2, 0x00); // Release

				  char chars[] = "Shutdown ;)";
				  LCD_PrintString16(100,320,chars,sizeof(chars)-1);

			  } else {
				  char chars[] = "            ";
				  LCD_PrintString16(100,320,chars,sizeof(chars)-1);
			  }
		  }
		  LCD_PrintUnsigned32Hex(70,320, last_cmd);
		  cmd_proc = 1;
	  }
	  HAL_Delay(200);

	  if (usb_is_configured == 10) {
		   // Wait long enough to see the effect
		  LCD_PrintString16(100,320,"command sent",13);
		  LCD_PrintUnsigned32Hex(130,320, loop_testing + 1);
		  HAL_Delay(1000);
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == IR_IN_Pin)  // Change to your IR pin
    {
    	// On falling edge (IR receiver goes LOW when signal starts)
		uint32_t now = __HAL_TIM_GET_COUNTER(&htim2);
		uint32_t pulse_duration = now >= last_falling_edge_time ?
								  now - last_falling_edge_time :
								  (0xFFFFFFFF - last_falling_edge_time + now);
		last_falling_edge_time = now;
//		printf("Pulse: %lu\n", pulse_duration);

		// 1. Check for NEC Start Pulse (~9ms + 4.5ms = ~13.5ms between pulses)
		if (!receiving && pulse_duration > 13300 && pulse_duration < 14000) {
			// Detected leading LOW pulse (~9ms)
			receiving = 1;
			bit_index = 0;
			ir_data = 0;
			return;
		}

		// 2. Decode bits if receiving
		if (receiving)
		{
			// NEC protocol sends 560us LOW + variable HIGH
			if (pulse_duration >= 2200 && pulse_duration <= 2350) {
				// Bit = 1
				ir_data |= (1UL << bit_index);
			}
			else if (pulse_duration >= 1100 && pulse_duration <= 1250) {
				// Bit = 0 → already zeroed in ir_data
			}
			else {
				// Invalid pulse → reset frame
				receiving = 0;
				bit_index = 0;
				ir_data = 0;
				return;
			}

			bit_index++;

			// 3. Full 32-bit code received
			if (bit_index >= 32)
			{
				receiving = 0;

				// Decode and verify
				uint8_t addr      = (ir_data >> 0) & 0xFF;
				uint8_t addr_inv  = (ir_data >> 8) & 0xFF;
				uint8_t cmd       = (ir_data >> 16) & 0xFF;
				uint8_t cmd_inv   = (ir_data >> 24) & 0xFF;

				if ((addr ^ addr_inv) == 0xFF && (cmd ^ cmd_inv) == 0xFF)
				{
					printf("NEC Address: 0x%02X, Command: 0x%02X\n", addr, cmd);
					last_cmd = cmd;
					cmd_proc = 0;
				}
				else
				{
					printf("Invalid NEC code received\n");
				}

				ir_data = 0;
				bit_index = 0;
			}
		}

    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
