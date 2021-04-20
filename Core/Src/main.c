/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gpio.h"
#include "lwip.h"

#include "lwip/opt.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
#include "lwip/dhcp.h"
#include "netif/etharp.h"

#include "tftpserver.h"
#include "client.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef void (*pFunction)(void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//static pFunction Jump_To_Application;
//volatile uint32_t JumpAddress;
extern struct netif gnetif;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//uint8_t load_bootloader();
static void reset_device(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//	uint32_t Data_R = *(__IO uint32_t*) LOAD_BOOTLOADER_ADDRESS;
//	if (Data_R == 1 || Data_R == 0xFFFFFFFF) {
//
//	} else {
//		pFunction Jump_To_Application;
//		uint32_t JumpAddress;
//
//		JumpAddress = *(__IO uint32_t*) (USER_FLASH_FIRST_PAGE_ADDRESS + 4);
//		__disable_irq();
//		/* Deinit everything */
//		//		HAL_RCC_DeInit();
//		//		HAL_DeInit();
//		SysTick->CTRL = 0;
//		SysTick->LOAD = 0;
//		SysTick->VAL = 0;
//		/* Jump to user application */
//		Jump_To_Application = (pFunction) JumpAddress;
//
//		/* Initialize user application's Stack Pointer */
//		__set_MSP(*(__IO uint32_t*) USER_FLASH_FIRST_PAGE_ADDRESS);
//		__set_PSP(*(__IO uint32_t*) USER_FLASH_FIRST_PAGE_ADDRESS);
//		__enable_irq();
//		Jump_To_Application();
//		/* do nothing */
//		while (1)
//			;
//	}
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
	MX_GPIO_Init();
	uint32_t Data_R = *(__IO uint32_t*) LOAD_BOOTLOADER_ADDRESS;
	if (Data_R == 1 || Data_R == 0xFFFFFFFF || HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin)) {
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
		SystemClock_Config();
		MX_LWIP_Init();
	} else {
		uint32_t JumpAddress = *(__IO uint32_t*) (USER_FLASH_FIRST_PAGE_ADDRESS + 4);
		pFunction Jump_To_Application = (pFunction) JumpAddress;

		/* Deinit everything */
		HAL_GPIO_DeInit(LD1_GPIO_Port, LD1_Pin);
		HAL_GPIO_DeInit(LD2_GPIO_Port, LD2_Pin);
		HAL_GPIO_DeInit(LD3_GPIO_Port, LD3_Pin);
		HAL_GPIO_DeInit(USER_Btn_GPIO_Port, USER_Btn_Pin);
		HAL_RCC_DeInit();
		HAL_DeInit();
		SysTick->CTRL = 0;
		SysTick->LOAD = 0;
		SysTick->VAL = 0;

		__disable_irq();

		/* Initialize user application's Stack Pointer */
		__set_MSP(*(__IO uint32_t*) USER_FLASH_FIRST_PAGE_ADDRESS);
		__enable_irq();
		/* Jump to user application */
		Jump_To_Application();
		/* do nothing */
		while (1)
			;
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint8_t server_init_state = 0;
	uint8_t status = 0;
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		MX_LWIP_Process();
		if (gnetif.ip_addr.addr != 0 && server_init_state == 0) {
			server_init_state = 1;
			client_connect();
		}
		status = client_is_connected();
		if (status == S_CONN_ABRT) {
			client_close_connect();
			HAL_Delay(500);
			client_connect();
			HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
			HAL_Delay(500);
		}
		if (firmware_downloaded()) {
			reset_device();
		}
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static void reset_device(void) {
	FLASH_EraseInitTypeDef FLASH_EraseInitStruct;
	uint32_t sectornb = 0;

	FLASH_EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	FLASH_EraseInitStruct.Sector = FLASH_SECTOR_11;
	FLASH_EraseInitStruct.NbSectors = 1;
	FLASH_EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	HAL_FLASH_Unlock();
	if (HAL_FLASHEx_Erase(&FLASH_EraseInitStruct, &sectornb) != HAL_OK) {
		Error_Handler();
	}
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t) LOAD_BOOTLOADER_ADDRESS, 0);
	HAL_FLASH_Lock();
	NVIC_SystemReset();
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
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
