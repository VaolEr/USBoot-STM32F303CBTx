/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bootloader.h"
#include "usbd_customhid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern	USBD_HandleTypeDef hUsbDeviceFS;
static FLASH_EraseInitTypeDef EraseInitStruct;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
bool UpdateFirmwareRequestFlag = false; // This flag true, if bootloader got update firmware request from USBoot app , else false
bool ClearFlashFlag = false;						// This flag true, if bootloader got clear flash request from USBoot app , else false
bool UpdatingInProcessFlag = false;			// This flag true, if the updating status online, else false
bool newSubPageFlag = false;						// This flag true, if bootloader got new part of qPageData from USBoot app, else false
bool MCURestartFlag = false;						// This flag true, if bootloader got restart MCU command from USBoot app, else false
bool MCURestartFromTimerFlag = false;		// This flag true, if bootloader got restart MCU command from Timer, else false
bool FlashEresed = false;

bool  USBdetectedFlag = false;					// This flag true, if USB bus power detected, else false

uint8_t THDBuff[CUSTOM_HID_EPOUT_SIZE]; 			// TO host data buffer
#define TH_RepoertID THDBuff[0]								// Repord_ID to host
uint8_t FHDBuff[CUSTOM_HID_EPOUT_SIZE];				// FROM host data buffer
#define FH_RepoertID FHDBuff[0]								// Repord_ID from host

/*
	* Page is 2048-bytes;
	*	1/4 page is 512 bytes = qPageData;
	* 512/32 = 16 => subPageNumber in range [0..15]
*/
uint8_t subPageData[32];		// Part of qPageData
uint32_t subPageNumber = 0;	// Number of part: 512/32 = 16 => subPageNumber in range [0..15]
uint8_t qPageData[512];			// Quarter of page data
uint8_t pageNumber = 0;			// Number of page in range [0..NUM_OF_PAGES] (see bootloader.h)
/*uint8_t testQPage[512] ={
0xD8, 0x0E, 0x00, 0x20, 0x9D, 0xA1, 0x00, 0x08, 0x9F, 0xB9, 0x00, 0x08, 0x0D, 0xB9, 0x00, 0x08, 
0x9D, 0xB9, 0x00, 0x08, 0x5F, 0xA2, 0x00, 0x08, 0x19, 0xC9, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x81, 0xBD, 0x00, 0x08,
0x6D, 0xA2, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xBD, 0x00, 0x08, 0x83, 0xBD, 0x00, 0x08, 
0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 
0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 
0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 
0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 
0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 
0x0D, 0xC9, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 
0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 
0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 
0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 
0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 
0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 
0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB7, 0xA1, 0x00, 0x08, 
0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 
0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 
0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB7, 0xA1, 0x00, 0x08, 0xB7, 0xA1, 0x00, 0x08, 
0xB7, 0xA1, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0xB7, 0xA1, 0x00, 0x08, 0xDF, 0xF8, 0x0C, 0xD0, 0x00, 0xF0, 0x2A, 0xF8, 
0x00, 0x48, 0x00, 0x47, 0x39, 0xC9, 0x00, 0x08, 0xD8, 0x0E, 0x00, 0x20, 0x06, 0x48, 0x80, 0x47, 
0x06, 0x48, 0x00, 0x47, 0xFE, 0xE7, 0xFE, 0xE7, 0xFE, 0xE7, 0xFE, 0xE7, 0xFE, 0xE7, 0xFE, 0xE7, 
0xFE, 0xE7, 0xFE, 0xE7, 0xFE, 0xE7, 0xFE, 0xE7, 0xE9, 0xBD, 0x00, 0x08, 0x89, 0xA1, 0x00, 0x08, 
0xD2, 0xB2, 0x01, 0xE0, 0x00, 0xF8, 0x01, 0x2B, 0x49, 0x1E, 0xFB, 0xD2, 0x70, 0x47, 0x00, 0x22, 
0xF6, 0xE7, 0x10, 0xB5, 0x13, 0x46, 0x0A, 0x46, 0x04, 0x46, 0x19, 0x46, 0xFF, 0xF7, 0xF0, 0xFF, 
0x20, 0x46, 0x10, 0xBD, 0x06, 0x4C, 0x07, 0x4D, 0x06, 0xE0, 0xE0, 0x68, 0x40, 0xF0, 0x01, 0x03, 
0x94, 0xE8, 0x07, 0x00, 0x98, 0x47, 0x10, 0x34, 0xAC, 0x42, 0xF6, 0xD3, 0xFF, 0xF7, 0xC8, 0xFF};*/


//uint32_t mainProgramPageAddress = MAIN_PROGRAM_START_ADDRESS;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void FLASH_EraseCountPages()
{	
	FlashEresed = false;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;//FLASH_TYPEERASE_MASSERASE;//
	EraseInitStruct.PageAddress = ADDR_FLASH_PAGE_20;//0x0800A000;//MAIN_PROGRAM_START_ADDRESS;
	EraseInitStruct.NbPages = NUM_OF_PAGES - MAIN_PROGRAM_PAGE_NUMBER; //108
	uint32_t eraseResult = 0;
	
	//HAL_FLASHEx_Erase(&flashType, &eraseResult);
	if(HAL_FLASHEx_Erase(&EraseInitStruct, &eraseResult) != HAL_OK) {
    //Erase error!
		while (1)
		{
			FlashEresed = false;
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			HAL_Delay(150);
		}
	}
	else{
		//HAL_Delay(250);
		FlashEresed = true;
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	}
//	for(int i = MAIN_PROGRAM_START_ADDRESS; i <  NUM_OF_PAGES - MAIN_PROGRAM_PAGE_NUMBER; i++){
//		FLASH_PageErase(i);
//	}

} //End of FLASH_EraseCountPages()
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

  /* USER CODE BEGIN 2 */
  MX_GPIO_Init();
	USBdetectedFlag = DetectUSB(USB_detect_GPIO_Port, USB_detect_Pin);
	
	if(USBdetectedFlag)
	{
		MX_USB_DEVICE_Init();
		MX_TIM2_Init();
		HAL_TIM_Base_Start_IT(&htim2); // Start WaitRequestUpdateTime seconds waiting timer
	} 
	else
	{
			//UpdateFirmwareRequestFlag = false;
			Boot_GoToMainProgram();
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
		//---------------------------------------------------------------//
		//------------- Detect Update Firmware Request flag -------------//
		//------------- and USB detected Flag 							-------------//
		//---------------------------------------------------------------//
		if(USBdetectedFlag && UpdateFirmwareRequestFlag)
		{
			
			HAL_TIM_Base_Stop_IT(&htim2); // Stop WaitRequestUpdateTime seconds waiting timer
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
			
			HAL_FLASH_Unlock();					
			
			Send_ReadyMessage_To_USBHost(); // Send to USBoot app ready message
							
			UpdatingInProcessFlag = true;		// Set updating status online
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
			
			UpdateFirmwareRequestFlag = false;	//Reset update firmware request flag
			
		}
		//---------------------------------------------------------------//
		//------------- ClearFlash request handler ----------------------//
		//---------------------------------------------------------------//
		if(ClearFlashFlag)
		{
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

			FLASH_EraseCountPages(); // delete old program

			Send_FlashErasedStatus_To_USBHost(FlashEresed);
			HAL_Delay(100);
			Send_FlashErasedStatus_To_USBHost(FlashEresed);
			HAL_Delay(100);
			Send_FlashErasedStatus_To_USBHost(FlashEresed);
			HAL_Delay(100);
			
			ClearFlashFlag = false;			
		}
		//---------------------------------------------------------------//
		//------------- newSubPage request handler ----------------------//
		//---------------------------------------------------------------//
		if(newSubPageFlag)
		{
			//----------------------------------------------------//
			//-----	Forming qPageData from subPagData arrays -----//
			//----------------------------------------------------//
			if(subPageNumber < 16)
			{
				for(int i = 0; i < 32; i++){
						int subPageInPagePosition = (32 * subPageNumber) + i;
						qPageData[subPageInPagePosition] = subPageData[i];
				}
				subPageNumber++;	// Count the number of subPages	of qPageData
			}
			//----------------------------------------------------//
			//-----	Write qPageData from USBoot app to flash -----//
			//----- **************************************** -----//
			//----- Send request new qPageData to USBoot app -----//
			//----------------------------------------------------//
			if(subPageNumber == 16) 
			{
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
				
				//HAL_FLASH_Unlock();		
				FLASH_WriteNewFirmwarePageData(qPageData);
				//HAL_FLASH_Lock();
				
				pageNumber++;	// Count the number of qPages of Pages 
				HAL_Delay(100);
				Send_RequestNewSubPage_To_USBHost();
				HAL_Delay(100);
				Send_RequestNewSubPage_To_USBHost();
				HAL_Delay(100);
				Send_RequestNewSubPage_To_USBHost();
								
				subPageNumber = 0;
				
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
			}
			newSubPageFlag = false;	// Reset new SubPage flag
		}
		//---------------------------------------------------------------//
		//------------- MCU restart request handler ---------------------//
		//---------------------------------------------------------------//
		if(MCURestartFlag || MCURestartFromTimerFlag)
		{
			HAL_FLASH_Lock();
			//__nop();
			Boot_GoToMainProgram();
			//MCURestartFlag = false;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 749;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 63999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USB_detect_Pin */
  GPIO_InitStruct.Pin = USB_detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_detect_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
