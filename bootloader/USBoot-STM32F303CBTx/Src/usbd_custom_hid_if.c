/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_custom_hid_if.c
  * @version        : v2.0_Cube
  * @brief          : USB Device Custom HID interface file.
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
#include "usbd_custom_hid_if.h"

/* USER CODE BEGIN INCLUDE */
#include "stdbool.h"
//#include "bootloader.h"
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern uint8_t FHDBuff[CUSTOM_HID_EPOUT_SIZE];				// FROM host data buffer
#define FH_RepoertID FHDBuff[0]

extern bool UpdateFirmwareRequestFlag;
extern bool ClearFlashFlag;
extern bool newSubPageFlag;
extern bool MCURestartFlag;
extern uint8_t subPageData[32];
extern uint8_t subPageNumber;
/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device.
  * @{
  */

/** @addtogroup USBD_CUSTOM_HID
  * @{
  */

/** @defgroup USBD_CUSTOM_HID_Private_TypesDefinitions USBD_CUSTOM_HID_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Defines USBD_CUSTOM_HID_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Macros USBD_CUSTOM_HID_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Variables USBD_CUSTOM_HID_Private_Variables
  * @brief Private variables.
  * @{
  */

/** Usb HID report descriptor. */
__ALIGN_BEGIN static uint8_t CUSTOM_HID_ReportDesc_FS[USBD_CUSTOM_HID_REPORT_DESC_SIZE] __ALIGN_END =
{
/* USER CODE BEGIN 0 */
		0x06, 0x00, 0xff,              // USAGE_PAGE (Generic Desktop)
    0x09, 0x01,                    // USAGE (Vendor Usage 1)
    0xa1, 0x00,                    // COLLECTION (Physical)
	//--------------------------------------------------------------//
	//-------------------- Input (to HOST) -------------------------//
	//--------------------------------------------------------------//
    0x85, 0xD0,                    //   REPORT_ID (208)
    0x09, 0x01,                    //   USAGE (Vendor Usage 1)
    0x19, 0x00,                    //   USAGE_MINIMUM (Undefined)
    0x29, 0x00,                    //   USAGE_MAXIMUM (Undefined)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x23,                    //   REPORT_COUNT (35)
    0x81, 0x06,                    //   INPUT (Data,Var,Rel)
	//--------------------------------------------------------------//
    0x85, 0xB0,                    //   REPORT_ID ()
    0x09, 0x01,                    //   USAGE (Vendor Usage 1)
    0x19, 0x00,                    //   USAGE_MINIMUM (Undefined)
    0x29, 0x00,                    //   USAGE_MAXIMUM (Undefined)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x87,                    //   REPORT_COUNT (135)
    0x81, 0x06,                    //   INPUT (Data,Var,Rel)
  //--------------------------------------------------------------//  
		0x85, 0x0F,                    //   REPORT_ID ()
    0x09, 0x01,                    //   USAGE (Vendor Usage 1)
    0x19, 0x00,                    //   USAGE_MINIMUM (Undefined)
    0x29, 0x00,                    //   USAGE_MAXIMUM (Undefined)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x23,                    //   REPORT_COUNT (35)
    0x81, 0x06,                    //   INPUT (Data,Var,Rel)
  //--------------------------------------------------------------//
	//-------------------- Output (from HOST) ----------------------//
	//--------------------------------------------------------------//
    0x85, 0x01,                    //   REPORT_ID (1)
    0x09, 0x01,                    //   USAGE (Vendor Usage 1)
    0x19, 0x00,                    //   USAGE_MINIMUM (Undefined)
    0x29, 0x00,                    //   USAGE_MAXIMUM (Undefined)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x80,                    //   REPORT_COUNT (128)
    0x91, 0x06,                    //   OUTPUT (Data,Var,Rel)
	//--------------------------------------------------------------//	
    0x85, 0x0A,                    //   REPORT_ID (10)
    0x09, 0x01,                    //   USAGE (Vendor Usage 1)
    0x19, 0x00,                    //   USAGE_MINIMUM (Undefined)
    0x29, 0x00,                    //   USAGE_MAXIMUM (Undefined)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x96, 0x02, 0x02,              //   REPORT_COUNT (514 = 1RI + 512PAGE + 1CRC)
    0x91, 0x06,                    //   OUTPUT (Data,Var,Rel)
	//--------------------------------------------------------------//	
    0x85, 0xCA,                    //   REPORT_ID (202)
    0x09, 0x01,                    //   USAGE (Vendor Usage 1)
    0x19, 0x00,                    //   USAGE_MINIMUM (Undefined)
    0x29, 0x00,                    //   USAGE_MAXIMUM (Undefined)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x80,                    //   REPORT_COUNT (128)
    0x91, 0x06,                    //   OUTPUT (Data,Var,Rel)
	//--------------------------------------------------------------//	
    0xc0                           // END_COLLECTION
  /* USER CODE END 0 */
  //0xC0    /*     END_COLLECTION	             */
};

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Exported_Variables USBD_CUSTOM_HID_Exported_Variables
  * @brief Public variables.
  * @{
  */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */
/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_FunctionPrototypes USBD_CUSTOM_HID_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CUSTOM_HID_Init_FS(void);
static int8_t CUSTOM_HID_DeInit_FS(void);
static int8_t CUSTOM_HID_OutEvent_FS(uint8_t event_idx, uint8_t state);

/**
  * @}
  */

USBD_CUSTOM_HID_ItfTypeDef USBD_CustomHID_fops_FS =
{
  CUSTOM_HID_ReportDesc_FS,
  CUSTOM_HID_Init_FS,
  CUSTOM_HID_DeInit_FS,
  CUSTOM_HID_OutEvent_FS
};

/** @defgroup USBD_CUSTOM_HID_Private_Functions USBD_CUSTOM_HID_Private_Functions
  * @brief Private functions.
  * @{
  */

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the CUSTOM HID media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_Init_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  DeInitializes the CUSTOM HID media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_DeInit_FS(void)
{
  /* USER CODE BEGIN 5 */
  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Manage the CUSTOM HID class events
  * @param  event_idx: Event index
  * @param  state: Event state
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_OutEvent_FS(uint8_t event_idx, uint8_t state)
{
  /* USER CODE BEGIN 6 */
	USBD_CUSTOM_HID_HandleTypeDef     *hhid = (USBD_CUSTOM_HID_HandleTypeDef*)hUsbDeviceFS.pClassData;
	//-------------------------------------------------------------//
	//------------------- Get data from USB HOST ------------------//
	//-------------------------------------------------------------//
	for (uint8_t i = 0; i < CUSTOM_HID_EPOUT_SIZE; i++)
		{
			FHDBuff[i] = hhid->Report_buf[i];
		}
	//-------------------------------------------------------------//
	//---- Switch/case state machine for hendling the requests ----//
	//---- from HOST.                                          ----//	
	//-------------------------------------------------------------//
	switch(FH_RepoertID)
	{
		//-------------------------------------------------------------//
		//-------- Get begin Update -----------------------------------//
		//-------------------------------------------------------------//
		case RepoertID_FH_BEGIN_UPDATE:
				if(FHDBuff[1] == 0xEF){
//					HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
//					HAL_FLASH_Unlock();
//					HAL_Delay(500);
//					FLASH_EraseCountPages(); // delete old program
//					HAL_Delay(500);
//					HAL_FLASH_Lock();
					ClearFlashFlag = true;
					//UpdateFirmwareRequestFlag = false;
				}
				else{
					UpdateFirmwareRequestFlag = true;
				}
		break;
		//-------------------------------------------------------------//
		//-------- Get new program page from HOST ---------------------//
		//-------------------------------------------------------------//
		case RepoertID_FH_NEW_PROGRAM_SUBPAGE:
			for(int i = 1; i < 33; i++){
				subPageData[i-1] = FHDBuff[i];
			}
			newSubPageFlag = true;
		break;
		//-------------------------------------------------------------//
		//-------- Get reset MCU request from HOST --------------------//
		//-------------------------------------------------------------//
		case RepoertID_FH_RESET_MCU:
			MCURestartFlag = true;
		break;
		
		default:
		break;
	}
	
  return (USBD_OK);
  /* USER CODE END 6 */
}

/* USER CODE BEGIN 7 */
/**
  * @brief  Send the report to the Host
  * @param  report: The report to be sent
  * @param  len: The report length
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
/*
static int8_t USBD_CUSTOM_HID_SendReport_FS(uint8_t *report, uint16_t len)
{
  return USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, report, len);
}
*/
/* USER CODE END 7 */

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

