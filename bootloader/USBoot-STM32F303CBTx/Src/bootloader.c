#include "bootloader.h"
#include "usbd_customhid.h"

uint32_t programBytesToRead;
uint32_t programBytesCounter;
uint32_t currentAddress;

uint8_t readBuffer[512];
uint8_t readBytes;

uint32_t currentAddress = MAIN_PROGRAM_START_ADDRESS;

extern	USBD_HandleTypeDef hUsbDeviceFS;

extern uint8_t THDBuff[CUSTOM_HID_EPOUT_SIZE];	// TO host data buffer 		(defined in main.c)
#define TH_RepoertID THDBuff[0]									// Repord_ID to host			(defined in main.c)
extern uint8_t FHDBuff[CUSTOM_HID_EPOUT_SIZE];	// FROM host data buffer	(defined in main.c)
#define FH_RepoertID FHDBuff[0]									// Repord_ID from host		(defined in main.c)

extern void    FLASH_PageErase(uint32_t PageAddress);
//static FLASH_EraseInitTypeDef EraseInitStruct;

/***************************************************************************************/
// Function                DetectUSB()
// Description             Detect's USB by 5V
// Parameters              None
// RetVal                  None
/***************************************************************************************/
bool DetectUSB(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	bool USBdetected = false;
	if(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_SET){
		USBdetected = true;
	}
	return USBdetected;
} // End of DetectUSB()

/***************************************************************************************/
// Function                Send_ReadyMessage_To_USBHost()
// Description             Send ready message to USBoot app
// Parameters              None
// RetVal                  None
/***************************************************************************************/
void Send_ReadyMessage_To_USBHost(){

	TH_RepoertID 	= RI_bootloader; // report id status from MCU
	THDBuff[1] 		= ReadyToUpdate; // device ready to update firmware flag
			
	USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, THDBuff, 2);
	HAL_Delay(50);

}//End of Send_ReadyMessage_To_USBHost()

/***************************************************************************************/
// Function                Send_RequestNewSubPage_To_USBHost()
// Description             Send New SubPage Request to USBoot app
// Parameters              None
// RetVal                  None
/***************************************************************************************/
void Send_RequestNewSubPage_To_USBHost(){

	TH_RepoertID 	= 0xB0; 			// report id ready to update firmware
	THDBuff[1] 		= RequestNewSubPage; 	// device ready to GetNewSubPage
			
	USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, THDBuff, 2);
	HAL_Delay(30);

}//End of Send_RequestNewSubPage_To_USBHost()

/***************************************************************************************/
// Function                Send_FlashErasedStatus_To_USBHost()
// Description             Send Flash Erased Status to USBoot app
// Parameters              bool FlashEraseResult
// RetVal                  None
/***************************************************************************************/
void Send_FlashErasedStatus_To_USBHost(bool FlashEraseResult){

	TH_RepoertID 	= RI_bootloader; 											// report id status from MCU
	if(FlashEraseResult == true) 	THDBuff[1]	= 0xFE; 	// flash erased status OK
	else 													THDBuff[1]	= 0xEE; 	// flash erased status ERROR
 		
	USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, THDBuff, 2);
	HAL_Delay(30);

}//End of Send_FlashErasedStatus_To_USBHost(bool FlashEraseResult)


/***************************************************************************************/
// Function                SetKey()
// Description             Sets bootloader key
// Parameters              None
// RetVal                  None
/***************************************************************************************/
void SetKey()
{
    HAL_FLASH_Unlock();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, BOOTLOADER_KEY_START_ADDRESS, BOOTLOADER_KEY_VALUE); 
		//FLASH_ProgramWord(BOOTLOADER_KEY_START_ADDRESS, BOOTLOADER_KEY_VALUE);
    HAL_FLASH_Lock();
} // End of SetKey()

/***************************************************************************************/
// Function                ResetKey()
// Description             Resets bootloader key
// Parameters              None
// RetVal                  None
/***************************************************************************************/
void ResetKey()
{
    HAL_FLASH_Unlock();
    FLASH_PageErase(BOOTLOADER_KEY_START_ADDRESS);
    HAL_FLASH_Lock();
} // End of ResetKey()

/***************************************************************************************/
// Function                ReadKey()
// Description             Reads bootloader key value
// Parameters              None
// RetVal                  None
/***************************************************************************************/
uint32_t ReadKey()
{
    return (*(__IO uint32_t*) BOOTLOADER_KEY_START_ADDRESS);
} // End of ReadKey()

/***************************************************************************************/
// Function                FLASH_EraseCountPages()
// Description             Erase (NUM_OF_PAGES - MAIN_PROGRAM_PAGE_NUMBER) Flash pages
// Parameters              None
// RetVal                  HAL_FLASH_Unlock() before using
/***************************************************************************************/
//void FLASH_EraseCountPages()
//{	
//	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;//FLASH_TYPEERASE_MASSERASE;//
//	EraseInitStruct.PageAddress = ADDR_FLASH_PAGE_20;//0x0800A000;//MAIN_PROGRAM_START_ADDRESS;
//	EraseInitStruct.NbPages = 108;//NUM_OF_PAGES - MAIN_PROGRAM_PAGE_NUMBER;
//	uint32_t eraseResult = 0;
//	
//	//HAL_FLASHEx_Erase(&flashType, &eraseResult);
//	if(HAL_FLASHEx_Erase(&EraseInitStruct, &eraseResult) != HAL_OK) {
//    //Erase error!
//		while (1)
//		{
//			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//			HAL_Delay(150);
//		}
//	}
//	else{
//		HAL_Delay(500);
//		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
//	}
////	for(int i = MAIN_PROGRAM_START_ADDRESS; i <  NUM_OF_PAGES - MAIN_PROGRAM_PAGE_NUMBER; i++){
////		FLASH_PageErase(i);
////	}

//} //End of FLASH_EraseCountPages()

/***************************************************************************************/
// Function                FLASH_WriteNewFirmware()
// Description             Write new firmware to Flash from MAIN_PROGRAM_START_ADDRESS
// Parameters              None
// RetVal                  HAL_FLASH_Unlock() and FLASH_EraseCountPages() before using
/***************************************************************************************/
void FLASH_WriteNewFirmware(uint32_t newProgramBytes,  uint8_t * newProgram )
{
	
	programBytesToRead = newProgramBytes;
	programBytesCounter = 0;
	currentAddress = MAIN_PROGRAM_START_ADDRESS;
	
	while ((programBytesToRead -  programBytesCounter) >= 512)
	{
		//f_read(&program, readBuffer, 512, &readBytes); //get data from newProgramDataArray
		programBytesCounter += 512;
		for (uint32_t i = 0; i < 512; i += 4)
		{
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, currentAddress, *(uint32_t*)&readBuffer[i]);
			//FLASH_ProgramWord(currentAddress, *(uint32_t*)&readBuffer[i]);
			currentAddress += 4;
		}
	}
	//HAL_FLASH_Lock();
}

void FLASH_WriteNewFirmwarePage(uint32_t mainProgramPageAddress, uint8_t * newProgramPage){
	
	//HAL_FLASH_Unlock();
	for (uint32_t i = 0; i < 512; i += 4)
		{
			if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, mainProgramPageAddress, *(uint32_t*)&newProgramPage[i]) == HAL_OK)
      {
        mainProgramPageAddress += 4;
      }
      else
      {
        /* Error occurred while writing data in Flash memory. 
           User can add here some code to deal with this error */
      }	
			//FLASH_ProgramWord(currentAddress, *(uint32_t*)&readBuffer[i]);
		}
	//HAL_FLASH_Lock();
}

void FLASH_WriteNewFirmwarePageData(uint8_t * newProgramPage){
	
	//HAL_FLASH_Unlock();
	for (uint32_t i = 0; i < 512; i += 4)
		{	
			//FLASH_ProgramWord(currentAddress, *(uint32_t*)&readBuffer[i]);
			if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, currentAddress, *(uint32_t*)&newProgramPage[i]) == HAL_OK)
      {
        currentAddress += 4;
      }
      else
      {
        /* Error occurred while writing data in Flash memory. 
           User can add here some code to deal with this error */
      }	
		}
	//HAL_FLASH_Lock();
}


/*ASCII to hex function*/
void Ascii_To_Hex( uint8_t* buff, uint8_t count)
{
	uint8_t i;
	
	for(i=0; i<count;i++)
	{
		if(buff[i] <= '9' && buff[i] >= '0' )
		{
			buff[i] -= 0x30;
		}
		else
		{
			buff[i] = buff[i] - 0x41 + 10;
		}	
	}	
}

void Boot_GoToMainProgram(void)
{
	//if(ReadKey() == BOOTLOADER_KEY_VALUE){
		uint32_t app_jump_address;
				
		typedef void(*pFunction)(void); // define new user defined type
		pFunction Jump_To_Application;	// create veriable of this type

		//__disable_irq();	// disable interrupts
		//NVIC_SetVectorTable(NVIC_VectTab_FLASH, MAIN_PROGRAM_START_ADDRESS);

	
		app_jump_address = *( uint32_t*) (MAIN_PROGRAM_START_ADDRESS + 4);	// get the jump address from Reset vector
		Jump_To_Application = (pFunction)app_jump_address;									// cast address to user defined type
				
		__set_MSP(*(__IO uint32_t*) MAIN_PROGRAM_START_ADDRESS);						// set up the app's SP
		SCB->VTOR = MAIN_PROGRAM_START_ADDRESS;

		//HAL_DeInit();
				
		Jump_To_Application();																			// run the app	
	//}
}

