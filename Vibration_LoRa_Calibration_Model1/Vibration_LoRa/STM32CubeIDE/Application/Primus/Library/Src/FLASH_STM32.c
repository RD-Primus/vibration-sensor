/*
 * FLASH_STM32.c
 *
 *  Created on: Oct 8, 2024
 *      Author: Nilniz
 */
#include "main.h"

#if defined(STM32WLxx_HAL_FLASH_H)
#include "stm32wlxx_hal_flash.h"
#elif defined(__STM32F0xx_HAL_FLASH_H)
#include "stm32f0xx_hal_flash.h"
#endif

void STM32_Flash_Write(uint16_t size, uint32_t *dptr, uint8_t Page) {

	uint8_t use_page = (size + sFLASH_PAGE_SIZE - 1) / sFLASH_PAGE_SIZE;
	if(Page + use_page > sFLASH_PAGE_NB + 1)
		return;

	uint16_t WRSize;
	uint32_t address ;
	uint32_t strAddr = (sFLASH_PAGE_SIZE * Page) + (sFLASH_START_ADDR);
	uint32_t endAddr = strAddr + (use_page * sFLASH_PAGE_SIZE) - 1 ;// strAddr + sFLASH_PAGE_SIZE - 1;

	size = (uint16_t) (size + 2) / 4; //Round up


	HAL_FLASH_Unlock();

	uint32_t error_sector_num = 0;

	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase = sFLASH_TYPEERASE_PAGES;

#if defined(STM32WLxx_HAL_FLASH_H)
	EraseInitStruct.Page = strAddr;
#elif defined(__STM32F0xx_HAL_FLASH_H)
	EraseInitStruct.PageAddress = strAddr;
#endif

	EraseInitStruct.NbPages = ((endAddr - strAddr) + 1) / sFLASH_PAGE_SIZE;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &error_sector_num) != HAL_OK)
	{
//		uint32_t error_code = HAL_FLASH_GetError();
		__ASM volatile("BKPT #01");
	}

	address = strAddr;
	WRSize = 0;
	while (WRSize < size) {
		if (HAL_FLASH_Program(sFLASH_TYPEPROGRAM, address, *dptr) == HAL_OK) {
			address = address + 4;
			dptr++;
			WRSize++;
		}
	}

	HAL_FLASH_Lock();

}

/*============================================================================*/

void STM32_Flash_Read(uint16_t size, uint32_t *dptr, uint8_t Page)
{
	uint8_t use_page = (size + sFLASH_PAGE_SIZE - 1) / sFLASH_PAGE_SIZE;
	if(Page + use_page > sFLASH_PAGE_NB + 1)
		return;
	uint16_t WRSize;
	uint32_t address;
	uint32_t strAddr = (sFLASH_PAGE_SIZE * Page) + (sFLASH_START_ADDR);


	size = (size + 2) / 4; //Round up

	address = strAddr;
	WRSize = 0;

	while (WRSize < size) {
		*dptr = *(uint32_t*) address;
		dptr++;
		WRSize++;
		address = address + 4;
	}
}

