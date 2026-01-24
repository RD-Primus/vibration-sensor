/*
 * FLASH_STM32.h
 *
 *  Created on: Oct 8, 2024
 *      Author: Nilniz
 */

#ifndef INC_FLASH_STM32_H_
#define INC_FLASH_STM32_H_

#if defined(STM32WLxx_HAL_FLASH_H)
#define sFLASH_PAGE_SIZE 				FLASH_PAGE_SIZE
#define sFLASH_START_ADDR 			FLASH_BASE
#define sFLASH_END_ADDR 				FLASH_END_ADDR
#define sFLASH_TYPEERASE_PAGES 	FLASH_TYPEERASE_PAGES
#define sFLASH_TYPEPROGRAM 			FLASH_TYPEPROGRAM_FAST

#define sFLASH_PAGE_NB 					(sFLASH_END_ADDR - sFLASH_START_ADDR) / sFLASH_PAGE_SIZE
#elif defined(__STM32F0xx_HAL_FLASH_H)
#define sFLASH_PAGE_SIZE 				FLASH_PAGE_SIZE
#define sFLASH_START_ADDR 			FLASH_BASE
#define sFLASH_END_ADDR 				FLASH_BANK1_END
#define sFLASH_TYPEERASE_PAGES 	FLASH_TYPEERASE_PAGES
#define sFLASH_TYPEPROGRAM 			FLASH_TYPEPROGRAM_WORD

#define sFLASH_PAGE_NB 					(sFLASH_END_ADDR - sFLASH_START_ADDR) / sFLASH_PAGE_SIZE
#endif


extern void STM32_Flash_Write(uint16_t size, uint32_t *dptr, uint8_t Page);
extern void STM32_Flash_Read(uint16_t size, uint32_t *dptr,  uint8_t Page);

#endif /* INC_FLASH_STM32_H_ */
