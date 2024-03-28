/*----------------------------------------------------------------------------*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef FLASH_H_IFND
#define FLASH_H_IFND


/* Includes ------------------------------------------------------------------*/

#include "stm32h7xx.h"


/* Defines -------------------------------------------------------------------*/

enum FLASH_STATUS{FLASH_RDY, FLASH_WRP_ERROR, FLASH_PGM_ERROR, FLASH_LATENCY_ERROR, FLASH_LOCK_ERROR};

enum FLASH_SECTOR{Sector0, Sector1, Sector2, Sector3, Sector4, Sector5, Sector6, Sector7};

#define FLASH_SECTOR_BOOTLOADER		Sector0
#define FLASH_SECTOR_CONFIG_DATA	Sector1
#define FLASH_SECTOR_USER_PROG		Sector2


#define TIMEOUT         				(0x0FFFFFFF)    // Timeout value

/* Base address of the Flash sectors Bank 1 */
#define ADDR_FLASH_SECTOR_0_BANK1    	((uint32_t)0x08000000) /* Sector 0, 128 Kbytes */
#define ADDR_FLASH_SECTOR_1_BANK1     	((uint32_t)0x08020000) /* Sector 1, 128 Kbytes */
#define ADDR_FLASH_SECTOR_2_BANK1     	((uint32_t)0x08040000) /* Sector 2, 128 Kbytes */
#define ADDR_FLASH_SECTOR_3_BANK1     	((uint32_t)0x08060000) /* Sector 3, 128 Kbytes */
#define ADDR_FLASH_SECTOR_4_BANK1     	((uint32_t)0x08080000) /* Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5_BANK1     	((uint32_t)0x080A0000) /* Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6_BANK1     	((uint32_t)0x080C0000) /* Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7_BANK1     	((uint32_t)0x080E0000) /* Sector 7, 128 Kbytes */


#define FLASH_FLAG_BSY_BANK1            FLASH_SR_BSY           /*!< FLASH Bank 1 Busy flag */
#define FLASH_FLAG_WBNE_BANK1           FLASH_SR_WBNE          /*!< Write Buffer Not Empty on Bank 1 flag */
#define FLASH_FLAG_QW_BANK1             FLASH_SR_QW            /*!< Wait Queue on Bank 1 flag */
#define FLASH_FLAG_CRC_BUSY_BANK1       FLASH_SR_CRC_BUSY      /*!< CRC Busy on Bank 1 flag */
#define FLASH_FLAG_EOP_BANK1            FLASH_SR_EOP           /*!< End Of Program on Bank 1 flag */
#define FLASH_FLAG_WRPERR_BANK1         FLASH_SR_WRPERR        /*!< Write Protection Error on Bank 1 flag */
#define FLASH_FLAG_PGSERR_BANK1         FLASH_SR_PGSERR        /*!< Program Sequence Error on Bank 1 flag */
#define FLASH_FLAG_STRBERR_BANK1        FLASH_SR_STRBERR       /*!< Strobe Error on Bank 1 flag */
#define FLASH_FLAG_INCERR_BANK1         FLASH_SR_INCERR        /*!< Inconsistency Error on Bank 1 flag */
#define FLASH_FLAG_OPERR_BANK1          FLASH_SR_OPERR         /*!< Operation Error on Bank 1 flag */
#define FLASH_FLAG_RDPERR_BANK1         FLASH_SR_RDPERR        /*!< Read Protection Error on Bank 1 flag */
#define FLASH_FLAG_RDSERR_BANK1         FLASH_SR_RDSERR        /*!< Read Secured Error on Bank 1 flag */
#define FLASH_FLAG_SNECCERR_BANK1       FLASH_SR_SNECCERR      /*!< Single ECC Error Correction on Bank 1 flag */
#define FLASH_FLAG_DBECCERR_BANK1       FLASH_SR_DBECCERR      /*!< Double Detection ECC Error on Bank 1 flag */
#define FLASH_FLAG_CRCEND_BANK1         FLASH_SR_CRCEND        /*!< CRC End of Calculation on Bank 1 flag */
//#define FLASH_FLAG_CRCRDERR_BANK1     FLASH_SR_CRCRDERR      /*!< CRC Read error on Bank 1 flag */

#define FLASH_FLAG_ALL_ERRORS_BANK1     (FLASH_FLAG_WRPERR_BANK1   | FLASH_FLAG_PGSERR_BANK1   | \
                                         FLASH_FLAG_STRBERR_BANK1  | FLASH_FLAG_INCERR_BANK1   | \
                                         FLASH_FLAG_OPERR_BANK1    | FLASH_FLAG_RDPERR_BANK1   | \
                                         FLASH_FLAG_RDSERR_BANK1   | FLASH_FLAG_SNECCERR_BANK1 | \
                                         FLASH_FLAG_DBECCERR_BANK1 ) /*!< All Bank 1 error flags */


#define NB_8BIT_IN_FLASHWORD    		(32U)                   // FLASHWORD = 256 bits
#define FLASHWORD_256					((uint16_t)256)			// STM32H743 Flash memory operates with 256 bits flash word (see ref.manual)


#define EIGHT_BITS						(8U)                    // bits in byte



/* Functions -----------------------------------------------------------------*/

enum FLASH_STATUS flash_WaitForLastOperation(void); //__attribute__ ((section(".fast")));


enum FLASH_STATUS flashUnlock(void);
enum FLASH_STATUS flashLock(void);

uint32_t flashRead(uint32_t address);

enum FLASH_STATUS flashWrite( uint32_t FlashAddress, uint32_t DataAddress, int DataSize);
enum FLASH_STATUS flash_EraseSector(uint32_t sectorNumb);
enum FLASH_STATUS flash_EraseAll(void);

#endif /* TIMERS_H_IFND */

