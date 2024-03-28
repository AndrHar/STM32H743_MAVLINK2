/**
  ******************************************************************************
  * @file           : flash.c
  * @brief          : Flash memory configuration for STM32H743
  ******************************************************************************
  *
  * Only BANK1 is used. For using BANK2 create functions that calling registers
  * with index '2'.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/

#include "flash.h"
#include "stm32h7xx.h"

/* Defines -------------------------------------------------------------------*/
/* Variables -----------------------------------------------------------------*/


/* Functions -----------------------------------------------------------------*/

/* flashUnlock ---------------------------------------------------------------*/
enum FLASH_STATUS flashUnlock(void)
{

	if(READ_BIT(FLASH->CR1, FLASH_CR_LOCK) != 0U)
	{
	    /* Authorize the FLASH Bank1 Registers access */
	    WRITE_REG(FLASH->KEYR1, 0x45670123);
	    WRITE_REG(FLASH->KEYR1, 0xCDEF89AB);

	    /* Verify Flash Bank1 is unlocked */
	    if (READ_BIT(FLASH->CR1, FLASH_CR_LOCK) != 0U){
	    	return FLASH_LOCK_ERROR;
	    }
	}

	return FLASH_RDY;
}
/* End flashUnlock -----------------------------------------------------------*/



/* flashLock -----------------------------------------------------------------*/
enum FLASH_STATUS flashLock(void)
{

	FLASH->CR1 |= FLASH_CR_LOCK;

	/* Verify Flash Bank1 is locked */
	if (READ_BIT(FLASH->CR1, FLASH_CR_LOCK) == 0U)
	{
		return FLASH_LOCK_ERROR;
	}

	return FLASH_RDY;
}
/* End flashLock -------------------------------------------------------------*/


/* flash_WaitForLastOperation ------------------------------------------------*/
enum FLASH_STATUS flash_WaitForLastOperation(void)
{
	enum FLASH_STATUS result;
	uint32_t timeout;
	uint32_t status;

    result = FLASH_PGM_ERROR;
    timeout = 0;
    status = FLASH->SR1;

    /* Wait for the FLASH operation to complete by polling on QW flag to be reset.
       Even if the FLASH operation fails, the QW flag will be reset and an error
       flag will be set */

    while((status & (FLASH_SR_BSY | FLASH_SR_WBNE | FLASH_SR_QW )) && (timeout < TIMEOUT)){
        timeout++;
        //IWDG1->KR = IWDG_RELOAD; // kick watchdog
        status = FLASH->SR1;
    }

    if (timeout < TIMEOUT){

    	if ( (status & FLASH_FLAG_ALL_ERRORS_BANK1) == 0){

            result = FLASH_RDY;
        }
        else if(status & FLASH_SR_WRPERR){
            result = FLASH_WRP_ERROR;
        }
        else if(status & (FLASH_SR_PGSERR)){
            result = FLASH_PGM_ERROR;
        }
    }


    /* Check FLASH End of Operation flag  */
    if ( (FLASH->SR1) & FLASH_FLAG_EOP_BANK1)
    {
    	/* Clear FLASH End of Operation pending bit */
    	FLASH->CCR1 |= FLASH_CCR_CLR_EOP;;
    }

    return(result);
}
/* End flash_WaitForLastOperation --------------------------------------------*/



/* flash_EraseSector ---------------------------------------------------------*/
enum FLASH_STATUS   flash_EraseSector(uint32_t sectorNumb)
{
	enum FLASH_STATUS status;
	if (sectorNumb > 7){return FLASH_PGM_ERROR;}

	status = flash_WaitForLastOperation();

	if(status == FLASH_RDY)
	{
		flashUnlock();
		FLASH->CR1 &= (~(FLASH_CR_PSIZE | FLASH_CR_SNB));	// clear
		FLASH->CR1 |= (sectorNumb << FLASH_CR_SNB_Pos);		// sector erase selection number
		FLASH->CR1 |= FLASH_CR_SER | FLASH_CR_PSIZE_1;    	// chose 'sector erase request' and  program size ( byte, half-word, word, double word)
		FLASH->CR1 |= FLASH_CR_START; 						// erase start control bit

		status = flash_WaitForLastOperation();
		FLASH->CR1 &= (~(FLASH_CR_SER | FLASH_CR_SNB));		// clear

		flashLock();
	}

	return(status);

}
/* End flash_EraseSector -----------------------------------------------------*/


/* flash_EraseAll ------------------------------------------------------------*/
enum FLASH_STATUS flash_EraseAll(void)
{
	enum FLASH_STATUS status;

    status = flash_WaitForLastOperation();
    if(status == FLASH_RDY){
    	flashUnlock();
        FLASH->CR1 = FLASH_CR_BER | FLASH_CR_PSIZE_1;
        FLASH->CR1 |= FLASH_CR_START;
        status = flash_WaitForLastOperation();
        FLASH->CR1 &= ~FLASH_CR_BER;
        flashLock();
    }

    return(status);
}
/* End flash_EraseAll --------------------------------------------------------*/




/* flashRead -----------------------------------------------------------------*/
uint32_t flashRead(uint32_t address)
{
  return (*(__IO uint32_t*)address);
}
/* End flashRead -------------------------------------------------------------*/




/* flashWrite ----------------------------------------------------------------*/
enum FLASH_STATUS flashWrite( uint32_t FlashAddress, uint32_t DataAddress, int DataSize)
{
	enum FLASH_STATUS status;

	/* The write operations are executed in the non-volatile memory only by 256-bit data Flash word.
	 * The application can decide to write as little as 8 bits to a  256 Flash word. In this case, a
	 * force-write mechanism is used (see FW1/2 bit of FLASH_CR1/2 register)
	 *
	 * To make possible write data less than 256-bit Flash word, I choose 'uint8_t' for 'dest_addr' and 'src_addr'.
	 * If change 'uint8_t' don't forget recalculate 'cyclesPerFlashWord'
	 * */

	__IO uint8_t *dest_addr = (__IO uint8_t *)FlashAddress;
	__IO uint8_t *src_addr = (__IO uint8_t*)DataAddress;
	uint16_t cyclesPerFlashWord = NB_8BIT_IN_FLASHWORD; //(256-bit flashWord)/'uint8_t') -> 256/8=32;
	uint32_t writtenBytes = 0;			// counter for bytes are already written to Flash
	uint32_t numb_flashword;			// amount of Flash words in input data

	/*calculate number of full flash words in data array*/
	numb_flashword = DataSize*EIGHT_BITS/FLASHWORD_256;    // integer result because both operands are integers

	flashUnlock();

  	/* Wait for last operation to be completed */
  	status = flash_WaitForLastOperation();

  	if(status == FLASH_RDY)
  	{
  		/* Enable the PG to the program operation */
  		SET_BIT(FLASH->CR1, FLASH_CR_PG);

  		do
  		{
  			__ISB();
  			__DSB();

  			/* Program the flash word */
  			cyclesPerFlashWord = NB_8BIT_IN_FLASHWORD;
  			do
  			{
  			   *dest_addr = *src_addr;
  			    dest_addr++;
  			    src_addr++;
  			    cyclesPerFlashWord--;
  			    writtenBytes++;
  			} while ( (cyclesPerFlashWord != 0U) && (writtenBytes < DataSize));

  			if (numb_flashword > 0){numb_flashword--;}

  			__ISB();
  			__DSB();

  			if ( (numb_flashword == 0) && (writtenBytes == DataSize) )
  			{
  				/* FW forces a write operation even if the write buffer is not full */
  			  	SET_BIT(FLASH->CR1, FLASH_CR_FW);
  			}

  			/* Wait for last operation to be completed */
  			status = flash_WaitForLastOperation();

  		} while (writtenBytes < DataSize);

  		/* If the program operation is completed, disable the PG */
  		CLEAR_BIT(FLASH->CR1, FLASH_CR_PG);


  	} // if(status == FLASH_RDY)

  	flashLock();

  return status;
}

/* End flashWrite ------------------------------------------------------------*/

		
//end
//end
//end


