/**
  ******************************************************************************
  * @file           : can.c
  * @brief          : CAN1 configuration for STM32H743
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* Variables -----------------------------------------------------------------*/
static uint32_t StdFilterSA = 0;
static uint32_t RxBufferSA = 0;
static uint32_t TxBufferSA = 0;


/* Functions -----------------------------------------------------------------*/

/* ---------------------------- InitCAN1 -------------------------------------*/
uint16_t InitCAN1()
{		

	uint32_t ExtStdFilterSA = 0;
	uint32_t RxFIFO0SA = 0;
	uint32_t RxFIFO1SA = 0;
	uint32_t TxEventFIFOSA = 0;
	uint32_t TxFIFOQSA = 0;
	uint32_t EndAddress = 0;
	uint32_t RAMcounter = 0;

	uint32_t wait_count = 0;
	
	/* FDCAN kernel clock source selection. Enable FDCAN Clock output generated from System PLL */
	MODIFY_REG(RCC->D2CCIP1R, RCC_D2CCIP1R_FDCANSEL, RCC_D2CCIP1R_FDCANSEL_0);
	
	RCC->APB1HENR|= RCC_APB1HENR_FDCANEN; 										//enable clock for bus APB1 (FDCAN)
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN; 	 									//enable clock for bus AHB4 (GPIO A) FDCAN1 - GPIO PA11, PA12
		
	/* PA11--> FDCAN1_RX;   PA12--> FDCAN1_TX;  CAN is AF9 (1001), see p80 of datasheet;
	   AFR register see p523 of ref manual */
	GPIOA->AFR[1] &= ~ (GPIO_AFRH_AFRH3 | GPIO_AFRH_AFRH4);						// reset bits for alternate function
	GPIOA->AFR[1] |= (GPIO_AF9_FDCAN << GPIO_AFRH_AFRH3_Pos); 					// choose alternate function on PA11
	GPIOA->AFR[1] |= (GPIO_AF9_FDCAN << GPIO_AFRH_AFRH4_Pos);					// choose alternate function on PA12
	GPIOA->MODER &= ~(GPIO_MODER_MODER11 | GPIO_MODER_MODER12); 				// reset bits MODER PA11, PA12
	GPIOA->MODER |= (GPIO_MODER_MODER11_1 | GPIO_MODER_MODER12_1);  			// set pins in alternate function mode
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_12; 										// set PA12 (CANTX) as  push-pull
	GPIOA->OSPEEDR &= (GPIO_OSPEEDER_OSPEEDR11 | GPIO_OSPEEDER_OSPEEDR12); 		// reset bits OSPEED PA11, PA12
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR11_1 | GPIO_OSPEEDER_OSPEEDR12_1; 	// set high speed
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR12; 										// No pull-up, pull-down

	/* reset FDCAN*/
	RCC->APB1HRSTR |= RCC_APB1HRSTR_FDCANRST;
	RCC->APB1HRSTR &= ~RCC_APB1HRSTR_FDCANRST;
		
	FDCAN1->IE &= 0x0; 															//Disable all interrupts
		
	wait_count = 0;
	FDCAN1->CCCR &= ~FDCAN_CCCR_CSR; 											//Exit from Sleep mode
	while (((FDCAN1->CCCR & FDCAN_CCCR_CSA) == FDCAN_CCCR_CSA) && (wait_count < CAN_TIMEOUT)) //wait Sleep mode acknowledge
	{
		wait_count++;
		if (wait_count >= CAN_TIMEOUT){return CAN_STATUS_ERROR;}
	}
		
		
	/* Request initialization */
	FDCAN1->CCCR |= FDCAN_CCCR_INIT;
		
	/* Wait until the INIT bit into CCCR register is set */
	wait_count = 0;
	while (((FDCAN1->CCCR & FDCAN_CCCR_INIT) != FDCAN_CCCR_INIT) && (wait_count < CAN_TIMEOUT))
	{
		wait_count++;
		if (wait_count >= CAN_TIMEOUT){return CAN_STATUS_ERROR;}
	}
		

	/* Enable configuration change */
	FDCAN1->CCCR |= FDCAN_CCCR_CCE;
	FDCAN1->CCCR &= (FDCAN_CCCR_INIT | FDCAN_CCCR_CCE); 						// clear all bits (except INIT and CCE)
	//FDCAN1->CCCR |= FDCAN_CCCR_DAR;											//no automatic retransmission
	FDCAN1->CCCR |= FDCAN_CCCR_PXHD; 											//Set the Protocol Exception Handling
				
	/* Set the nominal bit timing register */
	FDCAN1->NBTP = ((((uint32_t)CAN1_NSJW - 1) << 25)  | \
                    (((uint32_t)CAN1_NTSEG1 - 1) << 8) | \
                     ((uint32_t)CAN1_NTSEG2 - 1)       | \
                    (((uint32_t)CAN1_NBRP - 1) << 16));
		
	/* Configure Tx element size */
	FDCAN1->TXESC &= ~FDCAN_TXESC_TBDS;  /* 8 byte data field */

	/* Configure Rx element size */
	FDCAN1->RXESC &= ~FDCAN_RXESC_RBDS;
		
	/* Standard filter list start address */
	StdFilterSA = 0;
	FDCAN1->SIDFC = 0;	//reset register
	/* Standard filter elements number */
	FDCAN1->SIDFC |= (CAN_RX_STD_FILT_NBR << FDCAN_SIDFC_LSS_Pos);
		
	/* Extended filter list start address */
	ExtStdFilterSA = StdFilterSA + CAN_RX_STD_FILT_NBR;
	FDCAN1->XIDFC = 0; //reset register
	FDCAN1->XIDFC |= (ExtStdFilterSA << 2);
	/* Extended filter elements number */
	FDCAN1->XIDFC |= (CAN_RX_EXT_FILT_NBR << FDCAN_XIDFC_LSE_Pos);
		
	/* Rx FIFO 0 start address */
	RxFIFO0SA = ExtStdFilterSA + (CAN_RX_EXT_FILT_NBR * 2);
	FDCAN1->RXF0C = 0; //reset register
	FDCAN1->RXF0C |= (RxFIFO0SA << FDCAN_RXF0C_F0SA_Pos);
	/* Rx FIFO 0 elements number */
	FDCAN1->RXF0C |= (CAN_RX_FIFO0_ELMTS_NBR << FDCAN_RXF0C_F0S_Pos);
		
	/* Rx FIFO 1 start address */
	RxFIFO1SA = RxFIFO0SA + (CAN_RX_FIFO0_ELMTS_NBR * CAN_RX_FIFO0_ELMTS_SIZE);
	FDCAN1->RXF1C = 0; //reset register
	FDCAN1->RXF1C |= (RxFIFO1SA << FDCAN_RXF1C_F1SA_Pos);
	/* Rx FIFO 1 elements number */ 																												//reset bits
	FDCAN1->RXF1C |= (CAN_RX_FIFO1_ELMTS_NBR << FDCAN_RXF1C_F1S_Pos);
		
	/* Rx buffer list start address */
	RxBufferSA = RxFIFO1SA + (CAN_RX_FIFO1_ELMTS_NBR * CAN_RX_FIFO1_ELMTS_SIZE);
	FDCAN1->RXBC &= ~FDCAN_RXBC_RBSA;
	FDCAN1->RXBC |= (RxBufferSA << FDCAN_RXBC_RBSA_Pos);
		
	/* Tx event FIFO start address */
	TxEventFIFOSA = RxBufferSA + (CAN_RX_BUFFERS_NBR * CAN_RX_BUFFERS_SIZE);
	FDCAN1->TXEFC &= ~FDCAN_TXEFC_EFSA;
	FDCAN1->TXEFC |= (TxEventFIFOSA << FDCAN_TXEFC_EFSA_Pos);
		
	/* Tx event FIFO elements number */
	FDCAN1->TXEFC &= ~FDCAN_TXEFC_EFS; 																											//reset bits
	FDCAN1->TXEFC |= (CAN_TX_EVENTS_NBR << FDCAN_TXEFC_EFS_Pos);
		
	/* Tx buffer list start address */
	TxBufferSA = TxEventFIFOSA + (CAN_TX_EVENTS_NBR * 2);
	FDCAN1->TXBC = 0; //reset register
	FDCAN1->TXBC |= (TxBufferSA << FDCAN_TXBC_TBSA_Pos);
	/* Dedicated Tx buffers number */
	FDCAN1->TXBC |= (CAN_TX_BUFFERS_NBR << FDCAN_TXBC_NDTB_Pos);
	/* Tx FIFO/queue elements number */
	FDCAN1->TXBC |= (CAN_TX_FIFO_QUEUE_ELMTS_NBR << FDCAN_TXBC_TFQS_Pos);

	/* Tx FIFO/queue start address */
	TxFIFOQSA = TxBufferSA + (CAN_TX_BUFFERS_NBR * CAN_TX_ELMTS_SIZE);


	StdFilterSA = SRAMCAN_BASE; //+ (hfdcan->Init.MessageRAMOffset * 4);
	ExtStdFilterSA = StdFilterSA + (CAN_RX_STD_FILT_NBR * 4);
	RxFIFO0SA = ExtStdFilterSA + (CAN_RX_EXT_FILT_NBR * 2 * 4);
	RxFIFO1SA = RxFIFO0SA + (CAN_RX_FIFO0_ELMTS_NBR * CAN_RX_FIFO0_ELMTS_SIZE * 4);
	RxBufferSA = RxFIFO1SA + (CAN_RX_FIFO1_ELMTS_NBR * CAN_RX_FIFO1_ELMTS_SIZE * 4);
	TxEventFIFOSA = RxBufferSA + (CAN_RX_BUFFERS_NBR * CAN_RX_BUFFERS_SIZE * 4);
	TxBufferSA = TxEventFIFOSA + (CAN_TX_EVENTS_NBR * 2 * 4);
	TxFIFOQSA = TxBufferSA + (CAN_TX_BUFFERS_NBR * CAN_TX_ELMTS_SIZE * 4);

	EndAddress = TxFIFOQSA + (CAN_TX_FIFO_QUEUE_ELMTS_NBR * CAN_TX_ELMTS_SIZE * 4);

	if(EndAddress > 0x4000B5FC) /* Last address of the Message RAM */
	{
		/* Update error code. Message RAM overflow */
		return CAN_STATUS_ERROR_RAM;
	}
	else
	{
		/* Flush the allocated Message RAM area */
		for(RAMcounter = StdFilterSA; RAMcounter < EndAddress; RAMcounter += 4)
		{
			*(__IO uint32_t *)(RAMcounter) = 0x00000000;
		}
	}
		
	/* Reject Remote Frames Extended, Remote Frames Standard, Non-matching Frames Extended, Non-matching Frames Standard */
	FDCAN1->GFC |= FDCAN_GFC_RRFE | FDCAN_GFC_RRFS | FDCAN_GFC_ANFE | FDCAN_GFC_ANFS; //
		
	/* Set configuration of Rx & Tx filters */
	//Config_RxFilters(idArray);
	//Config_TxFilters();
		
	/* Request leave initialization */
	FDCAN1->CCCR &= ~FDCAN_CCCR_INIT;
		
	/* Wait until the INIT bit into CCCR register is reset */
	wait_count = 0;
	while (((FDCAN1->CCCR & FDCAN_CCCR_INIT) == FDCAN_CCCR_INIT)&& (wait_count < CAN_TIMEOUT))
	{
		wait_count++;
		if (wait_count >= CAN_TIMEOUT){return CAN_STATUS_ERROR;}
	}

		
	return CAN_STATUS_OK;
}
/* -------------------------- End InitCAN1 -----------------------------------*/



	
/* ------------------------- ReceiveCanMsg -----------------------------------*/
void ReceiveCanMsg (uint32_t RxLocation, uint8_t *pRxData, uint16_t CanModule)
{
	FDCAN_RxHeaderTypeDef pRxHeader;
	uint32_t *RxAddress;
	uint8_t  *pData;
	uint32_t ByteCounter;
	// uint32_t GetIndex = 0;
	
	
	/* Calculate Rx buffer address */
	RxAddress = (uint32_t *)(RxBufferSA + (RxLocation * CAN_RX_BUFFERS_SIZE * 4));
	
	/* Retrieve IdType */
	pRxHeader.IdType = *RxAddress & FDCAN_ELEMENT_MASK_XTD;

	/* Retrieve Identifier */
	pRxHeader.Identifier = ((*RxAddress & FDCAN_ELEMENT_MASK_STDID) >> 18);

	/* Retrieve RxFrameType */
	pRxHeader.RxFrameType = (*RxAddress & FDCAN_ELEMENT_MASK_RTR);

	/* Retrieve ErrorStateIndicator */
	pRxHeader.ErrorStateIndicator = (*RxAddress++ & FDCAN_ELEMENT_MASK_ESI);

	/* Retrieve RxTimestamp */
	pRxHeader.RxTimestamp = (*RxAddress & FDCAN_ELEMENT_MASK_TS);

	/* Retrieve DataLength */
	pRxHeader.DataLength = (*RxAddress & FDCAN_ELEMENT_MASK_DLC);

	/* Retrieve BitRateSwitch */
	pRxHeader.BitRateSwitch = (*RxAddress & FDCAN_ELEMENT_MASK_BRS);

	/* Retrieve FDFormat */
	pRxHeader.FDFormat = (*RxAddress & FDCAN_ELEMENT_MASK_FDF);

	/* Retrieve FilterIndex */
	pRxHeader.FilterIndex = ((*RxAddress & FDCAN_ELEMENT_MASK_FIDX) >> 24);

	/* Retrieve NonMatchingFrame */
	pRxHeader.IsFilterMatchingFrame = ((*RxAddress++ & FDCAN_ELEMENT_MASK_ANMF) >> 31);

	/* Retrieve Rx payload */
	pData = (uint8_t *)RxAddress;
	for(ByteCounter = 0; ByteCounter < DLCtoBytes[pRxHeader.DataLength >> 16]; ByteCounter++)
	{
      *pRxData++ = *pData++;
	}
 
	/* Clear the New Data flag of the current Rx buffer */
	if (CanModule == CAN_MODULE1)
	{
		//FDCAN1->NDAT1 = (1 << RxLocation);
		if(RxLocation < 32)
		{
			FDCAN1->NDAT1 = (1 << RxLocation);
		}
		else /* 32 <= RxBufferIndex <= 63 */
		{
			FDCAN1->NDAT2 = (1 << (RxLocation - 0x20));
		}
	}
	else
	{
		if(RxLocation < 32)
		{
			FDCAN2->NDAT1 = (1 << RxLocation);
		}
		else /* 32 <= RxBufferIndex <= 63 */
		{
			FDCAN2->NDAT2 = (1 << (RxLocation - 0x20));
		}
	
	}
    

}
/* ----------------------- End ReceiveCanMsg ---------------------------------*/




/* ----------------------- FDCAN_SendMessage ---------------------------------*/
 void FDCAN_SendMessage(FDCAN_TxHeaderTypeDef *pTxHeader, uint8_t *pTxData, uint32_t BufferIndex, uint16_t CanModule)
{
  if (!((FDCAN1->TXBRP) & (1 << BufferIndex)) )  // check transmit pending
  {
	
  uint32_t TxElementW1 =0;
  uint32_t TxElementW2=0;
  uint32_t *TxAddress;
  uint32_t ByteCounter;

  /* Build first word of Tx header element */
  TxElementW1 = (pTxHeader->ErrorStateIndicator |
                   FDCAN_STANDARD_ID |
                   pTxHeader->TxFrameType |
                   (pTxHeader->Identifier << 18));

  /* Build second word of Tx header element */
  TxElementW2 = ((pTxHeader->MessageMarker << 24) |
                 pTxHeader->TxEventFifoControl |
                 pTxHeader->FDFormat |
                 pTxHeader->BitRateSwitch |
                 pTxHeader->DataLength);

  /* Calculate Tx element address */
 	TxAddress = (uint32_t *)(TxBufferSA + (POSITION_VAL(BufferIndex) * CAN_TX_ELMTS_SIZE * 4));
	
  /* Write Tx element header to the message RAM */
  *TxAddress++ = TxElementW1;
  *TxAddress++ = TxElementW2;

  /* Write Tx payload to the message RAM */
  for(ByteCounter = 0; ByteCounter < DLCtoBytes[pTxHeader->DataLength >> 16]; ByteCounter += 4)
  {
    *TxAddress++ = ((pTxData[ByteCounter+3] << 24) |
                    (pTxData[ByteCounter+2] << 16) |
                    (pTxData[ByteCounter+1] << 8) |
                    pTxData[ByteCounter]);
  }


	/* Add transmission request */
	
	if (CanModule == CAN_MODULE1){
		 FDCAN1->TXBAR = BufferIndex;
	}
	else{
		 FDCAN2->TXBAR = BufferIndex;
	}
	
	}

}
/* --------------------- End FDCAN_SendMessage -------------------------------*/


/* -------------------- RxFilterRegisterConfig -------------------------------*/
void RxFilterRegisterConfig (FDCAN_FilterTypeDef *pRxFilter)
{
 	uint32_t FilterElementW1;
 	uint32_t *FilterAddress;

 	FilterElementW1 = ((FDCAN_FILTER_TO_RXBUFFER << 27)       |
                       (pRxFilter->FilterID1 << 16)       |
                       (pRxFilter->IsCalibrationMsg << 8) |
                        pRxFilter->RxBufferIndex            );

 	/* Calculate filter address */
 	FilterAddress = (uint32_t *)(StdFilterSA + (pRxFilter->FilterIndex * 4));

 	/* Write filter element to the message RAM */
 	*FilterAddress = FilterElementW1;

}
/* ------------------ End RxFilterRegisterConfig -----------------------------*/






	


