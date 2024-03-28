/**
  ******************************************************************************
  * @file           : uart.c
  * @brief          : UART Configuration
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "uart.h"


/* Variables -----------------------------------------------------------------*/



/* Functions -----------------------------------------------------------------*/

/* InitUART2 -----------------------------------------------------------------*/
uint16_t InitUART2 (void)
{

	//uint32_t wait_count = 0;
	
		/* UART2 kernel clock source selection.
	     Enable UART Clock output generated from rcc_pclk1 (default after reset) */
		RCC->D2CCIP2R &= ~RCC_D2CCIP2R_USART28SEL;
	
		RCC->APB1LENR|= RCC_APB1LENR_USART2EN; 												// enable clock for bus APB1 (UART2)
		RCC->AHB4ENR |= RCC_AHB4ENR_GPIODEN; 												// enable clock for bus AHB4 (GPIO D) UART2 - GPIO PD4, PD5
		
		/* PD5--> UART2_TX; PD6--> UART2_RX; UART2 is AF7 (0111), see p85 of datasheet;
		   AFR register see p523 of ref manual */
		GPIOD->AFR[0] &= ~(GPIO_AFRL_AFRL5 | GPIO_AFRL_AFRL6);								//reset bits
		GPIOD->AFR[0] |= ( GPIO_AFR_AF7_UART << GPIO_AFRL_AFRL5_Pos) | (GPIO_AFR_AF7_UART << GPIO_AFRL_AFRL6_Pos); // set alternate function
		GPIOD->MODER &= ~(GPIO_MODER_MODER5 | GPIO_MODER_MODER6); 							// reset bits MODER PE5, PE6
		GPIOD->MODER |= (GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1);  						// set PD5, PD6 in alternate function mode
		GPIOD->OTYPER &= ~GPIO_OTYPER_OT_5; 												// set PD5 (UART2_TX) as  push-pull
		GPIOD->OSPEEDR &= (GPIO_OSPEEDER_OSPEEDR5 | GPIO_OSPEEDER_OSPEEDR6); 				// reset bits OSPEED PE5, PE6
		GPIOD->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR5_1 | GPIO_OSPEEDER_OSPEEDR6_1; 				// set high speed
		GPIOD->PUPDR &= ~GPIO_PUPDR_PUPDR5; 												// No pull-up, pull-down

		/* reset UART2*/
		RCC->APB1LRSTR |= RCC_APB1LRSTR_USART2RST;
		RCC->APB1LRSTR &= ~RCC_APB1LRSTR_USART2RST;
		
		/* Word length - Bit 1 */
		USART2->CR1 &= ~USART_CR1_M1;  														// 00: 1 start bit, 8 Data bits, n Stop bit
		
		/* FIFO mode enable */
		USART2->CR1 |= USART_CR1_FIFOEN;
		
		/* RXFIFO threshold interrupt enable */
		//UART7->CR3 |= USART_CR3_RXFTIE;
		
		/* RXFTCFG [2:0]Receive FIFO threshold configuration */
		USART2->CR3 &= ~USART_CR3_RXFTCFG;													// 000:Receive FIFO reaches 1/8 of its depth
		//UART7->CR3 |= (USART_CR3_RXFTCFG_0 | USART_CR3_RXFTCFG_1);						// 011:TXFIFO reaches 3/4 of its depth
		USART2->CR3 |= USART_CR3_RXFTCFG_0;													// 001:TXFIFO reaches 1/4 of its depth
		
		/* DEAT[4:0] bits (Driver Enable Assertion Time) */
		USART2->CR1 &= ~USART_CR1_DEAT;
		
		/* DEDT[4:0] bits (Driver Enable Deassertion Time) */
		USART2->CR1 &= ~USART_CR1_DEDT;
		
		/* Oversampling by 8-bit or 16-bit mode */
		USART2->CR1 &= ~USART_CR1_OVER8;													// 0: Oversampling by 16
		
		/* Word length - Bit 0 */
		USART2->CR1 &= ~USART_CR1_M0;  														// 00: 1 start bit, 8 Data bits, n Stop bit
		
		/* Parity Control Enable */
		USART2->CR1 &= ~USART_CR1_PCE;   													// 0: Parity control disabled
		
		/* Parity Selection */
		USART2->CR1 &= ~USART_CR1_PS;   													//0: Even parity
		
		/* Transmission Complete Interrupt Enable */
		//UART7->CR1 |= USART_CR1_TCIE;   													// 1: USART interrupt generated whenever TC=1 in the USART_ISR register
		USART2->CR1 &= ~USART_CR1_TCIE;
		
		/* RXNE Interrupt Enable */
		//UART7->CR1 |= USART_CR1_RXNEIE; 													//USART interrupt generated whenever ORE=1 or RXNE/RXFNE=1
		
		/* Binary data inversion */
		//UART7->CR2 |= USART_CR2_DATAINV;
		
		/* TX pin active level inversion */
		//UART7->CR2 |= USART_CR2_TXINV;
		
		/* RX pin active level inversion */
		//UART7->CR2 |= USART_CR2_RXINV;
		
		
		/* Driver Enable Mode */
		USART2->CR3 &= ~USART_CR3_DEM;
		
		/* DMA Disable on Reception Error */
		USART2->CR3 &= ~USART_CR3_DDRE;  													//0: DMA is not disabled in case of reception error (p2064 refman)
		
		/* Overrun Disable */
		USART2->CR3 &= ~USART_CR3_OVRDIS;   												//0: Overrun Error Flag, ORE, is set
		//UART7->CR3 |= USART_CR3_OVRDIS;
		
		/* One sample bit method enable */
		USART2->CR3 &= ~USART_CR3_ONEBIT;  													//0: Three sample bit method
				
		/* DMA Enable Transmitter */
		USART2->CR3 &= ~USART_CR3_DMAT;  													//0: DMA mode is disabled for transmission
		USART2->CR3 |= USART_CR3_DMAT;
		
		/* DMA Enable Receiver */
		USART2->CR3 &= ~USART_CR3_DMAR;  													//0: DMA mode is disabled for reception
		//UART7->CR3 |= USART_CR3_DMAR; 
		
		
		/* Half-Duplex Selection */
		USART2->CR3 &= ~USART_CR3_HDSEL;  													//0: Half duplex mode is not selected
		
		/* USART baud rate  */   
		USART2->BRR &= ~(USART_BRR_DIV_FRACTION | USART_BRR_DIV_MANTISSA);
		
		/* clk 25MHz; presc=1; baudrate = 9600; BRR = 25000000/9600=2604,166=0xA2C (p2022 refman) */
		/* clk 25MHz; presc=1; baudrate = 115200; BRR = 25000000/115200=217,0138 =0xD9 (p2022 refman) */
		/* clk 36MHz; presc=1; baudrate = 57600; BRR = 36000000/57600=625 (p2022 refman) */
		USART2->BRR = UART2_BRR;
		
		/* PSC[7:0] bits (Prescaler value) */
		//UART7->GTPR &= ~USART_GTPR_PSC; 													//00000001: divides the source clock by 1
		
		/* PRESCALER[3:0] bits (Clock prescaler) */
		USART2->PRESC &= ~USART_PRESC_PRESCALER; 											// 0000: input clock not divided
		
		
		/* USART Enable */
		USART2->CR1 |= USART_CR1_UE; 														//1: USART enabled
		
		/* Transmitter Enable */
		USART2->CR1 |= USART_CR1_TE; // !!maybe need delay   								//1: Transmitter is enabled
		
		/* Receiver Enable */
		USART2->CR1 |= USART_CR1_RE;  														//1: Receiver is enabled and begins searching for a start bit
		
		
		//wait_count = 0;
		
		//NVIC_EnableIRQ(UART7_IRQn);
		

		return UART_STATUS_OK;
}
/* End InitUART2 -------------------------------------------------------------*/

/* InitDMA1_Stream0_UART2_Tx -------------------------------------------------*/
uint16_t InitDMA1_Stream0_UART2_Tx (void)
{
	uint32_t wait_count = 0;
	
	RCC->AHB1ENR|= RCC_AHB1ENR_DMA1EN; 					// enable clock for DMA1
	
	/* Double buffer mode enable */
	//DMA1_Stream0->CR |= DMA_SxCR_DBM;
	
	/* Priority level */
	DMA1_Stream0->CR &= ~DMA_SxCR_PL; 					// 00 - low priority
	DMA1_Stream0->CR |= DMA_SxCR_PL_1;  				// 10: high
	
	/* Memory data size */ 
	DMA1_Stream0->CR &= ~DMA_SxCR_MSIZE;				// 00 - 8 bit
	//DMA1_Stream0->CR |= DMA_SxCR_MSIZE_1;  			// 10: word (32-bit)
	
	/* Peripheral data size */
	DMA1_Stream0->CR &= ~DMA_SxCR_PSIZE;
	//DMA1_Stream0->CR |= DMA_SxCR_PSIZE_1;  			// 10: word (32-bit)
	
	/* Memory increment mode */
	//DMA1_Stream0->CR &= ~DMA_SxCR_MINC;				// 0: memory address pointer is fixed
	DMA1_Stream0->CR |= DMA_SxCR_MINC;	
	
	/* Peripheral increment mode */
	DMA1_Stream0->CR &= ~DMA_SxCR_PINC;					// 0: peripheral address pointer is fixed
	
	/* Circular mode */
	DMA1_Stream0->CR &= ~DMA_SxCR_CIRC;					// 0: circular mode disabled
	//DMA1_Stream0->CR |= DMA_SxCR_CIRC;
	
	/* Memory burst transfer configuration */
	DMA1_Stream0->CR &= ~DMA_SxCR_MBURST; 
	DMA1_Stream0->CR |= DMA_SxCR_MBURST_1;  			//10: INCR8 (incremental burst of 8 beats)
	
	/* Data transfer direction */
	DMA1_Stream0->CR &= ~DMA_SxCR_DIR;					// 00: peripheral-to-memory
	DMA1_Stream0->CR |= DMA_SxCR_DIR_0;					// 01: memory-to-peripheral
	
	/* Peripheral flow controller */
	DMA1_Stream0->CR &= ~DMA_SxCR_PFCTRL;				// 1: The peripheral is the flow controller; 0 - DMA is
	
	
	/* Transfer complete interrupt enable */
	DMA1_Stream0->CR &= ~DMA_SxCR_TCIE;					// 0: TC interrupt disabled
	//DMA1_Stream0->CR |= DMA_SxCR_TCIE;
	
	//DMA1_Stream0->CR |= DMA_SxCR_HTIE;
	
	/* Transfer error interrupt enable */
	DMA1_Stream0->CR &= ~DMA_SxCR_TEIE;					// 0: TE interrupt disabled
	
	/* Number of data items to transfer */
	DMA1_Stream0->NDTR &= ~DMA_SxNDT;
	//DMA1_Stream0->NDTR |= DMA_SxNDT_0;  				// 1 item
	//DMA1_Stream0->NDTR |= (DMA_SxNDT_2 | DMA_SxNDT_3);// 12 item
	//DMA1_Stream0->NDTR |= (DMA_SxNDT_2);  			// 4 item
	
	/* Peripheral Address */
	DMA1_Stream0->PAR &= ~DMA_SxPAR_PA;
	DMA1_Stream0->PAR = (uint32_t)&(USART2->TDR);
	
	/* Memory 0 Address */
	//DMA1_Stream0->M0AR &= ~DMA_SxM0AR_M0A;
	//DMA1_Stream0->M0AR = (uint32_t) &arUartToDisplay;  // reference to array with data is setting in main.c before enable dma_stream
	
	/* Memory 1 Address */
	DMA1_Stream0->M1AR &= ~DMA_SxM1AR_M1A;
	//DMA1_Stream0->M1AR = (uint32_t)&buf1;
	
	/* Direct mode disable */
	DMA1_Stream0->FCR |= DMA_SxFCR_DMDIS;  				// 1: direct mode disabled
	
	/* FIFO threshold selection */
	//DMA1_Stream0->FCR |= DMA_SxFCR_FTH_1;				// 10: 3/4 full FIFO
	DMA1_Stream0->FCR |= DMA_SxFCR_FTH; 				// 11: full FIFO
	
	/*  DMA request identification */
	DMAMUX1_Channel0->CCR &= ~DMAMUX_CxCR_DMAREQ_ID; 	//id=80 (p673 refman)
	DMAMUX1_Channel0->CCR = (uint32_t)44;
	
	
	
	/* Stream enable / flag stream ready when read low */
//	DMA1_Stream0->CR |= DMA_SxCR_EN;
//	while (((DMA1_Stream0->CR & DMA_SxCR_EN) != DMA_SxCR_EN) && (wait_count < UART_TIMEOUT)) //wait enable acknowledge
//		{ 
//			wait_count++;
//		} 
		
		if (wait_count >= UART_TIMEOUT){return UART_STATUS_ERROR;}	
		
		//NVIC_EnableIRQ(DMA1_Stream0_IRQn);

		return UART_STATUS_OK;
}
/* End InitDMA1_Stream0_UART2_Tx ---------------------------------------------*/





/* DMA1_Stream1_IRQHandler ---------------------------------------------------*/
void DMA1_Stream1_IRQHandler (void)
{
	/* Clear transfer complete interrupt flag */
	if ( (DMA1->LISR) & DMA_LISR_TCIF1){
		DMA1->LIFCR |= DMA_LIFCR_CTCIF1;
	}
	

}
/* End DMA1_Stream1_IRQHandler -----------------------------------------------*/





/* UART2_Send_Char -----------------------------------------------------------*/
void USART2_Send_Char(char chr)
{
	while ( !(USART2->ISR & USART_ISR_TC)){};
		
	USART2->TDR = chr;

}
/* End UART2_Send_Char -------------------------------------------------------*/



/* UART2_Send_Str ------------------------------------------------------------*/
void USART2_Send_Str(char* str)
{
		uint8_t i=0;
	
		while ( str[i]){
			USART2_Send_Char(str[i++]);
		};
		

}
/* End UART2_Send_Str --------------------------------------------------------*/



/* UART2_Send_arBytes --------------------------------------------------------*/
void USART2_Send_arBytes(uint8_t* arData)
{
	uint8_t i=0;
	
	for (i=0; i < sizeof(arData); i++)
	{
		while ( !(USART2->ISR & USART_ISR_TXE)){};
	  USART2->TDR = arData[i];
	}
}
/* End UART2_Send_arBytes ----------------------------------------------------*/


/* CheckForUartTxByteReplacement ---------------------------------------------*/
uint8_t CheckForUartTxByteReplacement (uint8_t dataByte, uint8_t curInd, uint8_t* arSendingData)
{
	if (dataByte == UART_MARKER)
	{
		arSendingData[++curInd]= UART_REPLACE_BYTE0;
		arSendingData[++curInd]= UART_REPLACE_BYTE1;
	}
	else
	{
		if (dataByte == UART_REPLACE_BYTE0)
		{
		arSendingData[++curInd]= UART_REPLACE_BYTE0;
		arSendingData[++curInd]= UART_REPLACE_BYTE2;
		}
		else
		{
			arSendingData[++curInd]= dataByte;
		}
	}

	return curInd;
}
/* End CheckForUartTxByteReplacement -----------------------------------------*/



/* CalculateCRC --------------------------------------------------------------*/
void CalculateCRC (void)
{

	////calculate CRC and write to array

	

}
/* End CalculateCRC ----------------------------------------------------------*/


/* UART7_IRQHandler ----------------------------------------------------------*/
void UART7_IRQHandler (void)
{
	
		
		if (UART7->ISR & USART_ISR_RXFT)
		{
			GPIOG->ODR |= GPIO_ODR_ODR_2; // GPIO G2=0
			//rcvD = UART7->RDR;
			//rcvD = UART7->RDR;
			
		}
		
		
}
/* End UART7_IRQHandler ------------------------------------------------------*/

	
/* UART8_IRQHandler ----------------------------------------------------------*/
void UART8_IRQHandler (void)
{
		
		if (UART8->ISR & USART_ISR_RXFT)
		{
			GPIOG->ODR ^= ~GPIO_ODR_ODR_2; // GPIO G2=0
			//TxData2[0] = UART8->RDR;
			//TxData2[1] = UART8->RDR;		

			
		}
		
}
/* End UART8_IRQHandler ------------------------------------------------------*/
	
	



	
