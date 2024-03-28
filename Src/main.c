/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program
  ******************************************************************************
  * 	Example of using Mavlink v2
  *
  * 	1. Receiving and sending HEARTBEAT messages
  * 	2. Receiving other messages (ATTITUDE, etc)
  * 	3. Request for sending message with determined interval
  * 	4. Read value of determined parameter
  *
  * 	Base algorith of this program:
  * 	- send CanMsg (0x600) in response to receiving a heartbeat_msg from mavlink (uart2);
  * 	- send CanMsg (0x510) in response to receiving a global_position_int_msg from mavlink (uart2);
  * 	- send CanMsg (0x520) in response to receiving a attitude_msg from mavlink (uart2);
  *
  * 	- send own heartbeat_msg with 1 Hz interval;
  * 	- send MAV_Request_MSG (with ATTITUDE id) in response to CanMsg(0x100, byte0=0x11)
  *
  *		Due to documentation of 'mavlink v2' it is normal to get
  *		warnings[-Waddress-of-packed-member] during compile
  *
  *		This program can be updated via CAN-bus with CAN_Bootloader
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Defines -------------------------------------------------------------------*/

#define BOARD_ID 					(0x0U)

/* defines for operations with FLASH */
#define APP_PROG_ADDRESS 			(0x8040000U)
#define APP_KONF_ADDRESS 			(0x8020000U)

#define FLASH_DATA_HEADER 			((uint32_t)0x0123fedc)

/* defines for operations with MAVLINK */
#define MAVLINK_MY_SYSTEM_ID 		((uint8_t)0x01)
#define MAVLINK_MY_COMPONENT_ID 	((uint8_t)0x33)
#define MAV_MSG_INTERVAL_1HZ		((int32_t)1000000)
enum MAVLINK_ERROR{MAVLINK_NO_ERRORS, MAVLINK_PARSE_ERROR};

/* use as index of array 'arErrors' */
enum ERROR_TYPE{FLASH_ERRORS, MAVLINK_ERRORS};

/* Variables -----------------------------------------------------------------*/

//uint8_t Error_status = FLASH_RDY;
uint8_t arErrors[] = {FLASH_RDY, MAVLINK_NO_ERRORS};
static mavlink_message_t msg = {0};  //fill structure by 0
static mavlink_status_t status = {0};

uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN];

/* ----------- CAN TxMsg headers ------------------*/

static FDCAN_TxHeaderTypeDef headerTxMsg_0x510;
static typeDefCanMessage CAN_TxMsg_0x510;

static FDCAN_TxHeaderTypeDef headerTxMsg_0x520;
static typeDefCanMessage CAN_TxMsg_0x520;

static FDCAN_TxHeaderTypeDef headerTxMsg_0x600;
static typeDefCanMessage CAN_TxMsg_0x600;


/* ---------- CAN RxMsg headers ------------------*/
uint32_t rxCANid[] = { 0x580 + BOARD_ID, 0x100 };

static FDCAN_FilterTypeDef headerRxMsg_0x58x;
static typeDefCanMessage CAN_RxMsg_0x58x;

static FDCAN_FilterTypeDef headerRxMsg_0x100;
static typeDefCanMessage CAN_RxMsg_0x100;


/* Functions -----------------------------------------------------------------*/

/* Main ----------------------------------------------------------------------*/
int main(void)
{

	/* After execution, bootloader transfers control to this program.
	 * New assignation of 'Vector Table Offset Register' should be done.
	 * */
	__disable_irq();
	SCB->VTOR = (uint32_t)APP_PROG_ADDRESS;
	__enable_irq();


	/* Set the power supply configuration */
	MODIFY_REG(PWR->CR3, (PWR_CR3_SCUEN | PWR_CR3_LDOEN | PWR_CR3_BYPASS), PWR_CR3_LDOEN);
	/* PWR_SetRegulVoltageScaling */
	MODIFY_REG(PWR->D3CR, PWR_D3CR_VOS, (PWR_D3CR_VOS_0 | PWR_D3CR_VOS_1));


	RCC_Init();

	// check if clock install is OK
	if (SystemCoreClock != 72000000)	//if SystemCoreClock not equal 72Mhz reset system
	{
		NVIC_SystemReset();
	}

	InitLEDs();

	/* read config data from flash */
	if (BOARD_ID != 0)
	{
		uint32_t config_data[] = {0,0};
		config_data[0] = flashRead(APP_KONF_ADDRESS);
		config_data[1] = flashRead(APP_KONF_ADDRESS+4);

		if ( (config_data[0] != FLASH_DATA_HEADER) || (config_data[1] != BOARD_ID) )
		{
			//LED3_ON();
			arErrors[FLASH_ERRORS] = WriteBoardIdToFlash();
		}

	}

	/* CAN config */
	InitCAN1();
	Config_CAN_Filters();

	/* Uart config */
	InitUART2();
	InitDMA1_Stream0_UART2_Tx();

	/* Timer config */
	TimerInit(1000);  //timer for 1kHz
	TimerStart();


	/* Loop forever */
	while(1)
	{
		CheckRxMessageCAN1();
		CheckUart2RxMsg();


		/* actions for 1 sec period */
		if (TimerGet().FLAGS.flag_1s)
		{
			TimerResetFlag(TIMER_FL_1S);
			Tick_1sec();
		}

		/* actions for 100 msec period */
		if (TimerGet().FLAGS.flag_100ms)
		{
			TimerResetFlag(TIMER_FL_100MS);
			Tick_100ms();
		}


	}
}
/* End main ------------------------------------------------------------------*/



/* MAV_ParseFrame ------------------------------------------------------------*/
void MAV_ParseFrame (uint8_t byte)
{
	// Try to parse a MAVLink message
	if ( mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status))
	{

		switch (msg.msgid)
		{
			case MAVLINK_MSG_ID_HEARTBEAT:

				mavlink_heartbeat_t heartbeat;
				mavlink_msg_heartbeat_decode(&msg, &heartbeat);

				CAN_TxMsg_0x600.data[0] = msg.msgid;
				CAN_TxMsg_0x600.data[1] = msg.sysid;
				CAN_TxMsg_0x600.data[2] = msg.compid;
				CAN_TxMsg_0x600.data[7] = heartbeat.type;
				FDCAN_SendMessage(&headerTxMsg_0x600, CAN_TxMsg_0x600.data, TxMsg_0x600_BUF_NUMBER, CAN_MODULE1);
				break;

			case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:

				mavlink_global_position_int_t global_position;
				mavlink_msg_global_position_int_decode(&msg, &global_position);

				CAN_TxMsg_0x510.data[0] = global_position.relative_alt >> 24;
				CAN_TxMsg_0x510.data[1] = global_position.relative_alt >> 16;
				CAN_TxMsg_0x510.data[2] = global_position.relative_alt >> 8;
				CAN_TxMsg_0x510.data[3] = global_position.relative_alt && 0xFF;
				CAN_TxMsg_0x510.data[4] = 0;
				CAN_TxMsg_0x510.data[5] = 0;
				CAN_TxMsg_0x510.data[6] = global_position.hdg >> 8;
				CAN_TxMsg_0x510.data[7] = global_position.hdg;
				FDCAN_SendMessage(&headerTxMsg_0x510, CAN_TxMsg_0x510.data, TxMsg_0x510_BUF_NUMBER, CAN_MODULE1);
				break;

			case MAVLINK_MSG_ID_ATTITUDE:

				mavlink_attitude_t attitude;
				mavlink_msg_attitude_decode(&msg, &attitude);

				float deg = attitude.pitch*(float)572957.79513;
				int32_t pitch = deg;

				CAN_TxMsg_0x520.data[0] = pitch >> 24;
				CAN_TxMsg_0x520.data[1] = pitch >> 16;
				CAN_TxMsg_0x520.data[2] = pitch >> 8;
				CAN_TxMsg_0x520.data[3] = pitch && 0xFF;

				deg = attitude.yaw*(float)572957.79513;
				int32_t yaw = deg;

				CAN_TxMsg_0x520.data[4] = yaw >> 24;
				CAN_TxMsg_0x520.data[5] = yaw >> 16;
				CAN_TxMsg_0x520.data[6] = yaw >> 8;
				CAN_TxMsg_0x520.data[7] = yaw;
				FDCAN_SendMessage(&headerTxMsg_0x520, CAN_TxMsg_0x520.data, TxMsg_0x520_BUF_NUMBER, CAN_MODULE1);
				break;

			case MAVLINK_MSG_ID_PARAM_VALUE:

				mavlink_param_value_t param_value;
				mavlink_msg_param_value_decode(&msg, &param_value);

				break;

		}
	}
	else    // mavlink_parse_char() == 0
	{
		if ( status.parse_error > 0 ){
			arErrors[MAVLINK_ERRORS] = MAVLINK_PARSE_ERROR;
		}
	}

}
/* End MAV_ParseFrame --------------------------------------------------------*/



/* CheckUart2RxMsg -----------------------------------------------------------*/
void CheckUart2RxMsg (void)
{

	while ( (USART2->ISR) & USART_ISR_RXNE ) //if RXNE = 1 Received data is ready to be read (some bytes may storage in FIFO)
	{
		//USART2->RDR;  /* read received byte */

		MAV_ParseFrame(USART2->RDR);

	} /* end while */

	/* clear flags */
	if ( ((USART2->ISR) & USART_ISR_FE) > 0)
	{
		USART2->ICR |= USART_ICR_FECF;
	}
}
/* End CheckUart2RxMsg -------------------------------------------------------*/



/* Tick_1ms ------------------------------------------------------------------*/
void Tick_100ms (void)
{
 //
}
/* End Tick_1ms --------------------------------------------------------------*/


/* Tick_1ms ------------------------------------------------------------------*/
void Tick_1ms (void)
{
	//
}
/* End Tick_1ms --------------------------------------------------------------*/


/* Tick_1sec -----------------------------------------------------------------*/
void Tick_1sec (void)
{

	LED1_TOOGLE();

	MAV_Send_HEARTBEAT();


	/* Check Errors */
	if ((arErrors[FLASH_ERRORS] != FLASH_RDY) || (arErrors[MAVLINK_ERRORS] != MAVLINK_NO_ERRORS)  )
	{
		LED3_ON();
	}

}
/* Tick_1sec -----------------------------------------------------------------*/




/* SendUart_DMA --------------------------------------------------------------*/
void SendUart_DMA(uint8_t txbuf[], const uint32_t length)
{
	while (DMA1_Stream0->NDTR != 0){};
	DMA1_Stream0->CR &= ~DMA_SxCR_EN;

	DMA1_Stream0->M0AR = (uint32_t) txbuf;
	DMA1_Stream0->NDTR = length;
	DMA1->LIFCR |= DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 |DMA_LIFCR_CTEIF0 |DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0;

	DMA1_Stream0->CR |= DMA_SxCR_EN;
}
/* End SendUart_DMA ----------------------------------------------------------*/



/* MAV_Send_HEARTBEAT --------------------------------------------------------*/
void MAV_Send_HEARTBEAT()
{
	static mavlink_heartbeat_t heartbeat;

	heartbeat.type = MAV_TYPE_GENERIC;
	heartbeat.autopilot = MAV_AUTOPILOT_INVALID;

	mavlink_msg_heartbeat_encode(MAVLINK_MY_SYSTEM_ID, MAVLINK_MY_COMPONENT_ID, &msg, &heartbeat);

	uint8_t len = mavlink_msg_to_send_buffer(tx_buf, &msg);

	SendUart_DMA(tx_buf, len);
}
/* End MAV_Send_HEARTBEAT ----------------------------------------------------*/



/* MAV_Request_PARAM_READ ----------------------------------------------------*/
void MAV_Request_PARAM_READ(const char* param_id)
{

	mavlink_msg_param_request_read_pack(MAVLINK_MY_SYSTEM_ID, MAVLINK_MY_COMPONENT_ID, &msg, 1, 1, param_id, 0);

	uint8_t len = mavlink_msg_to_send_buffer(tx_buf, &msg);

	SendUart_DMA(tx_buf, len);

}
/* End MAV_Request_PARAM_READ ------------------------------------------------*/


/* MAV_RequestMSG ------------------------------------------------------------*/
void MAV_Request_MSG( uint16_t msg_id, int32_t interval_us)
{
	mavlink_command_long_t cmd = {0};

	cmd.target_system = 1;
	cmd.target_component = 1;
	cmd.command = MAV_CMD_SET_MESSAGE_INTERVAL;
	cmd.confirmation = 0;
	cmd.param1 = msg_id;
	cmd.param2 = interval_us;  // 1 sec interval

	mavlink_msg_command_long_encode(MAVLINK_MY_SYSTEM_ID, MAVLINK_MY_COMPONENT_ID, &msg, &cmd);

	uint8_t len = mavlink_msg_to_send_buffer(tx_buf, &msg);

	SendUart_DMA(tx_buf, len);
}
/* End MAV_RequestMSG --------------------------------------------------------*/




/* CheckRxMessageCAN1 --------------------------------------------------------*/
void CheckRxMessageCAN1 (void)
{
	 uint32_t reg_NewDataFlags = FDCAN1->NDAT1; // for RxBufferIndex < 32

	/* Check Msg 0x58x reception */
	if((reg_NewDataFlags & (1 << headerRxMsg_0x58x.RxBufferIndex)) != 0)
	{
		ReceiveCanMsg(headerRxMsg_0x58x.RxBufferIndex, CAN_RxMsg_0x58x.data, CAN_MODULE1);

		/* example of jumping back to bootloader */
		if ( (CAN_RxMsg_0x58x.data[0] == 0x55) &&  (CAN_RxMsg_0x58x.data[1] == 0x66))
		{
			// MC always starts from bootloader after reset
			NVIC_SystemReset();
		}

		/* new bootloader delay value can be written to flash-memory */
		if ( (CAN_RxMsg_0x58x.data[0] == 0xCC) &&  (CAN_RxMsg_0x58x.data[1] == 0xDD))
		{
			uint32_t delay;

			delay = (CAN_RxMsg_0x58x.data[2] << 8) + CAN_RxMsg_0x58x.data[3];
			ChangeBootloaderDelay(delay);
		}


	}

	/* Check Msg 0x100 reception */
	if((reg_NewDataFlags & (1 << headerRxMsg_0x100.RxBufferIndex)) != 0)
	{
		ReceiveCanMsg(headerRxMsg_0x100.RxBufferIndex, CAN_RxMsg_0x100.data, CAN_MODULE1);

		/* command to send CMD_Msg */
		if ( (CAN_RxMsg_0x100.data[0] == 0x11) )
		{
			//const char* param_id = "MAX_ROLL_RATE";
			//SendMavlinkCmd(param_id);

			MAV_Request_MSG( MAVLINK_MSG_ID_ATTITUDE, MAV_MSG_INTERVAL_1HZ);
		}

	}


}
/* End CheckRxMessageCAN1 ----------------------------------------------------*/



/* WriteBoardIdToFlash -------------------------------------------------------*/
enum FLASH_STATUS WriteBoardIdToFlash(void)
{
	uint32_t buff[2];
	buff[0] = FLASH_DATA_HEADER;
	buff[1] = BOARD_ID;

	if ( flashUnlock() != FLASH_RDY ){ return FLASH_LOCK_ERROR;}


	if (flash_EraseSector(FLASH_SECTOR_CONFIG_DATA) == FLASH_RDY)
	{
		if (flashWrite(APP_KONF_ADDRESS, ((uint32_t)buff), sizeof(buff)) != FLASH_RDY)
		{
			return FLASH_PGM_ERROR;
		}

	}
	else {return FLASH_PGM_ERROR;}

	if ( flashLock() != FLASH_RDY ){ return FLASH_LOCK_ERROR;}

	return FLASH_RDY;

}
/* End WriteBoardIdToFlash ---------------------------------------------------*/



/* ChangeBootloaderDelay -----------------------------------------------------*/
enum FLASH_STATUS ChangeBootloaderDelay(uint32_t delay)
{
	uint32_t buff[3];

	buff[0] = FLASH_DATA_HEADER;
	buff[1] = BOARD_ID;
	buff[2] = delay;

	if ( flashUnlock() != FLASH_RDY ){ return FLASH_LOCK_ERROR;}

	if (flash_EraseSector(FLASH_SECTOR_CONFIG_DATA) == FLASH_RDY)
	{
		if (flashWrite(APP_KONF_ADDRESS, ((uint32_t)buff), sizeof(buff)) != FLASH_RDY)
		{
			return FLASH_PGM_ERROR;
		}

	}
	else {return FLASH_PGM_ERROR;}

	if ( flashLock() != FLASH_RDY ){ return FLASH_LOCK_ERROR;}

	return FLASH_RDY;
}
/* End ChangeBootloaderDelay -------------------------------------------------*/





/* InitLEDs ------------------------------------------------------------------*/
void InitLEDs(void)
{

	/* onboard leds: LED1, LED2, LED3 - PB0, PB7, PB14 */
	RCC->AHB4ENR|= RCC_AHB4ENR_GPIOBEN; 																														//enable clock for bus AHB4 (GPIO G)

	GPIOB->MODER &= ~ (GPIO_MODER_MODER0 | GPIO_MODER_MODER7 | GPIO_MODER_MODER14 ); 					// reset bits
	GPIOB->MODER |= (GPIO_MODER_MODER0_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER14_0 ); 				// output mode
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_0 | GPIO_OTYPER_OT_7 | GPIO_OTYPER_OT_14 );  						// output push-pull (reset state)
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 | GPIO_OSPEEDER_OSPEEDR7 | GPIO_OSPEEDER_OSPEEDR14 ); 	// low speed

	LED1_ON();
	LED2_OFF();
	LED3_OFF();
}
/* End InitLEDs --------------------------------------------------------------*/


/* Config_CAN_Filters --------------------------------------------------------*/
void Config_CAN_Filters(void)
{

	/**************** Config RxFilters ****************/

	uint32_t index;

	/* Configure Rx filter Msg ID 0x59x */
	index = FDCAN_RX_BUFFER0;
	headerRxMsg_0x58x.RxBufferIndex = index;
	headerRxMsg_0x58x.FilterIndex = index;
	headerRxMsg_0x58x.IdType = FDCAN_STANDARD_ID;
	headerRxMsg_0x58x.FilterType = FDCAN_FILTER_DUAL;
	headerRxMsg_0x58x.FilterConfig = FDCAN_FILTER_TO_RXBUFFER;
	headerRxMsg_0x58x.FilterID1 = rxCANid[index];
	headerRxMsg_0x58x.FilterID2 = 0x00;

	RxFilterRegisterConfig(&headerRxMsg_0x58x);

	/* Configure Rx filter Msg ID 0x100 */
	index = FDCAN_RX_BUFFER1;
	headerRxMsg_0x100.RxBufferIndex = index;
	headerRxMsg_0x100.FilterIndex = index;
	headerRxMsg_0x100.IdType = FDCAN_STANDARD_ID;
	headerRxMsg_0x100.FilterType = FDCAN_FILTER_DUAL;
	headerRxMsg_0x100.FilterConfig = FDCAN_FILTER_TO_RXBUFFER;
	headerRxMsg_0x100.FilterID1 = rxCANid[index];
	headerRxMsg_0x100.FilterID2 = 0x00;

	RxFilterRegisterConfig(&headerRxMsg_0x100);


	/**************** Config TxFilters ****************/

	headerTxMsg_0x510.Identifier = 0x510;
	headerTxMsg_0x510.IdType = FDCAN_STANDARD_ID;
	headerTxMsg_0x510.TxFrameType = FDCAN_DATA_FRAME;
	headerTxMsg_0x510.DataLength = FDCAN_DLC_BYTES_8;
	headerTxMsg_0x510.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	headerTxMsg_0x510.BitRateSwitch = FDCAN_BRS_OFF;
	headerTxMsg_0x510.FDFormat = FDCAN_CLASSIC_CAN;
	headerTxMsg_0x510.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	headerTxMsg_0x510.MessageMarker = headerTxMsg_0x510.Identifier;

	headerTxMsg_0x520.Identifier = 0x520;
	headerTxMsg_0x520.IdType = FDCAN_STANDARD_ID;
	headerTxMsg_0x520.TxFrameType = FDCAN_DATA_FRAME;
	headerTxMsg_0x520.DataLength = FDCAN_DLC_BYTES_8;
	headerTxMsg_0x520.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	headerTxMsg_0x520.BitRateSwitch = FDCAN_BRS_OFF;
	headerTxMsg_0x520.FDFormat = FDCAN_CLASSIC_CAN;
	headerTxMsg_0x520.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	headerTxMsg_0x520.MessageMarker = headerTxMsg_0x520.Identifier;

	headerTxMsg_0x600.Identifier = 0x600;
	headerTxMsg_0x600.IdType = FDCAN_STANDARD_ID;
	headerTxMsg_0x600.TxFrameType = FDCAN_DATA_FRAME;
	headerTxMsg_0x600.DataLength = FDCAN_DLC_BYTES_8;
	headerTxMsg_0x600.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	headerTxMsg_0x600.BitRateSwitch = FDCAN_BRS_OFF;
	headerTxMsg_0x600.FDFormat = FDCAN_CLASSIC_CAN;
	headerTxMsg_0x600.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	headerTxMsg_0x600.MessageMarker = headerTxMsg_0x600.Identifier;

}
/* End Config_CAN_Filters ----------------------------------------------------*/









