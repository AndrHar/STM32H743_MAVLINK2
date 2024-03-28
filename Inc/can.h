/*----------------------------------------------------------------------------*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CAN_H_IFND
#define CAN_H_IFND


/* Includes ------------------------------------------------------------------*/

#include "stm32h7xx.h"               

/* Defines -------------------------------------------------------------------*/

static const uint8_t DLCtoBytes[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};


/*----------------------------------------------------
For Clock 8MHz
250 kbit/sec SEG1 - 13; SEG2 - 2; BRP - 2;
250 kbit/sec SEG1 - 5; SEG2 - 2; BRP - 4;
500 kbit/sec SEG1 - 13; SEG2 - 2; BRP - 1;
1000 kbit/sec SEG1 - 6; SEG2 - 1; BRP - 1;


For Clock 32MHz
250 kbit/sec SEG1 - 13; SEG2 - 2; BRP - 8;
250 kbit/sec SEG1 - 13; SEG2 - 2; BRP - 4;  <--current
1000 kbit/sec SEG1 - 13; SEG2 - 2; BRP - 2;
-----------------------------------------------------*/


/* CAN1 settings values ------------------------------------------------------*/
#define CAN1_NSJW 								(1U)
#define CAN1_NTSEG1 							(13U)
#define CAN1_NTSEG2 							(2U)
#define CAN1_NBRP 								(4U)

/* CAN2 settings values ------------------------------------------------------*/
#define CAN2_NSJW 								(2U)
#define CAN2_NTSEG1 							(10U)
#define CAN2_NTSEG2 							(5U)
#define CAN2_NBRP 								(4U)

/* CAN1 and CAN2 share the same message RAM -> general parameters for Tx & Rx  */
#define CAN_RX_STD_FILT_NBR 					(32U) //maximum value 128
#define CAN_RX_EXT_FILT_NBR 					(0U)
#define CAN_RX_FIFO0_ELMTS_NBR 					(0U)
#define CAN_RX_FIFO1_ELMTS_NBR 					(0U)
#define	CAN_RX_BUFFERS_NBR 						CAN_RX_STD_FILT_NBR
#define CAN_RX_FIFO0_ELMTS_SIZE 				(0U)
#define CAN_RX_FIFO1_ELMTS_SIZE 				(0U)
#define CAN_RX_BUFFERS_SIZE 					(4U)
#define	CAN_TX_EVENTS_NBR 						(2U)
#define	CAN_TX_BUFFERS_NBR 						(32U) //change number of buffers if new added
#define CAN_TX_FIFO_QUEUE_ELMTS_NBR 			(0U)
#define CAN_TX_ELMTS_SIZE 						(4U)



/* CAN general definations ---------------------------------------------------*/
#define CAN_TIMEOUT 							((uint32_t)0xFF0000)
#define CAN_STATUS_OK							((uint16_t)0x00)
#define CAN_STATUS_ERROR						((uint16_t)0x01)
#define CAN_STATUS_ERROR_RAM					((uint16_t)0x02)
#define CAN_MODULE1								((uint16_t)0x00)
#define CAN_MODULE2								((uint16_t)0x01)

#define GPIO_AF9_FDCAN        					((uint8_t)0x09)  		/* FDCAN Alternate Function mapping   */

#define FDCAN_ELEMENT_MASK_STDID 				((uint32_t)0x1FFC0000U) /* Standard Identifier         */
#define FDCAN_ELEMENT_MASK_EXTID 				((uint32_t)0x1FFFFFFFU) /* Extended Identifier         */
#define FDCAN_ELEMENT_MASK_RTR   				((uint32_t)0x20000000U) /* Remote Transmission Request */
#define FDCAN_ELEMENT_MASK_XTD   				((uint32_t)0x40000000U) /* Extended Identifier         */
#define FDCAN_ELEMENT_MASK_ESI   				((uint32_t)0x80000000U) /* Error State Indicator       */
#define FDCAN_ELEMENT_MASK_TS    				((uint32_t)0x0000FFFFU) /* Timestamp                   */
#define FDCAN_ELEMENT_MASK_DLC   				((uint32_t)0x000F0000U) /* Data Length Code            */
#define FDCAN_ELEMENT_MASK_BRS   				((uint32_t)0x00100000U) /* Bit Rate Switch             */
#define FDCAN_ELEMENT_MASK_FDF   				((uint32_t)0x00200000U) /* FD Format                   */
#define FDCAN_ELEMENT_MASK_EFC   				((uint32_t)0x00800000U) /* Event FIFO Control          */
#define FDCAN_ELEMENT_MASK_MM    				((uint32_t)0xFF000000U) /* Message Marker              */
#define FDCAN_ELEMENT_MASK_FIDX  				((uint32_t)0x7F000000U) /* Filter Index                */
#define FDCAN_ELEMENT_MASK_ANMF  				((uint32_t)0x80000000U) /* Accepted Non-matching Frame */
#define FDCAN_ELEMENT_MASK_ET    				((uint32_t)0x00C00000U) /* Event type                  */

#define FDCAN_STANDARD_ID 						((uint32_t)0x00000000U) /*!< Standard ID element */
#define FDCAN_DATA_FRAME   						((uint32_t)0x00000000U) /*!< Data frame   */

#define FDCAN_DLC_BYTES_0  						((uint32_t)0x00000000U) /*!< 0 bytes data field  */
#define FDCAN_DLC_BYTES_1  						((uint32_t)0x00010000U) /*!< 1 bytes data field  */
#define FDCAN_DLC_BYTES_2  						((uint32_t)0x00020000U) /*!< 2 bytes data field  */
#define FDCAN_DLC_BYTES_3  						((uint32_t)0x00030000U) /*!< 3 bytes data field  */
#define FDCAN_DLC_BYTES_4  						((uint32_t)0x00040000U) /*!< 4 bytes data field  */
#define FDCAN_DLC_BYTES_5  						((uint32_t)0x00050000U) /*!< 5 bytes data field  */
#define FDCAN_DLC_BYTES_6  						((uint32_t)0x00060000U) /*!< 6 bytes data field  */
#define FDCAN_DLC_BYTES_7  						((uint32_t)0x00070000U) /*!< 7 bytes data field  */
#define FDCAN_DLC_BYTES_8  						((uint32_t)0x00080000U) /*!< 8 bytes data field  */

#define FDCAN_ESI_ACTIVE  						((uint32_t)0x00000000U) /*!< Transmitting node is error active  */
#define FDCAN_BRS_OFF 							((uint32_t)0x00000000U) /*!< FDCAN frames transmitted/received without bit rate switching */
#define FDCAN_CLASSIC_CAN 						((uint32_t)0x00000000U) /*!< Frame transmitted/received in Classic CAN format */
#define FDCAN_NO_TX_EVENTS    					((uint32_t)0x00000000U) /*!< Do not store Tx events */

/* FDCAN_Tx_location  */
#define FDCAN_TX_BUFFER0  						((uint32_t)0x00000001U) /*!< Add message to Tx Buffer 0  */
#define FDCAN_TX_BUFFER1  						((uint32_t)0x00000002U) /*!< Add message to Tx Buffer 1  */
#define FDCAN_TX_BUFFER2  						((uint32_t)0x00000004U) /*!< Add message to Tx Buffer 2  */
#define FDCAN_TX_BUFFER3  						((uint32_t)0x00000008U) /*!< Add message to Tx Buffer 3  */
#define FDCAN_TX_BUFFER4  						((uint32_t)0x00000010U) /*!< Add message to Tx Buffer 4  */
#define FDCAN_TX_BUFFER5  						((uint32_t)0x00000020U) /*!< Add message to Tx Buffer 5  */
#define FDCAN_TX_BUFFER6  						((uint32_t)0x00000040U) /*!< Add message to Tx Buffer 6  */
#define FDCAN_TX_BUFFER7  						((uint32_t)0x00000080U) /*!< Add message to Tx Buffer 7  */
#define FDCAN_TX_BUFFER8  						((uint32_t)0x00000100U) /*!< Add message to Tx Buffer 8  */
#define FDCAN_TX_BUFFER9  						((uint32_t)0x00000200U) /*!< Add message to Tx Buffer 9  */
#define FDCAN_TX_BUFFER10 						((uint32_t)0x00000400U) /*!< Add message to Tx Buffer 10 */
#define FDCAN_TX_BUFFER11 						((uint32_t)0x00000800U) /*!< Add message to Tx Buffer 11 */
#define FDCAN_TX_BUFFER12 						((uint32_t)0x00001000U) /*!< Add message to Tx Buffer 12 */
#define FDCAN_TX_BUFFER13 						((uint32_t)0x00002000U) /*!< Add message to Tx Buffer 13 */
#define FDCAN_TX_BUFFER14 						((uint32_t)0x00004000U) /*!< Add message to Tx Buffer 14 */
#define FDCAN_TX_BUFFER15 						((uint32_t)0x00008000U) /*!< Add message to Tx Buffer 15 */
#define FDCAN_TX_BUFFER16 						((uint32_t)0x00010000U) /*!< Add message to Tx Buffer 16 */
#define FDCAN_TX_BUFFER17 						((uint32_t)0x00020000U) /*!< Add message to Tx Buffer 17 */
#define FDCAN_TX_BUFFER18 						((uint32_t)0x00040000U) /*!< Add message to Tx Buffer 18 */
#define FDCAN_TX_BUFFER19 						((uint32_t)0x00080000U) /*!< Add message to Tx Buffer 19 */
#define FDCAN_TX_BUFFER20 						((uint32_t)0x00100000U) /*!< Add message to Tx Buffer 20 */
#define FDCAN_TX_BUFFER21 						((uint32_t)0x00200000U) /*!< Add message to Tx Buffer 21 */
#define FDCAN_TX_BUFFER22 						((uint32_t)0x00400000U) /*!< Add message to Tx Buffer 22 */
#define FDCAN_TX_BUFFER23 						((uint32_t)0x00800000U) /*!< Add message to Tx Buffer 23 */
#define FDCAN_TX_BUFFER24 						((uint32_t)0x01000000U) /*!< Add message to Tx Buffer 24 */
#define FDCAN_TX_BUFFER25 						((uint32_t)0x02000000U) /*!< Add message to Tx Buffer 25 */
#define FDCAN_TX_BUFFER26 						((uint32_t)0x04000000U) /*!< Add message to Tx Buffer 26 */
#define FDCAN_TX_BUFFER27 						((uint32_t)0x08000000U) /*!< Add message to Tx Buffer 27 */
#define FDCAN_TX_BUFFER28 						((uint32_t)0x10000000U) /*!< Add message to Tx Buffer 28 */
#define FDCAN_TX_BUFFER29 						((uint32_t)0x20000000U) /*!< Add message to Tx Buffer 29 */
#define FDCAN_TX_BUFFER30 						((uint32_t)0x40000000U) /*!< Add message to Tx Buffer 30 */
#define FDCAN_TX_BUFFER31 						((uint32_t)0x80000000U) /*!< Add message to Tx Buffer 31 */

/* FDCAN_Rx_location */
#define FDCAN_RX_BUFFER0  						((uint32_t)0x00000000U) /*!< Get received message from Rx Buffer 0  */
#define FDCAN_RX_BUFFER1  						((uint32_t)0x00000001U) /*!< Get received message from Rx Buffer 1  */
#define FDCAN_RX_BUFFER2  						((uint32_t)0x00000002U) /*!< Get received message from Rx Buffer 2  */
#define FDCAN_RX_BUFFER3  						((uint32_t)0x00000003U) /*!< Get received message from Rx Buffer 3  */
#define FDCAN_RX_BUFFER4  						((uint32_t)0x00000004U) /*!< Get received message from Rx Buffer 4  */
#define FDCAN_RX_BUFFER5  						((uint32_t)0x00000005U) /*!< Get received message from Rx Buffer 5  */
#define FDCAN_RX_BUFFER6  						((uint32_t)0x00000006U) /*!< Get received message from Rx Buffer 6  */
#define FDCAN_RX_BUFFER7  						((uint32_t)0x00000007U) /*!< Get received message from Rx Buffer 7  */
#define FDCAN_RX_BUFFER8  						((uint32_t)0x00000008U) /*!< Get received message from Rx Buffer 8  */
#define FDCAN_RX_BUFFER9  						((uint32_t)0x00000009U) /*!< Get received message from Rx Buffer 9  */
#define FDCAN_RX_BUFFER10 						((uint32_t)0x0000000AU) /*!< Get received message from Rx Buffer 10 */
#define FDCAN_RX_BUFFER11 						((uint32_t)0x0000000BU) /*!< Get received message from Rx Buffer 11 */
#define FDCAN_RX_BUFFER12 						((uint32_t)0x0000000CU) /*!< Get received message from Rx Buffer 12 */
#define FDCAN_RX_BUFFER13 						((uint32_t)0x0000000DU) /*!< Get received message from Rx Buffer 13 */
#define FDCAN_RX_BUFFER14 						((uint32_t)0x0000000EU) /*!< Get received message from Rx Buffer 14 */
#define FDCAN_RX_BUFFER15 						((uint32_t)0x0000000FU) /*!< Get received message from Rx Buffer 15 */
#define FDCAN_RX_BUFFER16 						((uint32_t)0x00000010U) /*!< Get received message from Rx Buffer 16 */
#define FDCAN_RX_BUFFER17 						((uint32_t)0x00000011U) /*!< Get received message from Rx Buffer 17 */
#define FDCAN_RX_BUFFER18 						((uint32_t)0x00000012U) /*!< Get received message from Rx Buffer 18 */
#define FDCAN_RX_BUFFER19 						((uint32_t)0x00000013U) /*!< Get received message from Rx Buffer 19 */
#define FDCAN_RX_BUFFER20 						((uint32_t)0x00000014U) /*!< Get received message from Rx Buffer 20 */
#define FDCAN_RX_BUFFER21 						((uint32_t)0x00000015U) /*!< Get received message from Rx Buffer 21 */
#define FDCAN_RX_BUFFER22 						((uint32_t)0x00000016U) /*!< Get received message from Rx Buffer 22 */
#define FDCAN_RX_BUFFER23 						((uint32_t)0x00000017U) /*!< Get received message from Rx Buffer 23 */
#define FDCAN_RX_BUFFER24 						((uint32_t)0x00000018U) /*!< Get received message from Rx Buffer 24 */
#define FDCAN_RX_BUFFER25 						((uint32_t)0x00000019U) /*!< Get received message from Rx Buffer 25 */
#define FDCAN_RX_BUFFER26 						((uint32_t)0x0000001AU) /*!< Get received message from Rx Buffer 26 */
#define FDCAN_RX_BUFFER27 						((uint32_t)0x0000001BU) /*!< Get received message from Rx Buffer 27 */
#define FDCAN_RX_BUFFER28 						((uint32_t)0x0000001CU) /*!< Get received message from Rx Buffer 28 */
#define FDCAN_RX_BUFFER29 						((uint32_t)0x0000001DU) /*!< Get received message from Rx Buffer 29 */
#define FDCAN_RX_BUFFER30 						((uint32_t)0x0000001EU) /*!< Get received message from Rx Buffer 30 */
#define FDCAN_RX_BUFFER31 						((uint32_t)0x0000001FU) /*!< Get received message from Rx Buffer 31 */
#define FDCAN_RX_BUFFER32 						((uint32_t)0x00000020U) /*!< Get received message from Rx Buffer 32 */
#define FDCAN_RX_BUFFER33 						((uint32_t)0x00000021U) /*!< Get received message from Rx Buffer 33 */
#define FDCAN_RX_BUFFER34 						((uint32_t)0x00000022U) /*!< Get received message from Rx Buffer 34 */
#define FDCAN_RX_BUFFER35 						((uint32_t)0x00000023U) /*!< Get received message from Rx Buffer 35 */
#define FDCAN_RX_BUFFER36 						((uint32_t)0x00000024U) /*!< Get received message from Rx Buffer 36 */
#define FDCAN_RX_BUFFER37 						((uint32_t)0x00000025U) /*!< Get received message from Rx Buffer 37 */
#define FDCAN_RX_BUFFER38 						((uint32_t)0x00000026U) /*!< Get received message from Rx Buffer 38 */
#define FDCAN_RX_BUFFER39 						((uint32_t)0x00000027U) /*!< Get received message from Rx Buffer 39 */
#define FDCAN_RX_BUFFER40 						((uint32_t)0x00000028U) /*!< Get received message from Rx Buffer 40 */
#define FDCAN_RX_BUFFER41 						((uint32_t)0x00000029U) /*!< Get received message from Rx Buffer 41 */
#define FDCAN_RX_BUFFER42 						((uint32_t)0x0000002AU) /*!< Get received message from Rx Buffer 42 */
#define FDCAN_RX_BUFFER43 						((uint32_t)0x0000002BU) /*!< Get received message from Rx Buffer 43 */
#define FDCAN_RX_BUFFER44 						((uint32_t)0x0000002CU) /*!< Get received message from Rx Buffer 44 */
#define FDCAN_RX_BUFFER45 						((uint32_t)0x0000002DU) /*!< Get received message from Rx Buffer 45 */
#define FDCAN_RX_BUFFER46 						((uint32_t)0x0000002EU) /*!< Get received message from Rx Buffer 46 */
#define FDCAN_RX_BUFFER47 						((uint32_t)0x0000002FU) /*!< Get received message from Rx Buffer 47 */
#define FDCAN_RX_BUFFER48 						((uint32_t)0x00000030U) /*!< Get received message from Rx Buffer 48 */
#define FDCAN_RX_BUFFER49 						((uint32_t)0x00000031U) /*!< Get received message from Rx Buffer 49 */
#define FDCAN_RX_BUFFER50 						((uint32_t)0x00000032U) /*!< Get received message from Rx Buffer 50 */
#define FDCAN_RX_BUFFER51 						((uint32_t)0x00000033U) /*!< Get received message from Rx Buffer 51 */
#define FDCAN_RX_BUFFER52 						((uint32_t)0x00000034U) /*!< Get received message from Rx Buffer 52 */
#define FDCAN_RX_BUFFER53 						((uint32_t)0x00000035U) /*!< Get received message from Rx Buffer 53 */
#define FDCAN_RX_BUFFER54 						((uint32_t)0x00000036U) /*!< Get received message from Rx Buffer 54 */
#define FDCAN_RX_BUFFER55 						((uint32_t)0x00000037U) /*!< Get received message from Rx Buffer 55 */
#define FDCAN_RX_BUFFER56 						((uint32_t)0x00000038U) /*!< Get received message from Rx Buffer 56 */
#define FDCAN_RX_BUFFER57 						((uint32_t)0x00000039U) /*!< Get received message from Rx Buffer 57 */
#define FDCAN_RX_BUFFER58 						((uint32_t)0x0000003AU) /*!< Get received message from Rx Buffer 58 */
#define FDCAN_RX_BUFFER59 						((uint32_t)0x0000003BU) /*!< Get received message from Rx Buffer 59 */
#define FDCAN_RX_BUFFER60 						((uint32_t)0x0000003CU) /*!< Get received message from Rx Buffer 60 */
#define FDCAN_RX_BUFFER61 						((uint32_t)0x0000003DU) /*!< Get received message from Rx Buffer 61 */
#define FDCAN_RX_BUFFER62 						((uint32_t)0x0000003EU) /*!< Get received message from Rx Buffer 62 */
#define FDCAN_RX_BUFFER63 						((uint32_t)0x0000003FU) /*!< Get received message from Rx Buffer 63 */

#define FDCAN_STANDARD_ID 				 		((uint32_t)0x00000000U) /*!< Standard ID element */
#define FDCAN_FILTER_TO_RXBUFFER   				((uint32_t)0x00000007U) /*!< Store into Rx Buffer, configuration of FilterType ignored */
#define FDCAN_FILTER_DUAL          				((uint32_t)0x00000001U) /*!< Dual ID filter for FilterID1 or FilterID2 */


#define TxMsg_0x510_BUF_NUMBER 					FDCAN_TX_BUFFER1
#define TxMsg_0x520_BUF_NUMBER 					FDCAN_TX_BUFFER2
#define TxMsg_0x600_BUF_NUMBER 					FDCAN_TX_BUFFER3





/* TypeDefines ---------------------------------------------------------------*/

/**
  * @brief  FDCAN filter structure definition
  */
typedef struct
{
  uint32_t IdType;           /*!< Specifies the identifier type.
                                  This parameter can be a value of @ref FDCAN_id_type       */

  uint32_t FilterIndex;      /*!< Specifies the filter which will be initialized.
                                  This parameter must be a number between:
                                   - 0 and 127, if IdType is FDCAN_STANDARD_ID
                                   - 0 and 63, if IdType is FDCAN_EXTENDED_ID               */

  uint32_t FilterType;       /*!< Specifies the filter type.
                                  This parameter can be a value of @ref FDCAN_filter_type.
                                  The value FDCAN_EXT_FILTER_RANGE_NO_EIDM is permitted
                                  only when IdType is FDCAN_EXTENDED_ID.
                                  This parameter is ignored if FilterConfig is set to
                                  FDCAN_FILTER_TO_RXBUFFER                                  */

  uint32_t FilterConfig;     /*!< Specifies the filter configuration.
                                  This parameter can be a value of @ref FDCAN_filter_config */

  uint32_t FilterID1;        /*!< Specifies the filter identification 1.
                                  This parameter must be a number between:
                                   - 0 and 0x7FF, if IdType is FDCAN_STANDARD_ID
                                   - 0 and 0x1FFFFFFF, if IdType is FDCAN_EXTENDED_ID       */

  uint32_t FilterID2;        /*!< Specifies the filter identification 2.
                                  This parameter is ignored if FilterConfig is set to
                                  FDCAN_FILTER_TO_RXBUFFER.
                                  This parameter must be a number between:
                                   - 0 and 0x7FF, if IdType is FDCAN_STANDARD_ID
                                   - 0 and 0x1FFFFFFF, if IdType is FDCAN_EXTENDED_ID       */

  uint32_t RxBufferIndex;    /*!< Contains the index of the Rx buffer in which the
                                  matching message will be stored.
                                  This parameter must be a number between 0 and 63.
                                  This parameter is ignored if FilterConfig is different
                                  from FDCAN_FILTER_TO_RXBUFFER                             */

  uint32_t IsCalibrationMsg; /*!< Specifies whether the filter is configured for
                                  calibration messages.
                                  This parameter is ignored if FilterConfig is different
                                  from FDCAN_FILTER_TO_RXBUFFER.
                                  This parameter can be:
                                   - 0 : ordinary message
                                   - 1 : calibration message                                */

}FDCAN_FilterTypeDef;


/**
  * @brief  FDCAN Rx header structure definition
  */
typedef struct
{
  uint32_t Identifier;            /*!< Specifies the identifier.
                                       This parameter must be a number between:
                                        - 0 and 0x7FF, if IdType is FDCAN_STANDARD_ID
                                        - 0 and 0x1FFFFFFF, if IdType is FDCAN_EXTENDED_ID               */

  uint32_t IdType;                /*!< Specifies the identifier type of the received message.
                                       This parameter can be a value of @ref FDCAN_id_type               */

  uint32_t RxFrameType;           /*!< Specifies the the received message frame type.
                                       This parameter can be a value of @ref FDCAN_frame_type            */

  uint32_t DataLength;            /*!< Specifies the received frame length.
                                        This parameter can be a value of @ref FDCAN_data_length_code     */

  uint32_t ErrorStateIndicator;   /*!< Specifies the error state indicator.
                                       This parameter can be a value of @ref FDCAN_error_state_indicator */

  uint32_t BitRateSwitch;         /*!< Specifies whether the Rx frame is received with or without bit
                                       rate switching.
                                       This parameter can be a value of @ref FDCAN_bit_rate_switching    */

  uint32_t FDFormat;              /*!< Specifies whether the Rx frame is received in classic or FD
                                       format.
                                       This parameter can be a value of @ref FDCAN_format                */

  uint32_t RxTimestamp;           /*!< Specifies the timestamp counter value captured on start of frame
                                       reception.
                                       This parameter must be a number between 0 and 0xFFFF              */

  uint32_t FilterIndex;           /*!< Specifies the index of matching Rx acceptance filter element.
                                       This parameter must be a number between:
                                        - 0 and 127, if IdType is FDCAN_STANDARD_ID
                                        - 0 and 63, if IdType is FDCAN_EXTENDED_ID                       */

  uint32_t IsFilterMatchingFrame; /*!< Specifies whether the accepted frame did not match any Rx filter.
                                         Acceptance of non-matching frames may be enabled via
                                         HAL_FDCAN_ConfigGlobalFilter().
                                         This parameter can be 0 or 1                                    */

}FDCAN_RxHeaderTypeDef;



/**
  * @brief  FDCAN Tx header structure definition
  */
typedef struct 
{
  uint32_t Identifier;          /*!< Specifies the identifier.
                                     This parameter must be a number between:
                                      - 0 and 0x7FF, if IdType is FDCAN_STANDARD_ID
                                      - 0 and 0x1FFFFFFF, if IdType is FDCAN_EXTENDED_ID               */

  uint32_t IdType;              /*!< Specifies the identifier type for the message that will be
                                     transmitted.
                                     This parameter can be a value of @ref FDCAN_id_type               */

  uint32_t TxFrameType;         /*!< Specifies the frame type of the message that will be transmitted.
                                     This parameter can be a value of @ref FDCAN_frame_type            */

  uint32_t DataLength;          /*!< Specifies the length of the frame that will be transmitted.
                                      This parameter can be a value of @ref FDCAN_data_length_code     */

  uint32_t ErrorStateIndicator; /*!< Specifies the error state indicator.
                                     This parameter can be a value of @ref FDCAN_error_state_indicator */

  uint32_t BitRateSwitch;       /*!< Specifies whether the Tx frame will be transmitted with or without
                                     bit rate switching.
                                     This parameter can be a value of @ref FDCAN_bit_rate_switching    */

  uint32_t FDFormat;            /*!< Specifies whether the Tx frame will be transmitted in classic or
                                     FD format.
                                     This parameter can be a value of @ref FDCAN_format                */

  uint32_t TxEventFifoControl;  /*!< Specifies the event FIFO control.
                                     This parameter can be a value of @ref FDCAN_EFC                   */

  uint32_t MessageMarker;       /*!< Specifies the message marker to be copied into Tx Event FIFO
                                     element for identification of Tx message status.
                                     This parameter must be a number between 0 and 0xFF                */

}FDCAN_TxHeaderTypeDef;







/* CAN Msg structures ----------------- */


typedef struct typeDefCanMessage
{
	uint8_t data[8];
	uint16_t onetime_transmit;
	uint16_t always_transmit;
}typeDefCanMessage;






/* Functions -----------------------------------------------------------------*/

uint16_t InitCAN1();

void RxFilterRegisterConfig (FDCAN_FilterTypeDef *pRxFilter);


void ReceiveCanMsg (uint32_t RxLocation, uint8_t *pRxData, uint16_t CanModule);
void FDCAN_SendMessage(FDCAN_TxHeaderTypeDef *pTxHeader, uint8_t *pTxData, uint32_t BufferIndex, uint16_t CanModule);


#endif /* CAN_H_IFND */


