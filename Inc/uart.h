/*----------------------------------------------------------------------------*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef UART_H_IFND
#define UART_H_IFND


/* Includes ------------------------------------------------------------------*/

#include "stm32h7xx.h"      
#include "can.h"  

/* Defines -------------------------------------------------------------------*/

#define UART_TIMEOUT 								(uint32_t)0x00FF0000
#define UART_STATUS_OK								(uint16_t)0x00
#define UART_STATUS_ERROR							(uint16_t)0x01

/* inp freq 36MHz, baudrate 57600
 * DIV = 36000000/57600 = 625
*/
#define UART2_BRR									(uint32_t)625

#define MSG_TO_DISPLAY_DLC							(uint8_t)12
#define UART_MARKER									(uint8_t)0xC0
#define UART_REPLACE_BYTE0							(uint8_t)0xDB
#define UART_REPLACE_BYTE1							(uint8_t)0xDC
#define UART_REPLACE_BYTE2							(uint8_t)0xDD

#define UART_DLC_MSG_OBSERVE						(uint8_t)27

#define GPIO_AFR_AF7_UART							(0x7U)


//#define KOEF_POS_HOR								(double)0.000021457
													/* Position horizont: +-180 degrees, we send with
														resolution 1 sign after point => +-1800;
														32767 - 1800
														1   -  x   */
#define KOEF_POS_TO_DISPLAY							(double)0.005493
	



/* TypeDefines ---------------------------------------------------------------*/

/* UART Msgs to & from Display structures ---------------- */

typedef union  {
	uint8_t all;
	struct   {      
		uint8_t      LED1_ON:1;		
		uint8_t      LED2_ON:1;
		uint8_t      LED3_ON:1;
		uint8_t      LED4_ON:1;
		uint8_t      LED5_ON:1;
		uint8_t      LED6_ON:1;
		uint8_t      LED7_ON:1;
		uint8_t      LED8_ON:1;		
	} BITS;
}LedBtn1_8_Byte;


typedef union  {
	uint8_t all;
	struct   {      
		uint8_t      LED9_ON:1;		
		uint8_t      LED10_ON:1;
		uint8_t      LED11_ON:1;
		uint8_t      Reserve:5;		
	} BITS;
}LedBtn9_11_Byte;


typedef union  {
	uint8_t all;
	struct   {      
		uint8_t      RESERVE:1;		
		uint8_t      SET_SETTINGS:1;
	} BITS;
}StateDispl_Byte;


typedef struct  
{    
	StateDispl_Byte			State;
	uint8_t							Highlight;
	uint8_t							Brightnes;
	uint8_t							Contrast;
	uint8_t							Color;
	LedBtn1_8_Byte			LED1_8;
	LedBtn9_11_Byte			LED9_11;
	uint8_t							ErrorCode;
	uint8_t							CRCH;
	uint8_t							CRCL;
}UART_MsgToDisplay_TypeDef;


typedef union  {
	uint8_t all;
	struct   {      
		uint8_t      BUTTON9:1;		
		uint8_t      BUTTON10:1;
		uint8_t      BUTTON11:1;		
		uint8_t      BUTTON_UN543_SCALE:1;	
		uint8_t      BUTTON_UN543_AUTOTRACKING:1;
		uint8_t      BUTTON_UN543_RANGEFINDER:1;	
		uint8_t      BUTTON_UN543_JOYSTICK:1;
		uint8_t      BUTTON_DRUM_RETURN:1;	
	} BITS;
}UART_Btn9_15_Byte;



typedef struct  
{    
	StateDispl_Byte			State;
	uint8_t							Highlight;
	uint8_t							Buttons1_8a;
	uint8_t							Buttons1_8b;
	UART_Btn9_15_Byte		Buttons9_11a;
	UART_Btn9_15_Byte		Buttons9_11b;
	uint8_t							Brightnes;
	uint8_t							Contrast;
	uint8_t							Color;
	LedBtn1_8_Byte			LED1_8;
	LedBtn9_11_Byte			LED9_11;
	uint8_t							ErrorCode;
	uint8_t							CRCH;
	uint8_t							CRCL;
}UART_MsgFromDisplay_TypeDef;


/* UART Msgs to Operator structures ---------------------- */

typedef union  {
	uint8_t all;
	struct   {      
		uint8_t      VIDEO_SOURCE:3;				
		uint8_t      TERMOVIS_PWR_ON:1;	
		uint8_t      AUTO_TRACKING_ON:1;
		uint8_t      DISTANCE_MEASUREMENT:1;	
		uint8_t      AUTO_SHOT:1;	
		uint8_t      TERMOVIS_READY:1;
	} BITS;
}UART_OperatorState_Byte;


typedef union  {
	uint8_t all;
	struct   {      
		uint8_t      GUIDANCE_MODE:1;				
		uint8_t      EJECTION_CONFIRM:1;	
		uint8_t      LASER_EMISSION:1;
		uint8_t      FIRE_READY:1;
		uint8_t      FIRE_RQ:1;
		uint8_t      LASER_OFF_RQ:1;
		uint8_t      RETURN_MISSILE_TO_DRUM_RQ:1;
		uint8_t      MISSILE_DROP_OUT_RQ:1;
	} BITS;
}UART_OperatorSubmode_Byte;


typedef union  {
	uint8_t all;
	struct   {      		
		uint8_t      PWM:5;
		uint8_t      VERIF_OPSN:1;
		uint8_t      SAVE_VERIF:1;	
		uint8_t      VERIF_LAUNCHER:1;		
	} BITS;
}UART_OperatorVerif_Byte;


typedef union  {
	uint8_t all;
	struct   {      
		uint8_t      POS1:2;	
		uint8_t      POS2:2;
		uint8_t      POS3:2;
		uint8_t      POS4:2;
	} BITS;
}UART_Missile1_4_Byte;


typedef union  {
	uint8_t all;
	struct   {      
		uint8_t      POS5:2;
		uint8_t      POS6:2;
		uint8_t      POS7:2;
		uint8_t      POS8:2;
	} BITS;
}UART_Missile5_8_Byte;


typedef union  {
	uint8_t all;
	struct   {      
		uint8_t      POS9:2;	
		uint8_t      POS10:2;
		uint8_t      POS11:2;
		uint8_t      POS12:2;
	} BITS;
}UART_Missile9_12_Byte;


typedef union  {
	uint8_t all;
	struct   {      
		uint8_t      POS13:2;	
		uint8_t      RESERVE:6;
	} BITS;
}UART_Missile13_Byte;


typedef union  {
	uint8_t all;
	struct   {      
		uint8_t      POS_NUMBER:4;	
		uint8_t      DRUM_RETURN_RQ:1;
		uint8_t      DRUM_ON_POS:1;
		//uint8_t      RETURN_IS_ACTIVE:1;
		uint8_t      RESERVE:2;
	} BITS;
}UART_Drum_Byte;


typedef union  {
	uint8_t all;
	struct   {      
		uint8_t      SYSTEM_MODE:4;	
		uint8_t      PROGRAM_MODE:1;
		uint8_t      STABILIZATION:1;
		uint8_t      INVERSION:1;
		uint8_t      MODE_READY:1;
	} BITS;
}UART_SystemMode_Byte;


typedef union  {
	uint8_t all;
	struct   {      
		uint8_t      AUTO_GAIN:1;	
		uint8_t      FLIP:2;						// 0-normal, 1-horiz, 2- vert, 3- horiz+vert
		uint8_t      MARK:2;
		uint8_t      OSD:1;
		uint8_t      RESERVE:2;
	} BITS;
}UART_TermoFlip_Byte;


typedef union  {
	uint8_t all;
	struct   {      
		uint8_t      SHARPNESS:4;	
		uint8_t      EDGE:4;						
	} BITS;
}UART_TermoSharp_Byte;


typedef union  {
	uint8_t all;
	struct   {      
		uint8_t      TrapdoorOpened:1;	
		uint8_t      TrapdoorClosed:1;
		uint8_t      LauncherLiftedUP:1;
		uint8_t      LauncherLiftedDOWN:1;
		uint8_t      MissileCaptured:1;
		uint8_t      MissileDroppedOut:1;
		uint8_t      DrumReady:1;
		uint8_t      LauncherParkingState:1;		
	} BITS;
}UART_VehDriverState_Byte;



typedef union  {
	uint8_t all;
	struct   {      
		uint8_t      ErrorIndex:4;	
		uint8_t      ErrorField1:1;
		uint8_t      ErrorField2:1;
		uint8_t      ErrorField3:1;
		uint8_t      ErrorField4:1;		
	} BITS;
}UART_Error1_Byte;



typedef union  {
	uint8_t all;
	struct   {      
		uint8_t      ErrorField5:1;
		uint8_t      ErrorField6:1;
		uint8_t      ErrorField7:1;
		uint8_t      ErrorField8:1;
		uint8_t      ErrorField9:1;
		uint8_t      ErrorField10:1;
		uint8_t      ErrorField11:1;
		uint8_t      ErrorField12:1;
	} BITS;
}UART_Error2_Byte;




typedef struct  
{    
	uint8_t											Counter;
	UART_SystemMode_Byte				SystemMode;
	UART_OperatorState_Byte			State;
	UART_Error1_Byte						ErrorCount;
	UART_Error2_Byte						ErrorCode;
	uint8_t											Buttons1_8;
	UART_Btn9_15_Byte						Buttons9_11;
	uint8_t											PosHoriz_MSB;
	uint8_t											PosHoriz_LSB;
	uint8_t											PosVert_MSB;
	uint8_t											PosVert_LSB;
	UART_Missile1_4_Byte				MissilePresence1_4;
	UART_Missile5_8_Byte				MissilePresence5_8;
	UART_Missile9_12_Byte				MissilePresence9_12;
	UART_Missile13_Byte					MissilePresence13;
	uint8_t											Distance_MSB;
	uint8_t											Distance_LSB;
	uint8_t											WindSpeed;
	uint8_t											CRCH;
	uint8_t											CRCL;
	
	UART_OperatorSubmode_Byte		Submode;
	uint8_t											MissileType;
	
	uint8_t											JoystickHoriz_MSB;
	uint8_t											JoystickHoriz_LSB;
	uint8_t											JoystickVert_MSB;
	uint8_t											JoystickVert_LSB;
	uint8_t											MonitorBrightness;
	uint8_t											MonitorContrast;
	uint8_t											MonitorColor;
	uint8_t											MonitorHighlight;
	uint8_t											TermovisBrightness;
	uint8_t											TermovisContrast;
	UART_TermoFlip_Byte					TermovisFlip;
	uint8_t											TermovisLinseTemp;
	uint8_t											TermovisBodyTemp;
	UART_TermoSharp_Byte				TermovisSharpness;
	uint8_t											TermovisVideoMode;
	
	UART_VehDriverState_Byte		VehicleDriversState;
	//UART_OperatorVerif_Byte			Verification;
	
	UART_Drum_Byte							DrumPosition;
	
}UART_MsgToOperator_TypeDef;





/* UART Msgs from Operator structures -------------------- */

typedef union  {
	uint8_t all;
	struct   {      
		uint8_t      TrapdoorOpenRQ:1;	
		uint8_t      TrapdoorCloseRQ:1;
		uint8_t      LauncherLiftUpRQ:1;
		uint8_t      LauncherLiftDownRQ:1;
		uint8_t      MissileCaptureRQ:1;
		uint8_t      MissileDropOutRQ:1;
		uint8_t      DrumReturnRQ:1;
		uint8_t      LauncherParkingRQ:1;		
	} BITS;
}UART_VehDriverControl_Byte;


typedef union  {
	uint8_t all;
	struct   {      
		uint8_t      ValueMSB:7;	
		uint8_t      Active:1;		
	} BITS;
}UART_AutoTrack_Byte;


typedef struct  
{    
	uint8_t											Counter;
	UART_SystemMode_Byte				SystemMode;
	UART_OperatorState_Byte			State;
	UART_AutoTrack_Byte					AutoTrackingX_MSB;
	uint8_t											AutoTrackingX_LSB;
	UART_AutoTrack_Byte					AutoTrackingY_MSB;
	uint8_t											AutoTrackingY_LSB;
	uint8_t											CameraPreset;
	uint8_t											Distance_MSB;
	uint8_t											Distance_LSB;
	uint8_t											ErrorCount;
	uint8_t											ErrorCode;
	uint8_t											CRCH;
	uint8_t											CRCL;
	
	UART_OperatorSubmode_Byte		Submode;
	UART_Drum_Byte							DrumPosition;
	
	uint8_t											ParameterNumber;
	uint8_t											ParameterValue;
	UART_OperatorVerif_Byte			Verification;
	
	uint8_t											StopZoomMSB;
	uint8_t											StopZoomLSB;
	UART_VehDriverControl_Byte	VehicleDriversControl;
	
}UART_MsgFromOperator_TypeDef;




/* Functions -----------------------------------------------------------------*/

uint16_t InitUART2 (void);


uint16_t InitDMA1_Stream0_UART2_Tx (void);

void USART2_Send_Char(char chr);
void USART2_Send_Str(char* str);
void USART2_Send_arBytes(uint8_t* arData);

uint8_t CheckForUartTxByteReplacement (uint8_t dataByte, uint8_t curInd, uint8_t* arSendingData);
void CalculateCRC (void);

	

#endif /* UART_H_IFND */



