/*----------------------------------------------------------------------------*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MAIN_H_IFND
#define MAIN_H_IFND


/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx.h"
#include "rcc.h"
#include "timer.h"
#include "flash.h"
#include "can.h"
#include "uart.h"

/* use mavlink.h specific to your autopilot*/
#include "ardupilotmega/mavlink.h"


/* Defines -------------------------------------------------------------------*/

// LEDs definitions ------------------------------------------------
#define LED1_ON()              		(GPIOB->ODR |=  GPIO_ODR_ODR_0)
#define LED1_OFF()              	(GPIOB->ODR &= ~GPIO_ODR_ODR_0)
#define LED1_TOOGLE()              	(GPIOB->ODR ^=  GPIO_ODR_ODR_0)
#define LED2_ON()              		(GPIOB->ODR |=  GPIO_ODR_ODR_7)
#define LED2_OFF()              	(GPIOB->ODR &= ~GPIO_ODR_ODR_7)
#define LED2_TOOGLE()              	(GPIOB->ODR ^=  GPIO_ODR_ODR_7)
#define LED3_ON()              		(GPIOB->ODR |=  GPIO_ODR_ODR_14)
#define LED3_OFF()              	(GPIOB->ODR &= ~GPIO_ODR_ODR_14)
#define LED3_TOOGLE()              	(GPIOB->ODR ^=  GPIO_ODR_ODR_14)



/* Functions -----------------------------------------------------------------*/

void CheckRxMessageCAN1 (void);
enum FLASH_STATUS ChangeBootloaderDelay(uint32_t delay);

void InitLEDs(void);
enum FLASH_STATUS WriteBoardIdToFlash(void);

void Tick_1ms(void);
void Tick_100ms(void);
void Tick_1sec(void);

void CheckUart2RxMsg (void);
void SendUart_DMA(uint8_t txbuf[], const uint32_t length);
void MAV_Send_HEARTBEAT();
void MAV_Request_PARAM_READ(const char* param_id);
void MAV_Request_MSG( uint16_t msg_id, int32_t interval_us);

void MAV_ParseFrame (uint8_t byte);

void Config_CAN_Filters(void);

#endif /* MAIN_H_IFND */
