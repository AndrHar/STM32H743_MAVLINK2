/*----------------------------------------------------------------------------*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef TIMER_H_IFND
#define TIMER_H_IFND


/* Includes ------------------------------------------------------------------*/

#include "stm32h7xx.h"


/* Defines -------------------------------------------------------------------*/

#define TIMER_FL_1MS_Pos		(0U)
#define TIMER_FL_1MS_Msk		(0x1U << TIMER_FL_1MS_Pos)
#define TIMER_FL_1MS			TIMER_FL_1MS_Msk
#define TIMER_FL_100MS_Pos		(1U)
#define TIMER_FL_100MS_Msk		(0x1U << TIMER_FL_100MS_Pos)
#define TIMER_FL_100MS			TIMER_FL_100MS_Msk
#define TIMER_FL_500MS_Pos		(2U)
#define TIMER_FL_500MS_Msk		(0x1U << TIMER_FL_500MS_Pos)
#define TIMER_FL_500MS			TIMER_FL_500MS_Msk
#define TIMER_FL_1S_Pos			(3U)
#define TIMER_FL_1S_Msk			(0x1U << TIMER_FL_1S_Pos)
#define TIMER_FL_1S				TIMER_FL_1S_Msk


typedef union  {
	uint32_t all;
	struct   {
		uint8_t flag_1ms:1;
		uint8_t flag_100ms:1;
		uint8_t flag_500ms:1;
		uint8_t flag_1s:1;
	}FLAGS;
}timerTypeDef;


/* Functions -----------------------------------------------------------------*/

void TimerInit (uint32_t Frequency_Hz);
void TimerStart (void);
timerTypeDef TimerGet (void);
void TimerResetFlag (uint32_t flagPos);
void SysTick_Handler (void);

#endif /* TIMER_H_IFND */


