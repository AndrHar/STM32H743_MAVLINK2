/**
  ******************************************************************************
  * @file           : timer.c
  * @brief          : SysTick timer configuration for STM32H743
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/

#include "timer.h"


/* Defines -------------------------------------------------------------------*/

#define BIT_SET		((uint8_t)1)
#define BIT_CLEAR	((uint8_t)0)


/* Variables ----------------------------------------------------------------*/

static uint32_t cnt_1sec = 0; 	//counter for 1 second detection
static uint32_t cnt_100ms = 0; 	//counter for 100 msecond detection
timerTypeDef Timer;


/* Functions -----------------------------------------------------------------*/

/* ---------------------------- TimerInit ------------------------------------*/
void TimerInit (uint32_t Frequency_Hz)
{
	/* Set the systick's registers into the default value. */
	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;

	/* configure system timer */
	SysTick->LOAD = SystemCoreClock/Frequency_Hz - 1;
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;   //interrupt is on

	/* SysTick not enabled there */
}

/* -------------------------- End TimerInit ----------------------------------*/


/* ---------------------------- TimerStart -----------------------------------*/
void TimerStart (void)
{
	/* start systick counter */
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}
/* -------------------------- End TimerStart ---------------------------------*/



/* ------------------------- SysTick_Handler ---------------------------------*/
void SysTick_Handler (void)
{
	/* ticks 1 ms*/

	cnt_1sec++;
	cnt_100ms++;

	Timer.FLAGS.flag_1ms = BIT_SET;


	if (cnt_1sec > 999){
		Timer.FLAGS.flag_1s = BIT_SET;
		cnt_1sec = 0;
	}

	if (cnt_100ms > 99){
			Timer.FLAGS.flag_100ms = BIT_SET;
			cnt_100ms = 0;
		}

}
/* ----------------------- End SysTick_Handler -------------------------------*/



/* ---------------------------- TimerGet -------------------------------------*/
timerTypeDef TimerGet (void)
{

	return Timer;
}
/* -------------------------- End TimerGet -----------------------------------*/



/* ------------------------- TimerResetFlag ----------------------------------*/
void TimerResetFlag (uint32_t flagPos)
{

	Timer.all &= ~flagPos;
}
/* ----------------------- End TimerResetFlag --------------------------------*/




