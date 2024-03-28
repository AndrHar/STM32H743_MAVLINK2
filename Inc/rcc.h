/*----------------------------------------------------------------------------*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef RCC_H_IFND
#define RCC_H_IFND

/* Includes ------------------------------------------------------------------*/

#include "stm32h7xx.h"

/* Defines -------------------------------------------------------------------*/

/* coefficients for 72MHz clock */
#define HSE_VALUE    				(8000000U) 	/*!< Value of the External oscillator in Hz */
#define RCC_PLLSOURCE_HSE           (2U)
#define PLLM1  						(8U)
#define	PLLN1  						(288U)
#define	PLLP1  						(4U)
#define	PLLQ1  						(9U)
#define	PLLR1   					(2U)
#define RCC_PLL1VCOWIDE             (0U)

/* Uncomment if PLL2 is used */
//#define PLL2

#if defined(PLL2)
	#define PLLM2					(5U)
	#define	PLLN2  					(60U)
	#define	PLLP2  					(5U)
	#define	PLLQ2  					(2U)
	#define	PLLR2   				(2U)
	#define RCC_PLL2VCOWIDE         (0U)
#endif

#define D1CPRE_PRESC                RCC_D1CFGR_D1CPRE_DIV1
#define HPRE_PRESC                  RCC_D1CFGR_HPRE_DIV2
#define D1PPRE_PRESC                RCC_D1CFGR_D1PPRE_DIV1
#define D2PPRE1_PRESC               RCC_D2CFGR_D2PPRE1_DIV1
#define D2PPRE2_PRESC               RCC_D2CFGR_D2PPRE2_DIV1
#define D3PPRE_PRESC                RCC_D3CFGR_D3PPRE_DIV1


/* Functions -----------------------------------------------------------------*/

void RCC_Init (void);


#endif /* RCC_H_IFND */


