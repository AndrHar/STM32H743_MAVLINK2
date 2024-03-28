/**
  ******************************************************************************
  * @file           : rcc.c
  * @brief          : RCC configuration for STM32H743
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "rcc.h"


/* Functions -----------------------------------------------------------------*/


/* ---------------------------- RCC_Init -------------------------------------*/
void RCC_Init (void)
{
		
	RCC->CR |= RCC_CR_HSEON; 								 // Enable HSE
	while (!(RCC->CR & RCC_CR_HSERDY)){};					 // Wait HSE Ready
		
	/* Configure the main PLL clock source, multiplication and division factors */
	MODIFY_REG(RCC->PLLCKSELR, (RCC_PLLCKSELR_PLLSRC | RCC_PLLCKSELR_DIVM1) , ((RCC_PLLSOURCE_HSE) | ( (PLLM1) <<4U)));  \
    WRITE_REG (RCC->PLL1DIVR , ( ((PLLN1 - 1U )& RCC_PLL1DIVR_N1) | (((PLLP1 -1U ) << 9U) & RCC_PLL1DIVR_P1) | \
                                (((PLLQ1 -1U) << 16U)& RCC_PLL1DIVR_Q1) | (((PLLR1 - 1U) << 24U)& RCC_PLL1DIVR_R1)));
		
	/* Configure PLL  PLL1FRACN */
    MODIFY_REG(RCC->PLL1FRACR, RCC_PLL1FRACR_FRACN1, (uint32_t)(0) << POSITION_VAL(RCC_PLL1FRACR_FRACN1));

    /* Select PLL1 input reference frequency range: VCI */ 
    MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLL1RGE, (RCC_PLLCFGR_PLL1RGE_2)) ;

    /* Select PLL1 output frequency range : VCO */
    MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLL1VCOSEL, (RCC_PLL1VCOWIDE)) ;

    /* Enable PLL System Clock output */
    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_DIVP1EN);

    /* Enable PLL1Q Clock output */
    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_DIVQ1EN);
 
    /* Enable PLL1R  Clock output */
    //SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_DIVR1EN);

    /* Enable PLL1FRACN */
    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL1FRACEN);
		
    /* Enable the main PLL1 */
    SET_BIT(RCC->CR, RCC_CR_PLL1ON);
    while((RCC->CR & RCC_CR_PLL1RDY) == 0){};    			// wait till PLL is ready


	#if defined(PLL2)
	/* Configure the main PLL2 clock source, multiplication and division factors. */
	MODIFY_REG(RCC->PLLCKSELR, (RCC_PLLCKSELR_PLLSRC | RCC_PLLCKSELR_DIVM2) , ((RCC_PLLSOURCE_HSE) | ( (PLLM2) <<12U)));  \
    WRITE_REG (RCC->PLL2DIVR , ( ((PLLN2 - 1U )& RCC_PLL2DIVR_N2) | (((PLLP2 -1U ) << 9U) & RCC_PLL2DIVR_P2) | \
                               (((PLLQ2 -1U) << 16U)& RCC_PLL2DIVR_Q2) | (((PLLR2 - 1U) << 24U)& RCC_PLL2DIVR_R2)));
		
	/* Configure PLL  PLL2FRACN */
    MODIFY_REG(RCC->PLL2FRACR, RCC_PLL2FRACR_FRACN2, (uint32_t)(0) << POSITION_VAL(RCC_PLL2FRACR_FRACN2));

    /* Select PLL2 input reference frequency range: VCI */ 
    MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLL2RGE, (RCC_PLLCFGR_PLL2RGE_2));

    /* Select PLL2 output frequency range : VCO */
    MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLL2VCOSEL, (RCC_PLL2VCOWIDE));

    /* Enable PLL2 System Clock output */
    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_DIVP2EN);

    /* Enable PLL2Q Clock output */ /*uncomment when need to use*/
    //SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_DIVQ2EN);

    /* Enable PLL2R  Clock output */ /*uncomment when need to use*/
    //SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_DIVR2EN);

    /* Enable PLL2FRACN */
    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL2FRACEN);
		
	/* Enable the PLL2 */
    SET_BIT(RCC->CR, RCC_CR_PLL2ON);
	while((RCC->CR & RCC_CR_PLL2RDY) == 0){};    // wait till PLL2 is ready

	#endif /* #if defined(PLL2) */


	/* System clock Switch configuration */
	MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL1); 			/* PLL1 selection as system clock */
	while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL1){}; 		// wait till CFGR is set

	/* D1CPRE configuration (Domain 1 Core prescaler) */
	MODIFY_REG(RCC->D1CFGR, RCC_D1CFGR_D1CPRE, D1CPRE_PRESC);
	while(((RCC->D1CFGR & RCC_D1CFGR_D1CPRE) != D1CPRE_PRESC)){};   // wait till D1CFGR is set

	/* D1HPRE configuration (AHB3 prescaler) */
    MODIFY_REG(RCC->D1CFGR, RCC_D1CFGR_HPRE, HPRE_PRESC);


    /* D1PPRE Configuration (APB3 prescaler) */
    MODIFY_REG(RCC->D1CFGR, RCC_D1CFGR_D1PPRE, D1PPRE_PRESC);

    /* D2PPRE1 configuration (APB1 prescaler) */
    MODIFY_REG(RCC->D2CFGR, RCC_D2CFGR_D2PPRE1, D2PPRE1_PRESC);

    /* D2PPRE2 configuration (APB2 prescaler) */
    MODIFY_REG(RCC->D2CFGR, RCC_D2CFGR_D2PPRE2, D2PPRE2_PRESC);

    /* D3PPRE configuration (APB4 prescaler) */
    MODIFY_REG(RCC->D3CFGR, RCC_D3CFGR_D3PPRE, D3PPRE_PRESC);


    /* Update the SystemCoreClock global variable */
	SystemCoreClockUpdate();

}
/* -------------------------- End RCC_Init -----------------------------------*/


