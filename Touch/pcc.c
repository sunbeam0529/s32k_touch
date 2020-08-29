/****************************************************************************//*!
*
* @file     timer.c
*
* @version  1.0.0.0
*
* @date     September-2017
*
* @brief    PCC routines
*
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include "S32K118.h"
#include "pcc.h"
#include "main.h"
#include "power_mode.h"
#include "ets.h"

/*****************************************************************************
*
* Function: void PCC_Init(uint8_t clkMode)
*
* Description: Init PCC
*
*****************************************************************************/
void PCC_Init(uint8_t clkMode)
{
    // PORTA 
    PCC->PCCn[PCC_PORTA_INDEX] = PCC_PCCn_CGC_MASK;
    // PORTB 
    PCC->PCCn[PCC_PORTB_INDEX] = PCC_PCCn_CGC_MASK;
    // PORTC 
    PCC->PCCn[PCC_PORTC_INDEX] = PCC_PCCn_CGC_MASK;
    // PORTD 
    PCC->PCCn[PCC_PORTD_INDEX] = PCC_PCCn_CGC_MASK;
    // PORTE 
    PCC->PCCn[PCC_PORTE_INDEX] = PCC_PCCn_CGC_MASK;


    // DMAMUX
    PCC->PCCn[PCC_DMAMUX_INDEX] |= PCC_PCCn_CGC_MASK; 		// clock enable

    // Configure PCC based on selected clock mode
    switch (clkMode)
    {
    	case (RUN_FIRC):
		{
    	    // LPTMR0

    		if(PCC->PCCn[PCC_LPTMR0_INDEX] & 0x40000000){PCC->PCCn[PCC_LPTMR0_INDEX]  = 0x80000000;}
    		PCC->PCCn[PCC_LPTMR0_INDEX] = 0xC3000001;		// enable clock, use FIRCDIV2 clock,  clock division 2

    		//PCC->PCCn[PCC_LPTMR0_INDEX] |= PCC_PCCn_CGC_MASK | PCC_PCCn_PCS(3);
    	    // ADC0
    	    if(PCC->PCCn[PCC_ADC0_INDEX] & 0x40000000){PCC->PCCn[PCC_ADC0_INDEX]  = 0x80000000;}
    	    PCC->PCCn[PCC_ADC0_INDEX] = 0xC3000000;			// enable clock, use FIRCDIV2 clock, no clock division

    	    //LPUART1

/*
    	    // FTM2
    	    if(PCC->PCCn[PCC_FTM2_INDEX] & 0x40000000){PCC->PCCn[PCC_FTM2_INDEX]  = 0x80000000;}
    		PCC->PCCn[PCC_FTM2_INDEX] = 0xC0000000;         // Enable clock, no clock division
*/
/*

    		//LPIT
    		if(PCC->PCCn[PCC_LPIT_INDEX] & 0x40000000){PCC->PCCn[PCC_LPIT_INDEX]  = 0x80000000;}
    		PCC->PCCn[PCC_LPIT_INDEX] = 0xC3000000;		// enable clock, use FIRCDIV2 clock, no clock division
*/
#if(LOW_POWER_MODE == LPM_DISABLE)
    	    // LPUART0
    	    PCC->PCCn[PCC_PORTC_INDEX] = PCC_PCCn_CGC_MASK;
    	    PCC->PCCn[PCC_LPUART0_INDEX] = 0xC3000000;		// enable clock, use FIRCDIV2 clock, no clock division
    	    PCC->PCCn[PCC_LPUART1_INDEX] = 0xC3000000;		// enable clock, use FIRCDIV2 clock, no clock division
#endif

    		break;
		}
    	case (RUN_PLL):
		{
    	    // LPTMR0
    		if(PCC->PCCn[PCC_LPTMR0_INDEX] & 0x40000000){PCC->PCCn[PCC_LPTMR0_INDEX]  = 0x80000000;}
    		PCC->PCCn[PCC_LPTMR0_INDEX] = 0xC6000000;		// enable clock, use SPLLDIV2 clock, no clock division

    	    // ADC0
    	    if(PCC->PCCn[PCC_ADC0_INDEX] & 0x40000000){PCC->PCCn[PCC_ADC0_INDEX]  = 0x80000000;}
    	    PCC->PCCn[PCC_ADC0_INDEX] = 0xC6000000;			// enable clock, use SPLLDIV2 clock, no clock division

/*
    	    // FTM2
    	    if(PCC->PCCn[PCC_FTM2_INDEX] & 0x40000000){PCC->PCCn[PCC_FTM2_INDEX]  = 0x80000000;}
    		PCC->PCCn[PCC_FTM2_INDEX] = 0xC0000000;         // Enable clock, no clock division
*/

#if FREQUENCY_HOPPING
    		//LPIT
    		if(PCC->PCCn[PCC_LPIT_INDEX] & 0x40000000){PCC->PCCn[PCC_LPIT_INDEX]  = 0x80000000;}
    		PCC->PCCn[PCC_LPIT_INDEX] = 0xC6000000;		// enable clock, use SPLLDIV2 clock, no clock division
#endif
#if(LOW_POWER_MODE == LPM_DISABLE)
    	    // LPUART0
    	    PCC->PCCn[PCC_PORTC_INDEX] = PCC_PCCn_CGC_MASK;
    	    PCC->PCCn[PCC_LPUART0_INDEX] = 0xC6000000;		// enable clock, use SPLLDIV2 clock, no clock division
#endif

    		break;
		}
    }
}
