/****************************************************************************//*!
*
* @file     lpuart.c
*
* @version  1.0.0.0
*
* @date     September-2017
*
* @brief    LPUART1 routines
*
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include "S32K118.h"
#include "lpuart.h"
#include "scg.h"
#include "main.h"
#include "power_mode.h"

/*****************************************************************************
*
* Function: void LPUART0_Init(uint8_t clkMode)
*
* Description: Init LPUART0
*
*****************************************************************************/
void LPUART0_Init(uint8_t clkMode)
{
	// Configure LPUART1 based on selected clock mode
	switch (clkMode)
    {
    	case (RUN_FIRC):
		{
    		// LPUART0 pins
			// PTB0, UART0_RX
			PORTA->PCR[2] = PORT_PCR_MUX(0x6);
			// PTB1, UART0_TX
			PORTA->PCR[3] = PORT_PCR_MUX(0x6);
    		// UART1 configured to run from FIRCDIV2
    	    LPUART0->BAUD = LPUART_BAUD_OSR(15) | LPUART_BAUD_SBR((48000000) / (16 * UART_BAUD_RATE));
    	    LPUART0->CTRL = LPUART_CTRL_RE_MASK | LPUART_CTRL_TE_MASK;

    		break;
		}
/*    	case (RUN_PLL):
		{
    		// UART1 configured to run from SPLLDIV2
    	    LPUART0->BAUD = LPUART_BAUD_OSR(15) | LPUART_BAUD_SBR((40000000) / (16 * UART_BAUD_RATE));
    	    LPUART0->CTRL = LPUART_CTRL_RE_MASK | LPUART_CTRL_TE_MASK;

    		break;
		}*/
    }
}
