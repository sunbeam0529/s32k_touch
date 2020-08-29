/*
 * lpit.c
 *
 *  Created on: May 25, 2020
 *      Author: nxf38186
 */
/*******************************************************************************
 * Includes
 *******************************************************************************/
#include "S32K118.h"
#include "ts_cfg.h"

extern uint8_t cpuFHScanFlag;
uint8_t lpit_clock;
/*****************************************************************************
 *
 * Function: void LPIT_Init(void)
 *
 * Description: Init LPIT
 *
 *****************************************************************************/
void LPIT_Init(void)
{
	LPIT0->MCR |= LPIT_MCR_M_CEN_MASK; // Enable module clk (allows writing other LPIT0 regs)
}

/*****************************************************************************
 *
 * Function: void LPIT_Enable(uint8_t channel, uint32_t timeout)
 *
 * Description: Enables LPIT
 *
 *****************************************************************************/
void LPIT_Enable(uint8_t channel, uint32_t timeout)
{
	LPIT0->MSR = (1 << channel); // Clear LPIT CH0 timer flag
	LPIT0->TMR[channel].TVAL = timeout; // Chan 0 Timeout period
	LPIT0->TMR[channel].TCTRL |= LPIT_TMR_TCTRL_T_EN_MASK;// LPIT ch0 Enable
}

/*****************************************************************************
 *
 * Function: void LPIT_Disable(uint8_t channel)
 *
 * Description: Disables LPIT
 *
 *****************************************************************************/
void LPIT_Disable(uint8_t channel)
{
	LPIT0->TMR[channel].TCTRL &= ~LPIT_TMR_TCTRL_T_EN_MASK;// LPIT ch0 Disable
}
