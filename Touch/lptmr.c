/*
 * lptmr.c
 *
 *  Created on: May 27, 2020
 *      Author: nxf38186
 */

/*******************************************************************************
 * Includes
 *******************************************************************************/
#include "S32K118.h"
#include "lptmr.h"
#include "ets.h"

/*****************************************************************************
 *
 * Function: void LPTMR0_Init(uint32_t timeout)
 *
 * Description: Init LPTMR
 *
 *****************************************************************************/
void LPTMR0_Init(uint32_t timeout)
{
	// Clear TCF LPTMR and disable whole CSR
	LPTMR0->CSR = 1 << 7;
	// Disable LPTMR
	LPTMR0->CSR = 0x00000000;
	// Bypass prescaler , clock 1 (LPO1K) selected
	LPTMR0->PSR = 5;
	// Set timeout period
	LPTMR0->CMR = timeout;
	// Clear TCF, enable interrupt, count rising edges, reset when TCF is set, counter mode, enable timer
	LPTMR0->CSR = 0x000000C1;
}

/*****************************************************************************
 *
 * Function: void LPTMR0_CMR_Update(uint32_t timeout)
 *
 * Description: Sets new timeout period of LPTMR
 *
 *****************************************************************************/
void LPTMR0_CMR_Update(uint32_t timeout)
{
	// Set new LPTMR timeout period
	LPTMR0->CMR = timeout;
}

/*****************************************************************************
 *
 * Function: void LPTMR0_IRQHandler(void)
 *
 * Description: LPTMR interrupt
 *
 *****************************************************************************/
void LPTMR0_IRQHandler(void)
{
	static uint8_t interrupttimer=0;
	// Initiate electrodes scan

	ElectrodesScanLPTMRHandlerRoutine();
	//LPTMR0->CSR |= 1 << 7;
}
