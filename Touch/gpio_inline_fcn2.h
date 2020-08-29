/****************************************************************************//*!
*
* @file     gpio_inline_fcn2.h
*
* @version  1.0.0.0
*
* @date     September-2017
*
* @brief    GPIO inline functions 2 header file
*
*******************************************************************************/
#ifndef __GPIO_INLINE_FCN2_H
#define __GPIO_INLINE_FCN2_H

/*******************************************************************************
* Includes
*******************************************************************************/
#include "S32K118.h"

/*****************************************************************************
*
* Function: static inline void LedRedON(void)
*
* Description: Turn ON RED LED
*
*****************************************************************************/
static inline void LedRedON(void)
{
	// Turn ON RED LED
	PTD->PSOR = 1 << 16;
}

/*****************************************************************************
*
* Function: static inline void LedRedOFF(void)
*
* Description: Turn OFF RED LED
*
*****************************************************************************/
static inline void LedRedOFF(void)
{
	// Turn OFF RED LED
	PTD->PCOR = 1 << 16;
}

/*****************************************************************************
*
* Function: static inline void LedGreenON(void)
*
* Description: Turn ON GREEN LED
*
*****************************************************************************/
static inline void LedGreenON(void)
{
	// Turn ON GREEN LED
	PTD->PSOR = 1 << 15;
}

/*****************************************************************************
*
* Function: static inline void LedGreenOFF(void)
*
* Description: Turn OFF GREEN LED
*
*****************************************************************************/
static inline void LedGreenOFF(void)
{
	// Turn OFF GREEN LED
	PTD->PCOR = 1 << 15;
}

/*****************************************************************************
*
* Function: static inline void LedBlueON(void)
*
* Description: Turn ON BLUE LED
*
*****************************************************************************/
static inline void LedBlueON(void)
{
	// Turn ON BLUE LED
	PTE->PSOR = 1 << 8;
}

/*****************************************************************************
*
* Function: static inline void LedBlueOFF(void)
*
* Description: Turn OFF BLUE LED
*
*****************************************************************************/
static inline void LedBlueOFF(void)
{
	// Turn OFF BLUE LED
	PTE->PCOR = 1 << 8;
}


#endif /* __GPIO_INLINE_FCN2_H */
