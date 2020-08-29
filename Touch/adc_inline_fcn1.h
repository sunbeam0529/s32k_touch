/****************************************************************************//*!
*
* @file     adc.h
*
* @version  1.0.0.0
*
* @date     September-2017
*
* @brief    ADC inline function 1 header file
*
*******************************************************************************/
#ifndef __ADC_INLINE_FCN1_H
#define __ADC_INLINE_FCN1_H

/*******************************************************************************
* Includes
*******************************************************************************/
#include "ets.h"
#include "S32K118.h"

/*****************************************************************************
*
* Function: static inline int16_t EquivalentVoltageDigitalization(tElecStruct *pElectrodeStruct)
*
* Input: Address of a single electrode structure in electrodes structure array
*
* Output: Result of the equivalent voltage conversion
*
* Description: Equivalent voltage conversion
*
*****************************************************************************/
static inline int16_t EquivalentVoltageDigitalization(tElecStruct *pElectrodeStruct)
{
	// Wait for conversion complete flag
	while(pElectrodeStruct->adcBasePtr->SC1[0] < 0x80)
	{}
	// Store result, clear COCO flag
	return pElectrodeStruct->adcBasePtr->R[0];
}

#endif /* __ADC_INLINE_FCN1_H */
