/*
 * ts_cfg_private.h
 *
 *  Created on: May 13, 2020
 *      Author: nxf38186
 */

#ifndef CFG_TS_CFG_PRIVATE_H_
#define CFG_TS_CFG_PRIVATE_H_

/*******************************************************************************
* Do not modify ! Demo hardware list
******************************************************************************/
#define S32K118_2PAD_EVB 			1
#define S32K144_7PAD_KEYPAD 		2
#define S32K144_6PAD_KEYPAD_SLIDER 	3

/*******************************************************************************
* Do not modify !
******************************************************************************/
#define LPM_ENABLE       1
#define LPM_DISABLE      0
#define ELECTRODE_ADC_CHANNEL_OFFSET 16

/*******************************************************************************
* Do not modify! Electrode data acquisition method
******************************************************************************/
#define CPU_ELECTRODE_DATA_ACQUISITION   0
#define DMA_ELECTRODE_DATA_ACQUISITION   1

/*******************************************************************************
* Do not modify ! Electrode types
******************************************************************************/
#define BUTTON 0
#define SLIDER 1
#define WAKEUP 2

/*******************************************************************************
* Do not modify ! Application Modes
******************************************************************************/
#define ACTIVE 		1
#define IDLE 		0
#define YES   		1
#define NO    		0
#define ON    		1
#define OFF   		0
#define INCREASE    1
#define DECREASE   	0

/*******************************************************************************
* Do not modify ! Slider data calculation method
******************************************************************************/
#define DIFFERENCE_DATA 	0
#define CENTROID			1

/*******************************************************************************
* Do not modify!
******************************************************************************/
// Required for IIR filter correct operation.
// SHIFT_NUMBER = (2^31 / (2^12 * NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_IDLE))
// Go to: https://www.wolframalpha.com/
// Calculate: IIR_FILTER_VALUE_SHIFT = round down [log2(SHIFT_NUMBER)]
// Calculated for max 128 samples
#define IIR_FILTER_VALUE_SHIFT  12

#endif /* CFG_TS_CFG_PRIVATE_H_ */
