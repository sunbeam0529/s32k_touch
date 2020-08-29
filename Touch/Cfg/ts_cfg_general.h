/*
 * @file     ts_cfg_general.h
*
* @version  1.0.0.0
*
* @date     January-2019
*
* @brief    Touch sense main configuration file
*
 */

#ifndef CFG_TS_CFG_GENERAL_H_
#define CFG_TS_CFG_GENERAL_H_

/*******************************************************************************
* Includes
*******************************************************************************/
#include "ts_cfg_private.h"

/*******************************************************************************
* Modify: Reference design Hardware Selection
* 	S32K118_2PAD_EVB 			- Two pads demo on S32K118 EVB
* 	S32K144_7PAD_KEYPAD 		- Reference design board - 7 pad keypad
* 	S32K144_6PAD_KEYPAD_SLIDER 	- Reference design board - 6 pad keypad with slider
******************************************************************************/
// Select the available type of hardware for TS application (S32K118_2PAD_EVB, S32K144_7PAD_KEYPAD or S32K144_6PAD_KEYPAD_SLIDER)
#define REFERENCE_DESIGN_BOARD 	S32K118_2PAD_EVB

/*******************************************************************************
* Modify: Electrode data acquisition method
*  If CPU_ELECTRODE_DATA_ACQUISITION chosen, the CPU drives the electrodes and gathers data of all electrodes
*  If DMA_ELECTRODE_DATA_ACQUISITION chosen, the DMA drives the electrodes and gathers data of all electrodes
******************************************************************************/
#define TS_ELECTRODE_DATA_ACQUISITION     CPU_ELECTRODE_DATA_ACQUISITION


#endif /* CFG_TS_CFG_GENERAL_H_ */
