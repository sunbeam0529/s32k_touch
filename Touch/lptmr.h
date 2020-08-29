/*
 * lptmr.h
 *
 *  Created on: May 27, 2020
 *      Author: nxf38186
 */

#ifndef PERIPHERALS_LPTMR_H_
#define PERIPHERALS_LPTMR_H_

/*******************************************************************************
* Includes
*******************************************************************************/
#include "ts_cfg.h"

/*******************************************************************************
* Defines LPTMR
******************************************************************************/
// Init LPTMR0 (num + 1 [ms])
#define LPTMR_ELEC_CAL      1  //  0 = 1ms timeout period, 1 = 2ms timeout period...Must be set to at least 1 = 2ms period else the pending flag in NVIC cannot be cleared
#define LPTMR_ELEC_SENSE    (ELECTRODES_SENSE_PERIOD - 1)

#if (WAKE_UP_ELECTRODE_PROXIMITY == YES)
#define LPTMR_ELEC_SENSE_PROXIMITY    (ELECTRODES_SENSE_PERIOD_PROXIMITY - 1)
#endif

#if DECIMATION_FILTER
#define LPTMR_ELEC_SENSE_DF   (ELECTRODES_SENSE_PERIOD_DF - 1)
#endif
/*******************************************************************************
* Function prototypes
******************************************************************************/
void LPTMR0_Init(uint32_t timeout);
void LPTMR0_CMR_Update(uint32_t timeout);


#endif /* PERIPHERALS_LPTMR_H_ */
