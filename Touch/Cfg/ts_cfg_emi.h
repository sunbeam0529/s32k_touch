/*
 * ts_cfg_emi.h
 *
 *  Created on: May 13, 2020
 *      Author: nxf38186
 */

#ifndef CFG_TS_CFG_EMI_H_
#define CFG_TS_CFG_EMI_H_

/*******************************************************************************
* Includes
*******************************************************************************/
#include "ts_cfg_general.h"
/*******************************************************************************
 * Modify: Jittering ON/OFF and jittering bits
 ******************************************************************************/
// ON or OFF for optional jittering sample rate control
#define JITTERING OFF

#if JITTERING
//  Jitter once at the beginning of LPTMR IRQ handler
// Modify: Value of 2 to 8
#define NUMBER_OF_JITTERING_BITS 3
// Do not modify!: the jittering_mask_macro
#define JITTERING_MASK ((1 << NUMBER_OF_JITTERING_BITS)-1)
#endif

/*******************************************************************************
 * Modify: Frequency hopping ON/OFF
 ******************************************************************************/
// ON  or OFF for optional frequency hopping
#define FREQUENCY_HOPPING OFF

#if FREQUENCY_HOPPING
// Values from 1 (min) to 4 (max)
#define NUMBER_OF_HOPPING_FREQUENCIES 4
#else
// Just the base frequency - at least one!
#define NUMBER_OF_HOPPING_FREQUENCIES 1
#endif

/*******************************************************************************
 * Modify: Hopping sampling periods (frequencies)
 * Note I: The sampling period must be at least 6us to work with DMA concept when sampling triggered by LPIT
 * Note II: Base period (frequency 0) must be defined 0 to fit into Low Power 100uA MCU average current requirement
 ******************************************************************************/
// Define base period (frequency 0)
// Values 0-5 - automatically using fastest period (2us CPU, 3.5 us DMA), sampling triggered by channel linking
#define FREQ_ID_0 							0
// Values > 5 - sampling triggered periodically by LPIT
//#define FREQ_ID_0 						    6 // one sample takes 6us (166 KHz)

// Define additional hopping periods (frequencies 1-3)
// Values > 5 !
#define FREQ_ID_1 							7 // one sample takes 7us (142 KHz)
#define FREQ_ID_2 							8 // one sample takes 8us (125 KHz)
#define FREQ_ID_3 							9 // one sample takes 9us (111 KHz)

/*******************************************************************************
 * Modify: Decimation filter ON/OFF
 *
 * WARNING_I: Decimation filter changes electrodes sensing period = LPMTR period (normally 30ms)
 *
 * WARNING_II: ELECTRODES_SENSE_PERIOD_DF must be chosen appropriately - so all the sampling on all used sampling frequencies fits in
 *
 ******************************************************************************/
// ON or OFF for optional decimation filter
#define DECIMATION_FILTER OFF

#if DECIMATION_FILTER
// Modify: LPTMR new wake-up period
// Value between 5 and 30
#define ELECTRODES_SENSE_PERIOD_DF 10

// Modify: Decimation filter filter factor addition for DCTracker
#define DF_DCTRACKER_FILTER_FACTOR 3

// Modify: Decimation filter step (minimal step 1)
#define DECIMATION_STEP 30

#else
// Do not Modify!: Decimation filter disabled - no filter factor addition for DCTracker
#define DF_DCTRACKER_FILTER_FACTOR 0
#endif

/*******************************************************************************
 * Modify: Define IIR1 filter parameters.
 *         Coefficients B0, B1 and A0 are dependent on filter cutoff frequency [Hz]
 *         and electrode sensing period, see ELECTRODES_SENSE_PERIOD above.
 *         Coefficient for filter are calculated with:
 *         http://engineerjs.com/?sidebar=docs/iir.html
 *
 ******************************************************************************/
#if DECIMATION_FILTER
// related to the ELECTRODES_SENSE_PERIOD_DF

// Touch button electrodes filter coeff.
#define FILTER_1                   0
// Cutoff frequency in Hertz [Hz]
#define FILTER_1_CUTOFF_FREQ_HZ    1
// Coefficient B0
#define FILTER_1_COEF_B0          0.030468747091254
// Coefficient B1
#define FILTER_1_COEF_B1         0.030468747091254
// Coefficient A0
#define FILTER_1_COEF_A0         -0.939062505817492

// Slider electrodes filter coeff.
#define FILTER_2  				   1
#define FILTER_2_CUTOFF_FREQ_HZ    1
#define FILTER_2_COEF_B0            0.030468747091254
#define FILTER_2_COEF_B1            0.030468747091254
#define FILTER_2_COEF_A0           -0.939062505817492

// Wakeup electrodes filter coeff.
#define FILTER_3                   2
#define FILTER_3_CUTOFF_FREQ_HZ    1
#define FILTER_3_COEF_B0           0.086364027013762
#define FILTER_3_COEF_B1           0.086364027013762
#define FILTER_3_COEF_A0           -0.827271945972476

#else
// related to the ELECTRODES_SENSE_PERIOD

// Touch button electrodes filter coeff.
#define FILTER_1                   0
// Cutoff frequency in Hertz [Hz]
#define FILTER_1_CUTOFF_FREQ_HZ    1
// Coefficient B0
#define FILTER_1_COEF_B0           0.086364027013762
// Coefficient B1
#define FILTER_1_COEF_B1           0.086364027013762
// Coefficient A0
#define FILTER_1_COEF_A0           -0.827271945972476

// Slider electrodes filter coeff.
#define FILTER_2  				   1
#define FILTER_2_CUTOFF_FREQ_HZ    1
#define FILTER_2_COEF_B0           0.086364027013762
#define FILTER_2_COEF_B1           0.086364027013762
#define FILTER_2_COEF_A0           -0.827271945972476

// Wakeup electrodes filter coeff.
#define FILTER_3                   2
#define FILTER_3_CUTOFF_FREQ_HZ    1
#define FILTER_3_COEF_B0           0.086364027013762
#define FILTER_3_COEF_B1           0.086364027013762
#define FILTER_3_COEF_A0           -0.827271945972476

#endif
// Number of used filters
#define NUMBER_OF_IIR_FILTERS_USED     3

/*******************************************************************************
 * Modify: Median Filter ON/OFF and its Window size
 * Values from 3 to Number of samples per electrode taken in active mode + number of presamples)
 ******************************************************************************/
#define FILTER_MEDIAN OFF
// Values of median window must be odd
#define WINDOW_MEDIAN_BUTTON	3
#define WINDOW_MEDIAN_SLIDER	3
#define WINDOW_MEDIAN_WAKEUP	3

/*******************************************************************************
 * Modify: Mean Filter ON/OFF and its Window size
 * Values from 3 to (Number of samples taken in active mode active mode + number of presamples)
 ******************************************************************************/
#define FILTER_MEAN  OFF
#define WINDOW_MEAN_BUTTON		3
#define WINDOW_MEAN_SLIDER		3
#define WINDOW_MEAN_WAKEUP		3

#endif /* CFG_TS_CFG_EMI_H_ */
