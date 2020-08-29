/*
 * @file     2pad_app.h
*
* @version  1.0.0.0
*
* @date     January-2019
*
* @brief    Application configuration file for 2pad EVB demo
*
 */

#ifndef CFG_2PAD_EVB_2PAD_APP_H_
#define CFG_2PAD_EVB_2PAD_APP_H_

/*******************************************************************************
* Includes
*******************************************************************************/
#include "ts_cfg_general.h"
#include "2pad_hw.h"

#if (REFERENCE_DESIGN_BOARD == S32K118_2PAD_EVB)
/*******************************************************************************
 * Modify: Number of samples to be taken and discarded (for all electrodes types)
 ******************************************************************************/
// Value from 1 to 2
#define NUMBER_OF_PRESAMPLES    1

/*******************************************************************************
 * BUTTON ELECTRODES APP CFG
 ******************************************************************************/
#if (NUMBER_OF_BUTTON_ELECTRODES > 0)
/*******************************************************************************
 * Modify: Number of samples to be taken per button electrode
 ******************************************************************************/
// IDLE MODE, when EGS/touch buttons are not touched
// Value from 1 to 4  to achieve MCU 70uA max average current consumption
#define NUMBER_OF_SAMPLES_PER_BUTTON_ELEC_IDLE   4
// ACTIVE MODE, when EGS/touch buttons are touched
// Number of samples, when module in active mode, max value 256
#define NUMBER_OF_SAMPLES_PER_BUTTON_ELEC_ACTIVE  16

/*******************************************************************************
 * Modify: Button electrodes common defines
 ******************************************************************************/
#if (NUMBER_OF_WAKEUP_ELECTRODES > 0)
// DC tracker response, the highest number, the slower response.
// Select values from 1 to 8
// DC tracker response, when in Idle mode
// Value 5 equals 1s at 30ms sampling period
#define BUTTON_DCTRACKER_FILTER_FACTOR_IDLE     1
// DC tracker response, when in Active mode
// Value 5 equals 1s at 30ms sampling period
#define BUTTON_DCTRACKER_FILTER_FACTOR_ACTIVE   (5 + DF_DCTRACKER_FILTER_FACTOR)
#else
// DC tracker response, the highest number, the slower response.
// Select values from 1 to 8
// DC tracker response, when in Idle mode
// Value 5 equals 1s at 30ms sampling period
#define BUTTON_DCTRACKER_FILTER_FACTOR_IDLE     5
// DC tracker response, when in Active mode
// Value 5 equals 1s at 30ms sampling period
#define BUTTON_DCTRACKER_FILTER_FACTOR_ACTIVE   (5 + DF_DCTRACKER_FILTER_FACTOR)
#endif

// Electrode touch threshold relative to DC tracker value
#define BUTTON_TOUCH_THRESHOLD_DELTA            2000
// Electrode release threshold relative to DC tracker value
#define BUTTON_RELEASE_THRESHOLD_DELTA          1800

/*******************************************************************************
 * Modify: Electrode 0 defines
 ******************************************************************************/
#ifdef ELEC0
// Electrode touch threshold relative to DC tracker value
#define ELEC0_TOUCH_THRESHOLD_DELTA       BUTTON_TOUCH_THRESHOLD_DELTA
// Electrode release threshold relative to DC tracker value
#define ELEC0_RELEASE_THRESHOLD_DELTA     BUTTON_RELEASE_THRESHOLD_DELTA
#endif

/*******************************************************************************
 * Modify: Electrode 1 defines
 ******************************************************************************/
#ifdef ELEC1
#define ELEC1_TOUCH_THRESHOLD_DELTA       BUTTON_TOUCH_THRESHOLD_DELTA
#define ELEC1_RELEASE_THRESHOLD_DELTA     BUTTON_RELEASE_THRESHOLD_DELTA
#endif

/*******************************************************************************
 * Modify: Electrode 2 defines
 ******************************************************************************/
#ifdef ELEC2
#define ELEC2_TOUCH_THRESHOLD_DELTA       BUTTON_TOUCH_THRESHOLD_DELTA
#define ELEC2_RELEASE_THRESHOLD_DELTA     BUTTON_RELEASE_THRESHOLD_DELTA
#endif

/*******************************************************************************
 * Modify: Electrode 3 defines
 ******************************************************************************/
#ifdef ELEC3
#define ELEC3_TOUCH_THRESHOLD_DELTA       BUTTON_TOUCH_THRESHOLD_DELTA
#define ELEC3_RELEASE_THRESHOLD_DELTA     BUTTON_RELEASE_THRESHOLD_DELTA
#endif

/*******************************************************************************
 * Modify: Electrode 4 defines
 ******************************************************************************/
#ifdef ELEC4
#define ELEC4_TOUCH_THRESHOLD_DELTA       BUTTON_TOUCH_THRESHOLD_DELTA
#define ELEC4_RELEASE_THRESHOLD_DELTA     BUTTON_RELEASE_THRESHOLD_DELTA
#endif

/*******************************************************************************
 * Modify: Electrode 5 defines
 ******************************************************************************/
#ifdef ELEC5
#define ELEC5_TOUCH_THRESHOLD_DELTA       BUTTON_TOUCH_THRESHOLD_DELTA
#define ELEC5_RELEASE_THRESHOLD_DELTA     BUTTON_RELEASE_THRESHOLD_DELTA
#endif

/*******************************************************************************
 * Modify: Electrode 6 defines
 ******************************************************************************/
#ifdef ELEC6
#define ELEC6_TOUCH_THRESHOLD_DELTA       BUTTON_TOUCH_THRESHOLD_DELTA
#define ELEC6_RELEASE_THRESHOLD_DELTA     BUTTON_RELEASE_THRESHOLD_DELTA
#endif

/*******************************************************************************
 * Modify: Electrode 7 defines
 ******************************************************************************/
#ifdef ELEC7
#define ELEC7_TOUCH_THRESHOLD_DELTA       BUTTON_TOUCH_THRESHOLD_DELTA
#define ELEC7_RELEASE_THRESHOLD_DELTA     BUTTON_RELEASE_THRESHOLD_DELTA
#endif

/*******************************************************************************
 * Modify: Electrode 8 defines
 ******************************************************************************/
#ifdef ELEC8
#define ELEC8_TOUCH_THRESHOLD_DELTA       BUTTON_TOUCH_THRESHOLD_DELTA
#define ELEC8_RELEASE_THRESHOLD_DELTA     BUTTON_RELEASE_THRESHOLD_DELTA
#endif

#endif

//No real wake-up (EGS) electrode - will use virtual wake-up for possible proximity detection
/*******************************************************************************
 * Modify: Virtual Wake-up Electrode touch threshold (for samples switching)
 ******************************************************************************/
// Virtual EGS touch threshold relative to DC tracker value
#define VIRTUAL_WAKEUP_TOUCH_THRESHOLD_DELTA 10

#endif
#endif /* CFG_2PAD_EVB_2PAD_APP_H_ */
