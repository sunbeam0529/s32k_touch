/*
 * @file     6pad_app.h
 *
 * @version  1.0.0.0
 *
 * @date     January-2019
 *
 * @brief    Application configuration file for 6pad keypad with slider reference design
 *
 */

#ifndef CFG_6PAD_SLIDER_6PAD_APP_H_
#define CFG_6PAD_SLIDER_6PAD_APP_H_

/*******************************************************************************
 * Includes
 *******************************************************************************/
#include "ts_cfg_general.h"
#include "6pad_hw.h"

#if (REFERENCE_DESIGN_BOARD == S32K144_6PAD_KEYPAD_SLIDER)
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
#define BUTTON_TOUCH_THRESHOLD_DELTA            300
// Electrode release threshold relative to DC tracker value
#define BUTTON_RELEASE_THRESHOLD_DELTA          250

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

#endif

/*******************************************************************************
 * SLIDER ELECTRODES APP CFG
 ******************************************************************************/
#if SLIDER_ENABLE
/*******************************************************************************
 * Modify: Number of samples to be taken per slider electrode
 ******************************************************************************/
// IDLE MODE, when EGS/touch electrodes are not touched
// Value from 1 to 4  to achieve MCU 70uA max average current consumption
#define NUMBER_OF_SAMPLES_PER_SLIDER_ELEC_IDLE   4
// ACTIVE MODE, when EGS/touch electrodes are touched
// Number of samples, when module in active mode, max value 256
#define NUMBER_OF_SAMPLES_PER_SLIDER_ELEC_ACTIVE  16

/*******************************************************************************
 * Modify: Slider electrodes common defines
 ******************************************************************************/
#if (NUMBER_OF_WAKEUP_ELECTRODES > 0)
// DC tracker response, the highest number, the slower response.
// Select values from 1 to 8
// DC tracker response, when in Idle mode
// Value 5 equals 1s at 30ms sampling period
#define SLIDER_DCTRACKER_FILTER_FACTOR_IDLE     1
// DC tracker response, when in Active mode
// Value 5 equals 1s at 30ms sampling period
#define SLIDER_DCTRACKER_FILTER_FACTOR_ACTIVE   (5 + DF_DCTRACKER_FILTER_FACTOR)
#else
// DC tracker response, the highest number, the slower response.
// Select values from 1 to 8
// DC tracker response, when in Idle mode
// Value 5 equals 1s at 30ms sampling period
#define SLIDER_DCTRACKER_FILTER_FACTOR_IDLE     5
// DC tracker response, when in Active mode
// Value 5 equals 1s at 30ms sampling period
#define SLIDER_DCTRACKER_FILTER_FACTOR_ACTIVE   (5 + DF_DCTRACKER_FILTER_FACTOR)
#endif

// Electrode touch threshold relative to DC tracker value
#define SLIDER_TOUCH_THRESHOLD_DELTA            150
// Electrode release threshold relative to DC tracker value
#define SLIDER_RELEASE_THRESHOLD_DELTA          120

/*******************************************************************************
 * Modify: Electrode 6 defines
 ******************************************************************************/
#ifdef ELEC6
#define ELEC6_TOUCH_THRESHOLD_DELTA       SLIDER_TOUCH_THRESHOLD_DELTA
#define ELEC6_RELEASE_THRESHOLD_DELTA     SLIDER_RELEASE_THRESHOLD_DELTA
#endif

/*******************************************************************************
 * Modify: Electrode 7 defines
 ******************************************************************************/
#ifdef ELEC7
#define ELEC7_TOUCH_THRESHOLD_DELTA       SLIDER_TOUCH_THRESHOLD_DELTA
#define ELEC7_RELEASE_THRESHOLD_DELTA     SLIDER_RELEASE_THRESHOLD_DELTA
#endif

/*******************************************************************************
 * Modify: Number of  slider segments
 ******************************************************************************/
// Number from 2 to 8 (5 is optimal)
#define NUMBER_OF_SLIDER_SEGMENTS 5

/*******************************************************************************
 * Do not Modify: Slider segments
 ******************************************************************************/
#if (NUMBER_OF_SLIDER_SEGMENTS > 0 && NUMBER_OF_SLIDER_SEGMENTS < 9 )
// Number of slider difference data thresholds
#define NUMBER_OF_SLIDER_THRESHOLDS (NUMBER_OF_SLIDER_SEGMENTS - 1)
// Define to alloc certain memory for sliderDiffThreshold array - Freemaster
#define NUMBER_OF_THRESHOLDS_FREEMASTER 7
#endif

/*******************************************************************************
 * Modify ONLY after tuning in Freemaster: DEFAULT Slider defines after power-up/reset
 ******************************************************************************/
/*******************************************************************************
 * Modify: Define slider data calculation method select:
 * CENTROID - centroid function of the two slider electrodes
 * DIFFERENCE_DATA - difference of the two slider electrodes
 ******************************************************************************/
#define SLIDER_DATA_CALC_METHOD 	CENTROID

#if (SLIDER_DATA_CALC_METHOD == CENTROID) // Centroid used?
// Modify centroid resolution
// Value 100 - 256
#define SLIDER_CENTROID_RESOLUTION 100

// How many percent of the centroid resolution can be realistically reached by the slider?
// Value 0-100
#define SLIDER_CENTROID_RESOLUTION_PCT 90

// Slider Sum
// Threshold of slider sum
#define SLIDER_SUM_THRESHOLD 	(-SLIDER_TOUCH_THRESHOLD_DELTA*2)

// Number of points above and below each data threshold, where the slider is insensitive to change
// Number from 1 to 30
#define NUMBER_OF_HYSTERESIS_BLIND_POINTS 6

#else // Difference data
// Number of points above and below each data threshold, where the slider is insensitive to change
// Number from 1 to 30
#define NUMBER_OF_HYSTERESIS_BLIND_POINTS 6

// Slider Sum
// Threshold of slider sum
#define SLIDER_SUM_THRESHOLD 	(-SLIDER_TOUCH_THRESHOLD_DELTA*2)

#endif

#endif

/*******************************************************************************
 * WAKEUP (EGS) ELECTRODES APP CFG
 ******************************************************************************/
#if (NUMBER_OF_WAKEUP_ELECTRODES > 0)
/*******************************************************************************
 * Modify: Additional wake-up electrode 5-10 cm proximity functionality
 ******************************************************************************/
// YES or NO FOR  optional wake-up electrode proximity
// If YES, the EGS electrode will we be scanned with huge number of samples to provide 5-10 cm proximity
// If NO, the EGS electrode will just provide wake up when touch electrodes touched
#define WAKE_UP_ELECTRODE_PROXIMITY   NO

/*******************************************************************************
 * Modify: Number of samples to be taken per wake-up electrode
 ******************************************************************************/
#if WAKE_UP_ELECTRODE_PROXIMITY // 5-10 cm proximity
// The wake-up electrode (EGS) only scanned in IDLE mode, when searching for possible touch event
// In ACTIVE mode, the wake-up electrode (EGS) no longer being scanned
// Value from 32 to 128  to achieve 5 - 10 cm proximity
#define NUMBER_OF_SAMPLES_PER_WAKEUP_ELEC   64

#else // max 1 cm proximity
// The wake-up electrode (EGS) only scanned in IDLE mode, when searching for possible touch event
// In ACTIVE mode, the wake-up electrode (EGS) no longer being scanned
// Value from 1 to 4  to achieve MCU 70uA max average current consumption at 30ms scanning period
#define NUMBER_OF_SAMPLES_PER_WAKEUP_ELEC   4
#endif

/*******************************************************************************
 * Modify: Wakeup electrodes common defines
 ******************************************************************************/
// DC tracker response, the highest number, the slower response.
// Select values from 1 to 8
// Value 5 equals 1s at 30ms sampling period
#define WAKEUP_DCTRACKER_FILTER_FACTOR    5

#if WAKE_UP_ELECTRODE_PROXIMITY
// Electrode touch threshold relative to DC tracker value
#define WAKEUP_TOUCH_THRESHOLD_DELTA       30
// Electrode release threshold relative to DC tracker value
#define WAKEUP_RELEASE_THRESHOLD_DELTA     20
#else
// Electrode touch threshold relative to DC tracker value
#define WAKEUP_TOUCH_THRESHOLD_DELTA       6
// Electrode release threshold relative to DC tracker value
#define WAKEUP_RELEASE_THRESHOLD_DELTA     4

#endif

/*******************************************************************************
 * Modify: Electrode 8 defines (wake-up EGS)
 ******************************************************************************/
#ifdef ELEC8
// Electrode touch threshold relative to DC tracker value
#define ELEC8_TOUCH_THRESHOLD_DELTA       WAKEUP_TOUCH_THRESHOLD_DELTA
// Electrode release threshold relative to DC tracker value
#define ELEC8_RELEASE_THRESHOLD_DELTA     WAKEUP_RELEASE_THRESHOLD_DELTA
#endif

#else // No real wake-up (EGS) electrode - will use virtual wake-up for possible proximity detection

/*******************************************************************************
 * Modify: Virtual Wake-up Electrode touch threshold (for samples switching)
 ******************************************************************************/
// Virtual EGS touch threshold relative to DC tracker value
#define VIRTUAL_WAKEUP_TOUCH_THRESHOLD_DELTA 10

#endif

#endif

#endif /* CFG_6PAD_SLIDER_6PAD_APP_H_ */
