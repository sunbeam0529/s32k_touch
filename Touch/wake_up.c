/*
 * wake_up.c
 *
 *  Created on: Apr 28, 2020
 *      Author: nxf38186
 */

/*******************************************************************************
* Includes
*******************************************************************************/
#include "ts_cfg.h"
#include "ets.h"
#include "filter.h"

#if (NUMBER_OF_WAKEUP_ELECTRODES > 0)
/*******************************************************************************
* Variables
*******************************************************************************/
// extern variables
extern int16_t   elecNumberOfSamples[NUMBER_OF_ELECTRODES];
extern int16_t   elecNumberOfSamplesActive[NUMBER_OF_ELECTRODES];
extern int32_t   elecRawData[NUMBER_OF_ELECTRODES][NUMBER_OF_HOPPING_FREQUENCIES];
extern int32_t   elecDCTrackerBuffer[NUMBER_OF_ELECTRODES];
extern uint8_t   elecDCTrackerShift[NUMBER_OF_ELECTRODES];
extern int32_t   elecDCTracker[NUMBER_OF_ELECTRODES];
extern int32_t   elecThresholdTouch[NUMBER_OF_ELECTRODES], elecThresholdRelease[NUMBER_OF_ELECTRODES];
extern int16_t   elecThresholdTouchDelta[NUMBER_OF_ELECTRODES], elecThresholdReleaseDelta[NUMBER_OF_ELECTRODES];
extern uint8_t   elecTouch[NUMBER_OF_ELECTRODES][NUMBER_OF_HOPPING_FREQUENCIES];
extern tFrac32   elecLPFilterData[NUMBER_OF_ELECTRODES][NUMBER_OF_HOPPING_FREQUENCIES];
extern uint8_t   firstWakeupElecNum;
extern uint8_t   elecNum;
extern uint16_t   electrodeWakeUpActivateCounter;

// FrequencyHopping
extern uint8_t   frequencyID,frequencyIDsave;
extern uint8_t proximityDetectedFlag;

// Decimation filter
extern int32_t   elecRawDataDF[NUMBER_OF_ELECTRODES][NUMBER_OF_HOPPING_FREQUENCIES];
/*****************************************************************************
 *
 * Function: void WakeupElecDSP(uint8_t electrodeNum)
 *
 * Description: Process raw data for single elec electrode
 *
 *****************************************************************************/
void WakeupElecDSP(uint8_t electrodeNum)
{
	// Update DC Tracker, if electrode not touched
	if (elecTouch[electrodeNum][frequencyID] == 0)
	{
		// Update DC Tracker
		elecDCTracker[electrodeNum] = DCTracker(elecRawData[electrodeNum][frequencyID], &(elecDCTrackerBuffer[electrodeNum]), elecDCTrackerShift[electrodeNum]);
	}

	// IIR LP filter fed by raw data
	elecLPFilterData[electrodeNum][frequencyID] = FilterIIR1(electrodeNum, (tFrac32)(elecRawData[electrodeNum][frequencyID]), frequencyID);

	// Calculate electrode touch and release thresholds
	elecThresholdTouch[electrodeNum] = elecDCTracker[electrodeNum] -  elecThresholdTouchDelta[electrodeNum];
	elecThresholdRelease[electrodeNum] = elecDCTracker[electrodeNum] -  elecThresholdReleaseDelta[electrodeNum];
}

/*****************************************************************************
 *
 * Function: void WakeupElecTouchDetect(uint8_t electrodeNum)
 *
 * Description: Detect touch event on single wakeup electrode
 *
 *****************************************************************************/
void WakeupElecTouchDetect(uint8_t electrodeNum)
{
	// Wakeup Electrode touched ?
	if (elecLPFilterData[electrodeNum][frequencyID] < elecThresholdTouch[electrodeNum])
	{
		// Touched
		elecTouch[electrodeNum][frequencyID] = 1;
		// Set proximity flag
		proximityDetectedFlag = 1;
	}
	else
	{
		// Released
		elecTouch[electrodeNum][frequencyID] = 0;
		// Clear proximity flag
		proximityDetectedFlag = 0;
	}

}

#endif
