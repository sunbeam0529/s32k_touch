/*
 * touch_buttons.c
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
#include "gpio_inline_fcn2.h"
#include "motor.h"
#if (NUMBER_OF_BUTTON_ELECTRODES > 0)
/*******************************************************************************
* Variables
*******************************************************************************/
// Touch qualification
uint8_t   buttonTouchQualified[NUMBER_OF_BUTTON_ELECTRODES];
uint8_t   buttonTouchQualifiedReport, buttonTouchQualifiedDisplay;
uint8_t   buttonTouchNumberPlusOne;

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
extern uint8_t   firstButtonElecNum;
extern uint8_t   elecNum;
extern uint16_t   electrodeWakeUpActivateCounter;

// FrequencyHopping
extern uint8_t   frequencyID;

// Decimation filter
extern int32_t   elecRawDataDF[NUMBER_OF_ELECTRODES][NUMBER_OF_HOPPING_FREQUENCIES];
/*****************************************************************************
 *
 * Function: void ButtonVarInit(void)
 *
 * Description: Init touch buttons specific variables
 *
 *****************************************************************************/
void ButtonVarInit(void)
{
	// Reset
	buttonTouchNumberPlusOne = 0;
	buttonTouchQualifiedReport = 0;
}
/*****************************************************************************
 *
 * Function: void ButtonElecDSP(uint8_t electrodeNum)
 *
 * Description: Process raw data for single elec electrode
 *
 *****************************************************************************/
void ButtonElecDSP(uint8_t electrodeNum)
{
	// Update DC Tracker, if electrode not touched
	if (elecTouch[electrodeNum][frequencyID] == 0)
	{
		// Update DC Tracker
		ElecDCTrackerUpdate(electrodeNum);
	}

	#if DECIMATION_FILTER
	// Decimation filtering
	DecimationFilter(electrodeNum);
	#endif

	#if DECIMATION_FILTER
	// IIR LP Filter fed by DF data
	elecLPFilterData[electrodeNum][frequencyID] = FilterIIR1(electrodeNum, (tFrac32)(elecRawDataDF[electrodeNum][frequencyID]), frequencyID);
	#else
	// IIR LP filter fed by raw data
	elecLPFilterData[electrodeNum][frequencyID] = FilterIIR1(electrodeNum, (tFrac32)(elecRawData[electrodeNum][frequencyID]), frequencyID);
	#endif

	// Calculate electrode touch and release thresholds
	elecThresholdTouch[electrodeNum] = elecDCTracker[electrodeNum] -  elecThresholdTouchDelta[electrodeNum];
	elecThresholdRelease[electrodeNum] = elecDCTracker[electrodeNum] -  elecThresholdReleaseDelta[electrodeNum];
}

/*****************************************************************************
 *
 * Function: void ButtonElecTouchDetect(uint8_t electrodeNum)
 *
 * Description: Detect touch event on single button electrode
 *
 *****************************************************************************/
void ButtonElecTouchDetect(uint8_t electrodeNum)
{
	// Button Electrode touched ?
	if (elecLPFilterData[electrodeNum][frequencyID] < elecThresholdTouch[electrodeNum])
	{
		// Touched
		elecTouch[electrodeNum][frequencyID] = 1;

		// Load counter to do not return to the wake-up function
		// Do not return to the wake-up function, when electrode released and electrode signal value is above touch and bellow release threshold
		// Expected SR <= 1s
		electrodeWakeUpActivateCounter = ELEC_WAKEUP_ACTIVATE_COUNTER;
	}
	// Electrode released?
	else if ((elecLPFilterData[electrodeNum][frequencyID] > elecThresholdRelease[electrodeNum]) && (elecTouch[electrodeNum][frequencyID] == 1))
	{
		// Released
		elecTouch[electrodeNum][frequencyID] = 0;
	}
}

/*****************************************************************************
 *
 * Function: void ButtonTouchQualify(void)
 *
 * Description: Qualify touch button electrodes touch
 *
 * Note: This function ensures that the first button touched electrode wins
 *
 *****************************************************************************/
void ButtonTouchQualify(void)
{
	uint8_t f_count;

	// For all touch elec electrodes check touch event
	for (elecNum = 0; elecNum < NUMBER_OF_BUTTON_ELECTRODES; elecNum++)
	{
#if FREQUENCY_HOPPING
		// Reset frequency count
		f_count = 0;

		for (uint8_t freq = 0; freq < NUMBER_OF_HOPPING_FREQUENCIES; freq++)
		{
			// If on the frequency proximity touch detected
			if((elecTouch[elecNum][freq] == 1) && (buttonTouchNumberPlusOne == 0) &&
					elecNumberOfSamples[elecNum] == elecNumberOfSamplesActive[elecNum])
			{
				f_count++;
			}
		}

		// Electrode has been touched on all frequencies?
		if (f_count == NUMBER_OF_HOPPING_FREQUENCIES)
#else
			// Electrode has been touched?
			if ((elecTouch[elecNum][frequencyID] == 1) && (buttonTouchNumberPlusOne == 0) &&
					elecNumberOfSamples[elecNum] == elecNumberOfSamplesActive[elecNum])
#endif
			{
				// Report button electrodes touch
				buttonTouchQualifiedReport = 1;
				// Set electrode touch qualified
				buttonTouchQualified[elecNum] = 1;
				// Load electrode number + 1
				buttonTouchNumberPlusOne = elecNum + 1;
			}

		// Any electrode touched?
		if (buttonTouchNumberPlusOne != 0)
		{
#if FREQUENCY_HOPPING
			// Reset frequency count
			f_count = 0;

			for (uint8_t freq= 0; freq< NUMBER_OF_HOPPING_FREQUENCIES; freq++)
			{
				// If on the frequency proximity touch detected
				if(elecTouch[buttonTouchNumberPlusOne - 1][freq] == 0)
				{
					f_count++;
				}
			}

			// Electrode, that was touched, has been released at least on one frequency?
			if(f_count > 0)
#else
				// Electrode, that was touched, has been released?
				if(elecTouch[(buttonTouchNumberPlusOne - 1)][frequencyID] == 0)
#endif
				{
					// Report button electrodes release
					buttonTouchQualifiedReport = 0;
					// Clear electrode touch qualified
					buttonTouchQualified[(buttonTouchNumberPlusOne - 1)] = 0;
					// Reset
					buttonTouchNumberPlusOne = 0;
				}
		}
	}
}

/*****************************************************************************
 *
 * Function: uint8_t Button_RGBLED_Ctrl(void)
 *
 * Description: Display button electrodes touch event
 *
 *****************************************************************************/
uint8_t Button_RGBLED_Ctrl(void)
{
	static uint8_t keypresslast=0;
	// No touch event
	buttonTouchQualifiedDisplay = 0x00;

	// Search for touch event
	for (elecNum = 0; elecNum < NUMBER_OF_BUTTON_ELECTRODES; elecNum++)//NUMBER_OF_BUTTON_ELECTRODES
	{
		// Electrode touched?
		if (elecTouch[elecNum][0] == 1)
		{
			// Load electrode number
			buttonTouchQualifiedDisplay |= 1<<elecNum;
			//break;
		}
	}
	if(keypresslast != (buttonTouchQualifiedDisplay?1:0))
	{
		keypresslast = (buttonTouchQualifiedDisplay?1:0);
		if(keypresslast == 1)
		{
			MotorEnable();
		}
	}

	

	return buttonTouchQualifiedDisplay;
}

uint8_t slider_Detect(uint8_t  buffer[])
{
	uint32_t signal[3],location;
	/*
	if(elecTouch[1][0]==0 && elecTouch[3][0]==0 && elecTouch[5][0]==0)
	{
		buffer[0] = 0;
		buffer[1] = 0;
		buffer[2] = 0;
		return 0xff;
	}
	*/
	if(elecDCTracker[1] > elecLPFilterData[1][0])
	{
		signal[0] = elecDCTracker[1] - elecLPFilterData[1][0];
		signal[0] >>= 8;   
	}	
	else
	{
		signal[0] = 0;
	}
	if(elecDCTracker[5] > elecLPFilterData[5][0])
	{
		signal[1] = elecDCTracker[5] - elecLPFilterData[5][0];
		signal[1] >>= 8;
	}	
	else
	{
		signal[1] = 0;
	}
	if(elecDCTracker[3] > elecLPFilterData[3][0])
	{
		signal[2] = elecDCTracker[3] - elecLPFilterData[3][0];
		signal[2] >>= 8;
	}	
	else
	{
		signal[2] = 0;
	}	            

	buffer[0] = signal[0];
	buffer[1] = signal[1];
	buffer[2] = signal[2];

	location = ((signal[2]*2 + signal[1])*50)/(signal[0]+signal[1]+signal[2]);
	return (uint8_t)location;
}

#endif
