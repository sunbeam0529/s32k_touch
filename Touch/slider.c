/****************************************************************************//*!
*
* @file  	slider.c
*
* @version  1.0.0.0
*
* @date     December-2017
*
* @brief    Slider electrode touch sense routines for S32K144
*
*******************************************************************************/

/*******************************************************************************
 * Includes
 *******************************************************************************/
#include "S32K118.h"
#include "ts_cfg.h"
#include "slider.h"
#include "gpio_inline_fcn2.h"
#include "filter.h"
#include "ets.h"

#if SLIDER_ENABLE
/*******************************************************************************
 * Variables
 ******************************************************************************/
// Touch qualification
uint8_t   sliderTouchQualified;
uint8_t	  sliderTouchQualifiedReport,sliderTouchQualifiedDisplay;
tBool hitHysteresis;
tBool positionDeciced;
tBool firstSliderTouch;
int8_t  positionCounter;
uint8_t thresholdNum;

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
extern uint8_t   firstSliderElecNum;
extern uint8_t   elecNum;
extern uint16_t   electrodeWakeUpActivateCounter;

// FrequencyHopping
extern uint8_t   frequencyID;

// Extern variables for slider electrode capacitance to equivalent voltage conversion

// Slider Touch qualification
// Slider data
int16_t sliderData;
int16_t sliderDiffData[NUMBER_OF_HOPPING_FREQUENCIES];
int16_t sliderSumData[NUMBER_OF_HOPPING_FREQUENCIES];
int16_t sliderSumLPFilterData[NUMBER_OF_HOPPING_FREQUENCIES];
int16_t sliderSumDataThreshold;
int16_t sliderStep;
int16_t sliderCentroidData[NUMBER_OF_HOPPING_FREQUENCIES];
tFrac32 sliderDiffLPFilterData[NUMBER_OF_HOPPING_FREQUENCIES];
tFrac32 sliderCentroidLPFilterData[NUMBER_OF_HOPPING_FREQUENCIES];

// Slider centroid filter buffer x(k-1), y(k-1)
extern tFrac32 FilterIIR1SliderDataBufferX[2][NUMBER_OF_HOPPING_FREQUENCIES], FilterIIR1SliderDataBufferY[2][NUMBER_OF_HOPPING_FREQUENCIES];

// Slider helper differential data thresholds for Freemaster
int16_t sliderDataThreshold[NUMBER_OF_THRESHOLDS_FREEMASTER + NUMBER_OF_SLIDER_THRESHOLDS] = {0};// PreInit to zero because of Freemaster
int16_t *sliderDataThresholdLast;

// Slider defines variables
int8_t numberOfHysteresisBlindPoints;

// Low power mode
extern uint8_t  lowPowerModeCtrl;

// FrequencyHopping
extern uint8_t   frequencyID,frequencyIDsave;
extern uint8_t   frequencyHoppingActivation;

// Decimation filter
extern int32_t   elecRawDataDF[NUMBER_OF_ELECTRODES][NUMBER_OF_HOPPING_FREQUENCIES];

// Oversampling
uint8_t sliderOversamplingActivationReport;


/*****************************************************************************
 *
 * Function: void SliderVarInit (void)
 *
 * Description: Init slider specific variables
 *
 *****************************************************************************/
void SliderVarInit(void)
{
	// Reset slider First touch indicator
	firstSliderTouch = TRUE;

	// Get default number of hysteresis blind points
	numberOfHysteresisBlindPoints = NUMBER_OF_HYSTERESIS_BLIND_POINTS;

	// Set the default slider addition data threshold of y-direction false touch canceling
	sliderSumDataThreshold = SLIDER_SUM_THRESHOLD;

	// Varying last difference threshold pointer assignment
	sliderDataThresholdLast = &sliderDataThreshold[NUMBER_OF_SLIDER_THRESHOLDS-1];

}
/*****************************************************************************
 *
 * Function: void Slider2PadsElectrodesSort(uint8_t electrodeNum, uint8_t* electrodeNumFirstPtr, uint8_t* electrodeNumSecondPtr)
 *
 * Description: Determine which electrode form slider electrode pair is first
 *
 * Note I: This function is suitable for slider consisting of 2 electrodes only
 *
 *****************************************************************************/
void Slider2PadsElectrodesSort(uint8_t electrodeNum, uint8_t* electrodeNumFirstPtr, uint8_t* electrodeNumSecondPtr)
{
	if (firstSliderElecNum % 2 == 0)// Is the total first electrode even numbered one?
	{
		// Decide which electrode of the 2  electrodes  is to be served
		if(electrodeNum % 2 == 0) // Is it the first one (even numbered one)?
		{
			// electrodeNum is even nembered electrode
			*electrodeNumFirstPtr = electrodeNum;
			*electrodeNumSecondPtr = electrodeNum + 1;
		}
		else if(electrodeNum % 2 == 1)// Is it the second one (odd numbered one)?
		{
			// electrodeNum is odd numbered electrode
			*electrodeNumFirstPtr = electrodeNum - 1;
			*electrodeNumSecondPtr = electrodeNum;
		}
	}
	else
	{
		// Decide which electrode of the 2  electrodes is to be served
		if(electrodeNum % 2 == 0) // Is it the first one (even numbered one)?
		{
			// electrodeNum is even nembered electrode
			*electrodeNumFirstPtr = electrodeNum - 1;
			*electrodeNumSecondPtr = electrodeNum;
		}
		else if(electrodeNum % 2 == 1)// Is it the second one (odd numbered one)?
		{
			// electrodeNum is odd numbered electrode
			*electrodeNumFirstPtr = electrodeNum;
			*electrodeNumSecondPtr = electrodeNum + 1;
		}
	}
}
/*****************************************************************************
 *
 * Function: void SliderSingleElecDSP(uint8_t electrodeNum)
 *
 * Description: Process raw data for single slider electrode
 *
 * Note I: This function is suitable for slider consisting of 2 electrodes only
 *
 *****************************************************************************/
void Slider2PadsElecDSP(uint8_t electrodeNum)
{
	uint8_t electrodeNumFirst, electrodeNumSecond;

	// Determine if the electrode served is first or second from the 2 slider electrodes
	Slider2PadsElectrodesSort(electrodeNum, &electrodeNumFirst, &electrodeNumSecond);

	// Update DC Tracker, if both electrodes not touched
	if (elecTouch[electrodeNumFirst][frequencyID] < 1 && elecTouch[electrodeNumSecond][frequencyID] < 1)
	{
		// Update DC Tracker
		ElecDCTrackerUpdate(electrodeNum);
	}

#if DECIMATION_FILTER
	// Decimation filtering
	DecimationFilter(electrodeNum);
#endif
	// Calculate the relative touch threshold of the electrode
	elecThresholdTouch[electrodeNum] = elecDCTracker[electrodeNum] - elecThresholdTouchDelta[electrodeNum];
	elecThresholdRelease[electrodeNum] = elecDCTracker[electrodeNum] - elecThresholdReleaseDelta[electrodeNum];

#if DECIMATION_FILTER
	// Filter Slider electrode DF signal using IIR LP filter
	elecLPFilterData[electrodeNum][frequencyID] = FilterIIR1(electrodeNum, (tFrac32)(elecRawDataDF[electrodeNum][frequencyID]), frequencyID);
#else
	// Filter Slider electrode raw signal using IIR LP filter
	elecLPFilterData[electrodeNum][frequencyID] = FilterIIR1(electrodeNum, (tFrac32)(elecRawData[electrodeNum][frequencyID]), frequencyID);
#endif

}


/*****************************************************************************
*
* Function: void Slider2PadsElecTouchDetect(uint8_t electrodeNum)
*
* Description: Detect slider electrode touch
*
* Note I: This function is suitable for slider consisting of 2 electrodes only
*
*****************************************************************************/
void Slider2PadsElecTouchDetect(uint8_t electrodeNum)
{
	// Slider electrode touched ? z-axis touch detection
	if (elecLPFilterData[electrodeNum][frequencyID] < elecThresholdTouch[electrodeNum])

	{
		// y-axis touch confirmation
		if (sliderSumLPFilterData[frequencyID] < sliderSumDataThreshold)
		{
			// Report touch detected and confirmed
			elecTouch[electrodeNum][frequencyID] = 2;

			// Load counter to do not return to the wake-up function
			// Do not return to the wake-up function, when electrode released and electrode signal value is above touch and bellow release threshold
			// Expected SR <= 1s
			electrodeWakeUpActivateCounter = ELEC_WAKEUP_ACTIVATE_COUNTER;

		}
		else
		{
			// Report touch detected, but not confirmed
			elecTouch[electrodeNum][frequencyID] = 1;
		}
	}
	// Electrode released?
	//else
	else if ((elecLPFilterData[electrodeNum][frequencyID] > elecThresholdRelease[electrodeNum]) && (elecTouch[electrodeNum][frequencyID] > 0))
	{
		// Report released
		elecTouch[electrodeNum][frequencyID] = 0;
	}

	// Fast release detection
	// Previously touched elec Electrode FAST RELEASED? z-axis touch detection
	if (elecTouch[electrodeNum][frequencyID] > 0)
	{
		// Slider electrode raw data risen above touch threshold?
		if (elecRawData[electrodeNum][frequencyID] > elecThresholdTouch[electrodeNum] || sliderSumData[frequencyID] > sliderSumDataThreshold)
		{
			// Report released
			elecTouch[electrodeNum][frequencyID] = 0;
		}
	}
}


/*****************************************************************************
 *
 * Function: void Slider2PadsSumDataCalculation(void)
 *
 * Description: Calculate slider sum data (el0-el1 etc.)
 *
 * Note I: This function is suitable for slider consisting of 2 electrodes only
 *
 *****************************************************************************/
void Slider2PadsSumDataCalculation(void)
{
	for (uint8_t electrodeNum = firstSliderElecNum; electrodeNum < (firstSliderElecNum + NUMBER_OF_SLIDER_ELECTRODES); electrodeNum = electrodeNum + NUMBER_OF_ELECTRODES_PER_SLIDER)
	{
	// Sum
	// Add the raw data of both electrodes, normalize them with respect to their different baselines
	sliderSumData[frequencyID] = (elecRawData[electrodeNum][frequencyID] + elecRawData[electrodeNum+1][frequencyID]) - (elecDCTracker[electrodeNum] + elecDCTracker[electrodeNum+1]);
	// Filter the sum data with 5Hz LPF IIR
	sliderSumLPFilterData[frequencyID] = FilterIIR1SliderData((tFrac32) sliderSumData[frequencyID], 0, frequencyID);
	}
}

#if (SLIDER_DATA_CALC_METHOD == DIFFERENCE_DATA)
/*****************************************************************************
 *
 * Function: int16_t Slider2PadsDiffDataCalculation(uint8_t freq)
 *
 * Description: Calculate slider difference data (el0-el1 etc.)
 *
 * Note I: This function is suitable for slider consisting of 2 electrodes only
 *
 *****************************************************************************/
int16_t Slider2PadsDiffDataCalculation(uint8_t freq)
{
	for (uint8_t electrodeNum = firstSliderElecNum; electrodeNum < (firstSliderElecNum + NUMBER_OF_SLIDER_ELECTRODES); electrodeNum = electrodeNum + NUMBER_OF_ELECTRODES_PER_SLIDER)
	{
	// Difference data
	// Subtract the raw data from each other, normalize them with respect to their different baselines
	sliderDiffData[freq] = (elecRawData[electrodeNum][freq] - elecRawData[electrodeNum+1][freq]) - (elecDCTracker[electrodeNum] - elecDCTracker[electrodeNum+1]);

	// Has slider been already touched?
	if (sliderTouchQualified > 0)
	{
		// Filter the diff data with 5Hz LPF IIR
		sliderDiffLPFilterData[freq] = FilterIIR1SliderData((tFrac32) sliderDiffData[freq], 1, freq);
	}
	else
	{
		// Init centroid IIR filter buffer
		FilterIIR1SliderDataBufferX[1][freq] = (tFrac32)sliderDiffData[freq]<< IIR_FILTER_VALUE_SHIFT;
		FilterIIR1SliderDataBufferY[1][freq] = (tFrac32)sliderDiffData[freq]<< IIR_FILTER_VALUE_SHIFT;

		// Set raw data as filtered for quick response
		sliderDiffLPFilterData[freq] = (tFrac32) sliderDiffData[freq];
	}


	}

	return sliderDiffLPFilterData[freq];
}
#endif

#if (SLIDER_DATA_CALC_METHOD == CENTROID)
/*****************************************************************************
 *
 * Function: int16_t Slider2PadsCentroidDataCalculation(uint8_t freq)
 *
 * Description: Calculate slider centroid data (el0-el1 etc.)
 *
 * Note I: This function is suitable for slider consisting of 2 electrodes only
 *
 *****************************************************************************/
int16_t Slider2PadsCentroidDataCalculation(uint8_t freq)
{
	for (uint8_t electrodeNum = firstSliderElecNum; electrodeNum < (firstSliderElecNum + NUMBER_OF_SLIDER_ELECTRODES); electrodeNum = electrodeNum + NUMBER_OF_ELECTRODES_PER_SLIDER)
	{
		int16_t centroid;

		// Centroid data = ((((elec0Num+1)*elec0Drop+(elec1Num+1)*elec1drop)/(elec0drop+elec1drop))*resolution)-resolution
		centroid = (int16_t) ((tFloat) (((1*(elecRawData[electrodeNum][freq]-elecDCTracker[electrodeNum]))+(2*(elecRawData[electrodeNum+1][freq]-elecDCTracker[electrodeNum+1]))))/((tFloat) (sliderSumData[freq]))*(SLIDER_CENTROID_RESOLUTION));
		centroid = centroid - SLIDER_CENTROID_RESOLUTION;

		// Correction
		if (centroid >= 0 && centroid <= SLIDER_CENTROID_RESOLUTION)
		{
			sliderCentroidData[freq] = (tFrac32) centroid;
		}

		// Has slider been already touched?
		if (sliderTouchQualified > 0)
		{
			// Filter the centroid data with 5Hz LPF IIR
			sliderCentroidLPFilterData[freq] = FilterIIR1SliderData((tFrac32) sliderCentroidData[freq], 1, freq);

		}
		else
		{
			// Init centroid IIR filter buffer
			FilterIIR1SliderDataBufferX[1][freq] = (tFrac32)sliderCentroidData[freq]<< IIR_FILTER_VALUE_SHIFT;
			FilterIIR1SliderDataBufferY[1][freq] = (tFrac32)sliderCentroidData[freq]<< IIR_FILTER_VALUE_SHIFT;

			// Set raw data as filtered for quick response
			sliderCentroidLPFilterData[freq] = (tFrac32) sliderCentroidData[freq];
		}

	}

	return sliderCentroidLPFilterData[freq];
}
#endif
/*****************************************************************************
 *
 * Function: void Slider2PadsTouchQualify(void)
 *
 * Description: Qualification of the slider touch event when using the differential data = ((el0-el1)-(el1-el0))
 *
 * Note I: This function is suitable for slider consisting of 2 electrodes only
 *
 *****************************************************************************/
void Slider2PadsTouchQualify(void)
{
	uint8_t f_count, freq;
	int16_t sliderDataTemp = 0;

	for (elecNum = firstSliderElecNum; elecNum < (firstSliderElecNum + NUMBER_OF_SLIDER_ELECTRODES); elecNum = elecNum + NUMBER_OF_ELECTRODES_PER_SLIDER)
	{
#if FREQUENCY_HOPPING
		// Reset frequency count
		f_count = 0;

		for (freq = 0; freq < NUMBER_OF_HOPPING_FREQUENCIES; freq++)
		{
			// At least one of the slider electrodes reported as touched AND the number of samples per cycle has been risen (from previous mcu duty cycle)
			if((elecTouch[elecNum][freq] > 1 || elecTouch[elecNum+1][freq] > 1) &&
					elecNumberOfSamples[elecNum] == elecNumberOfSamplesActive[elecNum])
			{
				f_count++;
			}
		}

		// Electrode has been touched on all frequencies?
		if (f_count == NUMBER_OF_HOPPING_FREQUENCIES)
#else
			// At least one of the slider electrodes reported as touched AND the number of samples per cycle has been risen (from previous mcu duty cycle)
			if ((elecTouch[elecNum][frequencyID] > 1 || elecTouch[elecNum+1][frequencyID] > 1) &&
					elecNumberOfSamples[elecNum] == elecNumberOfSamplesActive[elecNum])
#endif
			{
				// Report slider electrode touch for flextimer for background lightning, prevent entry to VLPS
				sliderTouchQualifiedReport = 1;
				// Reset  positionCounter of the slider RGB lights
				positionCounter = 1;
				// Set hysteresis detection to false
				hitHysteresis = FALSE;
				// Set decided position to false
				positionDeciced = FALSE;

				// Based on chosen slider calculation method
#if (SLIDER_DATA_CALC_METHOD == CENTROID)

				for (freq = 0; freq < NUMBER_OF_HOPPING_FREQUENCIES; freq++)
				{
					//Calculate Centroid for each freq
					sliderDataTemp += Slider2PadsCentroidDataCalculation(freq);
				}

				// Slider step recalculation
				sliderStep = (SLIDER_CENTROID_RESOLUTION*SLIDER_CENTROID_RESOLUTION_PCT/100) / NUMBER_OF_SLIDER_SEGMENTS;

				// Recalculate slider thresholds
				for(thresholdNum = 0; thresholdNum < NUMBER_OF_SLIDER_THRESHOLDS; thresholdNum++)
				{
					sliderDataThreshold[thresholdNum] = sliderStep*(thresholdNum+1) + (SLIDER_CENTROID_RESOLUTION - SLIDER_CENTROID_RESOLUTION*SLIDER_CENTROID_RESOLUTION_PCT/100)/2;
				}

#else
				int16_t sliderSumDataAvg = 0;

				for (uint8_t freq = 0; freq < NUMBER_OF_HOPPING_FREQUENCIES; freq++)
				{
					// Calculate Difference data for each freq
					sliderDataTemp += Slider2PadsDiffDataCalculation(freq);
					sliderSumDataAvg += sliderSumData[freq];
				}

				// Slider step recalculation
				sliderSumDataAvg = sliderSumDataAvg/NUMBER_OF_HOPPING_FREQUENCIES;

				sliderStep = sliderSumDataAvg/NUMBER_OF_SLIDER_SEGMENTS;

				// Recalculate slider thresholds - adaptive thresholds
				for(thresholdNum = 0; thresholdNum < NUMBER_OF_SLIDER_THRESHOLDS; thresholdNum++)
				{
					sliderDataThreshold[NUMBER_OF_SLIDER_THRESHOLDS-1-thresholdNum]=(sliderSumDataAvg/2)-sliderStep*(NUMBER_OF_SLIDER_THRESHOLDS-thresholdNum);
				}
#endif

				// Make avarage of the slider data from all frequencies
				sliderData = sliderDataTemp/NUMBER_OF_HOPPING_FREQUENCIES;
				// Decide the position of finger based on thresholds in cfg
				// At least one threshold defined in cfg?
#if (NUMBER_OF_SLIDER_THRESHOLDS > 0)

				// Are the data below the first threshold - threshold[0]?
				if (sliderData < sliderDataThreshold[0])
				{
					// Check if the data had hit the hysteresis interval below the first threshold
					if (sliderData > (sliderDataThreshold[0] - numberOfHysteresisBlindPoints))
					{
						// Is this a first touch of the slider? -> qualify the position anyway
						if(firstSliderTouch == TRUE)
						{
							sliderTouchQualified = positionCounter;
							positionDeciced = TRUE;
							firstSliderTouch = FALSE;
						}

						else
							// It hit the blind hysteresis interval and the first touch has already been done - set hitHysteresis True
							hitHysteresis = TRUE;
					}

					// Else qualify the position on slider
					else
					{
						sliderTouchQualified = positionCounter;
						positionDeciced = TRUE;
						firstSliderTouch = FALSE;
					}
				}

#if (NUMBER_OF_SLIDER_THRESHOLDS > 1)// More than one threshold defined in cfg?

				// Are the data somewhere between the threshold[0] and the last threshold then?
				else if((sliderData >= sliderDataThreshold[0]) &&
						(sliderData < *sliderDataThresholdLast))
				{

					// For cycle to determine the position across the thresholds - between the threshold[0] and the last threshold
					for(thresholdNum = 0; thresholdNum<NUMBER_OF_SLIDER_THRESHOLDS; thresholdNum++)
					{
						// Increase the  positionCounter of the RGB lights by one
						positionCounter++;

						if (sliderData >= sliderDataThreshold[thresholdNum] &&
								sliderData < sliderDataThreshold[thresholdNum+1])
						{
							// Now check if the data had hit the top or bottom hysteresis interval within the e.g first and e.g second threshold (under first and above second threshold)
							if (sliderData > (sliderDataThreshold[thresholdNum] + numberOfHysteresisBlindPoints) &&
									sliderData < (sliderDataThreshold[thresholdNum+1] - numberOfHysteresisBlindPoints))
							{
								// The data are in the active area = between the hysteresis intervals - thus qualify the position on slider
								sliderTouchQualified = positionCounter;
								positionDeciced = TRUE;
								firstSliderTouch = FALSE;
							}

							else
							{
								// Is this a first touch of the slider? -> qualify the position anyway
								if(firstSliderTouch == TRUE)
								{
									sliderTouchQualified = positionCounter;
									positionDeciced = TRUE;
									firstSliderTouch = FALSE;
								}

								else
									// It hit the blind hysteresis interval and the first touch has already been done - set hitHysteresis True
									hitHysteresis = TRUE;
							}

							// Exit for cycle
							break;
						}
					}
				}

#endif
				// The data did not fit between any thresholds - are the data above the last threshold?
				else
				{
					// Check if the data were previously marked as within hysteresis interval
					if(hitHysteresis == TRUE || positionDeciced == TRUE)
					{
						// Do not change the sliderTouchQualified
					}

					// Check for hysteresis above the last threshold (similar principle as checking for hysteresis below the first threshold)
					else
					{
						if(sliderData >= *sliderDataThresholdLast &&
								sliderData < (*sliderDataThresholdLast + numberOfHysteresisBlindPoints))
						{
							// Is this a first touch of the slider? -> qualify the position anyway as the highest
							if(firstSliderTouch == TRUE)
							{
								sliderTouchQualified = NUMBER_OF_SLIDER_SEGMENTS;
								positionDeciced = TRUE;
								firstSliderTouch = FALSE;
							}

							else
								// It hit the blind hysteresis interval and the first touch has already been done - set hitHysteresis True
								hitHysteresis = TRUE;

						}

						// The data were not within the hysteresis interval
						else
						{
							// Qualify the position on slider as the highest
							sliderTouchQualified = NUMBER_OF_SLIDER_SEGMENTS;
							positionDeciced = TRUE;
							firstSliderTouch = FALSE;
						}
					}
				}
#endif

			}

		// Else no touch?
			else
			{
				{
					// Report slider electrode release for flextimer for background lightning and for jumping to VLPS
					sliderTouchQualifiedReport = 0;

					// Report no slider qualified touch
					sliderTouchQualified = 0;

					// Reset the firstSliderTouch variable
					firstSliderTouch = TRUE;
				}
			}
	}
}

/*****************************************************************************
 *
 * Function: uint8_t Slider_RGBLED_Ctrl(void)
 *
 * Description: Display slider touch event
 *
 *****************************************************************************/
uint8_t Slider_RGBLED_Ctrl(void)
{

	// SLIDER RGB
	// No touch event
	sliderTouchQualifiedDisplay = 0;

	// Search for touch event
	// Slider Electrode touched?
	if (sliderTouchQualified > 0)
	{
		// Load slider electrode finger position
		sliderTouchQualifiedDisplay = sliderTouchQualified;
	}

	// Display slider touch event
	switch(sliderTouchQualifiedDisplay)
	{

	case 0:// Slider electrode not touched - no light
	{

		break;
	}
	case 1:// Slider electrodes touched, sliderDifferenceDataDouble < sliderDiffThreshold[0] - white
	{
		// Turn ON RED LED
		LedRedON();
		// Turn ON GREEN LED
		LedGreenON();
		// Turn ON BLUE LED
		LedBlueON();

		break;
	}

	case 2:// Slider electrodes touched, sliderDiffThreshold[0] < sliderDifferenceDataDouble < sliderDiffThreshold[1]- red
	{
		// Turn ON RED LED
		LedRedON();
		// Turn OFF GREEN LED
		LedGreenOFF();
		// Turn OFF BLUE LED
		LedBlueOFF();

		break;
	}



	case 3:// Slider electrodes touched, sliderDiffThreshold[1] < sliderDifferenceDataDouble < sliderDiffThreshold[2] - yellow
	{
		// Turn ON RED LED
		LedRedON();
		// Turn ON GREEN LED
		LedGreenON();
		// Turn OFF BLUE LED
		LedBlueOFF();

		break;
	}



	case 4:// Slider electrodes touched, sliderDiffThreshold[2]  < sliderDifferenceDataDouble < sliderDiffThreshold[3] - green
	{
		// Turn OFF RED LED
		LedRedOFF();
		// Turn ON GREEN LED
		LedGreenON();
		// Turn OFF BLUE LED
		LedBlueOFF();

		break;
	}



	case 5:// Slider electrodes touched, sliderDiffThreshold[3] < sliderDifferenceDataDouble < sliderDiffThreshold[4] - blue
	{
		// Turn OFF RED LED
		LedRedOFF();
		// Turn OFF GREEN LED
		LedGreenOFF();
		// Turn ON BLUE LED
		LedBlueON();

		break;
	}



	case 6:// Slider electrodes touched, sliderDiffThreshold[4] < sliderDifferenceDataDouble < sliderDiffThreshold[5] - purple
	{
		// Turn ON RED LED
		LedRedON();
		// Turn OFF GREEN LED
		LedGreenOFF();
		// Turn ON BLUE LED
		LedBlueON();

		break;
	}

	case 7:// Slider electrodes touched, sliderDiffThreshold[5] < sliderDifferenceDataDouble < sliderDiffThreshold[6] - azure
	{
		// Turn OFF RED LED
		LedRedOFF();
		// Turn ON GREEN LED
		LedGreenON();
		// Turn ON BLUE LED
		LedBlueON();

		break;
	}
	case 8:// Slider electrodes touched, sliderDifferenceDataDouble > sliderDiffThreshold[6] - white again
	{
		// Turn ON RED LED
		LedRedON();
		// Turn ON GREEN LED
		LedGreenON();
		// Turn ON BLUE LED
		LedBlueON();

		break;
	}

	}

	return sliderTouchQualifiedDisplay;
}

// Slider enable
#endif
