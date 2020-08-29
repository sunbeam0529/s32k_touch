/****************************************************************************//*!
*
* @file  	ets.c
*
* @version  1.0.0.0
*
* @date     September-2017
*
* @brief    Electrode touch sense routines for S32K144
*
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include "S32K118.h"
#include "ets.h"
#include "ts_cfg.h"
#include "filter.h"
#include "lptmr.h"
#include "gpio.h"
#include "gpio_inline_fcn1.h"
#include "adc.h"
#include "adc_inline_fcn1.h"
#include "touch_buttons.h"
#include "slider.h"
#include "wake_up.h"
#include "flextimer.h"
#include "dma.h"
#include "power_mode.h"
#include "lpit.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
// Button Electrode structure
tElecStruct  elecStruct[NUMBER_OF_ELECTRODES];
// Guard pin structure
tGuardStruct guardStruct[1];

// Electrode capacitance to equivalent voltage conversion
int16_t   elecNumberOfSamples[NUMBER_OF_ELECTRODES];
int32_t   elecRawData[NUMBER_OF_ELECTRODES][NUMBER_OF_HOPPING_FREQUENCIES];
uint32_t  elecRawDataCalc;

int16_t   elecNumberOfSamplesActive[NUMBER_OF_ELECTRODES];
int16_t   elecNumberOfSamplesIdle[NUMBER_OF_ELECTRODES];

uint8_t   firstButtonElecNum, firstSliderElecNum, firstWakeupElecNum;

#if (NUMBER_OF_BUTTON_ELECTRODES > 0)
uint16_t   buttonSamplesBuffer[NUMBER_OF_BUTTON_ELECTRODES][(NUMBER_OF_PRESAMPLES + NUMBER_OF_SAMPLES_PER_BUTTON_ELEC_ACTIVE)];
#endif
#if (NUMBER_OF_SLIDER_ELECTRODES > 0)
uint16_t   sliderSamplesBuffer[NUMBER_OF_SLIDER_ELECTRODES][(NUMBER_OF_PRESAMPLES + NUMBER_OF_SAMPLES_PER_SLIDER_ELEC_ACTIVE)];
#endif
#if (NUMBER_OF_WAKEUP_ELECTRODES > 0)
uint16_t   wakeupSamplesBuffer[NUMBER_OF_WAKEUP_ELECTRODES][(NUMBER_OF_PRESAMPLES + NUMBER_OF_SAMPLES_PER_WAKEUP_ELEC)];
#endif

// Electrode DC tracker self-trim after power-up or reset
int32_t   elecSelfTrimBuffer[NUMBER_OF_ELECTRODES];
uint16_t  elecSelfTrimBufferCounter[NUMBER_OF_ELECTRODES];

// DC Tracker
int32_t   elecDCTrackerBuffer[NUMBER_OF_ELECTRODES];
int32_t   elecDCTracker[NUMBER_OF_ELECTRODES];
uint8_t   elecDCTrackerShift[NUMBER_OF_ELECTRODES];
uint8_t   elecDCTrackerShiftActive[NUMBER_OF_ELECTRODES];
uint8_t   elecDCTrackerShiftIdle[NUMBER_OF_ELECTRODES];

// LP Filter
tFrac32   elecLPFilterData[NUMBER_OF_ELECTRODES][NUMBER_OF_HOPPING_FREQUENCIES];
uint8_t   elecLPFilterType[NUMBER_OF_ELECTRODES];

// Mean and Median Filter
// Electrodes window sizes for median and mean
extern uint16_t FilterMedianWindow[NUMBER_OF_ELECTRODES];
extern uint16_t FilterMeanWindow[NUMBER_OF_ELECTRODES];

// Detector
int32_t   elecThresholdTouch[NUMBER_OF_ELECTRODES], elecThresholdRelease[NUMBER_OF_ELECTRODES];
int16_t   elecThresholdTouchDelta[NUMBER_OF_ELECTRODES], elecThresholdReleaseDelta[NUMBER_OF_ELECTRODES];
uint8_t   elecTouch[NUMBER_OF_ELECTRODES][NUMBER_OF_HOPPING_FREQUENCIES];

// Decimation filter
int32_t   elecRawDataDF[NUMBER_OF_ELECTRODES][NUMBER_OF_HOPPING_FREQUENCIES];

uint16_t sampleNum;
uint16_t  chargeDistributionPeriod, chargeDistributionPeriodTmp;
uint8_t samplesReadyFlag;
uint8_t electrodesSelfTrimDoneFlag;
uint8_t proximityDetectedFlag;
uint8_t   elecNum, elecNumBaselineUpdate ;

// Wake up
uint16_t   electrodeWakeUpActivateCounter;

// Low power mode
extern uint8_t  lowPowerModeCtrl;

extern uint8_t dmaSeqScanFirstElec, dmaSeqScanLastElec;
extern uint8_t scanningIdleFlag;

// Jittering
int16_t jitterRead;
uint8_t jitterIndex;

// FrequencyHopping
uint8_t   frequencyID;
uint16_t freqencyIDtimeout[NUMBER_OF_HOPPING_FREQUENCIES];
uint8_t cpuFHScanFlag;
int32_t frequencyStd[NUMBER_OF_ELECTRODES][NUMBER_OF_HOPPING_FREQUENCIES];
extern uint8_t lpit_clock;

// Oversampling
uint8_t electrodesOversamplingActivationReport;

// Backlight PWM duty cycle from 0 to 100
extern uint16_t  backlightPWMDutyCycle;

// Backlight ON counter
uint32_t backlightCounter;
uint16_t  proximityDetectedCounter;

#if (GUARD)
/*****************************************************************************
*
* Function: void GuardStructureInit(void)
*
* Description: Guard structure init
*
*****************************************************************************/
void GuardStructureInit(void)
{
	// Load guard hardware data
	guardStruct[0].portBasePtr = GUARD_PORT;
	guardStruct[0].gpioBasePtr = GUARD_GPIO;
	guardStruct[0].pinNumberGuard = GUARD_GPIO_PIN;
	guardStruct[0].portMask = GUARD_PORT_MASK;

	// Guard pin as GPIO
    guardStruct[0].portBasePtr->PCR[guardStruct[0].pinNumberGuard] = PORT_PCR_MUX(0b01);
    // Guard pin output
    guardStruct[0].gpioBasePtr->PDDR |= 1 << guardStruct[0].pinNumberGuard;
}
#endif

/*****************************************************************************
 *
 * Function: void ElectrodesStructInit(void)
 *
 * Description: Init electrodes structures
 *
 * Note I: The electrodes are sorted in the structure in following order: buttons, slider, wakeup
 *
 *****************************************************************************/
void ElectrodesStructInit(void)
{
	elecNum = 0;

#ifdef ELEC0
	// Load electrode 0 hardware data
	elecStruct[elecNum].adcBasePtr = ELEC0_ADC;
	elecStruct[elecNum].adcChNum = ELEC0_ADC_CHANNEL;
	elecStruct[elecNum].portBasePtr = ELEC0_PORT;
	elecStruct[elecNum].gpioBasePtr = ELEC0_GPIO;
	elecStruct[elecNum].pinNumberElec = ELEC0_ELEC_GPIO_PIN;
	elecStruct[elecNum].pinNumberCext = ELEC0_CEXT_GPIO_PIN;
	elecStruct[elecNum].portMask = ELEC0_PORT_MASK;
	elecStruct[elecNum].ELEC_DMAMUX = ELEC0_DMAMUX;
	elecStruct[elecNum].ELEC_TRGMUX = ELEC0_TRGMUX;
	elecStruct[elecNum].type = ELEC0_TYPE;
	// Load elec electrode 0 application data
	elecThresholdTouchDelta[elecNum] = ELEC0_TOUCH_THRESHOLD_DELTA;
	elecThresholdReleaseDelta[elecNum] = ELEC0_RELEASE_THRESHOLD_DELTA;

	elecNum++;
#endif

#ifdef ELEC1
	// Load electrode 1 hardware data
	elecStruct[elecNum].adcBasePtr = ELEC1_ADC;
	elecStruct[elecNum].adcChNum = ELEC1_ADC_CHANNEL;
	elecStruct[elecNum].portBasePtr = ELEC1_PORT;
	elecStruct[elecNum].gpioBasePtr = ELEC1_GPIO;
	elecStruct[elecNum].pinNumberElec = ELEC1_ELEC_GPIO_PIN;
	elecStruct[elecNum].pinNumberCext = ELEC1_CEXT_GPIO_PIN;
	elecStruct[elecNum].portMask = ELEC1_PORT_MASK;
	elecStruct[elecNum].ELEC_DMAMUX = ELEC1_DMAMUX;
	elecStruct[elecNum].ELEC_TRGMUX = ELEC1_TRGMUX;
	elecStruct[elecNum].type = ELEC1_TYPE;
	// Load elec electrode 1 application data
	elecThresholdTouchDelta[elecNum] = ELEC1_TOUCH_THRESHOLD_DELTA;
	elecThresholdReleaseDelta[elecNum] = ELEC1_RELEASE_THRESHOLD_DELTA;

	elecNum++;
#endif

#ifdef ELEC2
	// Load electrode 2 hardware data
	elecStruct[elecNum].adcBasePtr = ELEC2_ADC;
	elecStruct[elecNum].adcChNum = ELEC2_ADC_CHANNEL;
	elecStruct[elecNum].portBasePtr = ELEC2_PORT;
	elecStruct[elecNum].gpioBasePtr = ELEC2_GPIO;
	elecStruct[elecNum].pinNumberElec = ELEC2_ELEC_GPIO_PIN;
	elecStruct[elecNum].pinNumberCext = ELEC2_CEXT_GPIO_PIN;
	elecStruct[elecNum].portMask = ELEC2_PORT_MASK;
	elecStruct[elecNum].ELEC_DMAMUX = ELEC2_DMAMUX;
	elecStruct[elecNum].ELEC_TRGMUX = ELEC2_TRGMUX;
	elecStruct[elecNum].type = ELEC2_TYPE;
	// Load elec electrode 2 application data
	elecThresholdTouchDelta[elecNum] = ELEC2_TOUCH_THRESHOLD_DELTA;
	elecThresholdReleaseDelta[elecNum] = ELEC2_RELEASE_THRESHOLD_DELTA;

	elecNum++;
#endif

#ifdef ELEC3
	// Load electrode 3 hardware data
	elecStruct[elecNum].adcBasePtr = ELEC3_ADC;
	elecStruct[elecNum].adcChNum = ELEC3_ADC_CHANNEL;
	elecStruct[elecNum].portBasePtr = ELEC3_PORT;
	elecStruct[elecNum].gpioBasePtr = ELEC3_GPIO;
	elecStruct[elecNum].pinNumberElec = ELEC3_ELEC_GPIO_PIN;
	elecStruct[elecNum].pinNumberCext = ELEC3_CEXT_GPIO_PIN;
	elecStruct[elecNum].portMask = ELEC3_PORT_MASK;
	elecStruct[elecNum].ELEC_DMAMUX = ELEC3_DMAMUX;
	elecStruct[elecNum].ELEC_TRGMUX = ELEC3_TRGMUX;
	elecStruct[elecNum].type = ELEC3_TYPE;
	// Load elec electrode 3 application data
	elecThresholdTouchDelta[elecNum] = ELEC3_TOUCH_THRESHOLD_DELTA;
	elecThresholdReleaseDelta[elecNum] = ELEC3_RELEASE_THRESHOLD_DELTA;

	elecNum++;
#endif

#ifdef ELEC4
	// Load electrode 4 hardware data
	elecStruct[elecNum].adcBasePtr = ELEC4_ADC;
	elecStruct[elecNum].adcChNum = ELEC4_ADC_CHANNEL;
	elecStruct[elecNum].portBasePtr = ELEC4_PORT;
	elecStruct[elecNum].gpioBasePtr = ELEC4_GPIO;
	elecStruct[elecNum].pinNumberElec = ELEC4_ELEC_GPIO_PIN;
	elecStruct[elecNum].pinNumberCext = ELEC4_CEXT_GPIO_PIN;
	elecStruct[elecNum].portMask = ELEC4_PORT_MASK;
	elecStruct[elecNum].ELEC_DMAMUX = ELEC4_DMAMUX;
	elecStruct[elecNum].ELEC_TRGMUX = ELEC4_TRGMUX;
	elecStruct[elecNum].type = ELEC4_TYPE;
	// Load elec electrode 4 application data
	elecThresholdTouchDelta[elecNum] = ELEC4_TOUCH_THRESHOLD_DELTA;
	elecThresholdReleaseDelta[elecNum] = ELEC4_RELEASE_THRESHOLD_DELTA;

	elecNum++;
#endif

#ifdef ELEC5
	// Load electrode 5 hardware data
	elecStruct[elecNum].adcBasePtr = ELEC5_ADC;
	elecStruct[elecNum].adcChNum = ELEC5_ADC_CHANNEL;
	elecStruct[elecNum].portBasePtr = ELEC5_PORT;
	elecStruct[elecNum].gpioBasePtr = ELEC5_GPIO;
	elecStruct[elecNum].pinNumberElec = ELEC5_ELEC_GPIO_PIN;
	elecStruct[elecNum].pinNumberCext = ELEC5_CEXT_GPIO_PIN;
	elecStruct[elecNum].portMask = ELEC5_PORT_MASK;
	elecStruct[elecNum].ELEC_DMAMUX = ELEC5_DMAMUX;
	elecStruct[elecNum].ELEC_TRGMUX = ELEC5_TRGMUX;
	elecStruct[elecNum].type = ELEC5_TYPE;
	// Load elec electrode 5 application data
	elecThresholdTouchDelta[elecNum] = ELEC5_TOUCH_THRESHOLD_DELTA;
	elecThresholdReleaseDelta[elecNum] = ELEC5_RELEASE_THRESHOLD_DELTA;

	elecNum++;
#endif

#ifdef ELEC6
	// Load electrode 6 hardware data
	elecStruct[elecNum].adcBasePtr = ELEC6_ADC;
	elecStruct[elecNum].adcChNum = ELEC6_ADC_CHANNEL;
	elecStruct[elecNum].portBasePtr = ELEC6_PORT;
	elecStruct[elecNum].gpioBasePtr = ELEC6_GPIO;
	elecStruct[elecNum].pinNumberElec = ELEC6_ELEC_GPIO_PIN;
	elecStruct[elecNum].pinNumberCext = ELEC6_CEXT_GPIO_PIN;
	elecStruct[elecNum].portMask = ELEC6_PORT_MASK;
	elecStruct[elecNum].ELEC_DMAMUX = ELEC6_DMAMUX;
	elecStruct[elecNum].ELEC_TRGMUX = ELEC6_TRGMUX;
	elecStruct[elecNum].type = ELEC6_TYPE;
	// Load electrode 6 application data
	elecThresholdTouchDelta[elecNum] = ELEC6_TOUCH_THRESHOLD_DELTA;
	elecThresholdReleaseDelta[elecNum] = ELEC6_RELEASE_THRESHOLD_DELTA;

	elecNum++;
#endif

#ifdef ELEC7
	// Load electrode 7 hardware data
	elecStruct[elecNum].adcBasePtr = ELEC7_ADC;
	elecStruct[elecNum].adcChNum = ELEC7_ADC_CHANNEL;
	elecStruct[elecNum].portBasePtr = ELEC7_PORT;
	elecStruct[elecNum].gpioBasePtr = ELEC7_GPIO;
	elecStruct[elecNum].pinNumberElec = ELEC7_ELEC_GPIO_PIN;
	elecStruct[elecNum].pinNumberCext = ELEC7_CEXT_GPIO_PIN;
	elecStruct[elecNum].portMask = ELEC7_PORT_MASK;
	elecStruct[elecNum].ELEC_DMAMUX = ELEC7_DMAMUX;
	elecStruct[elecNum].ELEC_TRGMUX = ELEC7_TRGMUX;
	elecStruct[elecNum].type = ELEC7_TYPE;
	// Load electrode 7 application data
	elecThresholdTouchDelta[elecNum] = ELEC7_TOUCH_THRESHOLD_DELTA;
	elecThresholdReleaseDelta[elecNum] = ELEC7_RELEASE_THRESHOLD_DELTA;

	elecNum++;
#endif

#ifdef ELEC8
	// Load electrode 8 hardware data
	elecStruct[elecNum].adcBasePtr = ELEC8_ADC;
	elecStruct[elecNum].adcChNum = ELEC8_ADC_CHANNEL;
	elecStruct[elecNum].portBasePtr = ELEC8_PORT;
	elecStruct[elecNum].gpioBasePtr = ELEC8_GPIO;
	elecStruct[elecNum].pinNumberElec = ELEC8_ELEC_GPIO_PIN;
	elecStruct[elecNum].pinNumberCext = ELEC8_CEXT_GPIO_PIN;
	elecStruct[elecNum].portMask = ELEC8_PORT_MASK;
	elecStruct[elecNum].ELEC_DMAMUX = ELEC8_DMAMUX;
	elecStruct[elecNum].ELEC_TRGMUX = ELEC8_TRGMUX;
	elecStruct[elecNum].type = ELEC8_TYPE;
	// Load electrode 8 application data
	elecThresholdTouchDelta[elecNum] = ELEC8_TOUCH_THRESHOLD_DELTA;
	elecThresholdReleaseDelta[elecNum] = ELEC8_RELEASE_THRESHOLD_DELTA;

	elecNum++;
#endif
}

/*****************************************************************************
 *
 * Function: void ElecAdcChannelFix(void)
 *
 * Description: Fixes ADC channel number of the electrode in case it is higher than 15
 *
 *****************************************************************************/
void ElecAdcChannelFix(uint8_t electrodeNum)
{
	// ADC channel number higher than 15?
	if(elecStruct[electrodeNum].adcChNum > 15)
	{
		// Add offset
		elecStruct[electrodeNum].adcChNum += ELECTRODE_ADC_CHANNEL_OFFSET;
	}
}

/*****************************************************************************
 *
 * Function: void ElecLPFilterTypeInit(uint8_t electrodeNum)
 *
 * Description: Init the LPFilter Type of the electrode
 *
 *****************************************************************************/
void ElecLPFilterTypeInit(uint8_t electrodeNum)
{
	// Set idle mode number of samples per electrode to be taken
#if (NUMBER_OF_BUTTON_ELECTRODES > 0)
	if (elecStruct[electrodeNum].type == BUTTON)
	{
		elecLPFilterType[electrodeNum] = FILTER_1;
	}
#endif
#if (NUMBER_OF_SLIDER_ELECTRODES > 0)
	if (elecStruct[electrodeNum].type == SLIDER)
	{
		elecLPFilterType[electrodeNum] = FILTER_2;
	}
#endif
#if (NUMBER_OF_WAKEUP_ELECTRODES > 0)
	if (elecStruct[electrodeNum].type == WAKEUP)
	{
		elecLPFilterType[electrodeNum] = FILTER_3;
	}
#endif
}

/*****************************************************************************
 *
 * Function: void ElecNumOfSamplesPerElecInit(uint8_t electrodeNum)
 *
 * Description: Init number of samples for each electrode type to be taken in various modes
 *
 *****************************************************************************/
void ElecNumOfSamplesPerElecInit(uint8_t electrodeNum)
{
	// Set idle mode number of samples per electrode to be taken
#if (NUMBER_OF_BUTTON_ELECTRODES > 0)
	if (elecStruct[electrodeNum].type == BUTTON)
	{
		elecNumberOfSamplesActive[electrodeNum] = NUMBER_OF_SAMPLES_PER_BUTTON_ELEC_ACTIVE;
		elecNumberOfSamplesIdle[electrodeNum] = NUMBER_OF_SAMPLES_PER_BUTTON_ELEC_IDLE;
		elecNumberOfSamples[electrodeNum] = elecNumberOfSamplesIdle[electrodeNum];
	}
#endif
#if (NUMBER_OF_SLIDER_ELECTRODES > 0)
	if (elecStruct[electrodeNum].type == SLIDER)
	{
		elecNumberOfSamplesActive[electrodeNum] = NUMBER_OF_SAMPLES_PER_SLIDER_ELEC_ACTIVE;
		elecNumberOfSamplesIdle[electrodeNum] = NUMBER_OF_SAMPLES_PER_SLIDER_ELEC_IDLE;
		elecNumberOfSamples[electrodeNum] = elecNumberOfSamplesIdle[electrodeNum];
	}
#endif
#if (NUMBER_OF_WAKEUP_ELECTRODES > 0)
	if (elecStruct[electrodeNum].type == WAKEUP)
	{
		elecNumberOfSamplesActive[electrodeNum] = NUMBER_OF_SAMPLES_PER_WAKEUP_ELEC;
		elecNumberOfSamplesIdle[electrodeNum] = NUMBER_OF_SAMPLES_PER_WAKEUP_ELEC;
		elecNumberOfSamples[electrodeNum] = elecNumberOfSamplesIdle[electrodeNum];
	}
#endif
}

/*****************************************************************************
 *
 * Function: void ElecOversamplingChange(uint8_t electrodeNum, uint8_t changeType)
 *
 * Description: Change electrodes DC tracker adjustments for oversampling
 *
 *****************************************************************************/
void ElecOversamplingDCTrackerChange(uint8_t electrodeNum, uint8_t changeType)
{
	// Idle mode?
	if (changeType == IDLE)
	{
		// Update DC tracker shift
		elecDCTrackerShift[electrodeNum] = elecDCTrackerShiftIdle[electrodeNum] ;

		// Set DCTracker value to correspond to number of samples taken in idle state
		elecDCTracker[electrodeNum] = ((elecDCTracker[electrodeNum]*elecNumberOfSamplesIdle[electrodeNum])/elecNumberOfSamplesActive[electrodeNum]);

		// Electrodes LP IIR filter init value (for FreeMASTER)
		elecLPFilterData[elecNum][frequencyID] = elecDCTracker[elecNum];
	}
	else if (changeType == ACTIVE)
	{
		// Update DC tracker shift
		elecDCTrackerShift[electrodeNum] = elecDCTrackerShiftActive[electrodeNum] ;

		// Set DCTracker value to correspond to number of samples taken in idle state
		elecDCTracker[electrodeNum] = ((elecDCTracker[electrodeNum]*elecNumberOfSamplesActive[electrodeNum])/elecNumberOfSamplesIdle[electrodeNum]);
	}

	// Load DC tracker buffer
	elecDCTrackerBuffer[electrodeNum] = (elecDCTracker[electrodeNum]) << ((elecDCTrackerShift[electrodeNum]));

	// Button electrodes touch and release thresholds recalculation
	elecThresholdTouch[electrodeNum] = elecDCTracker[electrodeNum] - elecThresholdTouchDelta[electrodeNum];
	elecThresholdRelease[electrodeNum] = elecDCTracker[electrodeNum] - elecThresholdReleaseDelta[electrodeNum];

#if DECIMATION_FILTER
	// All used scanning frequencies
	for (uint8_t freq = 0; freq < NUMBER_OF_HOPPING_FREQUENCIES; freq++)
	{

		// Load DF array with DC tracker value - used when oversampling with EGS
		elecRawDataDF[electrodeNum][freq] = elecDCTracker[electrodeNum];

	}
#endif
}

/*****************************************************************************
 *
 * Function: void ElecDCTrackerShiftInit(uint8_t electrodeNum)
 *
 * Description: Init DCTracker shift values for electrodes
 *
 *****************************************************************************/
void ElecDCTrackerShiftInit(uint8_t electrodeNum)
{
	// Load the shift values
#if (NUMBER_OF_BUTTON_ELECTRODES > 0)
	if (elecStruct[electrodeNum].type == BUTTON)
	{

		elecDCTrackerShiftActive[electrodeNum] = BUTTON_DCTRACKER_FILTER_FACTOR_ACTIVE;
		elecDCTrackerShiftIdle[electrodeNum] = BUTTON_DCTRACKER_FILTER_FACTOR_IDLE;
		elecDCTrackerShift[electrodeNum] = elecDCTrackerShiftIdle[electrodeNum];
	}
#endif
#if (NUMBER_OF_SLIDER_ELECTRODES > 0)
	if (elecStruct[electrodeNum].type == SLIDER)
	{

		elecDCTrackerShiftActive[electrodeNum] = SLIDER_DCTRACKER_FILTER_FACTOR_ACTIVE;
		elecDCTrackerShiftIdle[electrodeNum] = SLIDER_DCTRACKER_FILTER_FACTOR_IDLE;
		elecDCTrackerShift[electrodeNum] = elecDCTrackerShiftIdle[electrodeNum];
	}
#endif
#if (NUMBER_OF_WAKEUP_ELECTRODES > 0)
	if (elecStruct[electrodeNum].type == WAKEUP)
	{
		elecDCTrackerShiftActive[electrodeNum] = WAKEUP_DCTRACKER_FILTER_FACTOR;
		elecDCTrackerShiftIdle[electrodeNum] = WAKEUP_DCTRACKER_FILTER_FACTOR;
		elecDCTrackerShift[electrodeNum] = elecDCTrackerShiftIdle[electrodeNum];
	}
#endif
}

/*****************************************************************************
 *
 * Function: void ElecNumOfSamplesPerElecChange(uint8_t changeType)
 *
 * Description: Change number of samples for each electrode type to be taken
 *
 *****************************************************************************/
void ElecNumOfSamplesPerElecChange(uint8_t electrodeNum, uint8_t changeType)
{
	// Idle mode?
	if (changeType == IDLE)
	{
		elecNumberOfSamples[electrodeNum] = elecNumberOfSamplesIdle[electrodeNum];
	}
	// Active mode?
	else if (changeType == ACTIVE)
	{
		elecNumberOfSamples[electrodeNum] = elecNumberOfSamplesActive[electrodeNum];
	}
}

/*****************************************************************************
 *
 * Function: void ElectrodesTouchSenseInit(void)
 *
 * Description: Init electrodes structures and variables
 *
 *****************************************************************************/
void ElectrodesTouchSenseInit(void)
{
	// Init electrodes structure
	ElectrodesStructInit();

	// All electrodes
	for(elecNum = 0; elecNum < NUMBER_OF_ELECTRODES; elecNum++)
	{
		// Cext pin ADC channel number correction (in case higher ADC channel number used)
		ElecAdcChannelFix(elecNum);
		// Init number of samples to be taken per electrode
		ElecNumOfSamplesPerElecInit(elecNum);
		// DCTracker shift init
		ElecDCTrackerShiftInit(elecNum);
		// LP filter type init
		ElecLPFilterTypeInit(elecNum);
#if FILTER_MEDIAN
		// nit median pre-filter
		FilterMedianWindowInit(elecNum);
#endif
#if FILTER_MEAN
		// Init mean pre-filter
		FilterMeanWindowInit(elecNum);
#endif
	}

#if (NUMBER_OF_BUTTON_ELECTRODES > 0)
	// Init separate button variables
	ButtonVarInit();
	// Set first button electrode number for common electrode structures/arrays of DMA and LP filter
	firstButtonElecNum = 0;
#endif

#if SLIDER_ENABLE
	// Init separate slider variables
	SliderVarInit();
	// Set first slider electrode number for common electrode structures/arrays of DMA and LP filter
	firstSliderElecNum = NUMBER_OF_BUTTON_ELECTRODES;
#endif

#if (NUMBER_OF_WAKEUP_ELECTRODES > 0)
	// Init separate wakeup variables
	// Set first wakeup electrode number for common electrode structures/arrays of DMA and LP filter
	firstWakeupElecNum = NUMBER_OF_BUTTON_ELECTRODES + NUMBER_OF_SLIDER_ELECTRODES;
#endif

#if (GUARD)
	// Guard pin structure init
	GuardStructureInit();
#endif

	// Reset sample data flag
	samplesReadyFlag = 0;

	// Reset proximity detection flag
	proximityDetectedFlag = 0;

	// Electrode and Cext charge distribution period
	chargeDistributionPeriod = 0;

	// Init LP IIR Filter
	FilterIIR1Init();

	// Reset frequencyID
	frequencyID = 0;
}



/*****************************************************************************
 *
 * Function: void ElecBufferInitVal(int32_t inputSignal, int32_t *outputSignalPtr, uint16_t *sumCounterPtr)
 *
 * Description: Sum data
 *
 *****************************************************************************/
void ElecBufferInitVal(int32_t inputSignal, int32_t *outputSignalPtr, uint16_t *sumCounterPtr)
{
	int32_t  outputSignal;
	uint16_t sumCounter;


	outputSignal = *outputSignalPtr;
	sumCounter = *sumCounterPtr;

	sumCounter++;
	outputSignal = outputSignal + inputSignal;

	*outputSignalPtr = outputSignal;
	*sumCounterPtr = sumCounter;
}

/*****************************************************************************
 *
 * Function: void ElecCalInitVal(int32_t *outputSignalPtr, uint16_t *sumCounterPtr)
 *
 * Description: Calculate data average value
 *
 *****************************************************************************/
void ElecCalInitVal(int32_t *outputSignalPtr, uint16_t *sumCounterPtr)
{
	int32_t  outputSignal;
	uint16_t sumCounter;


	outputSignal = *outputSignalPtr;
	sumCounter = *sumCounterPtr;

	outputSignal = outputSignal / sumCounter;

	*outputSignalPtr = outputSignal;
	*sumCounterPtr = TOUCH_SENSE_APP_PWRUP_INIT_DONE;
}

/*****************************************************************************
 *
 * Function: int32_t DCTracker(int32_t inputSignal, int32_t *outputSignalRawPtr, uint8_t shift)
 *
 * Description: DC tracker (baseline) filter
 *
 *****************************************************************************/
int32_t DCTracker(int32_t inputSignal, int32_t *outputSignalRawPtr, uint8_t shift)
{	
	int32_t  outputSignalRaw;


	outputSignalRaw = *outputSignalRawPtr;

	if(inputSignal > (outputSignalRaw >> shift))
	{
		outputSignalRaw = outputSignalRaw + 1;
	}
	else if (inputSignal < (outputSignalRaw >> shift))
	{
		outputSignalRaw = outputSignalRaw - 1;
	}

	*outputSignalRawPtr = outputSignalRaw;

	return  (outputSignalRaw >> shift);
}

/*****************************************************************************
 *
 * Function: void ElectrodesFloat(void)
 *
 * Description: Set all electrodes float
 *
 *****************************************************************************/
void ElectrodesFloat(void)
{
	for (elecNum = 0; elecNum < NUMBER_OF_ELECTRODES; elecNum++)
	{
		// Set electrode as float
		ElectrodeFloat(&elecStruct[elecNum]);
	}
}

/*****************************************************************************
 *
 * Function: void ElectrodesGND(void)
 *
 * Description: Set all electrodes GND
 *
 *****************************************************************************/
void ElectrodesGND(void)
{
	for (elecNum = 0; elecNum < NUMBER_OF_ELECTRODES; elecNum++)
	{
		// Ground electrode
		ElectrodeGnd(&elecStruct[elecNum]);
	}
}

/*****************************************************************************
 *
 * Function: void ElecScanCPU(uint8_t electrodeNum, uint8_t samplingCtrl)
 *
 * Description: Convert single electrode self-capacitance to equivalent voltage by CPU
 *
 *****************************************************************************/
void ElecScanCPU(uint8_t electrodeNum, uint8_t samplingCtrl)
{
	// Clear ADCs gain (lower noise)
	ClearADCsGain();

	// Electrode capacitance to voltage conversion
	for (sampleNum = 0; sampleNum < (NUMBER_OF_PRESAMPLES + elecNumberOfSamples[electrodeNum]); sampleNum++)
	{
		if (samplingCtrl == 0)
		{
		while((LPIT0->MSR & LPIT_MSR_TIF0_MASK)!= 1){} // timeout?
		LPIT0->MSR = (1 << 0); // Clear LPIT CH0 timer flag
		}
		// Distribute Electrode and Cext charge
		ChargeDistribution(&elecStruct[electrodeNum]);
		// Delay to distribute charge
		chargeDistributionPeriodTmp = chargeDistributionPeriod;
		while (chargeDistributionPeriodTmp) {chargeDistributionPeriodTmp--;}

#if (GUARD)
		//Guard pin = 0
		guardStruct[0].gpioBasePtr->PCOR = 1 << guardStruct[0].pinNumberGuard;
#endif

		// If compiler optimization is not -O3 (is set to none -> -O0), then set TS_ASM_OPTIMIZE macro to 1 in ts_cfg.h
#if(TS_ASM_OPTIMIZE == 1)
		__asm volatile (
				/***** ASSEMBLY TEMPLATE ****************************************/
				// Start Cext voltage ADC conversion
				"str %1, [%0, #0]\n\t"    // store %1 to address in %0 with 0 byte offset
				// Redistribute Electrode and Cext charge
				"str %3, [%2, #0]"        // store %3 to address in %2 with 0 byte offset
#if (GUARD)
				// Guard pin = 0
				"\n\tstr %5, [%4, #0]"        // store %5 to address in %4 with 0 byte offset
#endif
				/***** LIST OF OUTPUT OPERANDS **********************************/
				:
				/***** LIST OF INPUT OPERANDS ***********************************/
				:  "l" (&elecStruct[electrodeNum].adcBasePtr->SC1[0]),  // operand %0
				   "l" (elecStruct[electrodeNum].adcChNum),             // operand %1
				   "l" (&elecStruct[electrodeNum].gpioBasePtr->PDDR),   // operand %2
				   "l" ((elecStruct[electrodeNum].gpioBasePtr->PDDR) & ~(elecStruct[electrodeNum].portMask))    // operand %3
#if (GUARD)
				   ,"l" (&guardStruct[0].gpioBasePtr->PSOR), // operand %4
				   "l" (guardStruct[0].portMask)  // operand %5
#endif
		);
#else
		// Start Cext voltage ADC conversion
		elecStruct[electrodeNum].adcBasePtr->SC1[0] = elecStruct[electrodeNum].adcChNum;

		// Redistribute Electrode and Cext charge
		ChargeRedistribution(&elecStruct[electrodeNum]);

#if (GUARD)
		//Guard pin = 1
		guardStruct[0].gpioBasePtr->PSOR = 1 << guardStruct[0].pinNumberGuard;
#endif

#endif
#if (NUMBER_OF_BUTTON_ELECTRODES > 0)
		// Every type of electrode can have different num of samples to be taken - determine the right buffer where to store the samples
		if (elecStruct[electrodeNum].type == BUTTON)
		{
			// Equivalent voltage digitalization
			buttonSamplesBuffer[electrodeNum - firstButtonElecNum][sampleNum] = EquivalentVoltageDigitalization(&elecStruct[electrodeNum]);
		}
#endif
#if (NUMBER_OF_SLIDER_ELECTRODES > 0)
		if (elecStruct[electrodeNum].type == SLIDER)
		{
			// Equivalent voltage digitalization
			sliderSamplesBuffer[electrodeNum - firstSliderElecNum][sampleNum] = EquivalentVoltageDigitalization(&elecStruct[electrodeNum]);
		}
#endif
#if (NUMBER_OF_WAKEUP_ELECTRODES > 0)
		if (elecStruct[electrodeNum].type == WAKEUP)
		{
			// Equivalent voltage digitalization
			wakeupSamplesBuffer[electrodeNum - firstWakeupElecNum][sampleNum] = EquivalentVoltageDigitalization(&elecStruct[electrodeNum]);
		}
#endif
	}

	// Set ADCs gain back to calibrated value
	SetADCsGain();
}

/*****************************************************************************
 *
 * Function: void  ElecSlider2PadsScanCPU(uint8_t electrodeNum, uint8_t samplingCtrl)
 *
 * Description: Drive both slider electrodes simultaneously, but convert slider electrodes one by one - with one ADC module at a time (SW trigger) by CPU
 *
 * Note I: This function is suitable for slider consisting of 2 electrodes only
 *
 * Note II: One slider electrode is scanned and the other (together with guard pin, if configured) serves as guard
 *
 * Note III: Both slider electrodes pins must be on the same port, each slider electrode Cext can be on the same or different ADC module
 *
 *****************************************************************************/
void  ElecSlider2PadsScanCPU(uint8_t electrodeNum, uint8_t samplingCtrl)
{
	uint8_t electrodeNumFirst, electrodeNumSecond;

#if SLIDER_ENABLE
	// Determine if the electrode scanned is first or second from the 2 slider electrodes
	Slider2PadsElectrodesSort(electrodeNum, &electrodeNumFirst, &electrodeNumSecond);
#endif
	// Clear ADCs gain (lower noise)
	ClearADCsGain();

	// Slider electrodes capacitance to voltage conversion
	for (sampleNum = 0; sampleNum < NUMBER_OF_PRESAMPLES + elecNumberOfSamples[electrodeNum]; sampleNum++)
	{
		if (samplingCtrl == 0)
		{
		while((LPIT0->MSR & LPIT_MSR_TIF0_MASK)!= 1){} // timeout?
		LPIT0->MSR = (1 << 0); // Clear LPIT CH0 timer flag
		}
		// Distribute Electrode and Cext charge for both slider electrodes
		SimultaneousChargeDistribution(&elecStruct[electrodeNumFirst],&elecStruct[electrodeNumSecond]);
		// Delay to distribute charge
		chargeDistributionPeriodTmp = chargeDistributionPeriod;
		while (chargeDistributionPeriodTmp) {chargeDistributionPeriodTmp--;}

#if (GUARD)
		// Guard pin = 0
		guardStruct[0].gpioBasePtr->PCOR = 1 << guardStruct[0].pinNumberGuard;
#endif

		// If compiler optimization is not -O3 (is set to none -> -O0), then set TS_ASM_OPTIMIZE macro to 1 in ts_cfg.h
#if(TS_ASM_OPTIMIZE == 1)
		__asm volatile (
				/***** ASSEMBLY TEMPLATE ****************************************/
				// Start Cext voltage ADC conversion
				"str %1, [%0, #0]\n\t"    // store %1 to address in %0 with 0 byte offset
				// Redistribute Electrode and Cext charge
				"str %3, [%2, #0]"        // store %3 to address in %2 with 0 byte offset
#if (GUARD)
				// Guard pin = 0
				"\n\tstr %5, [%4, #0]"        // store %5 to address in %4 with 0 byte offset
#endif
				/***** LIST OF OUTPUT OPERANDS **********************************/
				:
				/***** LIST OF INPUT OPERANDS ***********************************/
				:  "l" (&elecStruct[electrodeNum].adcBasePtr->SC1[0]),  // operand %0
				   "l" (elecStruct[electrodeNum].adcChNum),             // operand %1
				   "l" (&elecStruct[electrodeNumFirst].gpioBasePtr->PDDR),   // operand %2
				   "l" ((elecStruct[electrodeNumFirst].gpioBasePtr->PDDR) & ~(elecStruct[electrodeNumFirst].portMask + elecStruct[electrodeNumSecond].portMask))    // operand %3
#if (GUARD)
				   ,"l" (&guardStruct[0].gpioBasePtr->PSOR), // operand %4
				   "l" (guardStruct[0].portMask)  // operand %5
#endif
		);
#else
		// Start Cext voltage ADC conversion
		elecStruct[electrodeNum].adcBasePtr->SC1[0] = elecStruct[electrodeNum].adcChNum;

		// Redistribute Electrode and Cext charge for both slider electrodes
		SimultaneousChargeRedistribution(&elecStruct[electrodeNumFirst],&elecStruct[electrodeNumSecond]);

#if (GUARD)
		// Guard pin = 1
		guardStruct[0].gpioBasePtr->PSOR = 1 << guardStruct[0].pinNumberGuard;
#endif

#endif
#if (NUMBER_OF_SLIDER_ELECTRODES > 0)
		// Equivalent voltage digitalization
		sliderSamplesBuffer[electrodeNum - firstSliderElecNum][sampleNum] = EquivalentVoltageDigitalization(&elecStruct[electrodeNum]);
#endif
	}

	// Set ADCs gain back to calibrated value
	SetADCsGain();
}

/*****************************************************************************
 *
 * Function: void ElecRawDataCalc(uint8_t electrodeNum)
 *
 * Description: Calculate final raw data reading from gathered samples for single electrode
 *
 *****************************************************************************/
void ElecRawDataCalc(uint8_t electrodeNum)
{
	// Clear calc variable
	elecRawDataCalc = 0;

	// Pre-filtering the samples taken for each electrode before making SUM
	if (proximityDetectedFlag && (elecNumberOfSamples[electrodeNum] == elecNumberOfSamplesActive[electrodeNum]))
	{
#if (NUMBER_OF_BUTTON_ELECTRODES > 0)
		// Every type of electrode can have different num of samples to be taken - determine the right buffer from where to take the samples
		if (elecStruct[electrodeNum].type == BUTTON)
		{
#if FILTER_MEDIAN
			// Median filtering of the samples
			FilterMedian(buttonSamplesBuffer[electrodeNum - firstButtonElecNum], NUMBER_OF_PRESAMPLES + elecNumberOfSamples[electrodeNum], FilterMedianWindow[electrodeNum]);
#endif
#if FILTER_MEAN
			// Mean filtering of the samples
			FilterMean(buttonSamplesBuffer[electrodeNum - firstButtonElecNum], NUMBER_OF_PRESAMPLES + elecNumberOfSamples[electrodeNum],FilterMeanWindow[electrodeNum]);
#endif
			//FilterWeightedMean(buttonSamplesBuffer[electrodeNum - firstButtonElecNum], NUMBER_OF_PRESAMPLES + elecNumberOfSamples[electrodeNum], 7, elecDCTracker[electrodeNum]/(elecNumberOfSamples[electrodeNum]));
		}
#endif
#if (NUMBER_OF_SLIDER_ELECTRODES > 0)
		if (elecStruct[electrodeNum].type == SLIDER)
		{

#if FILTER_MEDIAN
			// Median filtering of the samples
			FilterMedian(sliderSamplesBuffer[electrodeNum - firstSliderElecNum], NUMBER_OF_PRESAMPLES + elecNumberOfSamples[electrodeNum],FilterMedianWindow[electrodeNum]);
#endif
#if FILTER_MEAN
			// Mean filtering of the samples
			FilterMean(sliderSamplesBuffer[electrodeNum - firstSliderElecNum], NUMBER_OF_PRESAMPLES + elecNumberOfSamples[electrodeNum],FilterMeanWindow[electrodeNum]);
#endif
			//FilterWeightedMean(sliderSamplesBuffer[electrodeNum - firstSliderElecNum], NUMBER_OF_PRESAMPLES + elecNumberOfSamples[electrodeNum], 7, elecDCTracker[electrodeNum]/(elecNumberOfSamples[electrodeNum]));
		}
#endif
#if (NUMBER_OF_WAKEUP_ELECTRODES > 0)
		if (elecStruct[electrodeNum].type == WAKEUP)
		{
#if FILTER_MEDIAN
			// Median filtering of the samples
			FilterMedian(wakeupSamplesBuffer[electrodeNum - firstWakeupElecNum], NUMBER_OF_PRESAMPLES + elecNumberOfSamples[electrodeNum],FilterMedianWindow[electrodeNum]);
#endif
#if FILTER_MEAN
			// Mean filtering of the samples
			FilterMean(wakeupSamplesBuffer[electrodeNum - firstWakeupElecNum], NUMBER_OF_PRESAMPLES + elecNumberOfSamples[electrodeNum],FilterMeanWindow[electrodeNum]);
#endif
			//FilterWeightedMean(wakeupSamplesBuffer[electrodeNum - firstWakeupElecNum], NUMBER_OF_PRESAMPLES + elecNumberOfSamples[electrodeNum], 7, elecDCTracker[electrodeNum]/(elecNumberOfSamples[electrodeNum]));
		}
#endif

	}

	// Calculate samples sum
	for (sampleNum = NUMBER_OF_PRESAMPLES; sampleNum < (NUMBER_OF_PRESAMPLES + elecNumberOfSamples[electrodeNum]); sampleNum++)
	{
#if (NUMBER_OF_BUTTON_ELECTRODES > 0)
		// Every type of electrode can have different num of samples to be taken - determine the right buffer from where to take the samples
		if (elecStruct[electrodeNum].type == BUTTON)
		{
			elecRawDataCalc += buttonSamplesBuffer[electrodeNum - firstButtonElecNum][sampleNum];
		}
#endif
#if (NUMBER_OF_SLIDER_ELECTRODES > 0)
		if (elecStruct[electrodeNum].type == SLIDER)
		{
			elecRawDataCalc += sliderSamplesBuffer[electrodeNum - firstSliderElecNum][sampleNum];
		}
#endif
#if (NUMBER_OF_WAKEUP_ELECTRODES > 0)
		if (elecStruct[electrodeNum].type == WAKEUP)
		{
			elecRawDataCalc += wakeupSamplesBuffer[electrodeNum - firstWakeupElecNum][sampleNum];
		}
#endif
	}

	// Put the sum of samples in the raw data
	elecRawData[electrodeNum][frequencyID] = (int32_t)(elecRawDataCalc);

}
/*****************************************************************************
 *
 * Function: void FilterIIR1BufferChange(uint8_t electrodeNum, uint8_t changeType, uint8_t freq)
 *
 * Description: Changes the values in Filter IIR1 buffer for single electrode
 *
 *****************************************************************************/
void FilterIIR1BufferChange(uint8_t electrodeNum, uint8_t changeType, uint8_t frequency)
{

	if (changeType == ACTIVE)// help LP filter data get closer to the threshold for faster reaction time
	{
		// Set IIR filter initial value to (DCTracker - threshold / 2)
		FilterIIR1BufferInit(electrodeNum, ((tFrac32)(elecDCTracker[electrodeNum] - (elecThresholdTouchDelta[electrodeNum] / 2))), \
				((tFrac32)(elecDCTracker[electrodeNum] - (elecThresholdTouchDelta[electrodeNum] / 2))), frequency);
	}
	else if (changeType == IDLE)
	{
		// Init IIR filter buffer to match DC Tracker value
		FilterIIR1BufferInit(electrodeNum, ((tFrac32)(elecDCTracker[electrodeNum])), ((tFrac32)(elecDCTracker[electrodeNum])), frequency);
	}

}
/*****************************************************************************
 *
 * Function: void ElectrodesSelfTrimSense(void)
 *
 * Description: Trigger electrodes self-trim scan after power-up or reset
 *
 *****************************************************************************/
void ElectrodesSelfTrimSense(void)
{
#ifdef DEBUG_ELECTRODE_SENSE
	// Pin set
	DES_GPIO->PSOR = 1 << DES_PIN;
#endif

	// Configure all electrodes floating
	ElectrodesFloat();

#if (NUMBER_OF_ELECTRODES > 0)
#if (TS_ELECTRODE_DATA_ACQUISITION == DMA_ELECTRODE_DATA_ACQUISITION)
	// Save first electrode number of the dma scanning sequence in the global variable (needed for DMA interrupt data sorting)
	dmaSeqScanFirstElec = 0;
	// Save last electrode electrode number of the dma scanning sequence in the global variable (needed for DMA interrupt data sorting)
	dmaSeqScanLastElec = NUMBER_OF_ELECTRODES - 1;
	if (dmaSeqScanFirstElec <= dmaSeqScanLastElec) // Sequence direction check
	{
		// Convert all electrodes (including wakeup elec) capacitance to equivalent voltage by DMA
		ElectrodesSequenceScanDMA(dmaSeqScanFirstElec, dmaSeqScanLastElec);
	}

#else
	// Decide sample control
	uint8_t sampleControl = SmplCtrl();

#if FREQUENCY_HOPPING
	if (sampleControl == 0)
	{
	LPIT_Enable(0, freqencyIDtimeout[frequencyID]*lpit_clock);
	}
#endif
	// All electrodes (including wakeup elec)
	for(elecNum = 0; elecNum < NUMBER_OF_ELECTRODES; elecNum++)
	{
		if (elecStruct[elecNum].type != SLIDER) // Button or wakeup electrode?
		{
			// Convert electrode capacitance to equivalent voltage by CPU
			ElecScanCPU(elecNum, sampleControl);
		}

		else // Slider electrode
		{
			// Convert electrode capacitance to equivalent voltage by CPU
			ElecScanCPU(elecNum, sampleControl);

			// Convert electrode capacitance to equivalent voltage by CPU
			// ElecSlider2PadsScanCPU(elecNum, sampleControl);
		}
	}
#if FREQUENCY_HOPPING
	LPIT_Disable(0);
#endif
	// Set the raw data flag
	samplesReadyFlag = 1;
#endif
#ifdef DEBUG_ELECTRODE_SENSE
			// Pin clear
			DES_GPIO->PCOR = 1 << DES_PIN;
#endif
#endif
}

/*****************************************************************************
 *
 * Function: uint8_t ElectrodesSelfTrim(void)
 *
 * Description: Run all electrodes self-trim algorithm
 *
 *****************************************************************************/
uint8_t ElectrodesSelfTrim(void)
{
	uint8_t selfTrimDone = 0;

	// Device after power-up / reset ?
	if (elecSelfTrimBufferCounter[0] < NUMBER_OF_CYCLES_DCTRACKER_PWRUP)
	{
		// All elec electrodes
		for (elecNum = 0; elecNum < (NUMBER_OF_ELECTRODES); elecNum++)
		{
			// Store touch electrode init value
			ElecBufferInitVal(elecRawData[elecNum][frequencyID], &(elecSelfTrimBuffer[elecNum]), &(elecSelfTrimBufferCounter[elecNum]));
		}
	}
	// Calculate and load init value
	else
	{
		// All elec electrodes
		for (elecNum = 0; elecNum < (NUMBER_OF_ELECTRODES); elecNum++)
		{
			// Calculate touch electrode init value
			ElecCalInitVal(&(elecSelfTrimBuffer[elecNum]), &(elecSelfTrimBufferCounter[elecNum]));
		}

		// Set elec electrodes status "self-trim done" flag
		selfTrimDone = YES;
	}

	// Elec electrodes self-trim done?
	if(selfTrimDone == YES)
	{
		// All elec electrodes
		for (elecNum = 0; elecNum < (NUMBER_OF_ELECTRODES); elecNum++)
		{
			// DC tracker data
			elecDCTracker[elecNum] = elecSelfTrimBuffer[elecNum];
			// Load DC tracker buffer
			elecDCTrackerBuffer[elecNum] = (elecDCTracker[elecNum]) << ((elecDCTrackerShift[elecNum]));
			// Electrode touch and release thresholds
			elecThresholdTouch[elecNum] = elecDCTracker[elecNum] -  elecThresholdTouchDelta[elecNum];
			elecThresholdRelease[elecNum] = elecDCTracker[elecNum] -  elecThresholdReleaseDelta[elecNum];
		}

		// All used scanning frequencies (freemaster init)
		for (uint8_t freq = 0; freq < NUMBER_OF_HOPPING_FREQUENCIES; freq++)
		{
			// All electrodes
			for (elecNum = 0; elecNum < (NUMBER_OF_ELECTRODES); elecNum++)
			{
				// Electrodes LP IIR filter init value (for FreeMASTER)
				elecLPFilterData[elecNum][freq] = elecDCTracker[elecNum];
				// Raw data preset as baseline
				elecRawData[elecNum][freq] = elecDCTracker[elecNum];

#if DECIMATION_FILTER
				// Pre-load DF array
				elecRawDataDF[elecNum][freq] = elecDCTracker[elecNum];
#endif
			}
		}

		// All used scanning frequencies
		for (uint8_t freq = 0; freq < NUMBER_OF_HOPPING_FREQUENCIES; freq++)
		{
			// All electrodes
			for (elecNum = 0; elecNum < (NUMBER_OF_ELECTRODES); elecNum++)
			{
				// Init elec electrodes IIR filter buffer to match DC tracker values
				FilterIIR1BufferChange(elecNum, IDLE, freq);
			}
		}
	}

	return selfTrimDone;
}

/*****************************************************************************
 *
 * Function: void ElectrodesSelfTrimProcess(void)
 *
 * Description: Continue processing of all electrodes raw data
 *
 *****************************************************************************/
void ElectrodesSelfTrimProcess(void)
{
	uint8_t selfTrimStatus;

	// Set ADCs gain back to calibrated value
	SetADCsGain();

	// Drive all electrodes to GND
	ElectrodesGND();

	// All electrodes
	for(elecNum = 0; elecNum < NUMBER_OF_ELECTRODES; elecNum++)
	{
		// Calculate raw data from scanned samples and perform a self trim to get the baselines
		ElecRawDataCalc(elecNum);
	}

	// All electrodes self trim
	selfTrimStatus = ElectrodesSelfTrim();

	// Self-trim done? (All electrodes have the same number of sensing cycles to be done)
	if (selfTrimStatus)
	{
		electrodesSelfTrimDoneFlag = 1;

#if (WAKE_UP_ELECTRODE_PROXIMITY == YES)
		// Enable LPTMR
		LPTMR0_Init(LPTMR_ELEC_SENSE_PROXIMITY);
#else
#if (DECIMATION_FILTER && NUMBER_OF_WAKEUP_ELECTRODES < 1)
		// Enable LPTMR
		LPTMR0_Init(LPTMR_ELEC_SENSE_DF);
#else
		// Enable LPTMR
		LPTMR0_Init(LPTMR_ELEC_SENSE);
#endif
#endif
		// Enable low power mode
		lowPowerModeCtrl = ON;
	}
}
/*****************************************************************************
 *
 * Function: ElecDCTrackerUpdate(uint8_t electrodeNum)
 *
 * Description: Update DCtracker for single electrode
 *
 *****************************************************************************/
void ElecDCTrackerUpdate(uint8_t electrodeNum)
{
	// Update DC Tracker
	elecDCTracker[electrodeNum] = DCTracker(elecRawData[electrodeNum][frequencyID], &(elecDCTrackerBuffer[electrodeNum]), elecDCTrackerShift[electrodeNum]);
}

/*****************************************************************************
 *
 * Function: void ElectrodesTouchElecSense(void)
 *
 * Description: Trigger sensing of all touch electrodes (all non-wakeup electrodes)
 *
 *****************************************************************************/
void ElectrodesTouchElecSense(void)
{
#ifdef DEBUG_ELECTRODE_SENSE
	// Pin set
	DES_GPIO->PSOR = 1 << DES_PIN;
#endif

	/* ---RAW DATA SAMPLING--- */
	// Configure all electrodes floating
	ElectrodesFloat();

#if (NUMBER_OF_TOUCH_ELECTRODES > 0)
#if (TS_ELECTRODE_DATA_ACQUISITION == DMA_ELECTRODE_DATA_ACQUISITION)
	// Save first electrode number of the dma scanning sequence in the global variable (needed for DMA interrupt data sorting)
	dmaSeqScanFirstElec = 0;
	// Save last electrode electrode number of the dma scanning sequence in the global variable (needed for DMA interrupt data sorting)
	dmaSeqScanLastElec = NUMBER_OF_TOUCH_ELECTRODES - 1;

	if (dmaSeqScanFirstElec <= dmaSeqScanLastElec) // Sequence direction check
	{
		// Convert all touch electrodes (excluding wakeup elec) capacitance to equivalent voltage by DMA
		ElectrodesSequenceScanDMA(dmaSeqScanFirstElec, dmaSeqScanLastElec);
	}
#else
	// Decide sample control
	uint8_t sampleControl = SmplCtrl();

#if FREQUENCY_HOPPING
	if (sampleControl == 0)
	{
	LPIT_Enable(0, freqencyIDtimeout[frequencyID]*lpit_clock);
	}
#endif
	// All touch electrodes (excluding wakeup elec)
	for(elecNum = 0; elecNum < NUMBER_OF_TOUCH_ELECTRODES; elecNum++)
	{
		if (elecStruct[elecNum].type != SLIDER) // Button electrode?
		{
			// Convert electrode capacitance to equivalent voltage by CPU
			ElecScanCPU(elecNum, sampleControl);
		}

		else // Slider electrode
		{
			// Convert electrode capacitance to equivalent voltage by CPU
			ElecScanCPU(elecNum, sampleControl);

			// Convert electrode capacitance to equivalent voltage by CPU
			//ElecSlider2PadsScanCPU(elecNum, sampleControl);
		}
	}
#if FREQUENCY_HOPPING
	LPIT_Disable(0);
#endif

	// Set the raw data flag
	samplesReadyFlag = 1;

#ifdef DEBUG_ELECTRODE_SENSE
			// Pin clear
			DES_GPIO->PCOR = 1 << DES_PIN;
#endif

#endif
#endif
}
/*****************************************************************************
 *
 * Function: void ElectrodesTouchElecProcess(void)
 *
 * Description: Continue processing of all touch electrodes raw data (all non-wakeup electrodes)
 *
 *****************************************************************************/
void ElectrodesTouchElecProcess(void)
{
	// Set ADCs gain back to calibrated value
	SetADCsGain();

	/* ---DIGITAL SIGNAL PROCESSING--- */

	// Drive all electrodes to GND
	ElectrodesGND();

	// Calculate raw data from scanned samples
#if (NUMBER_OF_TOUCH_ELECTRODES > 0)
	// All electrodes
	for(elecNum = 0; elecNum < NUMBER_OF_TOUCH_ELECTRODES; elecNum++)
	{
		// Calculate raw data from scanned samples
		ElecRawDataCalc(elecNum);
	}
#endif

	// All touch electrodes (excluding wakeup elec)
	for(elecNum = 0; elecNum < NUMBER_OF_TOUCH_ELECTRODES; elecNum++)
	{
#if (NUMBER_OF_BUTTON_ELECTRODES > 0)
		if (elecStruct[elecNum].type == BUTTON) // Button electrode?
		{
			// Process button electrode raw data
			ButtonElecDSP(elecNum);
		}
#endif
#if (NUMBER_OF_SLIDER_ELECTRODES > 0)
		if (elecStruct[elecNum].type == SLIDER)
		{
			// Process slider electrode raw data
			Slider2PadsElecDSP(elecNum);
		}
#endif
	}


	/* ---TOUCH DETECT ALGORITHM--- */
	for(elecNum = 0; elecNum < NUMBER_OF_TOUCH_ELECTRODES; elecNum++)
	{
#if (NUMBER_OF_BUTTON_ELECTRODES > 0)
		if (elecStruct[elecNum].type == BUTTON) // Button electrode?
		{
			// Detect and qualify button electrodes touch event
			ButtonElecTouchDetect(elecNum);
		}
#endif
#if (NUMBER_OF_SLIDER_ELECTRODES > 0)
		if (elecStruct[elecNum].type == SLIDER)
		{
			// Process slider electrode raw data
			Slider2PadsElecTouchDetect(elecNum);
		}
#endif
	}


#if (NUMBER_OF_BUTTON_ELECTRODES > 0)
	// Qualify button electrodes touch event
	ButtonTouchQualify();
#endif

#if SLIDER_ENABLE
	// Qualify slider electrodes touch event
	Slider2PadsSumDataCalculation();
	if (frequencyID == (NUMBER_OF_HOPPING_FREQUENCIES-1))
	{
	Slider2PadsTouchQualify();
	}
#endif
}


/*****************************************************************************
 *
 * Function: void ElectrodesWakeupEventSense(void)
 *
 * Description: Trigger Sensing of one wake-up electrode plus one more touch electrode for further processing:
 * 		a) if possible touch event detected - scan all touch electrodes
 * 		b) if possible touch event not detected, keep track of touch electrodes baselines
 *
 *****************************************************************************/
void ElectrodesWakeupEventSense(void)
{
#ifdef DEBUG_ELECTRODE_SENSE
	// Pin set
	DES_GPIO->PSOR = 1 << DES_PIN;
#endif
	/* ---RAW DATA SAMPLING--- */
	// Configure all electrodes floating
	ElectrodesFloat();

#if (TS_ELECTRODE_DATA_ACQUISITION == DMA_ELECTRODE_DATA_ACQUISITION) // DMA handles electrodes scanning?

#if (NUMBER_OF_WAKEUP_ELECTRODES > 0 && NUMBER_OF_TOUCH_ELECTRODES > 0) // At least one wakeup electrode and one touch electrode declared?

	if (elecStruct[elecNumBaselineUpdate].type != SLIDER) // Electrode to be updated is a button or wake up electrode?
	{
		// Convert single wakeup electrode capacitance to equivalent voltage by DMA and one touch electrode to keep track of touch electrodes baselines
		ElectrodesWakeupEventScanDMA(firstWakeupElecNum, elecNumBaselineUpdate);
	}
	else // Electrode to be updated is a slider electrode?
	{
#ifdef SLIDER_SELF_GUARDING	// Slider consists of only two pads and self-guarding defined?
		// Save first electrode number of the dma scanning sequence in the global variable (needed for DMA interrupt data sorting)
		dmaSeqScanFirstElec = firstSliderElecNum;
		// Save last electrode electrode number of the dma scanning sequence in the global variable (needed for DMA interrupt data sorting)
		dmaSeqScanLastElec = firstWakeupElecNum;

		if (dmaSeqScanFirstElec <= dmaSeqScanLastElec) // Sequence direction check
		{
			// Convert 2 slider electrodes and wakeup elec capacitance to equivalent voltage by DMA
			ElectrodesSequenceScanDMA(dmaSeqScanFirstElec, dmaSeqScanLastElec);
		}
#else // Slider self-guarding NOT defined

		// Convert single wakeup electrode capacitance to equivalent voltage by DMA and one touch electrode to keep track of touch electrodes baselines
		ElectrodesWakeupEventScanDMA(firstWakeupElecNum, elecNumBaselineUpdate);
#endif
	}
#endif

#else // CPU handles electrodes scanning

	// Decide sample control
	uint8_t sampleControl = SmplCtrl();

#if FREQUENCY_HOPPING
	if (sampleControl == 0)
	{
		LPIT_Enable(0, freqencyIDtimeout[frequencyID]*lpit_clock);
	}
#endif
#if (NUMBER_OF_WAKEUP_ELECTRODES > 0)
	// Convert single wakeup electrode self-capacitance to equivalent voltage (to detect possible touch event)
	ElecScanCPU(firstWakeupElecNum, sampleControl);
#endif

#if (NUMBER_OF_TOUCH_ELECTRODES > 0)
	if (elecStruct[elecNumBaselineUpdate].type == BUTTON) // Button electrode?
	{
		// Convert single button electrode self-capacitance to equivalent voltage (to keep track of touch electrodes baselines)
		ElecScanCPU(elecNumBaselineUpdate, sampleControl);
	}

	else if (elecStruct[elecNumBaselineUpdate].type == SLIDER) // slider electrode?
	{
#ifdef SLIDER_SELF_GUARDING // Slider consists of only two pads and self-guarding defined?
		for(elecNum = firstSliderElecNum; elecNum < firstWakeupElecNum; elecNum++)
		{
			// Convert 2 slider electrodes and wakeup elec capacitance to equivalent voltage by DMA
			ElecSlider2PadsScanCPU(elecNum, sampleControl);
		}
#else // Slider self-guarding NOT defined

		// Convert single slider electrode self-capacitance to equivalent voltage (to keep track of touch electrodes baselines)
		ElecScanCPU(elecNumBaselineUpdate, sampleControl);
#endif
	}
#endif
#if FREQUENCY_HOPPING
	LPIT_Disable(0);
#endif
	// Set the raw data ready flag
	samplesReadyFlag = 1;

#ifdef DEBUG_ELECTRODE_SENSE
			// Pin clear
			DES_GPIO->PCOR = 1 << DES_PIN;
#endif
#endif
}
/*****************************************************************************
 *
 * Function: void ElectrodesWakeupEventProcess(void)
 *
 * Description: Continue Processing of one wake-up electrode plus one more touch electrode data and process:
 * 		a) if possible touch event detected - scan all touch electrodes
 * 		b) if possible touch event not detected, keep track of touch electrodes baselines
 *
 *****************************************************************************/
void ElectrodesWakeupEventProcess(void)
{
	// Set ADCs gain back to calibrated value
	SetADCsGain();

	/* ---RAW DATA SAMPLING PART2--- */
	// Drive all electrodes to GND
	ElectrodesGND();

	/* ---DIGITAL SIGNAL PROCESSING--- */
#if (NUMBER_OF_WAKEUP_ELECTRODES > 0)
	// Calculate final raw data reading from gathered samples for single wake up electrode
	ElecRawDataCalc(firstWakeupElecNum);
	// Process single wakeup electrode raw data
	WakeupElecDSP(firstWakeupElecNum);
	// Detect single wake up electrode touch event
	WakeupElecTouchDetect(firstWakeupElecNum);
#endif

#if (NUMBER_OF_TOUCH_ELECTRODES > 0)
	// Calculate final raw data reading from gathered samples for single touch electrode
	ElecRawDataCalc(elecNumBaselineUpdate);
	// Update baseline of single touch electrode
	ElecDCTrackerUpdate(elecNumBaselineUpdate);
#endif

	// Next touch electrode for next baseline update scanning
	if (elecNumBaselineUpdate < (NUMBER_OF_TOUCH_ELECTRODES - 1))
	{
		elecNumBaselineUpdate++;
	}
	else
	{
		// Start from begin
		elecNumBaselineUpdate = 0;
	}
}
/*****************************************************************************
 *
 * Function: void ElectrodesWakeupEventCtrl(void)
 *
 * Description: Change essential touch sense application properties if proximity detected
 *
 *****************************************************************************/
void ElectrodesWakeupEventCtrl(void)
{
	// Proximity - wake up EGS electrode touched? (frequencyID at this stage of code should always be 0)
	if(proximityDetectedFlag == 1)
	{

#if (NUMBER_OF_TOUCH_ELECTRODES > 0)
		for(elecNum = 0; elecNum < NUMBER_OF_TOUCH_ELECTRODES; elecNum++)
		{
			// Change the number samples to be taken for each touch electrode to active
			ElecNumOfSamplesPerElecChange(elecNum, INCREASE);
		}
#endif
#if (NUMBER_OF_TOUCH_ELECTRODES > 0)
		for(elecNum = 0; elecNum < NUMBER_OF_TOUCH_ELECTRODES; elecNum++)
		{
			// Activate touch  electrodes DC tracker adjustments for oversampling - to active mode
			ElecOversamplingDCTrackerChange(elecNum, INCREASE);
		}
#endif
		/* IIR LP Filter Buffer initiation after EGS touch*/

		// All used scanning frequencies
		for (uint8_t freq = 0; freq < NUMBER_OF_HOPPING_FREQUENCIES; freq++)
		{
			for(elecNum = 0; elecNum < NUMBER_OF_TOUCH_ELECTRODES; elecNum++)
			{
			// Set IIR filter initial value to (DCTracker - threshold / 2) for touch electrodes for faster reaction
			//FilterIIR1BufferChange(elecNum, ACTIVE, freq);
			// Set IIR filter initial value to DCTracker for better EMI
			FilterIIR1BufferChange(elecNum, IDLE, freq);
			}
		}

		// Load counter to do not return to the wake-up function
		// When wake-up happens, 1 second period given to detect touch event
		electrodeWakeUpActivateCounter = ELEC_WAKEUP_ACTIVATE_COUNTER;

#if (NUMBER_OF_WAKEUP_ELECTRODES > 0)
#if DECIMATION_FILTER || (WAKE_UP_ELECTRODE_PROXIMITY == YES) // If decimation filter is On or proximity by wake up electrode is used
#if DECIMATION_FILTER
			// Set LPTMR timeout period of decimation filter (3ms)
			LPTMR0_CMR_Update(LPTMR_ELEC_SENSE_DF);
#else
			// Set LPTMR 30 ms timeout period
			LPTMR0_CMR_Update(LPTMR_ELEC_SENSE);
#endif
#endif
#endif
	}
}

/*****************************************************************************
 *
 * Function: void ElectrodesGoSleepEventCtrl(void)
 *
 * Description: Change essential touch sense application properties if proximity no longer detected
 *
 *****************************************************************************/
void ElectrodesGoSleepEventCtrl(void)
{
	// Release to wake-up electrode control
	if (electrodeWakeUpActivateCounter == 0)
	{
		// First time proximity not detected?
		if(proximityDetectedFlag != 0)
		{

#if (NUMBER_OF_TOUCH_ELECTRODES > 0)
			for(elecNum = 0; elecNum < NUMBER_OF_TOUCH_ELECTRODES; elecNum++)
			{
				// Change the number samples to be taken for each touch electrode to idle
				ElecNumOfSamplesPerElecChange(elecNum, DECREASE);
			}
#endif
#if (NUMBER_OF_TOUCH_ELECTRODES > 0)
			for(elecNum = 0; elecNum < NUMBER_OF_TOUCH_ELECTRODES; elecNum++)
			{
				// Deactivate touch electrodes DC tracker adjustments for oversampling - to idle mode
				ElecOversamplingDCTrackerChange(elecNum, DECREASE);
			}
#endif
			/* IIR LP Filter Buffer of EGS reset after EGS release*/

			// All (both) used scanning frequencies
			for (uint8_t freq = 0; freq < NUMBER_OF_HOPPING_FREQUENCIES; freq++)
			{
				for(elecNum = 0; elecNum < NUMBER_OF_ELECTRODES; elecNum++)
				{
					// Set IIR filter initial value to (DCtracker) for all electrodes
					FilterIIR1BufferChange(elecNum, IDLE, freq);
				}
				// Init wakeup electrodes IIR filter buffer to match DC tracker values
				//FilterIIR1BufferChange(firstWakeupElecNum, IDLE);
			}
#if (NUMBER_OF_WAKEUP_ELECTRODES > 0)
#if DECIMATION_FILTER || (WAKE_UP_ELECTRODE_PROXIMITY == YES) // If decimation filter is On or proximity by wake up electrode is used
#if (WAKE_UP_ELECTRODE_PROXIMITY == YES)
			// Set LPTMR proximity timeout period
			LPTMR0_CMR_Update(LPTMR_ELEC_SENSE_PROXIMITY);
#else
			// Set LPTMR 30 ms timeout period
			LPTMR0_CMR_Update(LPTMR_ELEC_SENSE);
#endif
#endif
#endif
		}

		// Proximity no longer detected
		proximityDetectedFlag = 0;

	}
	else
	{
#if FREQUENCY_HOPPING
		// On last frequency?
		if (frequencyID == NUMBER_OF_HOPPING_FREQUENCIES - 1)
#endif
		{
			// Decrement counter
			electrodeWakeUpActivateCounter--;
		}
	}
}

#if (NUMBER_OF_WAKEUP_ELECTRODES < 1)
/*****************************************************************************
 *
 * Function: void ElectrodesVirtualWakeupEventCtrl(void);
 *
 * Description: Search for wake-up event at the touch electrodes (any proximity at touch electrodes)
 *
 *****************************************************************************/
void ElectrodesVirtualWakeupEventCtrl(void)
{
	uint8_t count = 0;
	uint8_t f_count;
	// Detect if any of the button electrodes was reported as proximately touched
	for(elecNum = 0; elecNum < NUMBER_OF_TOUCH_ELECTRODES; elecNum++)
	{
#if FREQUENCY_HOPPING
		// Reset frequency count
		f_count = 0;

		for (uint8_t freq = 0; freq < NUMBER_OF_HOPPING_FREQUENCIES; freq++)
		{
			// If on the frequency proximity touch detected
			if(elecLPFilterData[elecNum][freq] < (elecDCTracker[elecNum] - VIRTUAL_WAKEUP_TOUCH_THRESHOLD_DELTA))
			{
				f_count++;
			}
		}

		// If at all frequencies proximity touch?
		if (f_count == NUMBER_OF_HOPPING_FREQUENCIES)
		{
			count++;
		}
#else
		// If proximity detected
		if(elecLPFilterData[elecNum][frequencyID] < (elecDCTracker[elecNum] - VIRTUAL_WAKEUP_TOUCH_THRESHOLD_DELTA))

		{
			count++; // increment count
		}
#endif
	}

	// At least one touch electrode proximity detected
	if (count > 0)
	{
		if (proximityDetectedFlag != 1) // First time detecting proximity?
		{
		proximityDetectedFlag = 1; // Set proximity detected flag

		// Change essential touch sense application properties if proximity detected
		ElectrodesWakeupEventCtrl();
		}
	}
	else
	{
		// Change essential touch sense application properties if proximity no longer detected
		ElectrodesGoSleepEventCtrl();
	}
}
#endif

#if JITTERING
#pragma GCC push_options
#pragma GCC optimize ("O0")
/*****************************************************************************
 *
 * Function: void Jitter(void)
 *
 * Description: Creates random short delay in order to jitter the sample rate
 *
 *
 *****************************************************************************/
void Jitter(void)
{
	int32_t jitter;

	// Mask the raw data
	jitter = elecRawData[jitterIndex][frequencyID] & JITTERING_MASK;
	// Store jitter value for Freemaster
	jitterRead = jitter;
	// Delay
	while(jitter--);

	// Move to next elec raw data to be the source of random jitter
	if (jitterIndex < NUMBER_OF_ELECTRODES-1)
	{
		jitterIndex++; // Increment jitter index
	}
	else
	{
		jitterIndex = 0; // Reset
	}

}
#pragma GCC pop_options
#endif

#if FREQUENCY_HOPPING
/*****************************************************************************
 *
 * Function: void FrequencyHop(void)
 *
 * Description: Changes scanning frequency/period of electrodes samples
 *
 *****************************************************************************/
void FrequencyHop(void)
{
	if (frequencyID < (NUMBER_OF_HOPPING_FREQUENCIES-1))
	{
		// Increment freq ID
		frequencyID++;
	}
	else
	{
		// Reset
		frequencyID = 0;
	}
}

/*****************************************************************************
 *
 * Function: void FrequencyHoppingInit(void)
 *
 * Description: Initiates the hopping frequencies
 *
 *****************************************************************************/
void FrequencyHoppingInit(uint8_t clockmode)
{

  if (clockmode == RUN_FIRC)
  {
	  // System clock is FIRC 48 MHz, LPIT clock 48 MHz
	  lpit_clock = 48;
  }
  else if (clockmode == RUN_PLL)
  {
	  // System clock is PLL 80 MHz (LPIT clock 40 MHz)
	  lpit_clock = 40;
  }

  	// Load hopping frequencies
#if NUMBER_OF_HOPPING_FREQUENCIES > 0
	freqencyIDtimeout[0] = FREQ_ID_0;
#endif
#if NUMBER_OF_HOPPING_FREQUENCIES > 1
	freqencyIDtimeout[1] = FREQ_ID_1;
#endif
#if NUMBER_OF_HOPPING_FREQUENCIES > 2
	freqencyIDtimeout[2] = FREQ_ID_2;
#endif
#if NUMBER_OF_HOPPING_FREQUENCIES > 3
	freqencyIDtimeout[3] = FREQ_ID_3;
#endif
}

#endif

#if DECIMATION_FILTER
/*****************************************************************************
 *
 * Function: void DecimationFilter(uint8_t electrodeNum)
 *
 * Description: Increment or decrement decimation filter value
 *
 *****************************************************************************/
void DecimationFilter(uint8_t electrodeNum)
{

	// Is the electrode raw data value still smaller than electrode decimation filter value?
	if(elecRawData[electrodeNum][frequencyID] < elecRawDataDF[electrodeNum][frequencyID])
	{
		// Decrement DF value - follow the raw data down
		elecRawDataDF[electrodeNum][frequencyID] -= DECIMATION_STEP ;
	}
	// Is the electrode raw data value same as electrode decimation filter value?
	else if ((elecRawData[electrodeNum][frequencyID] == elecRawDataDF[electrodeNum][frequencyID]))
	{
		// Do nothing
	}

	else
	{
		// Increment DF value - follow  the raw data back up to the baseline
		elecRawDataDF[electrodeNum][frequencyID] += DECIMATION_STEP;
	}

}

#endif

/*****************************************************************************
 *
 * Function: void ElectrodesBacklightAndLowPowerCtrl(void)
 *
 * Description: Control backlighting of the keyboard and also low power ctrl flag  based on detected proximity
 *
 *****************************************************************************/
void ElectrodesBacklightAndLowPowerCtrl(void)
{
	if (frequencyID == 0)
	{
		// Proximity detected?
		if (proximityDetectedFlag == 1)
		{
			// MCU in RUN mode only
			lowPowerModeCtrl = OFF;

			// Load counter period to keep backlight ON
			backlightCounter = electrodeWakeUpActivateCounter + KEYPAD_BACKLIGHT_OFFSET;

#if NUMBER_OF_WAKUP_ELECTRODES > 0
			// Turn ON backlight?
			if(backlightCounter == KEYPAD_BACKLIGHT_ON_PERIOD-1)
			{
				// Reset FTM 2 counter
				FTM2->CNT = 0;
				// Load PWM duty cycle
				LoadBacklightPWMDutyCycle(backlightPWMDutyCycle);
			}
#else
			// Turn ON backlight?
			if(backlightCounter == KEYPAD_BACKLIGHT_ON_PERIOD)
			{
				// Reset FTM 2 counter
				//FTM2->CNT = 0;
				// Load PWM duty cycle
				LoadBacklightPWMDutyCycle(backlightPWMDutyCycle);
			}
#endif
		}
		else
		{
			if (backlightCounter > 0)
			{
				// Decrement counter
				backlightCounter--;
			}
			else
			{
				// MCU enters the low power mode with periodical wake-up by LPTMR
				lowPowerModeCtrl = ON;
				// Reset counter
				backlightCounter = 0;
				// Turn OFF backlight
				LoadBacklightPWMDutyCycle(0);
			}
		}
	}
}
/*****************************************************************************
 *
 * Function: uint8_t SmplCtrl(void)
 *
 * Description: Decide how is the sampling frequency going to be controlled (triggered)
 *
 *****************************************************************************/
uint8_t SmplCtrl(void)
{
#if FREQUENCY_HOPPING
	// Actual frequency is the base freq(0) and low power (100uA MCu current) needed - (typically by using one or more wakeup electrodes)?
	if (freqencyIDtimeout[frequencyID] < 6) // sampling period < 6us ?
	{
		return 1; // triggered by linking
	}
	else
	{
		return 0; // triggered by LPIT
	}
#else
	return 1; // triggered by linking
#endif
}
/*****************************************************************************
 *
 * Function: void ElectrodesScanLPTMRHandlerRoutine(void)
 *
 * Description: Routine for triggering scanning
 *
 *****************************************************************************/
void ElectrodesScanLPTMRHandlerRoutine(void)
{
	// Clear scanning (DMA) idle flag
	scanningIdleFlag = 0;

#if(DECIMATION_FILTER != 1)
	// Clear TCF LPTMR
	LPTMR0->CSR |= 1 << 7;
#endif

#if JITTERING
	// Delay the electrodes sense period
	Jitter();
#endif

	// Electrodes self-trim done?
	if(electrodesSelfTrimDoneFlag == 1)
	{
#if (NUMBER_OF_WAKEUP_ELECTRODES > 0)
		// Wake-up electrode touched?
		if (proximityDetectedFlag == 1)
		{
			// Sense all touch electrodes
			ElectrodesTouchElecSense();
		}
		else
		{
			// Sense wake up electrode for wake up event plus sense one more touch electrode to keep track of baselines
			ElectrodesWakeupEventSense();
		}
#else // EGS not used
		// Sense all touch electrodes
		ElectrodesTouchElecSense();
#endif

	}
	else
	{
		// Self-trim all electrodes after power-up / reset
		ElectrodesSelfTrimSense();

	}
#if DECIMATION_FILTER
	// Clear TCF LPTMR
	//REG_RMW32(&LPTMR0->CSR, 1 << 7, 1 << 7);
	LPTMR0->CSR |= 1 << 7;
#endif
}

/*****************************************************************************
 *
 * Function: void  FilterMedianWindowInit(uint8_t electrodeNum)
 *
 * Description: Init window size of median filter for each electrode
 *
 *****************************************************************************/
void  FilterMedianWindowInit(uint8_t electrodeNum)
{
	// Set window size for median filtering to be used in active mode
#if (NUMBER_OF_BUTTON_ELECTRODES > 0)
	if (elecStruct[electrodeNum].type == BUTTON)
	{
		FilterMedianWindow[electrodeNum] = WINDOW_MEDIAN_BUTTON;
	}
#endif
#if (NUMBER_OF_SLIDER_ELECTRODES > 0)
	if (elecStruct[electrodeNum].type == SLIDER)
	{
		FilterMedianWindow[electrodeNum] = WINDOW_MEDIAN_SLIDER;
	}
#endif
#if (NUMBER_OF_WAKEUP_ELECTRODES > 0)
	if (elecStruct[electrodeNum].type == WAKEUP)
	{
		FilterMedianWindow[electrodeNum] = WINDOW_MEDIAN_WAKEUP;
	}
#endif
}
/*****************************************************************************
 *
 * Function: void  FilterMeanWindowInit(uint8_t electrodeNum)
 *
 * Description: Init window size of mean filter for each electrode
 *
 *****************************************************************************/
void  FilterMeanWindowInit(uint8_t electrodeNum)
{
	// Set window size for mean filtering to be used in active mode
#if (NUMBER_OF_BUTTON_ELECTRODES > 0)
	if (elecStruct[electrodeNum].type == BUTTON)
	{
		FilterMeanWindow[electrodeNum] = WINDOW_MEAN_BUTTON;
	}
#endif
#if (NUMBER_OF_SLIDER_ELECTRODES > 0)
	if (elecStruct[electrodeNum].type == SLIDER)
	{
		FilterMeanWindow[electrodeNum] = WINDOW_MEAN_SLIDER;
	}
#endif
#if (NUMBER_OF_WAKEUP_ELECTRODES > 0)
	if (elecStruct[electrodeNum].type == WAKEUP)
	{
		FilterMeanWindow[electrodeNum] = WINDOW_MEAN_WAKEUP;
	}
#endif
}

