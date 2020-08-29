/****************************************************************************//*!
*
* @file     ets.h
*
* @version  1.0.0.0
*
* @date     September-2017
*
* @brief    Electrode touch sense routines for S32K144
*
*******************************************************************************/
#ifndef __ETS_H
#define __ETS_H

/*******************************************************************************
* Includes
*******************************************************************************/
#include "S32K118.h"
#include "ts_cfg.h"

/*******************************************************************************
* Calibration after power-up and reset
* Define number of cycles to calculate DC tracker power-up value
* Electrodes sensing period 1ms period
******************************************************************************/
#define NUMBER_OF_CYCLES_DCTRACKER_PWRUP  100

/*******************************************************************************
* Touch sense application power-up init done
******************************************************************************/
#define TOUCH_SENSE_APP_PWRUP_INIT_DONE  0x0FFF

/*******************************************************************************
* Return to the wake-up function period (1s)
******************************************************************************/
#if DECIMATION_FILTER
	#define ELEC_WAKEUP_ACTIVATE_COUNTER    (1000 / ELECTRODES_SENSE_PERIOD_DF)
#else
	#define ELEC_WAKEUP_ACTIVATE_COUNTER    (1000 / ELECTRODES_SENSE_PERIOD)
#endif
/*******************************************************************************
* PCR defines
******************************************************************************/
// Configure pin as GPIO, clear ISF, ISF disabled, ALT1=GPIO, high drive strength, disable pulls, fast slew rate
#define PCR_GPIO	0x01000140
// Configure pin as analog input
#define PCR_ANA		0x00000000

/*******************************************************************************
* Type defines
******************************************************************************/
typedef struct
{
	// Hardware
	ADC_Type   *adcBasePtr;
	uint32_t   adcChNum:32;
	PORT_Type  *portBasePtr;
	GPIO_Type  *gpioBasePtr;
	uint32_t   pinNumberElec:32;
	uint32_t   pinNumberCext:32;
	uint32_t   portMask:32;
	uint32_t   ELEC_DMAMUX;
	uint32_t   ELEC_TRGMUX;
	uint8_t	   type;
}
tElecStruct;

// guard pin hardware config
typedef struct
{
	// Hardware
	PORT_Type  *portBasePtr;
	GPIO_Type  *gpioBasePtr;
	uint32_t   pinNumberGuard:32;
	uint32_t   portMask;

} tGuardStruct;

/*******************************************************************************
* Function prototypes
******************************************************************************/
void GuardStructureInit(void);
void ElectrodesStructInit(void);
void ElecAdcChannelFix(uint8_t electrodeNum);
void ElecNumOfSamplesPerElecInit(uint8_t electrodeNum);
void ElecLPFilterTypeInit(uint8_t electrodeNum);
void ElecOversamplingDCTrackerChange(uint8_t electrodeNum, uint8_t changeType);
void ElecDCTrackerShiftInit(uint8_t electrodeNum);
void ElecNumOfSamplesPerElecChange(uint8_t electrodeNum, uint8_t changeType);
void ElectrodesTouchSenseInit(void);
void ElecBufferInitVal(int32_t inputSignal, int32_t *outputSignalPtr, uint16_t *sumCounterPtr);
void ElecCalInitVal(int32_t *outputSignalPtr, uint16_t *sumCounterPtr);
int32_t DCTracker(int32_t inputSignal, int32_t *outputSignalRawPtr, uint8_t shift);
void ElectrodesFloat(void);
void ElectrodesGND(void);
void ElecScanCPU(uint8_t electrodeNum, uint8_t samplingCtrl);
void ElecSlider2PadsScanCPU(uint8_t electrodeNum, uint8_t samplingCtrl);
void ElecRawDataCalc(uint8_t electrodeNum);
void FilterIIR1BufferChange(uint8_t electrodeNum, uint8_t changeType, uint8_t frequency);
void ElectrodesSelfTrimSense(void);
uint8_t ElectrodesSelfTrim(void);
void ElectrodesSelfTrimProcess(void);
void ElecDCTrackerUpdate(uint8_t electrodeNum);
void ElectrodesTouchElecSense(void);
void ElectrodesTouchElecProcess(void);
void ElectrodesWakeupEventSense(void);
void ElectrodesWakeupEventProcess(void);
void ElectrodesWakeupEventCtrl(void);
void ElectrodesGoSleepEventCtrl(void);
void ElectrodesBacklightAndLowPowerCtrl(void);
void ElectrodesVirtualWakeupEventCtrl(void);
void DecimationFilter(uint8_t electrodeNum);
void Jitter(void);
void FrequencyHop(void);
void FrequencyHoppingInit(uint8_t clockmode);
void ElectrodesScanLPTMRHandlerRoutine(void);
void FrequencyHoppingSmplCtrl(void);
uint8_t SmplCtrl(void);
void  FilterMedianWindowInit(uint8_t electrodeNum);
void  FilterMeanWindowInit(uint8_t electrodeNum);

#endif /* __ETS_H */

