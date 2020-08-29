/****************************************************************************//*!
*
* @file     filter.c
*
* @version  1.0.0.0
*
* @date     September-2017
*
* @brief    Filter implementation
*
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include "filter.h"
#include "ets.h"
#include "ts_cfg.h"
#include "S32K118.h"
#include "main.h"
#include "math.h"

/*******************************************************************************
* Variables definition
*******************************************************************************/
// IIR1 filter coefficients
tFrac32 FilterIIR1CoeffB0[NUMBER_OF_IIR_FILTERS_USED], FilterIIR1CoeffB1[NUMBER_OF_IIR_FILTERS_USED], FilterIIR1CoeffA1[NUMBER_OF_IIR_FILTERS_USED];

// IIR1 filter buffer x(k-1), y(k-1)
tFrac32 FilterIIR1BufferX[NUMBER_OF_ELECTRODES][NUMBER_OF_IIR_FILTERS_USED][NUMBER_OF_HOPPING_FREQUENCIES], FilterIIR1BufferY[NUMBER_OF_ELECTRODES][NUMBER_OF_IIR_FILTERS_USED][NUMBER_OF_HOPPING_FREQUENCIES];
// IIR1 Filter type
extern uint8_t   elecLPFilterType[NUMBER_OF_ELECTRODES];

// Slider data filter buffer x(k-1), y(k-1)
tFrac32 FilterIIR1SliderDataBufferX[2][NUMBER_OF_HOPPING_FREQUENCIES], FilterIIR1SliderDataBufferY[2][NUMBER_OF_HOPPING_FREQUENCIES];

// Electrodes window sizes for median and mean
uint16_t FilterMedianWindow[NUMBER_OF_ELECTRODES];
uint16_t FilterMeanWindow[NUMBER_OF_ELECTRODES];

/****************************************************************************
* Math functions
****************************************************************************/
static inline tFrac32 MLIB_ShR_F32(register tFrac32 f32In1,register tU16 u16In2)
{
  return((tFrac32)(f32In1 >> u16In2));
}

static inline tFrac32 Mul_F32_C(register tFrac32 f32In1, register tFrac32 f32In2)
{
  register tS32 s32UpperIn1, s32UpperIn2;
  register tU32 u32LowerIn1, u32LowerIn2, u32ResultLower, u32ResultUpper;
  register tU32 u32Result0, u32Result1, u32Result2;

  s32UpperIn1 = MLIB_ShR_F32(f32In1,(tU16)16U);
  u32LowerIn1 = (tU32)(f32In1 & (tFrac32)0x0000FFFFU);
  s32UpperIn2 = MLIB_ShR_F32(f32In2,(tU16)16U);
  u32LowerIn2 = (tU32)(f32In2 & (tFrac32)0x0000FFFFU);
  u32ResultUpper = u32LowerIn1 * u32LowerIn2;
  u32ResultLower = u32ResultUpper >> (tU16)16U;

  u32ResultUpper = ((tU32)s32UpperIn1 * u32LowerIn2) + u32ResultLower;
  u32Result2 = u32ResultUpper & (tU32)0x0000FFFFU;
  u32Result1 = (tU32)((tS32)u32ResultUpper >> (tU16)16U);
  u32ResultUpper = (u32LowerIn1 * (tU32)s32UpperIn2) + u32Result2;

  u32ResultLower = (tU32)((tS32)u32ResultUpper >> (tU16)16U);
  u32Result0 = (((tU32)s32UpperIn1 * (tU32)s32UpperIn2) + u32Result1) + u32ResultLower;
  u32Result0 = (u32Result0 << (tU16)1U) | ((u32ResultUpper << (tU16)16U) >> (tU16)31U);

  return((tFrac32)u32Result0);
}

/*****************************************************************************
*
* Function: void FilterIIR1Init(void)
*
* Description: Init IIR1 low pass filter
*
*****************************************************************************/
void FilterIIR1Init(void)
{
#ifdef FILTER_1
	// Load coefficients
	FilterIIR1CoeffB0[FILTER_1] = FRAC32(FILTER_1_COEF_B0);
	FilterIIR1CoeffB1[FILTER_1] = FRAC32(FILTER_1_COEF_B1);
	FilterIIR1CoeffA1[FILTER_1] = FRAC32(FILTER_1_COEF_A0);
#endif

#ifdef FILTER_2
	// Load coefficients
	FilterIIR1CoeffB0[FILTER_2] = FRAC32(FILTER_2_COEF_B0);
	FilterIIR1CoeffB1[FILTER_2] = FRAC32(FILTER_2_COEF_B1);
	FilterIIR1CoeffA1[FILTER_2] = FRAC32(FILTER_2_COEF_A0);
#endif

#ifdef FILTER_3
	// Load coefficients
	FilterIIR1CoeffB0[FILTER_3] = FRAC32(FILTER_3_COEF_B0);
	FilterIIR1CoeffB1[FILTER_3] = FRAC32(FILTER_3_COEF_B1);
	FilterIIR1CoeffA1[FILTER_3] = FRAC32(FILTER_3_COEF_A0);
#endif

}

/*****************************************************************************
*
* void FilterIIR1BufferInit(uint8_t elec, tFrac32 valueBufferX, tFrac32 valueBufferY, uint8_t frequencyID)
*
* Description: Init IIR1 low pass filter buffer
*
*****************************************************************************/
void FilterIIR1BufferInit(uint8_t elec, tFrac32 valueBufferX, tFrac32 valueBufferY, uint8_t frequencyID)
{
	FilterIIR1BufferX[elec][elecLPFilterType[elec]][frequencyID] = valueBufferX << IIR_FILTER_VALUE_SHIFT;
	FilterIIR1BufferY[elec][elecLPFilterType[elec]][frequencyID] = valueBufferY << IIR_FILTER_VALUE_SHIFT;
}

/*****************************************************************************
*
* Function: tFrac32 FilterIIR1(uint8_t elec, tFrac32 x_k, uint8_t frequencyID)
*
* Description: IIR1 filter implementation
*              y(k) = b0*x(k) + b1*x(k-1) - a1*y(k-1)
*
*****************************************************************************/
tFrac32 FilterIIR1(uint8_t elec, tFrac32 x_k, uint8_t frequencyID)
{
	register tFrac32 M1;
	register tFrac32 M2;
	register tFrac32 M3;
	register tFrac32 Acc;
	register tFrac32 y_k;

	// Shift left input value to achieve calculation the highest resolution
	x_k = x_k << IIR_FILTER_VALUE_SHIFT;

    // M1 = b0 * x(k)
	M1 = Mul_F32_C(FilterIIR1CoeffB0[elecLPFilterType[elec]], x_k);

    // M2 = b1 * x(k-1)
    M2 = Mul_F32_C(FilterIIR1CoeffB1[elecLPFilterType[elec]], FilterIIR1BufferX[elec][elecLPFilterType[elec]][frequencyID]);

    // M3 = a1 * y(k-1)
    M3 = Mul_F32_C(FilterIIR1CoeffA1[elecLPFilterType[elec]], FilterIIR1BufferY[elec][elecLPFilterType[elec]][frequencyID]);

    // Acc = M2 - M3
    Acc = (tFrac32)(M2 - M3);

    // Acc = Acc + M1
    Acc = (tFrac32)(Acc + M1);

    // Load output
    y_k = Acc;

	// IIR1 filter buffer x(k-1), y(k-1)
	// x(k-1)
	FilterIIR1BufferX[elec][elecLPFilterType[elec]][frequencyID] = x_k;
	// y(k-1)
	FilterIIR1BufferY[elec][elecLPFilterType[elec]][frequencyID] = y_k;

	// Shift right the result to compensate value increase due to desired highest resolution
	y_k = y_k >> IIR_FILTER_VALUE_SHIFT;

    // Return value
    return(y_k);
}

/*****************************************************************************
*
* Function: tFrac32 FilterIIR1SliderData(tFrac32 x_k, uint8_t data_type)
*
* Description: IIR1 filter implementation
*              y(k) = b0*x(k) + b1*x(k-1) - a1*y(k-1)
*
* Note I: Additional function for final slider data
*
* T_sampling:30ms
* cutoff freq: 5 Hz
* f32B0 = FRAC32 (0.337540151883547);
* f32B1 = FRAC32 (0.337540151883547);
* f32A1 = FRAC32 (-0.324919696232906);
*
*****************************************************************************/
tFrac32 FilterIIR1SliderData(tFrac32 x_k, uint8_t data_type, uint8_t frequencyID)
{
	register tFrac32 M1;
	register tFrac32 M2;
	register tFrac32 M3;
	register tFrac32 Acc;
	register tFrac32 y_k;

	// Shift left input value to achieve calculation the highest resolution
	x_k = x_k << IIR_FILTER_VALUE_SHIFT;

    // M1 = b0 * x(k)
	M1 = Mul_F32_C(FRAC32(0.337540151883547), x_k);

    // M2 = b1 * x(k-1)
    M2 = Mul_F32_C(FRAC32(0.337540151883547), FilterIIR1SliderDataBufferX[data_type][frequencyID]);

    // M3 = a1 * y(k-1)
    M3 = Mul_F32_C(FRAC32(-0.324919696232906), FilterIIR1SliderDataBufferY[data_type][frequencyID]);

    // Acc = M2 - M3
    Acc = (tFrac32)(M2 - M3);

    // Acc = Acc + M1
    Acc = (tFrac32)(Acc + M1);

    // Load output
    y_k = Acc;

	// IIR1 filter buffer x(k-1), y(k-1)
	// x(k-1)
    FilterIIR1SliderDataBufferX[data_type][frequencyID] = x_k;
	// y(k-1)
    FilterIIR1SliderDataBufferY[data_type][frequencyID] = y_k;

	// Shift right the result to compensate value increase due to desired highest resolution
	y_k = y_k >> IIR_FILTER_VALUE_SHIFT;

    // Return value
    return(y_k);
}

/*****************************************************************************
*
* Function: uint16_t  Median(uint16_t element, uint16_t array[], uint16_t window_size)
*
* Description: Finds median within specific elements (window) of samples buffer
*
*****************************************************************************/
uint16_t  Median(uint16_t element, uint16_t array[], uint16_t window_size)
{
	uint16_t i,j,temp;

	// Create window
	uint16_t window[window_size];

	// put the samples into the window
	for (i = element; i < (element+window_size); i++)
	{
		window[i-element]= array[i];
	}

	// Sort the window to get median in the middle of the elements
	for(i=0; i<window_size; i++)
	{
		for(j=0; j<(window_size-1); j++)
		{
			if(window[j]>window[j+1])
			{
				temp        = window[j];
				window[j]    = window[j+1];
				window[j+1]  = temp;
			}
		}
	}

	// return middle value from the sorted window
	return window[window_size/2];
}

/*****************************************************************************
*
* Function: uint16_t  Mean(uint16_t element, uint16_t array[], uint16_t window_size)
*
* Description: Finds mean within specific elements (window) of samples buffer
*
*****************************************************************************/
uint16_t  Mean(uint16_t element, uint16_t array[], uint16_t window_size)
{
	// Create helper var
	uint32_t acc = 0;

	for (uint16_t i = element; i < (element+window_size); i++)
	{
		acc+= array[i];
	}
	return (acc/window_size);
}

/*****************************************************************************
*
* Function: void  FilterMedian(uint16_t input_buffer[], uint16_t buffer_size, uint16_t window_size)
*
* Description: Median filtering of input samples buffer
*
*****************************************************************************/
void  FilterMedian(uint16_t input_buffer[], uint16_t buffer_size, uint16_t window_size)
{
	if (window_size >= 3 && window_size < buffer_size)
	{
		// Create helper array
		uint16_t arr[buffer_size + window_size - 1];

		// Load input array into helper array
		for (uint16_t element = 0; element < (buffer_size + window_size - 1); element++)
		{
			if (element < buffer_size)
			{
				arr[element] = input_buffer[element];
			}
			else
			{
				arr[element] = input_buffer[element-buffer_size];
			}
		}

		for (uint16_t element = 0; element < buffer_size; element++)
		{
			// Median filtering
			input_buffer [element] = Median(element, arr, window_size);
		}
	}
	else if (window_size == buffer_size)
	{
		uint16_t i,j, temp, median;

		// Sort the input buffer to get median in the middle of the elements
			for(i=0; i<buffer_size; i++)
			{
				for(j=0; j<(buffer_size-1); j++)
				{
					if(input_buffer[j]>input_buffer[j+1])
					{
						temp        = input_buffer[j];
						input_buffer[j]    = input_buffer[j+1];
						input_buffer[j+1]  = temp;
					}
				}
			}

		// Median of all samples in the input buffer
		median = input_buffer[buffer_size/2];

		for (uint16_t element = 0; element < buffer_size; element++)
		{
			// Median is the same for all samples
			input_buffer[element] = median;
		}
	}

}

/*****************************************************************************
*
* Function: void  FilterMean(uint16_t input_buffer[], uint16_t buffer_size, uint16_t window_size)
*
* Description: Mean filtering of input samples buffer
*
*****************************************************************************/
void  FilterMean(uint16_t input_buffer[], uint16_t buffer_size, uint16_t window_size)
{
	if (window_size >= 2 && window_size < buffer_size)
	{
		// Create helper array
		uint16_t arr[buffer_size + window_size - 1];

		// Load input array into helper array
		for (uint16_t element = 0; element < (buffer_size + window_size - 1); element++)
		{
			if (element < buffer_size)
			{
				arr[element] = input_buffer[element];
			}
			else
			{
				arr[element] = input_buffer[element-buffer_size];
			}
		}


		for (uint16_t element = 0; element < buffer_size; element++)
		{
			// Mean filtering
			input_buffer[element] = Mean(element, arr, window_size);
		}
	}

	else if (window_size == buffer_size)
	{
		uint16_t mean;

		// Mean of all samples in the input buffer
		mean = Mean(0, input_buffer, window_size);

		for (uint16_t element = 0; element < buffer_size; element++)
		{
			// Mean is the same for all samples
			input_buffer[element] = mean;
		}
	}
}

/*****************************************************************************
*
* Function: void  FilterWeightedMean(uint16_t input_buffer[], uint16_t buffer_size, uint32_t average)
*
* Description: Mean filtering of input samples buffer
*
*****************************************************************************/
void  FilterWeightedMean(uint16_t input_buffer[], uint16_t buffer_size, uint16_t weight_size, int32_t average)
{
	int32_t nominator = 0;
	int32_t val = 0;
	int32_t denominator = 0;
	int32_t weighted_mean = 0;
	int16_t weight;
	uint16_t element;
	int16_t i;

	for (element = 0; element < buffer_size; element++)
	{
		val = input_buffer[element] - average;

		if (val < 0)
		{
			val = val*(-1);
		}

		for (i = 1; i <= weight_size; i++)
		{
			if (val < buffer_size*i)
			{
				weight = weight_size - i;
				break;
			}
			else
			{
				weight = 0;
			}
		}

		nominator += input_buffer[element]*weight;
		denominator += weight;
	}

	weighted_mean = nominator/ denominator;

	if (weighted_mean == 0)
	{
		weighted_mean = average;
	}

	for (element = 0; element < buffer_size; element++)
	{
		// Weighted mean is the same for all samples
		input_buffer[element] = weighted_mean;
	}
}


