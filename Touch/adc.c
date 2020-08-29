/****************************************************************************//*!
*
* @file     adc.c
*
* @version  1.0.0.0
*
* @date     September-2017
*
* @brief    ADC routines
*
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include "S32K118.h"
#include "adc.h"
#include "ts_cfg.h"
#include "power_mode.h"

/*******************************************************************************
* Variables
*******************************************************************************/
extern int16_t calibrationGainADC0;
/*****************************************************************************
*
* Function: void ADC0_Init(uint32_t sampleTime, uint32_t avgSel, uint8_t clkMode)
*
* Description: Init ADC0
*
* Note: sampleTime value in range from 2 to 255
*
*****************************************************************************/
void ADC0_Init(uint32_t sampleTime, uint32_t avgSel, uint8_t clkMode)
{
	// Configure ADC based on selected clock mode
	switch (clkMode)
    {
    	case (RUN_FIRC):
		{
#if (TS_ELECTRODE_DATA_ACQUISITION == DMA_ELECTRODE_DATA_ACQUISITION)
    		// set module connections between PDB and ADCs
    		SIM->ADCOPT |= SIM_ADCOPT_ADC0PRETRGSEL(0b00)			// ADC0 pretrigger source select: 0b00: PDB , 0b01= TRGMUX , 0b10=SW
    						| SIM_ADCOPT_ADC0SWPRETRG(0b000)		// ADC0 SW pretrigger source select: 0b000: SW pretrig disabled , 0b100= SW pretrig 0 , 0b101= SW pretrig 1 , 0b110= SW pretrig 2, 0b111= SW pretrig 3
							| SIM_ADCOPT_ADC0TRGSEL(0b0);

    	    // ADC input clock selected in PCC, FIRCDIV 48MHz
    	    // Should be max 64MHz, divide by 1
    	    // ADC input clock = 48/1 = 48MHz, tclk = 21ns
    	    // 12bit resolution, ADCK division by 1, no internal alternate clocks but only selectable via PCC
    	    ADC0->CFG1 = 0x00000004;
    	    // set sample time, 11 - 1 = 10, 10 x 18ns = 0.18us
    	    ADC0->CFG2 = sampleTime;
    	    //enable SW trigger, DMA on
    		ADC0->SC2 	= ADC_SC2_ADTRG(0b0)						// Trigger select: 0= software, 1=hardware
    					| ADC_SC2_ACFE(0b0)							// Compare Enable:  0=disabled , 1=enabled
    					| ADC_SC2_ACFGT(0b0)						// Compare Greater than Enable:  0= comp < than , 1=comp >= than
    					| ADC_SC2_ACREN(0b0)						// Compare Range Enable: 0=disabled , 1=enabled
    					| ADC_SC2_DMAEN(0b1)						// DMA Enable: 0=disabled , 1=enabled
    					| ADC_SC2_REFSEL(0b00);						// Reference Voltage Select: 0b00=Vrefh/Vrefl, 0b01= ALTh/Altl

    	    // Conv. Complete Interrupt enable: 0=disabled , 1=enabled
    	    ADC0->SC1[0] |= ADC_SC1_AIEN(0b0);

    		break;
#else
    	    // ADC input clock selected in PCC, FIRCDIV 48MHz
    	    // Should be max 64MHz, divide by 1
    	    // ADC input clock = 48/1 = 48MHz, tclk = 21ns
    	    // 12bit resolution, ADCK division by 1, no internal alternate clocks but only selectable via PCC
    	    ADC0->CFG1 = 0x00000004;
    	    // set sample time, 11 - 1 = 10, 10 x 18ns = 0.18us
    	    ADC0->CFG2 = sampleTime;
    	    // software trigger, no compare, no DMA, default VREFH/L
    	    ADC0->SC2 = 0x00000000;

    		break;
#endif
		}
    	case (RUN_PLL):
		{
#if (TS_ELECTRODE_DATA_ACQUISITION == DMA_ELECTRODE_DATA_ACQUISITION)
    		// set module connections between PDB and ADCs
    		SIM->ADCOPT |= SIM_ADCOPT_ADC0PRETRGSEL(0b00)			// ADC0 pretrigger source select: 0b00: PDB , 0b01= TRGMUX , 0b10=SW
    						| SIM_ADCOPT_ADC0SWPRETRG(0b000)		// ADC0 SW pretrigger source select: 0b000: SW pretrig disabled , 0b100= SW pretrig 0 , 0b101= SW pretrig 1 , 0b110= SW pretrig 2, 0b111= SW pretrig 3
							| SIM_ADCOPT_ADC0TRGSEL(0b0);

    	    // ADC input clock selected in PCC, SPLLDIV2 40MHz
    	    // Should be max 64MHz, divide by 1
    	    // ADC input clock = 40/1 = 40MHz, tclk = 25ns
    	    // 12bit resolution, ADCK division by 1, no internal alternate clocks but only selectable via PCC
    	    ADC0->CFG1 = 0x00000004;
    	    // set sample time, 11 - 1 = 10, 10 x 18ns = 0.18us
    	    ADC0->CFG2 = sampleTime;
    	    //enable SW trigger, DMA on
    		ADC0->SC2 	= ADC_SC2_ADTRG(0b0)						// Trigger select: 0= software, 1=hardware
    					| ADC_SC2_ACFE(0b0)							// Compare Enable:  0=disabled , 1=enabled
    					| ADC_SC2_ACFGT(0b0)						// Compare Greater than Enable:  0= comp < than , 1=comp >= than
    					| ADC_SC2_ACREN(0b0)						// Compare Range Enable: 0=disabled , 1=enabled
    					| ADC_SC2_DMAEN(0b1)						// DMA Enable: 0=disabled , 1=enabled
    					| ADC_SC2_REFSEL(0b00);						// Reference Voltage Select: 0b00=Vrefh/Vrefl, 0b01= ALTh/Altl

    	    // Conv. Complete Interrupt enable: 0=disabled , 1=enabled
    	    ADC0->SC1[0] |= ADC_SC1_AIEN(0b0);

    		break;
#else
    	    // ADC input clock selected in PCC, SPLLDIV2 40MHz
    	    // Should be max 64MHz, divide by 1
    	    // ADC input clock = 40/1 = 40MHz, tclk = 25ns
    	    // 12bit resolution, ADCK division by 1, no internal alternate clocks but only selectable via PCC
    	    ADC0->CFG1 = 0x00000004;
    	    // set sample time, 11 - 1 = 10, 10 x 18ns = 0.18us
    	    ADC0->CFG2 = sampleTime;
    	    // software trigger, no compare, no DMA, default VREFH/L
    	    ADC0->SC2 = 0x00000000;

    		break;
#endif
		}
    }
}



/*****************************************************************************
*
* Function: int16_t ADC0_Calibration(void)
*
* Description: Init ADC0 calibration
*
*****************************************************************************/
int16_t ADC0_Calibration(void){

	//ADC clock divided by 2 (48/2=24MHz)
	ADC0->CFG1=0x00000024;

	// Clear registers
	ADC0->CLPS=0x00000000;
	ADC0->CLP3=0x00000000;
	ADC0->CLP2=0x00000000;
	ADC0->CLP1=0x00000000;
	ADC0->CLP0=0x00000000;
	ADC0->CLPX=0x00000000;
	ADC0->CLP9=0x00000000;

	/*// Clear registers offset
	ADC0->CLP0_OFS=0x00000000;
	ADC0->CLP1_OFS=0x00000000;
	ADC0->CLP2_OFS=0x00000000;
	ADC0->CLP3_OFS=0x00000000;
	ADC0->CLPX_OFS=0x00000000;
	ADC0->CLP9_OFS=0x00000000;
	ADC0->CLPS_OFS=0x00000000;*/

	// Hardware averaging enable, 32 samples, start calibration
	ADC0->SC3 = ADC_SC3_AVGE(1) | ADC_SC3_AVGS(3) | ADC_SC3_CAL(1);

	// Wait for conversion complete flag
	while(ADC0->SC1[0] < 0x80)
	{}

	// Hardware averaging off
	ADC0->SC3 = ADC_SC3_AVGE(0) | ADC_SC3_AVGS(0);

	// Return the gain register value
	return ADC0->G;
}

/*****************************************************************************
*
* Function: void ClearADCsGain(void)
*
* Description: Clear ADC0 and ADC1 gain registers
*
*****************************************************************************/
void ClearADCsGain(void){

	ADC0->G = 0x00000000;

}

/*****************************************************************************
*
* Function: void SetADCsGain(void)
*
* Description: Set ADC0 and ADC1 gain registers on calibrated value
*
*****************************************************************************/
void SetADCsGain(void){

	ADC0->G = calibrationGainADC0;

}
