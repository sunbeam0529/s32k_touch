/*
 * dma.c
 *
 *  Created on: May 6, 2020
 *      Author: nxf38186
 */

/*******************************************************************************
 * Includes
 *******************************************************************************/
#include "dma.h"
#include "ts_cfg.h"
#include "ets.h"
#include "adc.h"
#include "slider.h"
#include "lpit.h"
#include "power_mode.h"
#include "edma_driver.h"
/*******************************************************************************
 * Variables
 *******************************************************************************/
// Guard pin structure
extern tGuardStruct guardStruct[1];
// Hardware config of electrodes
extern tElecStruct  elecStruct[NUMBER_OF_ELECTRODES];

// Electrodes variables
extern int16_t   elecNumberOfSamples[NUMBER_OF_ELECTRODES];
extern uint8_t  elecNum,elecNumBaselineUpdate;
extern uint16_t sampleNum;
extern uint8_t samplesReadyFlag;
extern uint8_t   firstButtonElecNum, firstSliderElecNum, firstWakeupElecNum;

// frequency hopping
extern uint16_t freqencyIDtimeout[NUMBER_OF_HOPPING_FREQUENCIES];
extern uint8_t   frequencyID;
extern uint8_t lpit_clock;

#if (NUMBER_OF_BUTTON_ELECTRODES > 0)
extern uint16_t   buttonSamplesBuffer[NUMBER_OF_BUTTON_ELECTRODES][(NUMBER_OF_PRESAMPLES + NUMBER_OF_SAMPLES_PER_BUTTON_ELEC_ACTIVE)];
#endif
#if (NUMBER_OF_SLIDER_ELECTRODES > 0)
extern uint16_t   sliderSamplesBuffer[NUMBER_OF_SLIDER_ELECTRODES][(NUMBER_OF_PRESAMPLES + NUMBER_OF_SAMPLES_PER_SLIDER_ELEC_ACTIVE)];
#endif
#if (NUMBER_OF_WAKEUP_ELECTRODES > 0)
extern uint16_t   wakeupSamplesBuffer[NUMBER_OF_WAKEUP_ELECTRODES][(NUMBER_OF_PRESAMPLES + NUMBER_OF_SAMPLES_PER_WAKEUP_ELEC)];
#endif


/* DMA CH0 - Scatter-Gather mode, Electrodes GPIO handling, ADC conversion SW triggering */
// Source and Destination addresses arrays in memory for Scatter - Gather TCDs
uint32_t dma_ch0_saddr_config[NUMBER_OF_ELECTRODES][4];
uint32_t* dma_ch0_daddr_config[NUMBER_OF_ELECTRODES][4];
// Scatter - Gather TCDs set for electrodes
edma_software_tcd_t1 dma_ch0_stcd[NUMBER_OF_ELECTRODES][4]  __attribute__((aligned(32)));

/* DMA CH1 - normal linking mode,  ADC Result register reading and storing in data buffer  */
// Data samples buffer for electrodes
uint16_t dma_samples_buffer[256*NUMBER_OF_ELECTRODES+NUMBER_OF_PRESAMPLES];

/* DMA CH2 - normal linking mode,  modifies SADDR and DADDR of TCD.ch0 to the first SG TCD of the next electrode */
// Source addresses arrays in memory for CH2
uint32_t dma_ch2_saddr_config[(NUMBER_OF_ELECTRODES + 1)*4];

/* DMA CH3 - normal linking mode,   TCD.ch1 to the  TCD of the next electrode and TCD.ch1 major iteration count according to num of samples of electrode to be taken */
// Source addresses arrays in memory for CH3
uint16_t dma_ch3_saddr_config[(NUMBER_OF_ELECTRODES + 1)*2];

uint8_t dmaSeqScanFirstElec, dmaSeqScanLastElec;
uint8_t dma_scanning_scenario;
/*****************************************************************************
 *
 * Function: void DMA_Scan_Start(uint8_t samplingCtrl)
 *
 * Description: Starts DMA touch sense algorithm (chain)
 *
 *****************************************************************************/
void DMA_Scan_Start(uint8_t samplingCtrl)
{
	DMA->CDNE = DMA_CDNE_CADN_MASK;	// Clear DONE status bit
	DMA->CERQ = DMA_CERQ_CERQ(1);	// Clear request for CH1
	DMA->CERR = DMA_CERR_CAEI_MASK;	// Clear errors
	DMA->SERQ = DMA_SERQ_SERQ(1); // CH1 enable request

#if FREQUENCY_HOPPING
	DMA->CERQ = DMA_CERQ_CERQ(0);	// Clear request for CH0
	DMA->SERQ = DMA_SERQ_SERQ(0);   // Enable request for CH0

	if (samplingCtrl == 0)
	{
		// Enable LPIT CH0 to periodically trigger DMA CH0 after each sample taken
		LPIT_Enable(0, freqencyIDtimeout[frequencyID]*lpit_clock);
	}
	else
	{
		// Start DMA manually and DMA CH1 will be triggering DMA CH0 after each minor loop done (= sample taken)
		DMA->TCD[0].CSR |= DMA_TCD_CSR_START(1);
	}
#else
	// Start DMA manually and DMA CH1 will be triggering DMA CH0 after each minor loop done (= sample taken)
	DMA->TCD[0].CSR |= DMA_TCD_CSR_START(1);
#endif
}

/*****************************************************************************
 *
 * Function: void DMA_Init(void)
 *
 * Description: Init DMA Control register, common for all electrodes scanning scenarios
 *
 *****************************************************************************/
void DMA_Init(void)
{
	DMA->CR |= DMA_CR_CLM(0) |							// Continuous Link mode disabled
			DMA_CR_CX(0)   |							// Cancel transfer off - normal operation
			DMA_CR_ECX(0)  |							// Error cancel transfer off - normal operation
			DMA_CR_EDBG(0) |							// Disabled debug - When in debug mode, the DMA continues to operate.
			DMA_CR_EMLM(1) |							// Enabled minor loop mapping
			DMA_CR_ERCA(0) |							// Round robin channel arbitration - Fixed priority arbitration is used for channel selection
			DMA_CR_HALT(0) |							// Halt DMA operations disabled - normal operation
			DMA_CR_HOE(0);								// Halt on error disabled - normal operation

	// Load sample control
	uint8_t sampleControl = SmplCtrl();

#if FREQUENCY_HOPPING
	// DMA CH0 can be periodically triggered by LPIT CH0
	DMAMUX->CHCFG[0] = DMAMUX_CHCFG_ENBL(0);		// disable DMAMUX
	DMAMUX->CHCFG[0] = 	DMAMUX_CHCFG_SOURCE(62) |   // always enabled as trigger comes via TRGMUX
			DMAMUX_CHCFG_TRIG(1) |
			DMAMUX_CHCFG_ENBL(1);		// enable DMAMUX
#endif

	// Initialize memory address arrays for DMA CH0
	DMA_CH0_All_Address_Sets_Load();
	// Initialize TCD sets for scatter-gather DMA CH0
	DMA_CH0_SG_TCDs_Load();

	// Initialize memory address arrays for DMA CH2
	DMA_CH2_All_Address_Sets_Load();

	// Initialize memory address arrays for DMA CH3
	DMA_CH3_All_Address_Sets_Load(sampleControl);
}

/*****************************************************************************
 *
 * Function: void DMA_CH0_All_Address_Sets_Load(void)
 *
 * Description: Loads sets of Source and Destination addresses to be operated by the DMA CH0 for each electrode
 *
 * Note I: All type of electrodes are sorted into SADDR and DADDR array in following order : button electrodes, slider electrodes, wake up electrodes
 *
 *****************************************************************************/
void DMA_CH0_All_Address_Sets_Load(void)
{
	uint8_t electrodeNumFirst, electrodeNumSecond;

	// DMA CH0 Scatter-Gather parameters configuration for electrodes
	for (elecNum = 0; elecNum < NUMBER_OF_ELECTRODES; elecNum++) // Run through all electrodes
	{
#ifdef SLIDER_SELF_GUARDING // Slider consists of only two pads and self-guarding defined?
		// Button or Wakeup electrode
		if (elecStruct[elecNum].type == BUTTON || elecStruct[elecNum].type == WAKEUP)
#endif
		{
			//SADDR_CONFIG - value to write to the register
			//DADDR_CONFIG - address of the register

#if (GUARD) // Guard pin as guard
			// Pre-configure guard pin as GPIO functionality
			guardStruct[0].portBasePtr->PCR[guardStruct[0].pinNumberGuard] = PORT_PCR_MUX(0b001);
#endif
			// Pre-configure electrode and cext pins as GPIO functionality (to save 2 SG TCDs = saved 700ns per single sample)
			elecStruct[elecNum].portBasePtr->PCR[elecStruct[elecNum].pinNumberElec] = PORT_PCR_MUX(0b001);
			elecStruct[elecNum].portBasePtr->PCR[elecStruct[elecNum].pinNumberCext] = PORT_PCR_MUX(0b001);
			// Pre-configure guard pin as GPIO functionality

			// Cext Pin = 1;(Electrodes pins PDOR are always 0)
			dma_ch0_saddr_config[elecNum][0] = (uint32_t) (1 << elecStruct[elecNum].pinNumberCext);
			dma_ch0_daddr_config[elecNum][0] = (uint32_t *) &elecStruct[elecNum].gpioBasePtr->PSOR;
			// Electrode and Cext pins = output
			dma_ch0_saddr_config[elecNum][1] = (uint32_t) ((elecStruct[elecNum].gpioBasePtr->PDDR) | (elecStruct[elecNum].portMask));
			dma_ch0_daddr_config[elecNum][1] = (uint32_t *) &elecStruct[elecNum].gpioBasePtr->PDDR;
			// ADC read Cext pin voltage
			dma_ch0_saddr_config[elecNum][2] = (uint32_t) ADC_SC1_ADCH(elecStruct[elecNum].adcChNum);
			dma_ch0_daddr_config[elecNum][2] = (uint32_t *) &elecStruct[elecNum].adcBasePtr->SC1[0];
			// Elec and Cext pins as input
			dma_ch0_saddr_config[elecNum][3] = (uint32_t) (elecStruct[elecNum].gpioBasePtr->PDDR) & ~(elecStruct[elecNum].portMask);
			dma_ch0_daddr_config[elecNum][3] = (uint32_t *) &elecStruct[elecNum].gpioBasePtr->PDDR;

		}
#ifdef SLIDER_SELF_GUARDING // Slider consists of only two pads and self-guarding defined?
		// Is this a slider electrode?
		else if (elecStruct[elecNum].type == SLIDER)
		{
			// Determine if the electrode scanned is first or second from the 2 slider electrodes
			Slider2PadsElectrodesSort(elecNum, &electrodeNumFirst, &electrodeNumSecond);

			//SADDR_CONFIG - value to write to the register
			//DADDR_CONFIG - address of the register

#if (GUARD) // Guard pin as guard
			// Pre-configure guard pin as GPIO functionality
			guardStruct[0].portBasePtr->PCR[guardStruct[0].pinNumberGuard] = PORT_PCR_MUX(0b001);
#endif
			// Pre-configure slider electrode and cext pins as GPIO functionality (to save 2 SG TCDs = saved 700ns per single sample)
			elecStruct[elecNum].portBasePtr->PCR[elecStruct[elecNum].pinNumberElec] = PORT_PCR_MUX(0b001);
			elecStruct[elecNum].portBasePtr->PCR[elecStruct[elecNum].pinNumberCext] = PORT_PCR_MUX(0b001);

			// Cext Pins of both slider electrodes = 1;(Electrodes pins PDOR are always 0)
			dma_ch0_saddr_config[elecNum][0] = (uint32_t) (1 << elecStruct[electrodeNumFirst].pinNumberCext | 1 << elecStruct[electrodeNumSecond].pinNumberCext);
			dma_ch0_daddr_config[elecNum][0] = (uint32_t *) &elecStruct[electrodeNumFirst].gpioBasePtr->PSOR;
			// Electrode and Cext pins of both slider electrodes = output
			dma_ch0_saddr_config[elecNum][1] = (uint32_t) ((elecStruct[electrodeNumFirst].gpioBasePtr->PDDR) | (elecStruct[electrodeNumFirst].portMask + elecStruct[electrodeNumSecond].portMask));
			dma_ch0_daddr_config[elecNum][1] = (uint32_t *) &elecStruct[electrodeNumFirst].gpioBasePtr->PDDR;
			// ADC read Cext pin voltage
			dma_ch0_saddr_config[elecNum][2] = (uint32_t) ADC_SC1_ADCH(elecStruct[elecNum].adcChNum);
			dma_ch0_daddr_config[elecNum][2] = (uint32_t *) &elecStruct[elecNum].adcBasePtr->SC1[0];
			// Elec and Cext pins of both slider electrodes as input
			dma_ch0_saddr_config[elecNum][3] = (uint32_t) (elecStruct[electrodeNumFirst].gpioBasePtr->PDDR) & ~(elecStruct[electrodeNumFirst].portMask + elecStruct[electrodeNumSecond].portMask);
			dma_ch0_daddr_config[elecNum][3] = (uint32_t *) &elecStruct[electrodeNumFirst].gpioBasePtr->PDDR;
		}
#endif

	}
}
/*****************************************************************************
 *
 * Function: void DMA_CH0_SG_TCDs_Load(void)
 *
 * Description: Loads set of Scatter-Gather TCDs (Transfer Control Descriptors) for DMA CH0 for each electrode
 *
 * Note I: All type of electrodes are sorted into one array in following order : button electrodes, slider electrodes, wake up electrodes
 *
 *****************************************************************************/
void DMA_CH0_SG_TCDs_Load(void)
{
	uint8_t tcdInstructionNum;

	for (elecNum = 0; elecNum < NUMBER_OF_ELECTRODES; elecNum++) // Run through all electrodes
	{
		// Scatter / gather TCDs configuration for each electrode
		for (tcdInstructionNum = 0; tcdInstructionNum < 4; tcdInstructionNum++)
		{
			dma_ch0_stcd[elecNum][tcdInstructionNum].CSR = DMA_TCD_CSR_BWC(0)         |         // BWC=0: No eDMA engine stalls - full bandwidth
					DMA_TCD_CSR_MAJORELINK(0)  |        										// The channel-to-channel linking is disabled
					DMA_TCD_CSR_MAJORLINKCH(0) |       										// No major channel linking
					DMA_TCD_CSR_ESG(1)         |         										// The current channel's TCD is in scatter/gather mode
					DMA_TCD_CSR_DREQ(0)        |         										// The channel's ERQ bit is not affected
					DMA_TCD_CSR_INTHALF(0)     |         										// The half-point interrupt is disabled
					DMA_TCD_CSR_INTMAJOR(0)    |         										// The end-of-major loop interrupt is disabled
					DMA_TCD_CSR_START(1);                										// The channel is explicitly started via a software initiated service request
			dma_ch0_stcd[elecNum][tcdInstructionNum].BITER = DMA_TCD_BITER_ELINKNO_BITER(1) | 					// Starting major iteration count is 1
					DMA_TCD_BITER_ELINKNO_ELINK(0);  										// The minor channel-to-channel linking is disabled
			dma_ch0_stcd[elecNum][tcdInstructionNum].SADDR = (uint32_t) &dma_ch0_saddr_config[elecNum][tcdInstructionNum];					// Source address
			dma_ch0_stcd[elecNum][tcdInstructionNum].SOFF = DMA_TCD_SOFF_SOFF(0);                				// Source Offset
			dma_ch0_stcd[elecNum][tcdInstructionNum].ATTR = DMA_TCD_ATTR_SMOD(0)  |              				// Source address modulo feature is disabled
					DMA_TCD_ATTR_SSIZE(2) |              										// Source data transfer size: 1: 16-bit, 2=32-bit
					DMA_TCD_ATTR_DMOD(0)  |              										// Destination address modulo feature: 0=disabled, x= x power of 2 buffer[DMOD=4->buffer of 16bytes]
					DMA_TCD_ATTR_DSIZE(2);               										// Destination data transfer size: 1: 16-bit, 2=32-bit
			dma_ch0_stcd[elecNum][tcdInstructionNum].NBYTES = DMA_TCD_NBYTES_MLNO_NBYTES(4); 					// Minor Byte Transfer Count is 4-bytes
			dma_ch0_stcd[elecNum][tcdInstructionNum].SLAST = DMA_TCD_SLAST_SLAST(0);     						// Last Source Address Adjustment is 0
			dma_ch0_stcd[elecNum][tcdInstructionNum].DADDR = (uint32_t) dma_ch0_daddr_config[elecNum][tcdInstructionNum];					// Destination address
			dma_ch0_stcd[elecNum][tcdInstructionNum].DOFF = DMA_TCD_DOFF_DOFF(0);                				// Destination Address Signed Offset is 0
			dma_ch0_stcd[elecNum][tcdInstructionNum].CITER = DMA_TCD_CITER_ELINKNO_CITER(1) | 					// Current Major Iteration Count is 1
					DMA_TCD_CITER_ELINKNO_ELINK(0);  										// The minor channel-to-channel linking is disabled
			dma_ch0_stcd[elecNum][tcdInstructionNum].DLASTSGA = (uint32_t) &dma_ch0_stcd[elecNum][tcdInstructionNum+1];					// Scatter gather link to the next TCD

		}
		dma_ch0_stcd[elecNum][0].CSR &= ~DMA_TCD_CSR_START_MASK;								// DMA channel 0 autostart off
#if (GUARD)
		// Guard pin = 0,  delay 60ns
		dma_ch0_stcd[elecNum][1].SOFF = DMA_TCD_SOFF_SOFF(((uint32_t) &guardStruct[0].portMask)-((uint32_t) &dma_ch0_saddr_config[elecNum][1])); // Source Offset to immediately jump on next source address - guard portmask source address
		dma_ch0_stcd[elecNum][1].DOFF = DMA_TCD_DOFF_DOFF(((uint32_t) &guardStruct[0].gpioBasePtr->PCOR)-((uint32_t) dma_ch0_daddr_config[elecNum][1])); // Destination Address Offset to immediately jump to next destination address - guard pin PCOR
		dma_ch0_stcd[elecNum][1].NBYTES = DMA_TCD_NBYTES_MLNO_NBYTES(8); // Minor Byte Transfer Count is 8-bytes

		// Guard pin = 1,  delay 60ns
		dma_ch0_stcd[elecNum][3].SOFF = DMA_TCD_SOFF_SOFF(((uint32_t) &guardStruct[0].portMask)-((uint32_t) &dma_ch0_saddr_config[elecNum][3])); // Source Offset to immediately jump on next source address - guard portmask source address
		dma_ch0_stcd[elecNum][3].DOFF = DMA_TCD_DOFF_DOFF(((uint32_t) &guardStruct[0].gpioBasePtr->PSOR)-((uint32_t) dma_ch0_daddr_config[elecNum][3])); // Destination Address Offset to immediately jump to next destination address - guard pin PSOR
		dma_ch0_stcd[elecNum][3].NBYTES = DMA_TCD_NBYTES_MLNO_NBYTES(8); // Minor Byte Transfer Count is 8-bytes

		dma_ch0_stcd[elecNum][3].DLASTSGA = (uint32_t)(&dma_ch0_stcd[elecNum][0]);    					// Scatter Gather loop to the 1st TCD
#else
		dma_ch0_stcd[elecNum][3].DLASTSGA = (uint32_t)(&dma_ch0_stcd[elecNum][0]);    					// Scatter Gather loop to the 1st TCD
#endif
	}
}
/*****************************************************************************
 *
 * Function: void DMA_CH2_All_Address_Sets_Load(void)
 *
 * Description: Loads sets of Source and Destination addresses to be operated by the DMA CH2 for each electrode
 *
 * Note I: All type of electrodes are sorted into SADDR array in following order : button electrodes, slider electrodes, wake up electrodes
 *
 *****************************************************************************/
void DMA_CH2_All_Address_Sets_Load(void)
{
	uint8_t index = 0;
	// DMA CH 2 - Source addresses arrays init for modifying SADDR, NBYTES, DADDR AND DLASTSGA of DMA TCD.ch0 to the first SG TCD of the next electrode
	for (elecNum = 0; elecNum < NUMBER_OF_ELECTRODES; elecNum++)
	{
		// Four instructions done using SOFF (4bytes) and DOFF(8bytes) in minor loop
		dma_ch2_saddr_config[index] = dma_ch0_stcd[elecNum][0].SADDR; // Load SADDR of first SG TCD for each electrode
		index++;
		dma_ch2_saddr_config[index] = dma_ch0_stcd[elecNum][0].NBYTES; // Load NBYTES of first SG TCD for each electrode
		index++;
		dma_ch2_saddr_config[index] = dma_ch0_stcd[elecNum][0].DADDR; // Load DADDR of first SG TCD for each electrode
		index++;
		dma_ch2_saddr_config[index] = dma_ch0_stcd[elecNum][0].DLASTSGA; // Load DLASTSGA of first SG TCD for each electrode
		index++;
	}
	// At the end of Major loop of DMA CH2 - automatically reset the tcd0. SADDR, NBYTES, DADDR and DLASTSGA to correspond with first electrode (elec0)
	dma_ch2_saddr_config[NUMBER_OF_ELECTRODES*4] = dma_ch0_stcd[0][0].SADDR;
	dma_ch2_saddr_config[(NUMBER_OF_ELECTRODES*4)+1] = dma_ch0_stcd[0][0].NBYTES;
	dma_ch2_saddr_config[(NUMBER_OF_ELECTRODES*4)+2] = dma_ch0_stcd[0][0].DADDR;
	dma_ch2_saddr_config[(NUMBER_OF_ELECTRODES*4)+3] = dma_ch0_stcd[0][0].DLASTSGA;
}
/*****************************************************************************
 *
 * Function: void DMA_CH3_All_Address_Sets_Load(void)
 *
 * Description: Loads sets of Source and Destination addresses to be operated by the DMA CH3 for each electrode
 *
 * Note I: All type of electrodes are sorted into SADDR array in following order : button electrodes, slider electrodes, wake up electrodes
 *
 *
 *****************************************************************************/
void DMA_CH3_All_Address_Sets_Load(uint8_t samplingCtrl)
{
	uint8_t index = 0;
	// DMA CH 3 - Source addresses arrays init for modifying BITER and CITER of TCD.Ch1 to change the number of samples to be taken for the scanned electrode
	for (elecNum = 0; elecNum < NUMBER_OF_ELECTRODES; elecNum++)
	{
		// Two instructions done using SOFF(4 bytes) and DOFF(8bytes) in minor loop
		dma_ch3_saddr_config[index] = DMA_TCD_CITER_ELINKYES_CITER_LE(elecNumberOfSamples[elecNum] + NUMBER_OF_PRESAMPLES) |		// Starting major iteration count is set to required samples quantity in one cycle
 	  	  	  	   DMA_TCD_CITER_ELINKYES_ELINK(samplingCtrl)	 	|			// The minor channel-to-channel linking is enabled
				   DMA_TCD_CITER_ELINKYES_LINKCH(0);
		index++;
		dma_ch3_saddr_config[index] = DMA_TCD_BITER_ELINKYES_BITER(elecNumberOfSamples[elecNum] + NUMBER_OF_PRESAMPLES) | 			// Starting major iteration count is set to required samples quantity in one cycle
				DMA_TCD_BITER_ELINKYES_ELINK(samplingCtrl) 	|				// The minor channel-to-channel linking is enabled
				DMA_TCD_BITER_ELINKYES_LINKCH(0);
		index++;
	}
	// At the end of Major loop of DMA CH3 - automatically reset the tcd1 BITER and CITER to load number of samples to be taken to correspond with first electrode (elec0)
	dma_ch3_saddr_config[NUMBER_OF_ELECTRODES*2] = DMA_TCD_CITER_ELINKYES_CITER_LE(elecNumberOfSamples[0] + NUMBER_OF_PRESAMPLES) |		// Starting major iteration count is set to required samples quantity in one cycle
	  	  	   DMA_TCD_CITER_ELINKYES_ELINK(samplingCtrl)	 	|			// The minor channel-to-channel linking is enabled
			   DMA_TCD_CITER_ELINKYES_LINKCH(0);
	dma_ch3_saddr_config[(NUMBER_OF_ELECTRODES*2)+1] = DMA_TCD_BITER_ELINKYES_BITER(elecNumberOfSamples[0] + NUMBER_OF_PRESAMPLES) | 			// Starting major iteration count is set to required samples quantity in one cycle
			DMA_TCD_BITER_ELINKYES_ELINK(samplingCtrl) 	|				// The minor channel-to-channel linking is enabled
			DMA_TCD_BITER_ELINKYES_LINKCH(0);
}

/*****************************************************************************
 *
 * Function: void DMA_TCDreg_Sequence_Scan_Init(uint8_t startingElecNum, uint8_t lastElecNum)
 *
 * Description: Init DMA TCD CH0-CH5 registers for DMA scanning scenario 1
 *
 * Note I: DMA scanning scenario 1 = Electrodes sequence to be scanned
 *
 * Note II : The electrodes in elecStruct numbered from startingElecNum to lastElecNum will be scanned
 *
 *****************************************************************************/
void DMA_TCDreg_Sequence_Scan_Init(uint8_t startingElecNum, uint8_t lastElecNum, uint8_t samplingCtrl)
{

	// DMA triggering by electrode's ADC config
	DMAMUX->CHCFG[1] = DMAMUX_CHCFG_ENBL(0);																						// disable DMAMUX
	DMAMUX->CHCFG[1] = DMAMUX_CHCFG_SOURCE(elecStruct[startingElecNum].ELEC_DMAMUX) | DMAMUX_CHCFG_TRIG(0) | DMAMUX_CHCFG_ENBL(1);	// adcX_COCO[0] flag triggers DMA_Ch1, enable DMAMUX

#if FREQUENCY_HOPPING
	if (samplingCtrl  == 0) // sampling controlled (triggered) by LPIT
	{
		// DMA CH0 periodically triggered by LPIT CH0
		DMAMUX->CHCFG[0] |= DMAMUX_CHCFG_ENBL(1);		// enable DMAMUX
	}
	else // sampling controlled by LPIT linking
	{
		// DMA CH0 periodically linked by CH1 and last DMA CH
		DMAMUX->CHCFG[0] |= DMAMUX_CHCFG_ENBL(0);		// disable DMAMUX
	}
#endif

	// DMA ch0 TCD config
	DMA->TCD[0].CSR &= 0xFFFFFFFF ^ DMA_TCD_CSR_DONE_MASK;  					// Clear Channel Done flag
	DMA->TCD[0].BITER.ELINKNO = DMA_TCD_BITER_ELINKNO_BITER(1) | 				// Starting major iteration count is 1
							    DMA_TCD_BITER_ELINKNO_ELINK(0);  				// The minor channel-to-channel linking is disabled
	DMA->TCD[0].SADDR = (uint32_t) dma_ch0_stcd[startingElecNum][0].SADDR;  	// The 1st TCD Source Address
	DMA->TCD[0].SOFF = DMA_TCD_SOFF_SOFF(0);                					// Source Offset
	DMA->TCD[0].ATTR = DMA_TCD_ATTR_SMOD(0)  |              					// Source address modulo feature is disabled
					   DMA_TCD_ATTR_SSIZE(2) |              					// Source data transfer size: 1: 16-bit, 2=32-bit
					   DMA_TCD_ATTR_DMOD(0)  |              					// Destination address modulo feature: 0=disabled, x= x power of 2 buffer[DMOD=4->buffer of 16bytes]
					   DMA_TCD_ATTR_DSIZE(2);               					// Destination data transfer size: 1: 16-bit, 2=32-bit
	DMA->TCD[0].NBYTES.MLNO = DMA_TCD_NBYTES_MLNO_NBYTES(4);					// Minor Byte Transfer Count is 4-bytes
	DMA->TCD[0].SLAST = DMA_TCD_SLAST_SLAST(0);             					// Last Source Address Adjustment is 0
	DMA->TCD[0].DADDR = (uint32_t) dma_ch0_stcd[startingElecNum][0].DADDR;     	// The 1st TCD Destination Address
	DMA->TCD[0].DOFF = DMA_TCD_DOFF_DOFF(0);                					// Destination Address Signed Offset is 0
	DMA->TCD[0].DLASTSGA = (uint32_t)(&dma_ch0_stcd[startingElecNum][1]);    	// Scatter-Gather link to the next TCD
	DMA->TCD[0].CITER.ELINKNO = DMA_TCD_CITER_ELINKNO_CITER(1) | 				// Current Major Iteration Count is 1
							    DMA_TCD_CITER_ELINKNO_ELINK(0);  				// The channel-to-channel linking is disabled
	DMA->TCD[0].CSR = DMA_TCD_CSR_BWC(0)         |         						// BWC=0: No eDMA engine stalls - full bandwidth
					  DMA_TCD_CSR_MAJORELINK(0)  |        						// The major channel-to-channel linking is disabled
					  DMA_TCD_CSR_MAJORLINKCH(0) |       						// No channel major linking
					  DMA_TCD_CSR_ESG(1)         |         						// The current channel's TCD is in scatter/gather mode
					  DMA_TCD_CSR_DREQ(0)        |         						// The channel's ERQ bit is not affected
					  DMA_TCD_CSR_INTHALF(0)     |         						// The half-point interrupt is disabled
					  DMA_TCD_CSR_INTMAJOR(0)    |         						// The end-of-major loop interrupt is disabled
					  DMA_TCD_CSR_START(0);                						// The channel is not explicitly started

	// DMA channel 1 config - triggered by ADCx COCO flag, reads the ADC result data from register and stores them in buffer
	DMA->TCD[1].CSR &= 0xFFFFFFFF ^ DMA_TCD_CSR_DONE_MASK;  							// Clear Channel Done flag
	DMA->TCD[1].CITER.ELINKYES = DMA_TCD_CITER_ELINKYES_CITER_LE(elecNumberOfSamples[startingElecNum] + NUMBER_OF_PRESAMPLES) |		// Starting major iteration count is set to required samples quantity in one cycle
				  	  	  	  	  	   DMA_TCD_CITER_ELINKYES_ELINK(samplingCtrl)	 	|			// The minor channel-to-channel linking is disabled/enabled
									   DMA_TCD_CITER_ELINKYES_LINKCH(0);				// The minor channel-to-channel link is to Ch 0
	DMA->TCD[1].BITER.ELINKYES = DMA_TCD_BITER_ELINKYES_BITER(elecNumberOfSamples[startingElecNum] + NUMBER_OF_PRESAMPLES) | 			// Starting major iteration count is set to required samples quantity in one cycle
									DMA_TCD_BITER_ELINKYES_ELINK(samplingCtrl) 	|				// The minor channel-to-channel linking is disabled/enabled
									DMA_TCD_BITER_ELINKYES_LINKCH(0);	 				// The minor channel-to-channel link is to Ch 0
	DMA->TCD[1].SADDR = (uint32_t) &(elecStruct[startingElecNum].adcBasePtr->R[0]);  	// Source Address of the ADC result
	DMA->TCD[1].SOFF = DMA_TCD_SOFF_SOFF(0);                							// Source Offset
	DMA->TCD[1].ATTR = DMA_TCD_ATTR_SMOD(0)  |              							// Source address modulo feature is disabled
	                    DMA_TCD_ATTR_SSIZE(1) |              							// Source data transfer size: 1: 16-bit, 2=32-bit
	                    DMA_TCD_ATTR_DMOD(0)  |              							// Destination address modulo feature: 0=disabled, x= x power of 2 buffer[DMOD=4->buffer of 16bytes]
	                    DMA_TCD_ATTR_DSIZE(1);               							// Destination data transfer size: 1: 16-bit, 2=32-bit
	DMA->TCD[1].NBYTES.MLNO = DMA_TCD_NBYTES_MLNO_NBYTES(2);							// Minor Byte Transfer Count is 2-bytes
	DMA->TCD[1].SLAST = DMA_TCD_SLAST_SLAST(0);             							// Last Source Address Adjustment is 0
	DMA->TCD[1].DADDR = (uint32_t) &dma_samples_buffer[0];     							// Destination Address of the storage array
	DMA->TCD[1].DOFF = DMA_TCD_DOFF_DOFF(2);                							// Destination Address Signed Offset is 2
	DMA->TCD[1].DLASTSGA = DMA_TCD_DLASTSGA_DLASTSGA(0);    							// Destination last address adjustment is 0 (will be done manually in interrupt routine)
	DMA->TCD[1].CSR = DMA_TCD_CSR_BWC(0)         |         								// BWC=0: No eDMA engine stalls - full bandwidth
	                    DMA_TCD_CSR_MAJORELINK(1)  |        							// The major channel-to-channel linking is enabled
	                    DMA_TCD_CSR_MAJORLINKCH(2) |       								// CH2 major linking
	                    DMA_TCD_CSR_ESG(0)         |         							// The current channel TCD mode is normal
	                    DMA_TCD_CSR_DREQ(0)        |         							// The channel's ERQ bit will NOT be cleared
	                    DMA_TCD_CSR_INTHALF(0)     |         							// The half-point interrupt is disabled
	                    DMA_TCD_CSR_INTMAJOR(0)    |         							// The end-of-major loop interrupt is disabled
	                    DMA_TCD_CSR_START(0);                							// The channel is not explicitly started

	/* DMA channel 2 config - linking mode, modifies SADDR, NBYTES, DADDR and DLASTSGA of TCD.ch0 to the first SG TCD of the next electrode, both minor and major links to ch3 */
	DMA->TCD[2].CSR &= 0xFFFFFFFF ^ DMA_TCD_CSR_DONE_MASK;  										// Clear Channel Done flag
	DMA->TCD[2].CITER.ELINKYES = DMA_TCD_CITER_ELINKYES_CITER_LE(lastElecNum + 1 - startingElecNum) |    // Current iteration count is set to number of pads
			DMA_TCD_CITER_ELINKYES_ELINK(1)	 	|													// The minor channel-to-channel linking is enabled
			DMA_TCD_CITER_ELINKYES_LINKCH(3);														// The minor channel-to-channel link is to Ch 3
	DMA->TCD[2].BITER.ELINKYES = DMA_TCD_BITER_ELINKYES_BITER(lastElecNum + 1 - startingElecNum) | 		// Starting major iteration count is set to number of pads
			DMA_TCD_BITER_ELINKYES_ELINK(1)|														// The minor channel-to-channel linking is enabled
			DMA_TCD_BITER_ELINKYES_LINKCH(3);  														// The minor channel-to-channel link is to Ch 3
	DMA->TCD[2].SADDR = (uint32_t) &dma_ch2_saddr_config[4*(1+startingElecNum)];  					// Source Address (for next - second electrode)
	DMA->TCD[2].SOFF = DMA_TCD_SOFF_SOFF(4);    													// Source Offset 4bytes
	DMA->TCD[2].ATTR = DMA_TCD_ATTR_SMOD(0)  |              										// Source address modulo feature is disabled
			DMA_TCD_ATTR_SSIZE(2) |              													// Source data transfer size: 1: 16-bit, 2=32-bit
			DMA_TCD_ATTR_DMOD(0)  |              													// Destination address modulo feature: 0=disabled, x= x power of 2 buffer[DMOD=4->buffer of 16bytes]
			DMA_TCD_ATTR_DSIZE(2);               													// Destination data transfer size: 1: 16-bit, 2=32-bit
	DMA->TCD[2].NBYTES.MLOFFYES = DMA_TCD_NBYTES_MLOFFYES_NBYTES(16) |								// Minor Byte Transfer Count is 16-bytes
										DMA_TCD_NBYTES_MLOFFYES_DMLOE(1)|							// Minor Loop offset destination enable
										DMA_TCD_NBYTES_MLOFFYES_MLOFF(-32);							// Minor loop offset value -32 (after minor loop done go back 32 bytes) - back to &DMA->TCD[0].SADDR
	DMA->TCD[2].SLAST = DMA_TCD_SLAST_SLAST(-16*(lastElecNum + 1 - startingElecNum));             				// Last Source Address Adjustment is back to dma_ch2_saddr_config[4]
	DMA->TCD[2].DADDR = (uint32_t) &DMA->TCD[0].SADDR;												// Destination Address is TCD0.SADDR
	DMA->TCD[2].DOFF = DMA_TCD_DOFF_DOFF(8); 														// Destination Address Signed Offset is 8 bytes
	DMA->TCD[2].DLASTSGA = DMA_TCD_DLASTSGA_DLASTSGA(-32);    										// Destination after major loop done -32 (after major loop done go back 32 bytes) - back to &DMA->TCD[0].SADDR
	DMA->TCD[2].CSR = DMA_TCD_CSR_BWC(0)         |         											// BWC=0: No eDMA engine stalls - full bandwidth
			DMA_TCD_CSR_MAJORELINK(1)  |        													// The major channel-to-channel linking is enabled
			DMA_TCD_CSR_MAJORLINKCH(3) |       														// channel 3 will be called from ch2
			DMA_TCD_CSR_ESG(0)         |         													// The current channel TCD mode is normal
			DMA_TCD_CSR_DREQ(0)        |         													// The channel's ERQ bit will not be cleared
			DMA_TCD_CSR_INTHALF(0)     |         													// The half-point interrupt is disabled
			DMA_TCD_CSR_INTMAJOR(0)    |         													// The end-of-major loop interrupt is disabled
			DMA_TCD_CSR_START(0);                													// The channel is not explicitly started

	/* DMA channel 3 config - linking mode, modifies TCD.ch1 CITER and BITER to change the number of samples to be taken for the next scanned electrode, both minor and major links to channel 4*/
	DMA->TCD[3].CSR &= 0xFFFFFFFF ^ DMA_TCD_CSR_DONE_MASK;  										// Clear Channel Done flag
	DMA->TCD[3].CITER.ELINKYES = DMA_TCD_CITER_ELINKYES_CITER_LE(lastElecNum + 1 - startingElecNum) |   // Current iteration count is set to number of pads
			DMA_TCD_CITER_ELINKYES_ELINK(samplingCtrl)	 	|													// The minor channel-to-channel linking is enabled/disabled
			DMA_TCD_CITER_ELINKYES_LINKCH(0);														// The minor channel-to-channel link is to Ch 0
	DMA->TCD[3].BITER.ELINKYES = DMA_TCD_BITER_ELINKYES_BITER(lastElecNum + 1 - startingElecNum) | 		// Starting major iteration count is set to number of pads
			DMA_TCD_BITER_ELINKYES_ELINK(samplingCtrl)|														// The minor channel-to-channel linking is enabled/disabled
			DMA_TCD_BITER_ELINKYES_LINKCH(0);  														// The minor channel-to-channel link is to Ch 0
	DMA->TCD[3].SADDR = (uint32_t) &dma_ch3_saddr_config[2*(1+startingElecNum)];  										// Source Address (for next - second electrode)
	DMA->TCD[3].SOFF = DMA_TCD_SOFF_SOFF(2);    													// Source Offset 2bytes
	DMA->TCD[3].ATTR = DMA_TCD_ATTR_SMOD(0)  |              										// Source address modulo feature is disabled
			DMA_TCD_ATTR_SSIZE(1) |              													// Source data transfer size: 1: 16-bit, 2=32-bit
			DMA_TCD_ATTR_DMOD(0)  |              													// Destination address modulo feature: 0=disabled, x= x power of 2 buffer[DMOD=4->buffer of 16bytes]
			DMA_TCD_ATTR_DSIZE(1);               													// Destination data transfer size: 1: 16-bit, 2=32-bit
	DMA->TCD[3].NBYTES.MLOFFYES = DMA_TCD_NBYTES_MLOFFYES_NBYTES(4) |								// Minor Byte Transfer Count is 8-bytes
											DMA_TCD_NBYTES_MLOFFYES_DMLOE(1)|						// Minor Loop offset destination enable
											DMA_TCD_NBYTES_MLOFFYES_MLOFF(-16);						// Minor loop offset value -16 (after minor loop done go back 16 bytes) - back to &DMA->TCD[1].CITER.ELINKYES
	DMA->TCD[3].SLAST = DMA_TCD_SLAST_SLAST(-4*(lastElecNum + 1 - startingElecNum));             				// Last Source Address Adjustment is back to dma_ch3_saddr_config[2]
	DMA->TCD[3].DADDR = (uint32_t) &DMA->TCD[1].CITER.ELINKYES;										// Destination Address is TCD1.CITER
	DMA->TCD[3].DOFF = DMA_TCD_DOFF_DOFF(8);                										// Destination Address Signed Offset is 8 (Distance between tcd.ch1 CITER and tcd.ch1 BITER)
	DMA->TCD[3].DLASTSGA = DMA_TCD_DLASTSGA_DLASTSGA(-16);    										// Destination after major loop done -16 (after major loop done go back 16 bytes) -  back to &DMA->TCD[1].CITER.ELINKYES
	DMA->TCD[3].CSR = DMA_TCD_CSR_BWC(0)         |         											// BWC=0: No eDMA engine stalls - full bandwidth
			DMA_TCD_CSR_MAJORELINK(0)  |        													// The major channel-to-channel linking is disabled
			DMA_TCD_CSR_MAJORLINKCH(0) |       														// No major channel linking
			DMA_TCD_CSR_ESG(0)         |         													// The current channel TCD mode is normal
			DMA_TCD_CSR_DREQ(0)        |         													// The channel's ERQ bit will not be cleared
			DMA_TCD_CSR_INTHALF(0)     |         													// The half-point interrupt is disabled
			DMA_TCD_CSR_INTMAJOR(1)    |         													// The end-of-major loop interrupt is enabled
			DMA_TCD_CSR_START(0);                													// The channel is not explicitly started
}

/*****************************************************************************
 *
 * Function: void DMA_TCDreg_Wakeup_Event_Scan_Init(uint8_t startingElecNum, uint8_t touchElecNum)
 *
 * Description: Init DMA TCD CH0-CH5 registers for DMA scanning scenario 2
 *
 * Note I: DMA scanning scenario 2 = wakeup event scan = one wakeup electrode and one touch electrode is scanned
 *
 *****************************************************************************/
void DMA_TCDreg_Wakeup_Event_Scan_Init(uint8_t startingElecNum, uint8_t touchElecNum, uint8_t samplingCtrl)
{
#if FREQUENCY_HOPPING
	if (samplingCtrl  == 0) // sampling controlled (triggered) by LPIT
	{
		// DMA CH0 periodically triggered by LPIT CH0
		DMAMUX->CHCFG[0] |= DMAMUX_CHCFG_ENBL(1);		// enable DMAMUX
	}
	else // sampling controlled by LPIT linking
	{
		// DMA CH0 periodically linked by CH1 and last DMA CH
		DMAMUX->CHCFG[0] |= DMAMUX_CHCFG_ENBL(0);		// disable DMAMUX
	}
#endif

	if (dma_scanning_scenario != 2)
	{
	// DMA triggering by electrode's ADC config
	DMAMUX->CHCFG[1] = DMAMUX_CHCFG_ENBL(0);																						// disable DMAMUX
	DMAMUX->CHCFG[1] = DMAMUX_CHCFG_SOURCE(elecStruct[startingElecNum].ELEC_DMAMUX) | DMAMUX_CHCFG_TRIG(0) | DMAMUX_CHCFG_ENBL(1);	// adcX_COCO[0] flag triggers DMA_Ch1, enable DMAMUX

	// DMA ch0 TCD config
	DMA->TCD[0].CSR &= 0xFFFFFFFF ^ DMA_TCD_CSR_DONE_MASK;  					// Clear Channel Done flag
	DMA->TCD[0].BITER.ELINKNO = DMA_TCD_BITER_ELINKNO_BITER(1) | 				// Starting major iteration count is 1
							    DMA_TCD_BITER_ELINKNO_ELINK(0);  				// The minor channel-to-channel linking is disabled
	DMA->TCD[0].SADDR = (uint32_t) dma_ch0_stcd[startingElecNum][0].SADDR;  	// The 1st TCD Source Address
	DMA->TCD[0].SOFF = DMA_TCD_SOFF_SOFF(0);                					// Source Offset
	DMA->TCD[0].ATTR = DMA_TCD_ATTR_SMOD(0)  |              					// Source address modulo feature is disabled
					   DMA_TCD_ATTR_SSIZE(2) |              					// Source data transfer size: 1: 16-bit, 2=32-bit
					   DMA_TCD_ATTR_DMOD(0)  |              					// Destination address modulo feature: 0=disabled, x= x power of 2 buffer[DMOD=4->buffer of 16bytes]
					   DMA_TCD_ATTR_DSIZE(2);               					// Destination data transfer size: 1: 16-bit, 2=32-bit
	DMA->TCD[0].NBYTES.MLNO = DMA_TCD_NBYTES_MLNO_NBYTES(4);					// Minor Byte Transfer Count is 4-bytes
	DMA->TCD[0].SLAST = DMA_TCD_SLAST_SLAST(0);             					// Last Source Address Adjustment is 0
	DMA->TCD[0].DADDR = (uint32_t) dma_ch0_stcd[startingElecNum][0].DADDR;     	// The 1st TCD Destination Address
	DMA->TCD[0].DOFF = DMA_TCD_DOFF_DOFF(0);                					// Destination Address Signed Offset is 0
	DMA->TCD[0].DLASTSGA = (uint32_t)(&dma_ch0_stcd[startingElecNum][1]);    	// Scatter-Gather link to the next TCD
	DMA->TCD[0].CITER.ELINKNO = DMA_TCD_CITER_ELINKNO_CITER(1) | 				// Current Major Iteration Count is 1
							    DMA_TCD_CITER_ELINKNO_ELINK(0);  				// The channel-to-channel linking is disabled
	DMA->TCD[0].CSR = DMA_TCD_CSR_BWC(0)         |         						// BWC=0: No eDMA engine stalls - full bandwidth
					  DMA_TCD_CSR_MAJORELINK(0)  |        						// The major channel-to-channel linking is disabled
					  DMA_TCD_CSR_MAJORLINKCH(0) |       						// No channel major linking
					  DMA_TCD_CSR_ESG(1)         |         						// The current channel's TCD is in scatter/gather mode
					  DMA_TCD_CSR_DREQ(0)        |         						// The channel's ERQ bit is not affected
					  DMA_TCD_CSR_INTHALF(0)     |         						// The half-point interrupt is disabled
					  DMA_TCD_CSR_INTMAJOR(0)    |         						// The end-of-major loop interrupt is disabled
					  DMA_TCD_CSR_START(0);                						// The channel is not explicitly started

	// DMA channel 1 config - triggered by ADCx COCO flag, reads the ADC result data from register and stores them in buffer
	DMA->TCD[1].CSR &= 0xFFFFFFFF ^ DMA_TCD_CSR_DONE_MASK;  							// Clear Channel Done flag
	DMA->TCD[1].CITER.ELINKYES = DMA_TCD_CITER_ELINKYES_CITER_LE(elecNumberOfSamples[startingElecNum]+ NUMBER_OF_PRESAMPLES) |		// Starting major iteration count is set to required samples quantity in one cycle
				  	  	  	  	  	   DMA_TCD_CITER_ELINKYES_ELINK(samplingCtrl)	 	|			// The minor channel-to-channel linking is disabled/enabled
									   DMA_TCD_CITER_ELINKYES_LINKCH(0);				// The minor channel-to-channel link is to Ch 0
	DMA->TCD[1].BITER.ELINKYES = DMA_TCD_BITER_ELINKYES_BITER(elecNumberOfSamples[startingElecNum]+ NUMBER_OF_PRESAMPLES) | 			// Starting major iteration count is set to required samples quantity in one cycle
									DMA_TCD_BITER_ELINKYES_ELINK(samplingCtrl) 	|				// The minor channel-to-channel linking is disabled/enabled
									DMA_TCD_BITER_ELINKYES_LINKCH(0);	 				// The minor channel-to-channel link is to Ch 0
	DMA->TCD[1].SADDR = (uint32_t) &(elecStruct[startingElecNum].adcBasePtr->R[0]);  	// Source Address of the ADC result
	DMA->TCD[1].SOFF = DMA_TCD_SOFF_SOFF(0);                							// Source Offset
	DMA->TCD[1].ATTR = DMA_TCD_ATTR_SMOD(0)  |              							// Source address modulo feature is disabled
	                    DMA_TCD_ATTR_SSIZE(1) |              							// Source data transfer size: 1: 16-bit, 2=32-bit
	                    DMA_TCD_ATTR_DMOD(0)  |              							// Destination address modulo feature: 0=disabled, x= x power of 2 buffer[DMOD=4->buffer of 16bytes]
	                    DMA_TCD_ATTR_DSIZE(1);               							// Destination data transfer size: 1: 16-bit, 2=32-bit
	DMA->TCD[1].NBYTES.MLNO = DMA_TCD_NBYTES_MLNO_NBYTES(2);							// Minor Byte Transfer Count is 2-bytes
	DMA->TCD[1].SLAST = DMA_TCD_SLAST_SLAST(0);             							// Last Source Address Adjustment is 0
	DMA->TCD[1].DADDR = (uint32_t) &dma_samples_buffer[0];     							// Destination Address of the storage array
	DMA->TCD[1].DOFF = DMA_TCD_DOFF_DOFF(2);                							// Destination Address Signed Offset is 2
	DMA->TCD[1].DLASTSGA = DMA_TCD_DLASTSGA_DLASTSGA(0);    							// Destination last address adjustment is 0 (will be done manually in interrupt routine)
	DMA->TCD[1].CSR = DMA_TCD_CSR_BWC(0)         |         								// BWC=0: No eDMA engine stalls - full bandwidth
	                    DMA_TCD_CSR_MAJORELINK(1)  |        							// The major channel-to-channel linking is enabled
	                    DMA_TCD_CSR_MAJORLINKCH(2) |       								// CH2 major linking
	                    DMA_TCD_CSR_ESG(0)         |         							// The current channel TCD mode is normal
	                    DMA_TCD_CSR_DREQ(0)        |         							// The channel's ERQ bit will NOT be cleared
	                    DMA_TCD_CSR_INTHALF(0)     |         							// The half-point interrupt is disabled
	                    DMA_TCD_CSR_INTMAJOR(0)    |         							// The end-of-major loop interrupt is disabled
	                    DMA_TCD_CSR_START(0);                							// The channel is not explicitly started


	/* DMA channel 2 config - linking mode, modifies SADDR, NBYTES, DADDR and DLASTSGA of TCD.ch0 to the first SG TCD of the next electrode, both minor and major links to ch3 */
	DMA->TCD[2].CSR &= 0xFFFFFFFF ^ DMA_TCD_CSR_DONE_MASK;  										// Clear Channel Done flag
	DMA->TCD[2].CITER.ELINKYES = DMA_TCD_CITER_ELINKYES_CITER_LE(2) |    // Current iteration count is set to number of pads
			DMA_TCD_CITER_ELINKYES_ELINK(1)	 	|													// The minor channel-to-channel linking is enabled
			DMA_TCD_CITER_ELINKYES_LINKCH(3);														// The minor channel-to-channel link is to Ch 3
	DMA->TCD[2].BITER.ELINKYES = DMA_TCD_BITER_ELINKYES_BITER(2) | 		// Starting major iteration count is set to number of pads
			DMA_TCD_BITER_ELINKYES_ELINK(1)|														// The minor channel-to-channel linking is enabled
			DMA_TCD_BITER_ELINKYES_LINKCH(3);  														// The minor channel-to-channel link is to Ch 3
	DMA->TCD[2].SADDR = (uint32_t) &dma_ch2_saddr_config[4*touchElecNum];  										// Source Address (for next - second electrode)
	DMA->TCD[2].SOFF = DMA_TCD_SOFF_SOFF(4);    													// Source Offset 4bytes
	DMA->TCD[2].ATTR = DMA_TCD_ATTR_SMOD(0)  |              										// Source address modulo feature is disabled
			DMA_TCD_ATTR_SSIZE(2) |              													// Source data transfer size: 1: 16-bit, 2=32-bit
			DMA_TCD_ATTR_DMOD(0)  |              													// Destination address modulo feature: 0=disabled, x= x power of 2 buffer[DMOD=4->buffer of 16bytes]
			DMA_TCD_ATTR_DSIZE(2);               													// Destination data transfer size: 1: 16-bit, 2=32-bit
	DMA->TCD[2].NBYTES.MLOFFYES = DMA_TCD_NBYTES_MLOFFYES_NBYTES(16) |								// Minor Byte Transfer Count is 16-bytes
										DMA_TCD_NBYTES_MLOFFYES_DMLOE(1)|							// Minor Loop offset destination enable
										DMA_TCD_NBYTES_MLOFFYES_MLOFF(-32);							// Minor loop offset value -32 (after minor loop done go back 32 bytes) - back to &DMA->TCD[0].SADDR
	DMA->TCD[2].SLAST = DMA_TCD_SLAST_SLAST(-16*(2));             				// Last Source Address Adjustment is back to dma_ch2_saddr_config[4]
	DMA->TCD[2].DADDR = (uint32_t) &DMA->TCD[0].SADDR;												// Destination Address is TCD0.SADDR
	DMA->TCD[2].DOFF = DMA_TCD_DOFF_DOFF(8); 														// Destination Address Signed Offset is 8 bytes
	DMA->TCD[2].DLASTSGA = DMA_TCD_DLASTSGA_DLASTSGA(-32);    										// Destination after major loop done -32 (after major loop done go back 32 bytes) - back to &DMA->TCD[0].SADDR
	DMA->TCD[2].CSR = DMA_TCD_CSR_BWC(0)         |         											// BWC=0: No eDMA engine stalls - full bandwidth
			DMA_TCD_CSR_MAJORELINK(1)  |        													// The major channel-to-channel linking is enabled
			DMA_TCD_CSR_MAJORLINKCH(3) |       														// channel 3 will be called from ch2
			DMA_TCD_CSR_ESG(0)         |         													// The current channel TCD mode is normal
			DMA_TCD_CSR_DREQ(0)        |         													// The channel's ERQ bit will not be cleared
			DMA_TCD_CSR_INTHALF(0)     |         													// The half-point interrupt is disabled
			DMA_TCD_CSR_INTMAJOR(0)    |         													// The end-of-major loop interrupt is disabled
			DMA_TCD_CSR_START(0);                													// The channel is not explicitly started

	/* DMA channel 3 config - linking mode, modifies TCD.ch1 CITER and BITER to change the number of samples to be taken for the next scanned electrode, both minor and major links to channel 4*/
	DMA->TCD[3].CSR &= 0xFFFFFFFF ^ DMA_TCD_CSR_DONE_MASK;  										// Clear Channel Done flag
	DMA->TCD[3].CITER.ELINKYES = DMA_TCD_CITER_ELINKYES_CITER_LE(2) |   // Current iteration count is set to number of pads
			DMA_TCD_CITER_ELINKYES_ELINK(samplingCtrl)	 	|													// The minor channel-to-channel linking is enabled/disabled
			DMA_TCD_CITER_ELINKYES_LINKCH(0);														// The minor channel-to-channel link is to Ch 0
	DMA->TCD[3].BITER.ELINKYES = DMA_TCD_BITER_ELINKYES_BITER(2) | 		// Starting major iteration count is set to number of pads
			DMA_TCD_BITER_ELINKYES_ELINK(samplingCtrl)|														// The minor channel-to-channel linking is enabled/disabled
			DMA_TCD_BITER_ELINKYES_LINKCH(0);  														// The minor channel-to-channel link is to Ch 0
	DMA->TCD[3].SADDR = (uint32_t) &dma_ch3_saddr_config[2*touchElecNum];  										// Source Address (for next - second electrode)
	DMA->TCD[3].SOFF = DMA_TCD_SOFF_SOFF(2);    													// Source Offset 2bytes
	DMA->TCD[3].ATTR = DMA_TCD_ATTR_SMOD(0)  |              										// Source address modulo feature is disabled
			DMA_TCD_ATTR_SSIZE(1) |              													// Source data transfer size: 1: 16-bit, 2=32-bit
			DMA_TCD_ATTR_DMOD(0)  |              													// Destination address modulo feature: 0=disabled, x= x power of 2 buffer[DMOD=4->buffer of 16bytes]
			DMA_TCD_ATTR_DSIZE(1);               													// Destination data transfer size: 1: 16-bit, 2=32-bit
	DMA->TCD[3].NBYTES.MLOFFYES = DMA_TCD_NBYTES_MLOFFYES_NBYTES(4) |								// Minor Byte Transfer Count is 8-bytes
											DMA_TCD_NBYTES_MLOFFYES_DMLOE(1)|						// Minor Loop offset destination enable
											DMA_TCD_NBYTES_MLOFFYES_MLOFF(-16);						// Minor loop offset value -16 (after minor loop done go back 16 bytes) - back to &DMA->TCD[1].CITER.ELINKYES
	DMA->TCD[3].SLAST = DMA_TCD_SLAST_SLAST(-4*(2));             				// Last Source Address Adjustment is back to dma_ch3_saddr_config[2]
	DMA->TCD[3].DADDR = (uint32_t) &DMA->TCD[1].CITER.ELINKYES;										// Destination Address is TCD1.CITER
	DMA->TCD[3].DOFF = DMA_TCD_DOFF_DOFF(8);                										// Destination Address Signed Offset is 8 (Distance between tcd.ch1 CITER and tcd.ch1 BITER)
	DMA->TCD[3].DLASTSGA = DMA_TCD_DLASTSGA_DLASTSGA(-16);    										// Destination after major loop done -16 (after major loop done go back 16 bytes) -  back to &DMA->TCD[1].CITER.ELINKYES
	DMA->TCD[3].CSR = DMA_TCD_CSR_BWC(0)         |         											// BWC=0: No eDMA engine stalls - full bandwidth
			DMA_TCD_CSR_MAJORELINK(0)  |        													// The major channel-to-channel linking is disabled
			DMA_TCD_CSR_MAJORLINKCH(0) |       														// No major channel linking
			DMA_TCD_CSR_ESG(0)         |         													// The current channel TCD mode is normal
			DMA_TCD_CSR_DREQ(0)        |         													// The channel's ERQ bit will not be cleared
			DMA_TCD_CSR_INTHALF(0)     |         													// The half-point interrupt is disabled
			DMA_TCD_CSR_INTMAJOR(1)    |         													// The end-of-major loop interrupt is enabled
			DMA_TCD_CSR_START(0);                													// The channel is not explicitly started

	}
	else
	{
		// DMA triggering by electrode's ADC config
		DMAMUX->CHCFG[1] = DMAMUX_CHCFG_ENBL(0);																						// disable DMAMUX
		DMAMUX->CHCFG[1] = DMAMUX_CHCFG_SOURCE(elecStruct[startingElecNum].ELEC_DMAMUX) | DMAMUX_CHCFG_TRIG(0) | DMAMUX_CHCFG_ENBL(1);	// adcX_COCO[0] flag triggers DMA_Ch1, enable DMAMUX

		DMA->TCD[0].SADDR = (uint32_t) dma_ch0_stcd[startingElecNum][0].SADDR;  	// The 1st TCD Source Address
		DMA->TCD[0].DADDR = (uint32_t) dma_ch0_stcd[startingElecNum][0].DADDR;     	// The 1st TCD Destination Address
		DMA->TCD[0].DLASTSGA = (uint32_t)(&dma_ch0_stcd[startingElecNum][1]);    	// Scatter-Gather link to the next TCD

		DMA->TCD[1].CITER.ELINKYES = DMA_TCD_CITER_ELINKYES_CITER_LE(elecNumberOfSamples[startingElecNum]+ NUMBER_OF_PRESAMPLES) |		// Starting major iteration count is set to required samples quantity in one cycle
				DMA_TCD_CITER_ELINKYES_ELINK(samplingCtrl)	 	|			// The minor channel-to-channel linking is disabled/enabled
				DMA_TCD_CITER_ELINKYES_LINKCH(0);				// The minor channel-to-channel link is to Ch 0
		DMA->TCD[1].BITER.ELINKYES = DMA_TCD_BITER_ELINKYES_BITER(elecNumberOfSamples[startingElecNum]+ NUMBER_OF_PRESAMPLES) | 			// Starting major iteration count is set to required samples quantity in one cycle
				DMA_TCD_BITER_ELINKYES_ELINK(samplingCtrl) 	|				// The minor channel-to-channel linking is disabled/enabled
				DMA_TCD_BITER_ELINKYES_LINKCH(0);	 				// The minor channel-to-channel link is to Ch 0
		DMA->TCD[1].SADDR = (uint32_t) &(elecStruct[startingElecNum].adcBasePtr->R[0]);  	// Source Address of the ADC result
		DMA->TCD[1].DADDR = (uint32_t) &dma_samples_buffer[0];     							// Destination Address of the storage array

		DMA->TCD[2].SADDR = (uint32_t) &dma_ch2_saddr_config[4*touchElecNum];  				// Source Address (for next - second electrode)

		DMA->TCD[3].SADDR = (uint32_t) &dma_ch3_saddr_config[2*touchElecNum];  				// Source Address (for next - second electrode)
	}
}
/*****************************************************************************
 *
 * Function: void ElectrodesAllElecScanDMA(void);
 *
 * Description: Convert sequence of electrodes capacitance to equivalent voltage by DMA
 *
 * Note I : The electrodes in elecStruct numbered from startingElecNum to lastElecNum will be scanned
 *
 * Note I : Works even for one electrode - if(startingElecNum = lastElecNum)
 *
 *****************************************************************************/
void ElectrodesSequenceScanDMA(uint8_t startingElecNum, uint8_t lastElecNum)
{
	// Clear ADCs gain (lower noise)
	ClearADCsGain();

	// Decide sample control
	uint8_t sampleControl = SmplCtrl();

	// Load number of samples to be taken per electrode
	DMA_CH3_All_Address_Sets_Load(sampleControl);

	// Reconfigure DMA chain
	DMA_TCDreg_Sequence_Scan_Init(startingElecNum, lastElecNum, sampleControl);

	// Set DMA scanning mode
	dma_scanning_scenario = 1;

	// Start (trigger) DMA chain
	DMA_Scan_Start(sampleControl);
}

/*****************************************************************************
 *
 * Function: void ElectrodesWakeupEventScanDMA(uint8_t wakeupNum, uint8_t touchElecNum)
 *
 * Description: Convert single wakeup electrode and one touch electrode capacitance to equivalent voltage by DMA
 *
 * Note I : The scan is not sequential because the wakeupNum and touchElecNum have irregular offset in elecStruct each time the function is called
 *
 *****************************************************************************/
void ElectrodesWakeupEventScanDMA(uint8_t wakeupNum, uint8_t touchElecNum)
{
	// Clear ADCs gain (lower noise)
	ClearADCsGain();

	// Decide sample control
	uint8_t sampleControl = SmplCtrl();

	// Fisrt time doing wake-up scan?
	if (dma_scanning_scenario != 2)
	{
	// Load number of samples to be taken per electrode
	DMA_CH3_All_Address_Sets_Load(sampleControl);
	}

	// Reconfigure DMA chain
	DMA_TCDreg_Wakeup_Event_Scan_Init(wakeupNum, touchElecNum, sampleControl);

	// Set DMA scanning mode
	dma_scanning_scenario = 2;

	// Start (trigger) DMA chain
	DMA_Scan_Start(sampleControl);
}

/*****************************************************************************
 *
 * Function: void ElectrodesWakeupEventScanDMA(uint8_t wakeupNum, uint8_t touchElecNum)
 *
 * Description: Convert single electrode capacitance to equivalent voltage by DMA
 *
 *****************************************************************************/
void ElectrodesSingleElecScanDMA(uint8_t electrodeNum)
{
	// Clear ADCs gain (lower noise)
	ClearADCsGain();

	// Decide sample control
	uint8_t sampleControl = SmplCtrl();

	// Set DMA scanning mode
	dma_scanning_scenario = 3;

	// Set ADCs gain back to calibrated value
	SetADCsGain();
}

/*****************************************************************************
 *
 * Function: void DMA_ADC_Samples_Sort(void)
 *
 * Description: Sort out the samples taken by DMA to the respective electrodes buffers
 *
 *****************************************************************************/
void DMA_ADC_Samples_Sort(void)
{
	switch(dma_scanning_scenario)
	{

	case 0:
	{
		// Nothing
		break;
	}
	case 1: // Sequence elec scan
	{
		uint16_t index = 0;

		// All electrodes were scanned
		for (elecNum = dmaSeqScanFirstElec; elecNum < (dmaSeqScanLastElec + 1); elecNum++) // Run through all electrodes
		{
			for (sampleNum = index; sampleNum < (index + NUMBER_OF_PRESAMPLES + elecNumberOfSamples[elecNum]); sampleNum++)
			{
#if (NUMBER_OF_BUTTON_ELECTRODES > 0)
				if (elecStruct[elecNum].type == BUTTON)
				{
					// Transfer the samples into correct electrodes samples buffer
					buttonSamplesBuffer[elecNum - firstButtonElecNum][sampleNum-index] = dma_samples_buffer[sampleNum];
				}
#endif
#if (NUMBER_OF_SLIDER_ELECTRODES > 0)
				if (elecStruct[elecNum].type == SLIDER)
				{
					// Transfer the samples into correct electrodes samples buffer
					sliderSamplesBuffer[elecNum - firstSliderElecNum][sampleNum-index] = dma_samples_buffer[sampleNum];
				}
#endif
#if (NUMBER_OF_WAKEUP_ELECTRODES > 0)
				if (elecStruct[elecNum].type == WAKEUP)
				{
					// Transfer the samples into correct electrodes samples buffer
					wakeupSamplesBuffer[elecNum - firstWakeupElecNum][sampleNum-index] = dma_samples_buffer[sampleNum];
				}
#endif
			}
			index += (elecNumberOfSamples[elecNum]+NUMBER_OF_PRESAMPLES); // Increment index
		}
		break;
	}
	case 2: // wake-up elec scan (one wakeup electrode and one touch electrode
	{
		uint16_t index = 0;

		// Sort out the wake-up electrode data first
		for (sampleNum = index; sampleNum < (index + NUMBER_OF_PRESAMPLES + elecNumberOfSamples[firstWakeupElecNum]); sampleNum++)
		{
#if (NUMBER_OF_WAKEUP_ELECTRODES > 0)
			// Transfer the samples into correct electrodes samples buffer
			wakeupSamplesBuffer[firstWakeupElecNum-firstWakeupElecNum][sampleNum-index] = dma_samples_buffer[sampleNum];
#endif
		}

		index += (elecNumberOfSamples[firstWakeupElecNum]+NUMBER_OF_PRESAMPLES); // Increment index

		// Sort out the touch electrode data
#if (NUMBER_OF_BUTTON_ELECTRODES > 0)
		if (elecStruct[elecNumBaselineUpdate].type == BUTTON)
		{
			for (sampleNum = index; sampleNum < (index + NUMBER_OF_PRESAMPLES + elecNumberOfSamples[elecNumBaselineUpdate]); sampleNum++)
			{
				// Transfer the samples into correct electrodes samples buffer
				buttonSamplesBuffer[elecNumBaselineUpdate - firstButtonElecNum][sampleNum-index] = dma_samples_buffer[sampleNum];
			}
		}
#endif
#if (NUMBER_OF_SLIDER_ELECTRODES > 0)
		if (elecStruct[elecNumBaselineUpdate].type == SLIDER)
		{
			for (sampleNum = index; sampleNum < (index + NUMBER_OF_PRESAMPLES + elecNumberOfSamples[elecNumBaselineUpdate]); sampleNum++)
			{
				// Transfer the samples into correct electrodes samples buffer
				sliderSamplesBuffer[elecNumBaselineUpdate - firstSliderElecNum][sampleNum-index] = dma_samples_buffer[sampleNum];
			}
		}
#endif
		break;
	}
	case 3:
	{
		break;
	}
	}
}
/*****************************************************************************
 *
 * Function: void DMA3_IRQHandler(void)
 *
 * Description: DMA Channel 3 Interrupt Routine. Measurement ready flag sets
 *
 *****************************************************************************/
void DMA3_IRQHandler (void)
{
#if FREQUENCY_HOPPING
	// Regardless the sampling control, clear DMA CH 0request, disable PIT and disable DMAMUX CH0 - faster than checking SmplCtrl()
	DMA->CERQ = DMA_CERQ_CERQ(0); // Clear request for CH0
	LPIT_Disable(0); // Disable LPIT
	// Clear and disable DMAMUX CH0 to completely reset in case last LPIT trigger is pending
	DMAMUX->CHCFG[0] = DMAMUX_CHCFG_ENBL(0);
	// Set DMAMUX CH0 trigger source back
	DMAMUX->CHCFG[0] = 	DMAMUX_CHCFG_SOURCE(62) |   // always enabled as trigger comes via TRGMUX
			DMAMUX_CHCFG_TRIG(1);
#endif

	DMA->CERQ = DMA_CERQ_CERQ(1);	// Clear request for CH1

	// ISR is triggered after all samples are stored in the array
	DMA->CINT = DMA_CINT_CINT(3);  	// Clear CH3 Interrupt Request flag
	DMA->CDNE = 0;  				// Clear Channel 0 Done flag
	DMA->CDNE = 1;  				// Clear Channel 1 Done flag
	DMA->CDNE = 2;  				// Clear Channel 2 Done flag
	DMA->CDNE = 3;  				// Clear Channel 3 Done flag

	// Sort out the data
	DMA_ADC_Samples_Sort();

	// measurement ready flag
	samplesReadyFlag = 1;

#ifdef DEBUG_ELECTRODE_SENSE
	// Pin clear
	DES_GPIO->PCOR = 1 << DES_PIN;
#endif
}
