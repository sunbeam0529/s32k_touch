/****************************************************************************//*!
*
* @file     main.c
*
* @version  1.0.0.0
*
* @date     September-2017
*
* @brief    Touch sense main for S32K144. Compiler optimization -O3.
*
*******************************************************************************/

/*******************************************************************************
 * This version is compatible with S32DS 2018R1
 *******************************************************************************/
/*******************************************************************************
 * Includes
 *******************************************************************************/
#include "S32K118.h"
#include "main.h"
#include "scg.h"
#include "lptmr.h"
#include "adc.h"
#include "pcc.h"
#include "lpuart.h"
#include "freemaster.h"
#include "ets.h"
#include "power_mode.h"
#include "ts_cfg.h"
#include "gpio.h"
#include "gpio_inline_fcn2.h"
#include "flextimer.h"
#include "slider.h"
#include "touch_buttons.h"
#include "wake_up.h"
#include "dma.h"
#include "lpit.h"
#include "lin_common_api.h"
#include "lptmr_driver.h"
#include "lptmr1.h"
/*******************************************************************************
 * Variables
 ******************************************************************************/
extern int16_t   numberOfElectrodeSensingCyclesPerSample;
extern uint8_t   electrodeTouchQualified[NUMBER_OF_ELECTRODES];
extern uint8_t   elecNum;
extern uint8_t   sliderElectrodeTouchQualified;
extern uint8_t samplesReadyFlag, proximityDetectedFlag, electrodesSelfTrimDoneFlag;

#define TIMER_COMPARE_VAL             (uint16_t)(2000U)
#define TIMER_TICKS_1US               (uint16_t)(4U)

uint8_t SPIReadBuf;
uint8_t scanningIdleFlag;

uint8_t  clockMode;

// Low power mode
extern uint8_t  lowPowerModeCtrl, lowPowerModeEnable;

// Guard pin structure
extern tGuardStruct guardStruct[1];

// RGB LED display
uint8_t  electrodeTouchQualifiedDisplay;
uint8_t  sliderElectrodeTouchQualifiedDisplay;

// Backlight PWM duty cycle from 0 to 100
extern uint16_t  backlightPWMDutyCycle;

// Freq hopping
extern uint8_t  frequencyID;

// ADC0 and ADC1 gain register results after calibration
int16_t calibrationGainADC0;
int16_t calibrationGainADC1;


volatile bool linEnabled = false;
// Datalog synchronization, PTD1, 4k7 pull-up
//uint8_t datalogON;
uint8_t one=0;
volatile uint16_t capturedValue = 0U;
uint16_t timerOverflowInterruptCount = 0U;
/*****************************************************************************
 *
 * Function: void NVIC_Init(void)
 *
 * Description: Init NVIC
 *
 * Note : IMPORTANT - DMA IRQ MUST BE ENABLED BEFORE LPTMR IRQ!!!
 *****************************************************************************/
void NVIC_Init(void)
{
	NVIC_SET_IRQ_PRIORITY_LEVEL(DMA3_IRQn,1);// set higher priority for dma
	NVIC_SET_IRQ_PRIORITY_LEVEL(LPTMR0_IRQn,2);// set lower priority for lptmr

	NVIC_IRQ_CLEARPENDING(DMA3_IRQn); // clear pending DMA ch3 Interrupts
	NVIC_IRQ_CLEARPENDING(LPTMR0_IRQn); // clear pending LPTMR0 interrupts in NVIC

	NVIC_IRQ_ENABLE(DMA3_IRQn); // Enable DMA ch3 Interrupts
	NVIC_IRQ_ENABLE(LPTMR0_IRQn); // Enable LPTMR0 interrupts in NVIC
}

/*!
 * @brief Callback function to get time interval in nano seconds
 * @param[out] ns - number of nanoseconds passed since the last call of the function
 * @return dummy value
 */
uint32_t timerGetTimeIntervalCallback0(uint32_t *ns)
{
    static uint32_t previousCountValue = 0UL;
    uint32_t counterValue;
    counterValue = capturedValue;
    *ns = ((uint32_t)(counterValue + timerOverflowInterruptCount * TIMER_COMPARE_VAL - previousCountValue)) * 1000UL / TIMER_TICKS_1US;
    timerOverflowInterruptCount = 0UL;
    previousCountValue = counterValue;
    return 0UL;
}

/*****************************************************************************
 *
 * Function: void RGBLED_Ctrl(void)
 *
 * Description: Display touch event
 *
 *****************************************************************************/
void RGBLED_Ctrl(void)
{
	uint8_t buttonsTouched = 0xFF;
	uint8_t sliderTouched = 0;

#if NUMBER_OF_BUTTON_ELECTRODES
	// Display button electrodes touch event
	buttonsTouched = Button_RGBLED_Ctrl();
#endif


	if (buttonsTouched == 0xFF && sliderTouched == 0)
	{
		// RGB LED turn OFF
		// Turn OFF RED LED
		LedRedOFF();
		// Turn OFF GREEN LED
		LedGreenOFF();
		// Turn OFF BLUE LED
		LedBlueOFF();
	}

}

void PORT_IRQHandler(void)
{
    static bool autoBaudComplete = false;

    /* Check if the interrupt is triggered by the LPUART Rx Pin */
    if((PORTC->ISFR) & (1UL << 6))
    {
        /* Capture transition time stamp */
        capturedValue = LPTMR_DRV_GetCounterValueByCount(INST_LPTMR1);
        /* If the auto baud process is not completed and lin is enabled, call
         * LIN_DRV_AutoBaudCapture
         */

        if((!autoBaudComplete) && (linEnabled))
        {
            if (LIN_DRV_AutoBaudCapture(INST_LIN1) == STATUS_SUCCESS)
            {
                autoBaudComplete = true;
            }
            autoBaudComplete = true;
        }

        /* Clear PORT interrupt register */
        PORTC->ISFR = PORT_ISFR_ISF_MASK;
    }


}

void LPUART0_transmit_char(char send) {    /* Function to Transmit single Char */
	while((LPUART0->STAT & LPUART_STAT_TDRE_MASK)>>LPUART_STAT_TDRE_SHIFT==0);
	/* Wait for transmit buffer to be empty */
	LPUART0->DATA=send;              /* Send data */
}

void LPUART0_transmit_string(char data_string[])  {  /* Function to Transmit whole string */
	uint32_t i=0;
	while(data_string[i] != '\0')  {           /* Send chars one at a time */
		LPUART0_transmit_char(data_string[i]);
		i++;
	}
}

void LPTMR_ISR(void)
{
    /* Static variable, used to count if the timeout has passed to
     * provide the LIN scheme tick.
     */
    static uint32_t interruptCount = 0UL;
    /* Timer Interrupt Handler */
    lin_lld_timeout_service(LI0);

    /* Update motor temperature every 0.1s, based on the
     * motor speed trend.
     */
    if (++interruptCount >= 200u)
    {
        interruptCount = 0u;

        /* Increase temp */
        if (Motor1_Selection == MOTOR_SELECTION_INCREASE)
        {
            Motor1_temp++;
        }
        /* Decrease temp */
        else if (Motor1_Selection == MOTOR_SELECTION_DECREASE)
        {
            Motor1_temp--;
        }
    }

    /* Increment overflow count */
    timerOverflowInterruptCount++;
    /* Clear compare flag */
    LPTMR_DRV_ClearCompareFlag(INST_LPTMR1);
}


/*****************************************************************************
 *
 * Function: void main(void)
 *
 * Description: Main routine
 *
 *****************************************************************************/
void main(void)
{ 
	uint32_t regValue;
	// Disable interrupts
	DisableInterrupts;

	// Load clock mode
	clockMode = CLOCKMODE;

	// System clock generator init
	SCG_Init(clockMode);

	// Peripheral clock enable
	PCC_Init(clockMode);

	// Flash and enable I/D cache and write buffer
	LMEM->PCCCR = LMEM_PCCCR_GO_MASK | LMEM_PCCCR_INVW1_MASK | \
			LMEM_PCCCR_INVW0_MASK | LMEM_PCCCR_ENCACHE_MASK;

    // Wake-up timer init
	LPTMR0_Init(LPTMR_ELEC_CAL);

#if FREQUENCY_HOPPING
#if (TS_ELECTRODE_DATA_ACQUISITION == DMA_ELECTRODE_DATA_ACQUISITION)
	 TRGMUX->TRGMUXn[TRGMUX_DMAMUX0_INDEX] |= 17; 			// set LPIT ch0 as trigger source for DMAch0
#endif
	// Init hopping frequencies
	FrequencyHoppingInit(clockMode);
	// (Pre)Init LPIT
	LPIT_Init();
#endif

	// ADC0 calibration init
	calibrationGainADC0 = ADC0_Calibration();

	// ADC0 init (sample time, samples number to average)
	ADC0_Init(ADC_SWTRIGGER_SAMPLE_TIME, 0, clockMode);

	// FTM2 init
	//FTM2_Init();

	// GPIO init
	GPIO_Init();

	// Init electrodes touch sense
	ElectrodesTouchSenseInit();

#if (TS_ELECTRODE_DATA_ACQUISITION == DMA_ELECTRODE_DATA_ACQUISITION)
    /* Configure the SRAM_L and SRAM_U arbitration scheme to fixed priority (backdoor has highest, processor has lowest) */
	/*Crossbar Round-robin Arbitration Enable*/
   // MCM->CPCR = MCM_CPCR_SRAMLAP(3) | MCM_CPCR_SRAMUAP(3) | MCM_CPCR_CBRR(1) ;
	 MCM->CPCR = MCM_CPCR_CBRR(1);
    // DMA Module init
	DMA_Init();
#endif

	// Low power mode control disabled
	lowPowerModeCtrl = OFF;

#if(LOW_POWER_MODE == LPM_DISABLE)
	// Init FreeMASTER UART0
	LPUART0_Init(clockMode);

	// Init FreeMASTER, poll driven
	FMSTR_Init();

	// Disable low power mode
	lowPowerModeEnable = NO;
#else
	// Enable low power mode
	lowPowerModeEnable = YES;
#endif

	// Init backlight PWM dutycycle
	backlightPWMDutyCycle = BACKLIGHT_PWM_DUTYCYCLE;

	// Init NVIC
	NVIC_Init();

	// Enable interrupts
	EnableInterrupts;

	// Loop forever
	SPIReadBuf = SPI_WriteByte(0x30,0x55);
	SPIReadBuf = SPI_ReadByte(0x10);
	SPIReadBuf = SPI_ReadByte(0x11);
	SPIReadBuf = SPI_ReadByte(0x12);
	SPIReadBuf = SPI_ReadByte(0x13);
	SPIReadBuf = SPI_ReadByte(0x14);
	SPIReadBuf = SPI_ReadByte(0x15);
	SPIReadBuf = SPI_ReadByte(0x30);
	SPIReadBuf = SPI_ReadByte(0x31);
	SPIReadBuf = SPI_WriteByte(0x30,0x02);
	SPIReadBuf = SPI_ReadByte(0x30);
	SPIReadBuf = SPI_WriteByte(0x30,0x37);
	SPIReadBuf = SPI_ReadByte(0x30);
	LIN_STBN(0);
	LIN_EN(1);
	PORTC->PCR[6] = PORT_PCR_MUX(2);
	PORTC->PCR[7] = PORT_PCR_MUX(2);

	/* For auto baud rate, we need to activate the interrupt for the LPUART Rx Pin,
	 * which needs to be triggered on both edges of the signal.*/
	PINS_DRV_SetPinIntSel(PORTC, 6UL, PORT_INT_EITHER_EDGE);
	/*regValue = PORTC->PCR[6];
	regValue &= ~(PORT_PCR_IRQC_MASK);
	regValue |= PORT_PCR_IRQC(0x0B);
	PORTC->PCR[6] = regValue;*/

	/* Enable PORT interrupt */
	NVIC_IRQ_ENABLE(PORT_IRQn);

	/* Initialize LIN network interface */
	l_sys_init();
	l_ifc_init(LI0);
	linEnabled = true;

	while(1)
	{
		// Electrodes samples set scanned and ready?
		if (samplesReadyFlag == 1) // Yes, do touch digital processing and touch detecting routine
		{

			// Electrodes self-trim done?
			if(electrodesSelfTrimDoneFlag == 1)
			{
				one = 1;
				// Process all touch electrodes raw data
				ElectrodesTouchElecProcess();
				// Check for virtual wake up event
				ElectrodesVirtualWakeupEventCtrl();
				// Data have been processed, continue
				samplesReadyFlag = 0;
				// Backlighting of the keyboard and also low power ctrl flag based on detected proximity
				ElectrodesBacklightAndLowPowerCtrl();

#if(LOW_POWER_MODE == LPM_DISABLE)
				// Recorder time base defined by ELECTRODES_SENSE_PERIOD
				FMSTR_Recorder();
#endif
			}
			else
			{
				// Self-trim all electrodes after power-up / reset
				ElectrodesSelfTrimProcess();
				// Data have been processed, continue
				samplesReadyFlag = 0;
				// Self-trimming done?
				if (electrodesSelfTrimDoneFlag == 1)
				{
					// Reset frequencyID
					frequencyID = 0;
				}
			}

			// Are we back on the base scanning frequency?
			if (frequencyID == 0)
			{
				// Set scanning (DMA) Idle flag
				scanningIdleFlag = 1;
			}
		}
		else // No electrodes data set ready, do whatever else you want
		{
			if (proximityDetectedFlag == 1)
			{
				// Control RGB LED - touch events display
				RGBLED_Ctrl();
			}

			// Enable FreeMASTER poll, when low power mode disabled
			if (lowPowerModeEnable == NO)
			{
				FMSTR_Poll();
			}
			if(one == 1)
			{
				one = 0;
				LPUART0_transmit_char(Button_RGBLED_Ctrl());
			}


		}

	}
}
