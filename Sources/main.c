/*
 * Copyright 2018 NXP
 * All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */
/* ###################################################################
**     Filename    : main.c
**     Project     : lin_slave_s32k118
**     Processor   : S32K118_64
**     Version     : Driver 01.00
**     Compiler    : GNU C Compiler
**     Date/Time   : 2018-03-03, 10:58, # CodeGen: 0
**     Abstract    :
**         Main module.
**         This module contains user's application code.
**     Settings    :
**     Contents    :
**         No public methods
**
** ###################################################################*/
/*!
** @file main.c
** @version 01.00
** @brief
**         Main module.
**         This module contains user's application code.
*/
/*!
**  @addtogroup main_module main module documentation
**  @{
*/
/* MODULE main */


/* Including needed modules to compile this module/procedure */
#include "Cpu.h"

#if CPU_INIT_CONFIG
  #include "Init_Config.h"
#endif
volatile int exit_code = 0;
/* User includes (#include below this line is not maintained by Processor Expert) */

#include "sbc.h"
#include "pcc.h"
#include "main.h"
#include "power_mode.h"
#include "adc.h"
#include "ts_cfg.h"
#include "lpuart.h"
#include "ets.h"
#include "freemaster.h"
#include "touch_buttons.h"
#include "lptmr.h"
#include "scg.h"
#include "motor.h"
uint8_t SPIReadBuf;
/* This example is setup to work by default with EVB. To use it with other boards
   please comment the following line
*/
#define EVB


    #define USE_LIN_XCVR              (1)
    #define LIN_XCVR_ENABLE_PIN       (7UL)
    #define LIN_XCVR_ENABLE_MASK      (0x1u << LIN_XCVR_ENABLE_PIN)
    #define LIN_XCVR_ENABLE_GPIO_PORT (PTA)



/* (CLK (MHz)* timer period (us) / Prescaler) */
#define TIMER_COMPARE_VAL             (uint16_t)(2000U)
#define TIMER_TICKS_1US               (uint16_t)(4U)

#define MOTOR_SELECTION_INCREASE      (1u)
#define MOTOR_SELECTION_DECREASE      (2u)
#define MOTOR_SELECTION_STOP          (3u)

#define MOTOR1_OVER_TEMP              (200u)
#define MOTOR1_MAX_TEMP               (100u)
#define MOTOR1_MIN_TEMP               (30u)

uint16_t timerOverflowInterruptCount = 0U;
volatile uint16_t capturedValue = 0U;
volatile bool linEnabled = false;
l_u8 Motor1_Selection = 0U;
l_u8 Motor1_temp = 30U;

// ADC0 and ADC1 gain register results after calibration
int16_t calibrationGainADC0;
uint8_t scanningIdleFlag;
// Low power mode
extern uint8_t  lowPowerModeCtrl, lowPowerModeEnable;
extern int16_t   numberOfElectrodeSensingCyclesPerSample;
extern uint8_t   electrodeTouchQualified[NUMBER_OF_ELECTRODES];
extern uint8_t   elecNum;
extern uint8_t   sliderElectrodeTouchQualified;
extern uint8_t samplesReadyFlag, proximityDetectedFlag, electrodesSelfTrimDoneFlag;
extern uint8_t  frequencyID;
uint8_t one=0;





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

/*!
 * @brief LPTMR Interrupt Service Routine
 * The ISR will call LIN timeout service every 500us
 * and will increase or decrease motor temperature every
 * 0.1s, depending on the motor speed trend.
 */
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
    if (++interruptCount >= 2u)
    {
        interruptCount = 0u;
        //ElectrodesScanLPTMRHandlerRoutine();
    }
    /* Increment overflow count */
    timerOverflowInterruptCount++;
    /* Clear compare flag */
    LPTMR_DRV_ClearCompareFlag(INST_LPTMR1);
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

/*!
 * @brief Interrupt Service Routine for buttons and LPUART Rx pin port event
 * With button depending on which buttons were pressed LIN scheme will be
 * set to sleep mode or normal mode.
 * With LPUART Px pin will capture the time stamp of the Rx pin level
 * transition and will run Auto Baudrate function if required.
 */

void PORT_IRQHandler(void)
{
    static bool autoBaudComplete = false;

    /* Check if the interrupt is triggered by the LPUART Rx Pin */
    if(PINS_DRV_GetPortIntFlag(PORTC) & (1UL << 6))
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
        }

        /* Clear PORT interrupt register */
        PINS_DRV_ClearPortIntFlagCmd(PORTC);
    }


}


void I2C_Init(void)
{
    PCC->PCCn[PCC_PORTA_INDEX] |= PCC_PCCn_CGC_MASK;
    PCC->PCCn[PCC_LPI2C0_INDEX] |= PCC_PCCn_CGC_MASK | PCC_PCCn_PCS(3);
	PORTA->PCR[2] = PORT_PCR_MUX(3);
    PORTA->PCR[3] = PORT_PCR_MUX(3);

    LPI2C0->MCFGR1 = LPI2C_MCFGR1_PRESCALE(2) | LPI2C_MCFGR1_IGNACK(0);

    LPI2C0->MCCR0 = LPI2C_MCCR0_CLKLO(18)
                    | LPI2C_MCCR0_CLKHI(6)
                    | LPI2C_MCCR0_SETHOLD(6)
                    | LPI2C_MCCR0_DATAVD(3);

    LPI2C0->MFCR = LPI2C_MFCR_TXWATER(0) | LPI2C_MFCR_RXWATER(3);

    LPI2C0->MCR |= LPI2C_MCR_MEN_MASK | LPI2C_MCR_DBGEN_MASK;

}

void delay(uint32_t time)
{
    while(time--);
}

uint8_t ReadReg(uint8_t reg)
{
	uint8_t regval=0;
    LPI2C0->MTDR = LPI2C_MTDR_CMD(4) | LPI2C_MTDR_DATA((0X46<<1)|0);
    delay(1000);
    LPI2C0->MTDR = LPI2C_MTDR_CMD(0) | LPI2C_MTDR_DATA(reg);
    delay(1000);
    LPI2C0->MTDR = LPI2C_MTDR_CMD(4) | LPI2C_MTDR_DATA((0X46<<1)|1);
    delay(1000);
    LPI2C0->MTDR = LPI2C_MTDR_CMD(1) | LPI2C_MTDR_DATA(0x00);
    delay(1000);
    LPI2C0->MTDR = LPI2C_MTDR_CMD(2) | LPI2C_MTDR_DATA(0x00);
    delay(1000);
    regval = LPI2C0->MRDR;

    return regval;
}
void WriteReg(uint8_t reg,uint8_t data)
{
    LPI2C0->MTDR = LPI2C_MTDR_CMD(4) | LPI2C_MTDR_DATA((0X46<<1)|0);
    delay(1000);
    LPI2C0->MTDR = LPI2C_MTDR_CMD(0) | LPI2C_MTDR_DATA(reg);
    delay(1000);
    LPI2C0->MTDR = LPI2C_MTDR_CMD(0) | LPI2C_MTDR_DATA(data);
    delay(1000);
    LPI2C0->MTDR = LPI2C_MTDR_CMD(2) | LPI2C_MTDR_DATA(0x00);
    delay(1000);
}

void gotosleep(void)
{
	LIN_STBN(1);
	LIN_EN(0);
}

/*!
 * @brief Main LIN slave task
 * This function will initialize the LIN interface and manipulate
 * the received data from the master.
 * Depending on the received data, the motor speed will be increased/decreased
 * or the motor will be stopped.
 */
void lin_slave_task(void)
{
	uint8_t key,buf[3],slider,temp;
	uint16_t press;
    /* Initialize LIN network interface */
    l_sys_init();
    l_ifc_init(LI0);
    linEnabled = true;
    /* Infinite loop */
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

				// Recorder time base defined by ELECTRODES_SENSE_PERIOD
				FMSTR_Recorder();
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

			// Enable FreeMASTER poll, when low power mode disabled
			if (lowPowerModeEnable == NO)
			{
				FMSTR_Poll();
			}
			if (LIN_NODE_STATE_SLEEP_MODE == lin_lld_get_state(LI0))
			{
				gotosleep();
			}
			if(one == 1)
			{

				one = 0;
				//LPUART0_transmit_char(Button_RGBLED_Ctrl());
				key = Button_RGBLED_Ctrl();
				slider = slider_Detect(buf);
				//read press value
				press = 0;
				temp = ReadReg(0x03);
				press = temp;
				press<<=4;
				temp = ReadReg(0x04);
				temp >>= 4;
				press |= temp;
				if(press > 2047)
					press = 0;
				else if(press > 255)
					press = 255;

				
				l_u8_wr_LI0_temp5((uint8_t)press)
				l_u8_wr_LI0_temp1(buf[0]);
				l_u8_wr_LI0_temp2(buf[1]);
				l_u8_wr_LI0_temp3(buf[2]);
				l_u8_wr_LI0_temp4(slider);
				if(key & 0x01)//1
				{
					l_u8_wr_LI0_MFS_Speech_switch_signal(1);
				}
				else
				{
					l_u8_wr_LI0_MFS_Speech_switch_signal(0);
				}
				if(key & 0x02)//2
				{
					l_u8_wr_LI0_MFS_RES_switch_signal(1);
				}
				else
				{
					l_u8_wr_LI0_MFS_RES_switch_signal(0);
				}
				if(key & 0x04)//3
				{
					l_u8_wr_LI0_MFS_OK_switch_signal(1);
				}
				else
				{
					l_u8_wr_LI0_MFS_OK_switch_signal(0);
				}
				if(key & 0x08)//4
				{
					l_u8_wr_LI0_MFS_SET_switch_signal(1);
				}
				else
				{
					l_u8_wr_LI0_MFS_SET_switch_signal(0);
				}
				if(key & 0x10)//5
				{
					l_u8_wr_LI0_MFS_LIM_switch_signal(1);
				}
				else
				{
					l_u8_wr_LI0_MFS_LIM_switch_signal(0);
				}
				if(key & 0x20)//6
				{
					//l_u8_wr_LI0_MFS_Mode_switch_signal(1);
				}
				else
				{
					//l_u8_wr_LI0_MFS_Mode_switch_signal(0);
				}
				if(key & 0x40)//7
				{
					l_u8_wr_LI0_MFS_ACC_switch_signal(1);
				}
				else
				{
					l_u8_wr_LI0_MFS_ACC_switch_signal(0);
				}
				if(key & 0x80)//8
				{
					l_u8_wr_LI0_MFS_DIST_switch_signal(1);
				}
				else
				{
					l_u8_wr_LI0_MFS_DIST_switch_signal(0);
				}
			}


		}

	}
    for (; ; )
    {


        /* Check node state */
        if (LIN_NODE_STATE_SLEEP_MODE == lin_lld_get_state(LI0))
        {
            /* Turn off all LEDs */
            //PINS_DRV_WritePin(LED0_GPIO_PORT, PORT_LED0_INDEX, 0U);
            //PINS_DRV_WritePin(LED1_GPIO_PORT, PORT_LED1_INDEX, 0U);
            //PINS_DRV_WritePin(LED2_GPIO_PORT, PORT_LED2_INDEX, 0U);
        }
    }
}



/*!
  \brief The main function for the project.
  \details The startup initialization sequence is the following:
 * - startup asm routine
 * - main()
*/
int main(void)
{
  /* Write your local variable definition here */

  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  #ifdef PEX_RTOS_INIT
    PEX_RTOS_INIT();                   /* Initialization of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of Processor Expert internal initialization.                    ***/

    /* Initialize and configure clocks
     *  -   Setup system clocks, dividers
     *  -   Configure LPUART clock, GPIO clock
     *  -   see clock manager component for more details
     */
    //CLOCK_SYS_Init(g_clockManConfigsArr, CLOCK_MANAGER_CONFIG_CNT, g_clockManCallbacksArr, CLOCK_MANAGER_CALLBACK_CNT);
    //CLOCK_SYS_UpdateConfiguration(0U, CLOCK_MANAGER_POLICY_AGREEMENT);
    SCG_Init(RUN_FIRC);
    PCC_Init(RUN_FIRC);
    // Wake-up timer init
	LPTMR0_Init(LPTMR_ELEC_CAL);
    /* Initialize LPTMR */

	//LPTMR_DRV_Init(INST_LPTMR1, &lpTmr1_config0, false);
	//INT_SYS_InstallHandler(LPTMR0_IRQn, LPTMR_ISR, (isr_t *)NULL);
	//INT_SYS_EnableIRQ(LPTMR0_IRQn);
	//LPTMR_DRV_StartCounter(INST_LPTMR1);
    // ADC0 calibration init
	calibrationGainADC0 = ADC0_Calibration();
	// ADC0 init (sample time, samples number to average)
	ADC0_Init(ADC_SWTRIGGER_SAMPLE_TIME, 0, RUN_FIRC);
	// Init electrodes touch sense
	ElectrodesTouchSenseInit();

	// Low power mode control disabled
	lowPowerModeCtrl = OFF;
	//LPUART0_Init(RUN_FIRC);
	// Init FreeMASTER, poll driven
	//FMSTR_Init();
	// Init NVIC
	NVIC_Init();
	MotorInit();
	FlexIO_UART_txchar(0x55);


    SPI_GPIO_Init();
    SPIReadBuf = 0;
	SPI_Delay(100000);
	SPIReadBuf = SPI_WriteByte(0x11,0x10);
	SPIReadBuf = SPI_ReadByte(0x10);
	FlexIO_UART_txchar(0x10);
	FlexIO_UART_txchar(SPIReadBuf);
	SPIReadBuf = SPI_ReadByte(0x11);
	FlexIO_UART_txchar(0x11);
	FlexIO_UART_txchar(SPIReadBuf);
	SPIReadBuf = SPI_ReadByte(0x12);
	FlexIO_UART_txchar(0x12);
	FlexIO_UART_txchar(SPIReadBuf);
	SPIReadBuf = SPI_ReadByte(0x13);
	FlexIO_UART_txchar(0x13);
	FlexIO_UART_txchar(SPIReadBuf);
	SPIReadBuf = SPI_ReadByte(0x14);
	FlexIO_UART_txchar(0x14);
	FlexIO_UART_txchar(SPIReadBuf);
	SPIReadBuf = SPI_ReadByte(0x15);
	FlexIO_UART_txchar(0x15);
	FlexIO_UART_txchar(SPIReadBuf);
	SPIReadBuf = SPI_ReadByte(0x30);
	FlexIO_UART_txchar(0x30);
	FlexIO_UART_txchar(SPIReadBuf);
	SPIReadBuf = SPI_ReadByte(0x31);
	FlexIO_UART_txchar(0x31);
	FlexIO_UART_txchar(SPIReadBuf);
	SPIReadBuf = SPI_WriteByte(0x30,0xED);
	SPIReadBuf = SPI_ReadByte(0x30);
	LIN_STBN(0);
	LIN_EN(1);
    /* Initialize pins
     *  -   Init LPUART and GPIO pins
     *  -   See PinSettings component for more info
     */
    PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);
    LIN_EN(1);

#if USE_LIN_XCVR
    /* Set LIN transceiver sleep pin direction */
    PINS_DRV_SetPinsDirection(LIN_XCVR_ENABLE_GPIO_PORT, LIN_XCVR_ENABLE_MASK);
    /* Wake up LIN transceiver */
    PINS_DRV_SetPins(LIN_XCVR_ENABLE_GPIO_PORT, LIN_XCVR_ENABLE_MASK);
#endif

    /* For auto baud rate, we need to activate the interrupt for the LPUART Rx Pin,
     * which needs to be triggered on both edges of the signal.
     */
    PINS_DRV_SetPinIntSel(PORTC, 6UL, PORT_INT_EITHER_EDGE);
    /* Enable PORT interrupt */
    INT_SYS_EnableIRQ(PORT_IRQn);
	I2C_Init();


    /* Start LIN slave task */
    lin_slave_task();

  /*** Don't write any code pass this line, or it will be deleted during code generation. ***/
  /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of RTOS startup code.  ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;) {
    if(exit_code != 0) {
      break;
    }
  }
  return exit_code;
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/

} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/


/* END main */
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.1 [05.21]
**     for the Freescale S32K series of microcontrollers.
**
** ###################################################################
*/
