/*
 * @file     ts_cfg.h
*
* @version  1.0.0.0
*
* @date     January-2019
*
* @brief    Touch sense configuration file which gathers and includes all other cfg files into TS application source codes
*
 */

#ifndef CFG_TS_CFG_H_
#define CFG_TS_CFG_H_
/*******************************************************************************
* Includes
*******************************************************************************/
#include "S32K118.h"
#include "ts_cfg_general.h"
#include "ts_cfg_emi.h"

#if (REFERENCE_DESIGN_BOARD == S32K118_2PAD_EVB)
#include "2pad_hw.h"
#include "2pad_app.h"
#elif (REFERENCE_DESIGN_BOARD == S32K144_6PAD_KEYPAD_SLIDER)
#include "6pad_hw.h"
#include "6pad_app.h"
#elif (REFERENCE_DESIGN_BOARD == S32K144_7PAD_KEYPAD)
#include "7pad_hw.h"
#include "7pad_app.h"
#else
#error Please select valid reference design board in ts_cfg_general.h
#endif

/*******************************************************************************
* Modify: Low power mode enable
*         If low power mode is enabled (LPM_ENABLE), the application debug and
*         FreeMASTER data visualization are disabled.
*         If low power mode is disabled (LPM_DISABLE), the application debug and
*         FreeMASTER data visualization are enabled.
******************************************************************************/
#define LOW_POWER_MODE   LPM_DISABLE

/*******************************************************************************
* Modify: Assembly optimization for CPU to avoid dependency on (-O3) compilation optimization (ON,OFF)
* 		  Modify if lower than -O3 compiler optimization is used (for example -O0, -O1, -O2)
* Note:	  This assembly optimization for CPU ensures (determines) a constant minimal number of instructions between
* 		  the start of ADC conversion command and switching the pins to input state command (charge redistribution).
* 		  At all levels of compiler optimization (-O0, -O1, -O2. -O3) this assembly optimization for CPU prevents
* 		  the compiler to put any unnecessary instructions in this crucial period between the two commands
* 		  and thus shift the ADC conversion moment before/long time after the equivalent voltage is set.
* WARNING:The -O3 compiler optimization itself has almost the same efficiency as this assembly optimization,
* 		  however sometimes even on -O3 there might be a delay between the ADC conversion command and switching the pins to input state command
* 		  Therefore, the best solution is to keep -O3 compiler optimization and put the assembly optimization ON (1)
******************************************************************************/
#define TS_ASM_OPTIMIZE     ON

/*******************************************************************************
* Modify: Keypad backlight PWM dutycycle from 0 to 100
******************************************************************************/
#define BACKLIGHT_PWM_DUTYCYCLE    30

/*******************************************************************************
* Modify: Define ADC software trigger sample time for all electrodes
******************************************************************************/
#if TS_ELECTRODE_DATA_ACQUISITION == CPU_ELECTRODE_DATA_ACQUISITION
	// CPU used for electrodes data gathering
	// Value from 14 to 255
	#define ADC_SWTRIGGER_SAMPLE_TIME 14
#else
	// DMA used for electrodes data gathering
	// Value from 50 to 255
	#define ADC_SWTRIGGER_SAMPLE_TIME 60
#endif

/*******************************************************************************
* Modify: If needed, configure debug pins. Uncomment required option.
******************************************************************************/
// Debug electrode sensing cycle
//#define DEBUG_ELECTRODE_SENSE
// Debug application algorithm
//#define DEBUG_ALGORITHM

// Defined?
#ifdef DEBUG_ELECTRODE_SENSE
	// Assign PORT
	#define DES_PORT  PORTE
	// Assign GPIO
	#define DES_GPIO  PTE
	// Assign pin
	#define DES_PIN   10

#endif

// Defined?
#ifdef DEBUG_ALGORITHM
	// Assign PORT
	#define DA_PORT  PORTE
	// Assign GPIO
	#define DA_GPIO  PTE
	// Assign pin
	#define DA_PIN   11
#endif

/*******************************************************************************
* Modify: Define clock mode: either CLOCKMODE_FIRC_48MHZ or CLOCKMODE_PLL_80MHZ
******************************************************************************/
//#define CLOCKMODE_PLL_80MHZ // CAN ONLY BE USED AT S32K14* family
#define CLOCKMODE_FIRC_48MHZ
/*******************************************************************************
* Modify: Electrodes sense period for Reaction time
* Note: Response time stands for period of electrodes sensing (scan period)
*       Reaction time stands for period between electrode touch and application report "electrode touched"
******************************************************************************/
//Define electrodes sensing (scanning) cycle in milliseconds [ms].
// Value from 2 to 65535.
#define ELECTRODES_SENSE_PERIOD   30 // 30 ms ideal for keeping Low Power < 100 uA and Reaction time < 100 ms

#if (NUMBER_OF_WAKEUP_ELECTRODES > 0 && WAKE_UP_ELECTRODE_PROXIMITY == YES)
// Value from 100 to 65535.
#define ELECTRODES_SENSE_PERIOD_PROXIMITY   100
#endif

#endif /* CFG_TS_CFG_H_ */
