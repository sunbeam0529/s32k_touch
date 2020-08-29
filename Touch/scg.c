/****************************************************************************//*!
*
* @file     scg.c
*
* @version  1.0.0.0
*
* @date     September-2017
*
* @brief    SCG routines
*
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include "S32K118.h"
#include "scg.h"
#include "main.h"
#include "power_mode.h"

/*****************************************************************************
*
* Function: void SCG_Init(uint8_t clkMode)
*
* Description: Init SCG
*
*****************************************************************************/
void SCG_Init(uint8_t clkMode)
{
	// Configure SCG based on selected clock mode
	switch(clkMode)
	{
		case(RUN_FIRC):
		{
	    	/* FIRC mode after reset */
	    	/* FIRC 48MHz, core clock 48MHz, system clock 48MHz, bus clock 48MHz, Flash clock 24MHz */
			/* FIRC Configuration 48MHz */
			SCG->FIRCDIV = SCG_FIRCDIV_FIRCDIV1(0b01)	   /*FIRC DIV1=1  48MHz*/
						  |SCG_FIRCDIV_FIRCDIV2(0b01);	   /*FIRC DIV2=1  48MHz*/

			SCG->FIRCCFG =SCG_FIRCCFG_RANGE(0b00);	/* Fast IRC trimmed 48MHz*/
			
			SCG->SIRCCFG = SCG_SIRCCFG_RANGE(0);//SIRC -> 2MHz
    		SCG->SIRCDIV = SCG_SIRCDIV_SIRCDIV1(1) | SCG_SIRCDIV_SIRCDIV2(2);//SIRC2 = 2MHz / 2 = 1 MHz

			while(SCG->FIRCCSR & SCG_FIRCCSR_LK_MASK); /*Is PLL control and status register locked?*/

			SCG->FIRCCSR = SCG_FIRCCSR_FIRCEN_MASK;  /*Enable FIRC*/

			while(!(SCG->FIRCCSR & SCG_FIRCCSR_FIRCVLD_MASK)); /*Check that FIRC clock is valid*/

			/* RUN Clock Configuration */
			SCG->RCCR=SCG_RCCR_SCS(0b0011) /* FIRC as clock source*/
					  |SCG_RCCR_DIVCORE(0b00) /* DIVCORE=1, Core clock = 48 MHz*/
					  |SCG_RCCR_DIVBUS(0b00)  /* DIVBUS=1, Bus clock = 48 MHz*/
					  |SCG_RCCR_DIVSLOW(0b01);/* DIVSLOW=2, Flash clock= 24 MHz*/


			break;
		}

/*		case(RUN_PLL):
		{
			SCG->SOSCDIV=0x00000101;   SOSCDIV1 & SOSCDIV2 =1: divide by 1
			SCG->SOSCCFG=0x00000024;
			 Range=2: Medium freq (SOSC between 1MHz-8MHz)
			 HGO=0:   Config xtal osc for low power  EREFS=1: Input is external XTAL
			while(SCG->SOSCCSR & SCG_SOSCCSR_LK_MASK);  Ensure SOSCCSR unlocked
			SCG->SOSCCSR=0x00000001;
			 LK=0:          SOSCCSR can be written
			 SOSCCMRE=0:    OSC CLK monitor IRQ if enabled
			 SOSCCM=0:      OSC CLK monitor disabled
			 SOSCERCLKEN=0: Sys OSC 3V ERCLK output clk disabled
			 SOSCLPEN=0:    Sys OSC disabled in VLP modes
			 SOSCSTEN=0:    Sys OSC disabled in Stop modes
			 SOSCEN=1:      Enable oscillator
			while(!(SCG->SOSCCSR & SCG_SOSCCSR_SOSCVLD_MASK));  Wait for sys OSC clk valid

			 Step 1. Setup dividers.
			SCG->SPLLDIV = SCG_SPLLDIV_SPLLDIV1(2) // DIVIDE BY 2 so SPLL_DIV1_CLK is 80MHz
			  |SCG_SPLLDIV_SPLLDIV2(3);	// DIVIDE BY 4 so SPLL_DIV2_CLK is 40MHz

		     Step 2. Set PLL configuration.
		    SCG->SPLLCFG = SCG_SPLLCFG_PREDIV(0)  |
		                    SCG_SPLLCFG_MULT(24);

		     Step 3. Enable clock.
		    SCG->SPLLCSR = SCG_SPLLCSR_SPLLEN(1U);

		     Step 4. Enable monitor.
		    SCG->SPLLCSR = (SCG->SPLLCSR & ~(SCG_SPLLCSR_SPLLCM_MASK   |
		                                      SCG_SPLLCSR_SPLLCMRE_MASK |
		                                      SCG_SPLLCSR_SPLLERR_MASK));


			 Change to normal RUN mode with 8MHz SOSC, 80 MHz PLL
			SCG->RCCR=SCG_RCCR_SCS(6)       PLL as clock source
				|SCG_RCCR_DIVCORE(0b01)       DIVCORE=1, div. by 2: Core clock = 160/2 MHz = 80 MHz
				|SCG_RCCR_DIVBUS(0b01)        DIVBUS=1, div. by 2: bus clock = 40 MHz
				|SCG_RCCR_DIVSLOW(0b10);      DIVSLOW=2, div. by 3: SCG slow, flash clock= 26 2/3 MHz
				while (((SCG->CSR   & SCG_CSR_SCS_MASK) >> SCG_CSR_SCS_SHIFT ) != 6) {} Wait for sys clk src = SPLL

			break;
		}*/
	}

	// Enable very low power modes
	SMC->PMPROT = SMC_PMPROT_AVLP_MASK;
}
