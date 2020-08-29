/*
 * motor.c
 *
 *  Created on: 2020��8��19��
 *      Author: dm01
 */


#include "motor.h"

static uint8_t MotorDrvTime;

void FlexIO_UART_txchar(char send)
{
    while(!(FLEXIO->SHIFTSTAT & 0x01 ));
    FLEXIO->SHIFTBUF[0] = send;
}

void MotorInit(void)
{
    PCC->PCCn[PCC_PORTB_INDEX] |= PCC_PCCn_CGC_MASK;
    PCC->PCCn[PCC_PORTD_INDEX] |= PCC_PCCn_CGC_MASK;
    PCC->PCCn[PCC_FlexIO_INDEX] |= PCC_PCCn_CGC_MASK | PCC_PCCn_PCS(2);

    //MODE
    PORTD->PCR[2] = PORT_PCR_MUX(1);
    //PWM
    PORTD->PCR[3] = PORT_PCR_MUX(4);//FlexIO_D5
    //EN
    PORTB->PCR[13] = PORT_PCR_MUX(1);
    PTD->PDDR |= 1<< 2;
    PTD->PDDR |= 1<< 3;
    PTB->PDDR |= 1<< 13;
    //TEST UART_TX
    PORTA->PCR[10] = PORT_PCR_MUX(4);//FlexIO_D0
    PTA->PDDR |= 1<<10;//A10 output
    PORTE->PCR[5] = PORT_PCR_MUX(6);//FlexIO_D7
    PTE->PDDR |= 1<<5;//E5 output

    //FLEXIO->CTRL = FLEXIO_CTRL_FLEXEN(1);//Enable FlexIO
    FLEXIO->TIMCTL[0] = FLEXIO_TIMCTL_TIMOD(2)
                        |FLEXIO_TIMCTL_PINSEL(5)
                        |FLEXIO_TIMCTL_PINCFG(3);
    //FLEXIO->TIMCFG[0] |= 0; 
    FLEXIO->TIMCMP[0] = 49<<8 | 49;//DUTY

    //UART
    FLEXIO->TIMCFG[1] |= FLEXIO_TIMCFG_TIMRST(6)
                        |FLEXIO_TIMCFG_TIMOUT(0)
                        |FLEXIO_TIMCFG_TIMDIS(2)
                        |FLEXIO_TIMCFG_TIMENA(2)
                        |FLEXIO_TIMCFG_TSTART(1)
                        |FLEXIO_TIMCFG_TSTOP(3);
    FLEXIO->TIMCTL[1] |= FLEXIO_TIMCTL_TRGSEL(1)
                        |FLEXIO_TIMCTL_TRGPOL(1)
                        |FLEXIO_TIMCTL_TIMOD(1)
                        |FLEXIO_TIMCTL_TRGSRC(1)
                        |FLEXIO_SHIFTCTL_PINSEL(7)
                        |FLEXIO_SHIFTCTL_PINCFG(0);
    FLEXIO->TIMCMP[1] = 0x0F19;

    FLEXIO->SHIFTCFG[0] |= FLEXIO_SHIFTCFG_SSTART(2) | FLEXIO_SHIFTCFG_SSTOP(3);
    FLEXIO->SHIFTCTL[0] |= FLEXIO_SHIFTCTL_TIMSEL(1)
                          |FLEXIO_SHIFTCTL_PINSEL(0)
                          |FLEXIO_SHIFTCTL_PINCFG(3)
                          |FLEXIO_SHIFTCTL_SMOD(2);

    FLEXIO->CTRL = FLEXIO_CTRL_FLEXEN(1);//Enable FlexIO

    //LPIT0 
    PCC->PCCn[PCC_LPIT_INDEX] = PCC_PCCn_CGC_MASK | PCC_PCCn_PCS(2);
    LPIT0->MCR = LPIT_MCR_M_CEN(1);
    LPIT0->MIER = LPIT_MIER_TIE0(1) | LPIT_MIER_TIE1(1);
    LPIT0->TMR[0].TVAL = 11111;
    LPIT0->TMR[0].TCTRL = LPIT_TMR_TCTRL_T_EN(1);
    LPIT0->TMR[1].TVAL = 2000;
	LPIT0->TMR[1].TCTRL = LPIT_TMR_TCTRL_T_EN(1);
    NVIC_IRQ_CLEARPENDING(LPIT0_IRQn);
    NVIC_IRQ_ENABLE(LPIT0_IRQn);
}

void MotorEnable(void)
{
    if((PTB->PDOR & (1<<13)) == 0)
    {
        PTB->PSOR = 1 << 13;
        MotorDrvTime = 0;
    }
    
}

void MotorDisable(void)
{
    PTB->PCOR = 1 << 13;
    MotorDrvTime = 0;
}

void LPIT0_IRQHandler(void)
{
	if(LPIT0->MSR & LPIT_MSR_TIF0_MASK)
	{
		PTD->PTOR = 1 << 2;

		if((PTB->PDOR & (1<<13)) != 0)
		{
			MotorDrvTime++;
			if(MotorDrvTime > 20)
			{
				MotorDisable();
			}
		}

		LPIT0->MSR |= LPIT_MSR_TIF0_MASK;
	}
	if(LPIT0->MSR & LPIT_MSR_TIF1_MASK)
	{
		lin_lld_timeout_service(LI0);
		//PTC->PTOR = 1<<5;
		LPIT0->MSR |= LPIT_MSR_TIF1_MASK;
	}


}
