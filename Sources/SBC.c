/*
 * SBC.c
 *
 *  Created on: 2020Äê8ÔÂ12ÈÕ
 *      Author: dm01
 */

#include "sbc.h"


void SPI_GPIO_Init(void)
{
    // PORTA
    PCC->PCCn[PCC_PORTA_INDEX] = PCC_PCCn_CGC_MASK;
    // PORTB
    PCC->PCCn[PCC_PORTB_INDEX] = PCC_PCCn_CGC_MASK;
    // PORTC
    PCC->PCCn[PCC_PORTC_INDEX] = PCC_PCCn_CGC_MASK;
    // PORTD
    PCC->PCCn[PCC_PORTD_INDEX] = PCC_PCCn_CGC_MASK;
    // PORTE
    PCC->PCCn[PCC_PORTE_INDEX] = PCC_PCCn_CGC_MASK;

	//SDO
	// PTC6 GPIO pin
	PORTC->PCR[6] = PORT_PCR_MUX(1);
	// PTC6 configured as input
	PTC->PDDR &= ~(1 << 6);

	//SDI
	// PTC7 GPIO pin
	PORTC->PCR[7] = PORT_PCR_MUX(1);
	// Turn OFF
	PTC->PCOR = 1 << 7;
	// PTC7 configured as output
	PTC->PDDR |= 1 << 7;

	//SCK
	// PTD15 GPIO pin
	PORTD->PCR[15] = PORT_PCR_MUX(1);
	// Turn OFF
	PTD->PCOR = 1 << 15;
	// PTD15 configured as output
	PTD->PDDR |= 1 << 15;

	//SCNS
	// PTB5 GPIO pin
	PORTB->PCR[5] = PORT_PCR_MUX(1);
	// Turn OFF
	PTB->PCOR = 1 << 5;
	// PTB5 configured as output
	PTB->PDDR |= 1 << 5;

	/*
	//PTA10
	// PTA10 GPIO pin
	PORTC->PCR[5] = PORT_PCR_MUX(1);
	// Turn OFF
	PTC->PCOR = 1 << 5;
	// PTA10 configured as output
	PTC->PDDR |= 1 << 5;
	*/
}

void SDI_write1bit(uint8_t bits)
{
	if(bits == 0)
	{
		PTC->PCOR = 1 << 7;
	}
	else
	{
		PTC->PSOR = 1 << 7;
	}
}

void SCK_write1bit(uint8_t bits)
{
	if(bits == 0)
	{
		PTD->PCOR = 1 << 15;
	}
	else
	{
		PTD->PSOR = 1 << 15;
	}
}

void SCSN_write1bit(uint8_t bits)
{
	if(bits == 0)
	{
		PTB->PCOR = 1 << 5;
	}
	else
	{
		PTB->PSOR = 1 << 5;
	}
}

uint8_t SDO_read1bit(void)
{
	if(PTC->PDIR & (1<<6))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void SPI_Delay(uint32_t time)
{
	while(time--);
}

uint8_t SPI_ReadByte(uint8_t addr)
{
	uint8_t buf=0;
	SCSN_write1bit(0);
	addr <<= 1;
	addr |= 0x01;// read only
	for(uint8_t i=0;i<8;i++)
	{
		SCK_write1bit(1);
		SDI_write1bit(addr & (0x80>>i));
		SPI_Delay(100);
		SCK_write1bit(0);
		SPI_Delay(100);
	}
	for(uint8_t i=0;i<8;i++)
	{
		SCK_write1bit(1);
		SPI_Delay(100);
		buf <<= 1;
		buf |= SDO_read1bit();
		SCK_write1bit(0);
		SPI_Delay(100);


	}

	SCSN_write1bit(1);

	return buf;
}
uint8_t SPI_WriteByte(uint8_t addr,uint8_t data)
{
	uint8_t buf=0;
	SCSN_write1bit(0);
	addr <<= 1;
	addr |= 0x00;// read & write
	for(uint8_t i=0;i<8;i++)
	{
		SCK_write1bit(1);
		SDI_write1bit(addr & (0x80>>i));
		SPI_Delay(100);
		SCK_write1bit(0);
		SPI_Delay(100);
	}
	for(uint8_t i=0;i<8;i++)
	{
		SCK_write1bit(1);
		SDI_write1bit(data & (0x80>>i));
		SPI_Delay(100);
		buf <<= 1;
		buf |= SDO_read1bit();
		SCK_write1bit(0);
		SPI_Delay(100);


	}

	SCSN_write1bit(1);

	return buf;
}
void LIN_EN(uint8_t en)
{
	SCK_write1bit(en);
}

void LIN_STBN(uint8_t stbn)
{
	SCSN_write1bit(stbn);
}
