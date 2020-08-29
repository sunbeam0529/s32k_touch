/****************************************************************************//*!
*
* @file     gpio.c
*
* @version  1.0.0.0
*
* @date     September-2017
*
* @brief    GPIO routines
*
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include "gpio.h"
#include "S32K118.h"
#include "ts_cfg.h"

/*****************************************************************************
 *
 * Function: void GPIO_Init(void)
 *
 * Description: Init GPIO
 *
 *****************************************************************************/
void GPIO_Init(void)
{
#ifdef DEBUG_ELECTRODE_SENSE
	// Configure as GPIO pin
	DES_PORT->PCR[DES_PIN] = PORT_PCR_MUX(1);
	// Drive GPIO low
	DES_GPIO->PCOR = 1 << DES_PIN;
	// Configure as an output
	DES_GPIO->PDDR |= 1 << DES_PIN;

#endif

#ifdef DEBUG_ALGORITHM
	// Configure as GPIO pin
	DA_PORT->PCR[DA_PIN] = PORT_PCR_MUX(1);
	// Drive GPIO low
	DA_GPIO->PCOR = 1 << DA_PIN;
	// Configure as an output
	DA_GPIO->PDDR |= 1 << DA_PIN;
#endif

#if(LOW_POWER_MODE == LPM_DISABLE)
	// LPUART0 pins
	// PTB0, UART0_RX
	PORTA->PCR[2] = PORT_PCR_MUX(0x6);
	// PTB1, UART0_TX
	PORTA->PCR[3] = PORT_PCR_MUX(0x6);
#endif
	SPI_GPIO_Init();
	// Init RGB LED pins
	//RGB_LED_Init();

	// Init backlight LED pins
	//Backlight_LED_Init();

	/*  // Datalog pin
    // Configure as GPIO pin
    PORTD->PCR[1] = PORT_PCR_MUX(1);
    // Configure as an input
    //BITBAND_ACCESS32(&(PTD)->PDDR,1) = 0;
    PTD->PDDR &= ~(1 << 1);*/
}

/*****************************************************************************
*
* Function: void RGB_LED_Init(void)
*
* Description: Init RGB LED pins
*
*****************************************************************************/
void RGB_LED_Init(void)
{
    // PTE8, RGB LED BLUE
    // PTE8 GPIO pin
    PORTE->PCR[8] = PORT_PCR_MUX(1);
    // Turn OFF LED
    PTE->PCOR = 1 << 8;
    // PTE8 configured as output
    PTE->PDDR |= 1 << 8;

	// PTD15, RGB LED GEEN
    // PTD15 GPIO pin
    PORTD->PCR[15] = PORT_PCR_MUX(1);
    // Turn OFF LED
    PTD->PCOR = 1 << 15;
    // PTD15 configured as output
    PTD->PDDR |= 1 << 15;

    // PTD16, RGB LED RED
    // PTD16 GPIO pin
    PORTD->PCR[16] = PORT_PCR_MUX(1);
    // Turn OFF LED
    PTD->PCOR = 1 << 16;
    // PTD16 configured as output
    PTD->PDDR |= 1 << 16;
}

/*****************************************************************************
*
* Function: void Backlight_LED_Init(void)
*
* Description: Init backlight LED pins
*
*****************************************************************************/
void Backlight_LED_Init(void)
{
    // LED0
    // PTD12 GPIO pin
    PORTD->PCR[12] = PORT_PCR_MUX(2);

    // LED1
    // PTC12 GPIO pin
    PORTC->PCR[12] = PORT_PCR_MUX(3);

    // LED2
    // PTD5 GPIO pin
    PORTD->PCR[5] = PORT_PCR_MUX(2);

    // LED3
    // PTD10 GPIO pin
    PORTD->PCR[10] = PORT_PCR_MUX(2);

    // LED4
    // PTD13 GPIO pin
    PORTD->PCR[13] = PORT_PCR_MUX(2);

    // LED5
    // PTD11 GPIO pin
    PORTD->PCR[11] = PORT_PCR_MUX(2);

    // LED6
    // PTD14 GPIO pin
    PORTD->PCR[14] = PORT_PCR_MUX(2);
}
/*
void SPI_GPIO_Init(void)
{
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
*/

