/*
 * lpit.h
 *
 *  Created on: May 25, 2020
 *      Author: nxf38186
 */

#ifndef PERIPHERALS_LPIT_H_
#define PERIPHERALS_LPIT_H_


/*******************************************************************************
* Function prototypes
******************************************************************************/
void LPIT_Init(void);
void LPIT_Enable(uint8_t channel, uint32_t timeout);
void LPIT_Disable(uint8_t channel);
#endif /* PERIPHERALS_LPIT_H_ */
