/*
 * sbc.h
 *
 *  Created on: 2020Äê8ÔÂ12ÈÕ
 *      Author: dm01
 */

#ifndef SBC_H_
#define SBC_H_

#include "Cpu.h"

uint8_t SPI_ReadByte(uint8_t addr);
void SPI_Delay(uint32_t time);
void SPI_GPIO_Init(void);
uint8_t SPI_WriteByte(uint8_t addr,uint8_t data);
void LIN_EN(uint8_t en);
void LIN_STBN(uint8_t stbn);

#endif /* SBC_H_ */
