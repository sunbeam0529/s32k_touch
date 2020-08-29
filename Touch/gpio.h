/****************************************************************************//*!
*
* @file     gpio.h
*
* @version  1.0.0.0
*
* @date     September-2017
*
* @brief    GPIO routines header file
*
*******************************************************************************/
#ifndef __GPIO_H
#define __GPIO_H

/*******************************************************************************
* Includes
*******************************************************************************/
#include "S32K118.h"
/*******************************************************************************
* Function prototypes
******************************************************************************/
void GPIO_Init(void);
void RGB_LED_Init(void);
void Backlight_LED_Init(void);
uint8_t SPI_ReadByte(uint8_t addr);
void SPI_Delay(uint32_t time);
void SPI_GPIO_Init(void);
uint8_t SPI_WriteByte(uint8_t addr,uint8_t data);
void LIN_EN(uint8_t en);
void LIN_STBN(uint8_t stbn);


#endif /* __GPIO_H */
