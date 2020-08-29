/*
 * touch_buttons.h
 *
 *  Created on: Apr 28, 2020
 *      Author: nxf38186
 */

#ifndef TOUCH_BUTTONS_H_
#define TOUCH_BUTTONS_H_

/*******************************************************************************
* Function prototypes
******************************************************************************/
void ButtonVarInit(void);
void ButtonElecDSP(uint8_t electrodeNum);
void ButtonElecTouchDetect(uint8_t electrodeNum);
void ButtonTouchQualify(void);
uint8_t Button_RGBLED_Ctrl(void);
uint8_t slider_Detect(uint8_t  buffer[]);
#endif /* TOUCH_BUTTONS_H_ */
