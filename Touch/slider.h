/****************************************************************************//*!
*
* @file  	slider.h
*
* @version  1.0.0.0
*
* @date     December-2017
*
* @brief    Slider electrode touch sense routines for S32K144
*
*******************************************************************************/

#ifndef SLIDER_H_
#define SLIDER_H_

/******************************************************************************
* Typedefs and structures       (scope: module-local)
******************************************************************************/
typedef unsigned char       tBool;          /*!< basic boolean type */

#ifndef FALSE
#define FALSE ((tBool)0)                    /*!< Boolean type FALSE constant */
#endif

#ifndef TRUE
#define TRUE ((tBool)1)                     /*!< Boolean type TRUE constant */
#endif

/*******************************************************************************
* Function prototypes
******************************************************************************/
void SliderVarInit(void);
void SliderDifferenceDataHelperThresholds (void);
void Slider2PadsElectrodesSort(uint8_t electrodeNum, uint8_t* electrodeNumFirstPtr, uint8_t* electrodeNumSecondPtr);
void Slider2PadsElecDSP(uint8_t electrodeNum);
void Slider2PadsElecTouchDetect(uint8_t electrodeNum);
void Slider2PadsSumDataCalculation(void);
int16_t Slider2PadsDiffDataCalculation(uint8_t freq);
int16_t Slider2PadsCentroidDataCalculation(uint8_t freq);
void Slider2PadsTouchQualify(void);
uint8_t Slider_RGBLED_Ctrl(void);
uint8_t SliderDirection(void);
#endif /* SLIDER_H_ */
