/*
 * dma.h
 *
 *  Created on: May 6, 2020
 *      Author: nxf38186
 */

#ifndef PERIPHERALS_DMA_H_
#define PERIPHERALS_DMA_H_

/*******************************************************************************
 * Includes
 *******************************************************************************/
#include "S32K118.h"

/*******************************************************************************
* Type defines
******************************************************************************/
//TCD for DMA Scatter - Gather implementation

typedef struct
{
    uint32_t SADDR;
    int16_t SOFF;
    uint16_t ATTR;
    uint32_t NBYTES;
    int32_t SLAST;
    uint32_t DADDR;
    int16_t DOFF;
    uint16_t CITER;
    int32_t DLASTSGA;
    uint16_t CSR;
    uint16_t BITER;
} edma_software_tcd_t1;

/*******************************************************************************
* Function prototypes
******************************************************************************/
void DMA_Scan_Start(uint8_t samplingCtrl);
void DMA_Init(void);
void DMA_CH0_All_Address_Sets_Load(void);
void DMA_CH0_SG_TCDs_Load(void);
void DMA_CH2_All_Address_Sets_Load(void);
void DMA_CH3_All_Address_Sets_Load(uint8_t samplingCtrl);
void DMA_TCDreg_All_Electrodes_Scan_Init(uint8_t startingElecNum);
void DMA_TCDreg_Sequence_Scan_Init(uint8_t startingElecNum, uint8_t lastElecNum, uint8_t samplingCtrl);
void DMA_CH2_Wakeup_Event_Address_Sets_Load(uint8_t startingElecNum, uint8_t touchElecNum, uint8_t samplingCtrl);
void DMA_TCDreg_Wakeup_Event_Scan_Init(uint8_t startingElecNum, uint8_t touchElecNum, uint8_t samplingCtrl);
void ElectrodesSequenceScanDMA(uint8_t startingElecNum, uint8_t lastElecNum);
void ElectrodesWakeupEventScanDMA(uint8_t wakeupNum, uint8_t touchElecNum);
void ElectrodesSingleElecScanDMA(uint8_t electrodeNum);
#endif /* PERIPHERALS_DMA_H_ */
