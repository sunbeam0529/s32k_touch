/****************************************************************************//*!
*
* @file     main.h
*
* @version  1.0.0.0
*
* @date     September-2017
*
* @brief    Touch sense main for S32K144
*
*******************************************************************************/
#ifndef __MAIN_H
#define __MAIN_H

/*******************************************************************************
* Interrupts
*******************************************************************************/
/*!< Macro to enable all interrupts. */
// This function enables IRQ interrupts by clearing the I-bit in the CPSR.
#define EnableInterrupts __asm(" CPSIE i");

/*!< Macro to disable all interrupts. */
// This function disables IRQ interrupts by setting the I-bit in the CPSR.
#define DisableInterrupts __asm(" CPSID i");

/* Set IRQ priority */
/* Accepts NVIC IRQ number (not the Cortex-M0+ vector number), priority 0-3 */
#define NVIC_SET_IRQ_PRIORITY_LEVEL(IrqNum,Priority)    (S32_NVIC->IPR[IrqNum / 4] = ((Priority & 0x3) << (8*(IrqNum % 4) + 6)))

/* Enable IRQ */
/* Accepts NVIC IRQ number (not the Cortex-M4F vector number) */
#define NVIC_IRQ_ENABLE(IrqNum)                         (S32_NVIC->ISER[IrqNum / 32] |= (1 << (IrqNum % 32)))

/* Disable IRQ */
/* Accepts NVIC IRQ number (not the Cortex-M4F vector number) */
#define NVIC_IRQ_DISABLE(IrqNum)                        (S32_NVIC->ICER[IrqNum / 32] |= (1 << (IrqNum % 32)))

/* Clear pending IRQ */
/* Accepts NVIC IRQ number (not the Cortex-M4F vector number) */
#define NVIC_IRQ_CLEARPENDING(IrqNum)                        (S32_NVIC->ICPR[IrqNum / 32] = (1 << (IrqNum % 32)))

/* Modify 32bit register bit: Read, Modify, Write */
//#define R_RMW32(address, bit, value)   (REG_WRITE32((address), ((REG_READ32(address)& ((uint32_t)~((uint32_t)(1 << bit))))| ((uint32_t)(value << bit)))))

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void NVIC_Init(void);
void RGBLED_Ctrl(void);

#endif /* __MAIN_H */
