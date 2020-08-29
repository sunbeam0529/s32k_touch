/*
 * motor.h
 *
 *  Created on: 2020��8��19��
 *      Author: dm01
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "Cpu.h"


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


void MotorInit(void);
void MotorEnable(void);
void MotorDisable(void);
void FlexIO_UART_txchar(char send);

#endif /* MOTOR_H_ */
