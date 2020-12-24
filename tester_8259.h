/*
 * tester_8259.h
 *
 *  Created on: Dec 24, 2020
 *      Author: idgar
 */

#ifndef TESTER_8259_H_
#define TESTER_8259_H_

void raise_irq(volatile const cpu_state *cpu, char num);
void clear_irq(volatile const cpu_state *cpu, char num);
void mask_interrupts(char mask);

#endif /* TESTER_8259_H_ */
