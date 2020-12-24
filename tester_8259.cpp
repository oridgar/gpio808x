/*
 * tester_8259.cpp
 *
 *  Created on: Dec 24, 2020
 *      Author: idgar
 */

#include "gpiocpu.h"

char irr = 0;
volatile char isr = 0;
char imr = 0;

bool wait_inta = false;
unsigned int interrupt_counter = 0;

//peripheral interface

void raise_irq(volatile const cpu_state *cpu, char num) {
	irr |= 1 << num;
	cpu_interrupt_on((const cpu_state *)cpu);
}

void clear_irq(volatile const cpu_state *cpu, char num) {
	irr &= ~(1 << num);
}

//software interface (through command register)

void mask_interrupts(char mask) {
	imr = mask;
}

static inline void priority_resolver() {
	uint8_t result = irr & ~imr;
	isr = result;
	result >>= 1;
	for(int i=0; result; i++) {
		isr &= ~(1 << i);
		result >>= 1;
	}
}

//-------------------------------
//gpiocpu related implementations
//-------------------------------

void interrupt_acknoledge(struct cpu_state *cpu) {
	priority_resolver();
	wait_inta = false;
	interrupt_counter++;
	cpu_interrupt_off(cpu);
}

char read_interrupt_controller_vector_index() {
	uint8_t int_num = 0;
	uint8_t temp_isr = isr;
	//TODO inefficient. improve later
	while(temp_isr) {
		int_num++;
		temp_isr >>= 1;
	}
	//if irr is 1, IRQ0 is on. IRQ0 is mapped to
	//interrupt vector 8
	return --int_num + 8;
}
