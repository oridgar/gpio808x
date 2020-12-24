/*
 * gpiocpu.h
 *
 *  Created on: Dec 22, 2020
 *      Author: idgar
 */

#ifndef GPIOCPU_H_
#define GPIOCPU_H_

#include <stdint.h>
#include <stdbool.h>

#define BUS_DTR_BIT 0
#define BUS_IO_M_BIT 1
#define BUS_INTA_BIT 2

#define BUS_DTR_MASK 1 << BUS_DTR_BIT
#define BUS_IO_M_MASK 1 << BUS_IO_M_BIT
#define BUS_INTA_MASK 1 << BUS_INTA_BIT

struct cpu_state {
	uint8_t state;
	bool clock_up;
	uint32_t address;
	uint32_t temp_address;
	char control_bus;
	bool ale;
	uint32_t cycles;
	void *gpio;
	uint32_t operations;
	bool first_inta;
};

//CPU logic implementation
void init(struct cpu_state *cpu);
void start(struct cpu_state *cpu);
void cpu_run_half_cycle(struct cpu_state *cpu, bool clock_is_up);

//environment-specific implementations
char read_address(uint32_t address);
char read_io_address(uint16_t address);
void write_to_io_address(uint16_t address, char data);
void write_to_address(uint32_t address, char data);
char read_interrupt_controller_vector_index();
void interrupt_acknoledge(struct cpu_state *cpu);

//gpio-specific implementations.
void cpu_init(struct cpu_state *cpu);
uint32_t address_bus_read(struct cpu_state *cpu);
uint8_t control_bus_read(struct cpu_state *cpu);
void data_bus_direction_out(struct cpu_state *cpu);
void data_bus_write(struct cpu_state *cpu, char Byte);
char data_bus_read(struct cpu_state *cpu);
void data_bus_direction_in(struct cpu_state *cpu);
void cpu_change_clock(struct cpu_state *cpu, bool clock_up);
void clock_half_cycle(uint32_t hz);
void cpu_reset_on(struct cpu_state *cpu);
void cpu_reset_off(struct cpu_state *cpu);
void cpu_delay(struct cpu_state *cpu, unsigned int msec);
bool cpu_read_ale(struct cpu_state *cpu);
void cpu_interrupt_on(const struct cpu_state *cpu);
void cpu_interrupt_off(const struct cpu_state *cpu);

#endif /* GPIOCPU_H_ */
