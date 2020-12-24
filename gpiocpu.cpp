/*
 * gpiocpu.cpp
 *
 *  Created on: Dec 22, 2020
 *      Author: idgar
 */

#include "gpiocpu.h"

static inline void tick_tock(struct cpu_state *state, bool clock_is_up) {
	cpu_change_clock(state, clock_is_up);
	state->clock_up = clock_is_up;
	if (clock_is_up && state->state != 0) {
			state->state++;
			state->cycles++;
	}
}

void init(struct cpu_state *cpu) {
	//Sets up Ports
	cpu_init(cpu);
	cpu->state = 0;
	cpu->ale = false;
	cpu->clock_up = 1;
	cpu->address = 0;
	cpu->cycles = 0;
	cpu->operations = 0;
	cpu->first_inta = false;
}

void start(struct cpu_state *cpu) {
	cpu_change_clock(cpu, true);
	cpu_reset_on(cpu);
	for (int i=0; i < 8 * 2; i++) {
		cpu_run_half_cycle(cpu, !cpu->clock_up);
		cpu_delay(cpu, 1);
	}
   cpu_reset_off(cpu);
}

void action_t2(struct cpu_state *state) {
	state->temp_address = address_bus_read(state);
	state->operations++;
}

void action_t3(struct cpu_state *cpu) {
	cpu->control_bus = control_bus_read(cpu);
	// avoid writing garbage to address field while
	// there is an interrupt
	if (cpu->control_bus != BUS_IO_M_MASK) {
		cpu->address = cpu->temp_address;
	}
	switch (cpu->control_bus) {
      //Read Mem
      case BUS_INTA_MASK:
	  data_bus_direction_out(cpu);
	  data_bus_write(cpu, read_address(cpu->address));
	 break;
      //Write Mem
      case (BUS_INTA_MASK | BUS_DTR_MASK):
      write_to_address(cpu->address, data_bus_read(cpu));
	  break;
      //Read IO
      case (BUS_INTA_MASK | BUS_IO_M_MASK):
	  data_bus_direction_out(cpu);
	  data_bus_write(cpu, read_io_address(cpu->address));
	  break;
      //Write IO
      case (BUS_INTA_MASK | BUS_IO_M_MASK | BUS_DTR_MASK):
	  write_to_io_address(cpu->address, data_bus_read(cpu));
	  break;
      //Interrupt
      case BUS_IO_M_MASK:
    	  if(!cpu->first_inta) {
    		  cpu->first_inta = true;
    		  interrupt_acknoledge(cpu);
    	  }
    	  else {
    		  data_bus_direction_out(cpu);
    		  data_bus_write(cpu, read_interrupt_controller_vector_index());
    		  cpu->first_inta = false;
    	  }
	 break;
     default:
	 break;
    }
}

void action_t4(struct cpu_state *state) {
	//only when data direction is out we need
	//to put bus direction back to in.
	if (!(state->control_bus & BUS_DTR_MASK)) {
		data_bus_direction_in(state);
	}
}

void cpu_run_half_cycle(struct cpu_state *cpu, bool clock_is_up) {
	cpu->ale = cpu_read_ale(cpu);
	//wait for syncing the CPU state with code
	if (cpu->state == 0 && !cpu->ale) {
		goto tick;
	}

	if (cpu->ale) {
		cpu->state = 1;
	}

	if (cpu->state == 2 && cpu->clock_up)
		action_t2(cpu);
	else if (cpu->state == 3 && !cpu->clock_up)
		action_t3(cpu);
	else if (cpu->state == 4 && !cpu->clock_up)
		action_t4(cpu);
tick:
	tick_tock(cpu, clock_is_up);
	return;
}
