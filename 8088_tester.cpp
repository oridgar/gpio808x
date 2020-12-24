#include <fcntl.h>    /* For O_RDWR */
#include <unistd.h>   /* For open(), creat() */
#include <pthread.h>
#include <assert.h>

#include <string>
#include <fstream>

#include "gpiocpu.h"

#include "tester_clock.h"
#include "tester_8259.h"

using namespace std;

extern bool wait_inta;
extern unsigned int interrupt_counter;

volatile unsigned char RAM[0x100000];
volatile unsigned char IO[0x10000];

bool enable_interrupts = false;
unsigned int addr_error_counter = 0;

struct start_cpu_params {
	struct cpu_state *cpu;
	uint32_t hz;
};

// --------
// IO Ports
// --------

char read_io_address(uint16_t address) {
	return IO[address];
}

void write_to_io_address(uint16_t address, char data) {
	IO[address] = data;
}

//-----------------------
//Address module functions
//-----------------------

char read_address(uint32_t address) {
	return RAM[address];
}

void write_to_address(uint32_t address, char data) {
	RAM[address] = data;
}

//------------
//Bootstraping
//------------

unsigned int load_data_to_mem(string filename, unsigned int start_address) {
	unsigned int filesize;
	std::ifstream MemoryFile;                 //New ifstream
	MemoryFile.open(filename);              //Open Rom.bin
	MemoryFile.seekg (0, ios::end);           //Find the end of the file
	filesize = MemoryFile.tellg();        //Get the size of the file
	MemoryFile.seekg (0, MemoryFile.beg);     //Start reading at the begining
	MemoryFile.read((char*)&RAM[start_address], filesize);           //Read the file
	MemoryFile.close();                       //Close the file
	return filesize;
}

void fill_ivt() {
	uint16_t *offset;
	uint16_t *segment;
	int i = 0;

	//CPU internal interrupts
	for(; i < 8; i++) {
		offset = (uint16_t*)&RAM[i * 4];
		segment = offset + 1;
		*offset = 0;
		*segment = 0x4000;
	}

	//rest of interrupts
	for(; i < 256; i++) {
		offset = (uint16_t*)&RAM[i * 4];
		segment = offset + 1;
		*offset = 0x0;
		*segment = 0x3000;
	}

	load_data_to_mem("asm/dummy_isr.bin", 0x30000);
	load_data_to_mem("asm/dummy_hw_isr.bin", 0x40000);
}

// -------------
// tester engine
// -------------

void *start_cpu(void *args) {
	struct start_cpu_params *params =
			(struct start_cpu_params *)args;
	start(params->cpu);
	while(true) {
		cpu_run_half_cycle(params->cpu, !params->cpu->clock_up);
		clock_half_cycle(params->hz);
	}
}

void test1(volatile struct cpu_state *cpu, unsigned int hz, unsigned int start_addr,
		unsigned int code_size) {
	bool addr_sampled = false;
	uint next_intr_cycle = 100;
	uint mul_mark_counter = 0;
	uint8_t curr_irq = 0;

	printf("legend: DTR=0 read, DTR=1 write\n"
		   "        IO_M=0 memory, IO_M=1 port\n"
		   "        INTA=0 acknoledge active, INTA=1 acknoledge not active\n");

	//wait for cpu sync
	while (cpu->state == 0);
	while (cpu->address != 0xffff0);

	while (true) {
		if (cpu->state > 3 && !addr_sampled) {
			if (!(
					(cpu->control_bus & BUS_IO_M_MASK)
				|| (0xffff0 <= cpu->address && cpu->address <= 0xffff5)
				|| (0x00000 <= cpu->address && cpu->address <= 0x40010)
			)) {
				addr_error_counter++;
			}
			addr_sampled = true;
		} else if (cpu->state < 2) {
			addr_sampled = false;
		}

		if (hz < 1000 || cpu->cycles % 200 == 0) {
			if( IO[0] & 1) {
				IO[0] = 0;
				mul_mark_counter++;
			}
			printf(	"\rDTR=%u IO_M=%u INTA=%u operations=%09u "
					"cycles=%09u addr=0x%05x mul_result=%09u mul_mark=%09u "
					"intr_cnt=%09u err=%09u",
					(cpu->control_bus & BUS_DTR_MASK) >> BUS_DTR_BIT,
					(cpu->control_bus & BUS_IO_M_MASK) >> BUS_IO_M_BIT,
					(cpu->control_bus & BUS_INTA_MASK) >> BUS_INTA_BIT,
					cpu->operations, cpu->cycles, cpu->address,
					(uint32_t)*(uint32_t*)(&RAM[0x2F000]), mul_mark_counter,
					interrupt_counter, addr_error_counter);
		}

		if (enable_interrupts && !wait_inta && cpu->cycles > next_intr_cycle) {
			wait_inta = true;
			clear_irq(cpu, (uint8_t)(curr_irq - 1) % 8);
			raise_irq(cpu, curr_irq++);
			curr_irq %= 8;
			next_intr_cycle += hz < 1000 ? 200 : hz / 10;
		}
	}
}

int main(int argc, char *argv[]) {
	unsigned int code_size;
	uint8_t curr_add_idx = 0;
	uint32_t cycles = 0;
	uint32_t error_counter = 0;
	pthread_t cpu_thread;
    cpu_set_t cpuset;
	int s;
	struct cpu_state cpu;
	struct start_cpu_params cpu_thread_params =
	{
			.cpu = &cpu,
			.hz = 2
	};

	if (argc > 1) {
		cpu_thread_params.hz = atoi(argv[1]);
	}
	if (argc == 3) {
		enable_interrupts = true;
	}

	printf("hz=%d\n", cpu_thread_params.hz);
	fill_ivt();
	load_data_to_mem("asm/reset_vector.bin", 0xFFFF0);
	*(uint16_t*)(&RAM[0x2F004]) = 1; //value for AX
	*(uint16_t*)(&RAM[0x2F006]) = 32; //value for CX
	init(&cpu);
	code_size = load_data_to_mem("asm/test1.bin", 0x10000);
	if(pthread_create(&cpu_thread, NULL, start_cpu, &cpu_thread_params)) {
		fprintf(stderr, "Error creating thread\n");
		return 1;
	}

    CPU_ZERO(&cpuset);
    CPU_SET(3, &cpuset);

    s = pthread_setaffinity_np(cpu_thread, sizeof(cpuset), &cpuset);
    if (s != 0)
        fprintf(stderr, "cannot set affinity! result=%d", s);

    /* Check the actual affinity mask assigned to the thread */

    s = pthread_getaffinity_np(cpu_thread, sizeof(cpuset), &cpuset);
    if (s != 0)
        fprintf(stderr, "cannot get affinity! result=%d", s);

    printf("Set returned by pthread_getaffinity_np() contained:\n");
    for (int j = 0; j < CPU_SETSIZE; j++)
        if (CPU_ISSET(j, &cpuset))
            printf("    CPU %d\n", j);

	test1(&cpu, cpu_thread_params.hz, 0x10000, code_size);
	pthread_join(cpu_thread, NULL);
}

