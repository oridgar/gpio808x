/*
 * gpiocpu_gpiomem.c
 *
 *  Created on: Dec 22, 2020
 *      Author: idgar
 */

#include "gpiocpu.h"
#include <wiringPi.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <stddef.h>
#include "gpiocpu_pins.h"

#define GPIOMEM_PIN_INPUT 0x000
#define GPIOMEM_PIN_OUTPUT 0x001

#define GPLEV0_FIELD 13
#define GPSET0_FIELD 7
#define GPCLR0_FIELD 10
#define GPFSEL0_FIELD 0
#define GPFSEL1_FIELD 1
#define GPFSEL2_FIELD 2
#define GPFSEL3_FIELD 3

struct cpu_gpiomem {
	int gpiomem_fd;
	uint32_t *gpio_mmap;
};

//Sets up Raspberry PI pins in the beginning
void cpu_init(struct cpu_state *cpu)
{
	cpu->gpio = malloc(sizeof(struct cpu_gpiomem));
	struct cpu_gpiomem *gpio = (struct cpu_gpiomem *) cpu->gpio;
	wiringPiSetup();

	gpio->gpiomem_fd = open("/dev/gpiomem", O_RDWR | O_SYNC);
	gpio->gpio_mmap = (uint32_t*)mmap(NULL, 4096, PROT_READ|PROT_WRITE, MAP_SHARED, gpio->gpiomem_fd, 0x00200000);

   pinMode (PIN_CLK, OUTPUT);
   pinMode (PIN_RESET, OUTPUT);
   pinMode (PIN_ALE, INPUT);
   pinMode (PIN_IO_M, INPUT);
   pinMode (PIN_DTR, INPUT);
   pinMode (PIN_BHE, INPUT);

   pinMode (PIN_INTR, OUTPUT);
   pinMode (PIN_INTA, INPUT);
   digitalWrite (PIN_INTR, LOW);

   pinMode (AD0, INPUT);
   pinMode (AD1, INPUT);
   pinMode (AD2, INPUT);
   pinMode (AD3, INPUT);
   pinMode (AD4, INPUT);
   pinMode (AD5, INPUT);
   pinMode (AD6, INPUT);
   pinMode (AD7, INPUT);

   pinMode (A8, INPUT);
   pinMode (A9, INPUT);
   pinMode (A10, INPUT);
   pinMode (A11, INPUT);
   pinMode (A12, INPUT);
   pinMode (A13, INPUT);
   pinMode (A14, INPUT);
   pinMode (A15, INPUT);

   pinMode (A16, INPUT);
   pinMode (A17, INPUT);
   pinMode (A18, INPUT);
   pinMode (A19, INPUT);
}

void cpu_interrupt_on(const struct cpu_state *cpu)
{
   digitalWrite (PIN_INTR, HIGH);
}

void cpu_interrupt_off(const struct cpu_state *cpu)
{
   digitalWrite (PIN_INTR, LOW);
}

void cpu_reset_on(struct cpu_state *cpu) {
    digitalWrite (PIN_RESET, HIGH);
}

void cpu_reset_off(struct cpu_state *cpu) {
	digitalWrite (PIN_RESET, LOW);
}

bool cpu_read_ale(struct cpu_state *cpu) {
	 return digitalRead(PIN_ALE);
}

void cpu_change_clock(struct cpu_state *cpu, bool clock_up) {
	struct cpu_gpiomem *gpiommap = (struct cpu_gpiomem *)cpu->gpio;
	uint32_t *gp_field0 = &gpiommap->gpio_mmap[clock_up ? GPSET0_FIELD : GPCLR0_FIELD];
	*gp_field0 = 1 << PIN_CLK_BCM;
}

//---------
// Data bus
//---------

void data_bus_direction_in(struct cpu_state *cpu) {
	struct cpu_gpiomem *gpiommap = (struct cpu_gpiomem *)cpu->gpio;
	volatile uint32_t *gpfsel0 = &gpiommap->gpio_mmap[GPFSEL0_FIELD];
	volatile uint32_t *gpfsel1 = &gpiommap->gpio_mmap[GPFSEL1_FIELD];
	volatile uint32_t *gpfsel2 = &gpiommap->gpio_mmap[GPFSEL2_FIELD];

	*gpfsel0 = *gpfsel0 &
			~(GPIOMEM_PIN_OUTPUT << (AD3_BCM_BIT % 10) * 3 |
			GPIOMEM_PIN_OUTPUT << (AD4_BCM_BIT % 10)* 3 |
			GPIOMEM_PIN_OUTPUT << (AD5_BCM_BIT % 10) * 3 |
			GPIOMEM_PIN_OUTPUT << (AD7_BCM_BIT % 10) * 3);
	*gpfsel1 = *gpfsel1 &
			~(GPIOMEM_PIN_OUTPUT << (AD1_BCM_BIT % 10) * 3 |
			GPIOMEM_PIN_OUTPUT << (AD2_BCM_BIT % 10) * 3 |
			GPIOMEM_PIN_OUTPUT << (AD6_BCM_BIT % 10) * 3);
	*gpfsel2 = *gpfsel2 &
			~(GPIOMEM_PIN_OUTPUT << (AD0_BCM_BIT % 10) * 3);
}

void data_bus_direction_out(struct cpu_state *cpu) {
	struct cpu_gpiomem *gpiommap = (struct cpu_gpiomem *)cpu->gpio;
	volatile uint32_t *gpfsel0 = &gpiommap->gpio_mmap[GPFSEL0_FIELD];
	volatile uint32_t *gpfsel1 = &gpiommap->gpio_mmap[GPFSEL1_FIELD];
	volatile uint32_t *gpfsel2 = &gpiommap->gpio_mmap[GPFSEL2_FIELD];

	*gpfsel0 = *gpfsel0 |
			GPIOMEM_PIN_OUTPUT << (AD3_BCM_BIT % 10) * 3 |
			GPIOMEM_PIN_OUTPUT << (AD4_BCM_BIT % 10)* 3 |
			GPIOMEM_PIN_OUTPUT << (AD5_BCM_BIT % 10) * 3 |
			GPIOMEM_PIN_OUTPUT << (AD7_BCM_BIT % 10) * 3;
	*gpfsel1 = *gpfsel1 |
			GPIOMEM_PIN_OUTPUT << (AD1_BCM_BIT % 10) * 3 |
			GPIOMEM_PIN_OUTPUT << (AD2_BCM_BIT % 10) * 3 |
			GPIOMEM_PIN_OUTPUT << (AD6_BCM_BIT % 10) * 3;
	*gpfsel2 = *gpfsel2 |
			GPIOMEM_PIN_OUTPUT << (AD0_BCM_BIT % 10) * 3;
}

char data_bus_read(struct cpu_state *cpu) {
   char data_bus;
	struct cpu_gpiomem *gpiommap = (struct cpu_gpiomem *)cpu->gpio;
	uint32_t gplev0 = gpiommap->gpio_mmap[GPLEV0_FIELD];

	data_bus =
			(gplev0 & AD0_BCM_MASK) >> AD0_BCM_BIT << 0 |
			(gplev0 & AD1_BCM_MASK) >> AD1_BCM_BIT << 1 |
			(gplev0 & AD2_BCM_MASK) >> AD2_BCM_BIT << 2 |
			(gplev0 & AD3_BCM_MASK) >> AD3_BCM_BIT << 3 |
			(gplev0 & AD4_BCM_MASK) >> AD4_BCM_BIT << 4 |
			(gplev0 & AD5_BCM_MASK) >> AD5_BCM_BIT << 5 |
			(gplev0 & AD6_BCM_MASK) >> AD6_BCM_BIT << 6 |
			(gplev0 & AD7_BCM_MASK) >> AD7_BCM_BIT << 7
	;
   return data_bus;
}

//Writes Data to Data Port 0-7
void data_bus_write(struct cpu_state *cpu, char Byte)
{
	uint32_t only_ad_bits;
	uint32_t val_before;
	uint32_t val1;
	uint32_t val2;
	struct cpu_gpiomem *gpiommap = (struct cpu_gpiomem *)cpu->gpio;
	uint32_t *gpclr0 = &gpiommap->gpio_mmap[GPCLR0_FIELD];
	uint32_t *gpset0 = &gpiommap->gpio_mmap[GPSET0_FIELD];

	only_ad_bits = (
			 ((Byte     ) & 1) << AD0_BCM_BIT |
			 ((Byte >> 1) & 1) << AD1_BCM_BIT |
			 ((Byte >> 2) & 1) << AD2_BCM_BIT |
			 ((Byte >> 3) & 1) << AD3_BCM_BIT |
			 ((Byte >> 4) & 1) << AD4_BCM_BIT |
			 ((Byte >> 5) & 1) << AD5_BCM_BIT |
			 ((Byte >> 6) & 1) << AD6_BCM_BIT |
			 ((Byte >> 7) & 1) << AD7_BCM_BIT);

	*gpclr0 = (AD0_BCM_MASK |
			AD1_BCM_MASK |
			AD2_BCM_MASK |
			AD3_BCM_MASK |
			AD4_BCM_MASK |
			AD5_BCM_MASK |
			AD6_BCM_MASK |
			AD7_BCM_MASK);

	*gpset0 = only_ad_bits;
}

// -----------
// Control bus
// -----------

//Reads the IO_M, RD, WR pins
uint8_t control_bus_read(struct cpu_state *cpu) {
	uint8_t control_bus;

	struct cpu_gpiomem *gpiommap = (struct cpu_gpiomem *)cpu->gpio;
	uint32_t gplev0 = gpiommap->gpio_mmap[GPLEV0_FIELD];

	control_bus =
			(gplev0 & PIN_DTR_BCM_MASK) >> PIN_DTR_BCM_BIT << BUS_DTR_BIT |
			(gplev0 & PIN_IO_M_BCM_MASK) >> PIN_IO_M_BCM_BIT << BUS_IO_M_BIT |
			(gplev0 & PIN_INTA_BCM_MASK) >> PIN_INTA_BCM_BIT << BUS_INTA_BIT
	;
	return control_bus;
}

// -----------
// Address bus
// -----------

uint32_t address_bus_read(struct cpu_state *cpu) {
	uint32_t addr;
   struct cpu_gpiomem *gpiommap = (struct cpu_gpiomem *)cpu->gpio;
   uint32_t gplev0 = gpiommap->gpio_mmap[GPLEV0_FIELD];

   addr =
		   (gplev0 & AD0_BCM_MASK) >> AD0_BCM_BIT |
		   (gplev0 & AD1_BCM_MASK) >> AD1_BCM_BIT << 1  |
		   (gplev0 & AD2_BCM_MASK) >> AD2_BCM_BIT << 2 |
		   (gplev0 & AD3_BCM_MASK) >> AD3_BCM_BIT << 3 |
		   (gplev0 & AD4_BCM_MASK) >> AD4_BCM_BIT << 4 |
		   (gplev0 & AD5_BCM_MASK) >> AD5_BCM_BIT << 5 |
		   (gplev0 & AD6_BCM_MASK) >> AD6_BCM_BIT << 6 |
		   (gplev0 & AD7_BCM_MASK) >> AD7_BCM_BIT << 7 |
		   (gplev0 & A8_BCM_MASK) >> A8_BCM_BIT << 8 |
		   (gplev0 & A9_BCM_MASK) >> A9_BCM_BIT << 9 |
		   (gplev0 & A10_BCM_MASK) >> A10_BCM_BIT << 10 |
		   (gplev0 & A11_BCM_MASK) >> A11_BCM_BIT<< 11 |
		   (gplev0 & A12_BCM_MASK) >> A12_BCM_BIT << 12 |
		   (gplev0 & A13_BCM_MASK) >> A13_BCM_BIT << 13 |
		   (gplev0 & A14_BCM_MASK) >> A14_BCM_BIT << 14 |
		   (gplev0 & A15_BCM_MASK) >> A15_BCM_BIT<< 15 |
		   (gplev0 & A16_BCM_MASK) >> A16_BCM_BIT << 16 |
		   (gplev0 & A17_BCM_MASK) >> A17_BCM_BIT << 17 |
		   (gplev0 & A18_BCM_MASK) >> A18_BCM_BIT << 18 |
		   (gplev0 & A19_BCM_MASK) >> A19_BCM_BIT << 19
   ;
   return addr;
}

void cpu_delay(struct cpu_state *cpu, unsigned int msec) {
	delay(msec);
}
