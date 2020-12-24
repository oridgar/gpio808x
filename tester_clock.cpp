/*
 * tester_clock.c
 *
 *  Created on: Dec 24, 2020
 *      Author: idgar
 */

#include <stdint.h>
#include <wiringPi.h>
#include "tester_clock.h"

void clock_half_cycle(uint32_t hz) {
	delayMicroseconds(1000000 / 2 / hz);
}
