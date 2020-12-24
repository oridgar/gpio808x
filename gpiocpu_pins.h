/*
 * gpiocpu_pins.h
 *
 *  Created on: Dec 23, 2020
 *      Author: idgar
 */

#ifndef GPIOCPU_PINS_H_
#define GPIOCPU_PINS_H_

#define PIN_CLK   29
#define PIN_RESET 27
#define PIN_ALE   26
#define PIN_IO_M  10
#define PIN_DTR   11
#define PIN_BHE   6

#define PIN_INTR  28
#define PIN_INTA  31

#define AD0   25
#define AD1   24
#define AD2   23
#define AD3   22
#define AD4   21
#define AD5   30
#define AD6   14
#define AD7   13

#define A8    12
#define A9    3
#define A10   2
#define A11   0
#define A12   7
#define A13   9
#define A14   8
#define A15   15

#define A16   16
#define A17   1
#define A18   4
#define A19   5

#define AD0_BCM_BIT 26
#define AD1_BCM_BIT 19
#define AD2_BCM_BIT 13
#define AD3_BCM_BIT 6
#define AD4_BCM_BIT 5
#define AD5_BCM_BIT 0
#define AD6_BCM_BIT 11
#define AD7_BCM_BIT 9

#define A8_BCM_BIT 10
#define A9_BCM_BIT 22
#define A10_BCM_BIT 27
#define A11_BCM_BIT 17
#define A12_BCM_BIT 4
#define A13_BCM_BIT 3
#define A14_BCM_BIT 2
#define A15_BCM_BIT 14

#define A16_BCM_BIT 15
#define A17_BCM_BIT 18
#define A18_BCM_BIT 23
#define A19_BCM_BIT 24

#define AD0_BCM_MASK 1<<AD0_BCM_BIT
#define AD1_BCM_MASK 1<<AD1_BCM_BIT
#define AD2_BCM_MASK 1<<AD2_BCM_BIT
#define AD3_BCM_MASK 1<<AD3_BCM_BIT
#define AD4_BCM_MASK 1<<AD4_BCM_BIT
#define AD5_BCM_MASK 1<<AD5_BCM_BIT
#define AD6_BCM_MASK 1<<AD6_BCM_BIT
#define AD7_BCM_MASK 1<<AD7_BCM_BIT

#define A8_BCM_MASK 1<<A8_BCM_BIT
#define A9_BCM_MASK 1<<A9_BCM_BIT
#define A10_BCM_MASK 1<<A10_BCM_BIT
#define A11_BCM_MASK 1<<A11_BCM_BIT
#define A12_BCM_MASK 1<<A12_BCM_BIT
#define A13_BCM_MASK 1<<A13_BCM_BIT
#define A14_BCM_MASK 1<<A14_BCM_BIT
#define A15_BCM_MASK 1<<A15_BCM_BIT

#define A16_BCM_MASK 1<<A16_BCM_BIT
#define A17_BCM_MASK 1<<A17_BCM_BIT
#define A18_BCM_MASK 1<<A18_BCM_BIT
#define A19_BCM_MASK 1<<A19_BCM_BIT

#define PIN_DTR_BCM_BIT 7
#define PIN_IO_M_BCM_BIT  8
#define PIN_INTA_BCM_BIT  1

#define PIN_DTR_BCM_MASK 1<<PIN_DTR_BCM_BIT
#define PIN_IO_M_BCM_MASK 1<<PIN_IO_M_BCM_BIT
#define PIN_INTA_BCM_MASK 1<<PIN_INTA_BCM_BIT

#define PIN_CLK_BCM 21

#endif /* GPIOCPU_PINS_H_ */
