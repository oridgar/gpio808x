CPPFLAGS += -ggdb 
#CPPFLAGS += -O0
CPPFLAGS += -O3
LDFLAGS += -lstdc++ -lwiringPi -lpthread

all: asm/reset_vector.bin tests 8088_tester

8088_tester: gpiocpu.o gpiocpu_gpiomem.o 8088_tester.o tester_clock.o tester_8259.o

tests: asm/test1.bin asm/dummy_isr.bin asm/dummy_hw_isr.bin

clean:
	@$(RM) 8088_tester asm/*.bin *.o
	
%.bin: %.asm 
	nasm $^ -o $@
