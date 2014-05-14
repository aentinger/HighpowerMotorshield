PROJECT=firmware_rob

CC=avr-g++
LDFLAGS= -Wl,-Map,$(TRG).map -mmcu=$(MCU) -lm
OBJCOPY=avr-objcopy

MCU=atmega328p
PROG_MCU=m328p

TRG=$(PROJECT).out
HEXFORMAT=ihex
HEXROMTRG= $(PROJECT).hex
AVRDUDE_PROGID=dragon_isp
AVRDUDE=avrdude
AVRDUDE_PORT= usb

all: adc control input linear_mapper motor main

adc: adc.cpp
	$(CC) $(LDFLAGS) -c adc.cpp

control: control.cpp
	$(CC) $(LDFLAGS) -c control.cpp

input: input.cpp
	$(CC) $(LDFLAGS) -c input.cpp

linear_mapper: linear_mapper.cpp
	$(CC) $(LDFLAGS) -c linear_mapper.cpp

motor: motor.cpp
	$(CC) $(LDFLAGS) -c motor.cpp

main: main.cpp
	$(CC) $(LDFLAGS) *.o main.cpp -o $(PROJECT).out

install: writeflash

hex:
	$(OBJCOPY) -j .text -j .data -O $(HEXFORMAT) $(TRG) $(HEXROMTRG)

writeflash: hex
	$(AVRDUDE) -c $(AVRDUDE_PROGID) \
		-p $(PROG_MCU) -P $(AVRDUDE_PORT) -e \
		-U flash:w:$(HEXROMTRG)

clean:
	rm -rf *.o
	rm -rf $(PROJECT).out
	rm -rf $(PROJECT).out.map
	rm -rf $(PROJECT).hex

