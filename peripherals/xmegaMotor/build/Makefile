# Copyright 2015 Andre Pool
# Licensed under the Apache License version 2.0
# you may not use this file except in compliance with this License
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

INC = -I../inc
INC += -I../asf/common/boards
INC += -I../asf/common/boards/user_board
INC += -I../asf/common/boards/user_board/board_config
INC += -I../asf/common/services/clock
INC += -I../asf/common/services/clock/xmega
INC += -I../asf/common/services/ioport
INC += -I../asf/common/services/ioport/xmega
INC += -I../asf/common/services/sleepmgr
INC += -I../asf/common/services/spi/xmega_spi
INC += -I../asf/common/services/spi/xmega_spi/module_config
INC += -I../asf/common/utils
INC += -I../asf/xmega/drivers/adc
INC += -I../asf/xmega/drivers/cpu
INC += -I../asf/xmega/drivers/nvm
INC += -I../asf/xmega/drivers/pmic
INC += -I../asf/xmega/drivers/qdec
INC += -I../asf/xmega/drivers/sleep
INC += -I../asf/xmega/drivers/spi
INC += -I../asf/xmega/drivers/tc
INC += -I../asf/xmega/drivers/usart
INC += -I../asf/xmega/utils
INC += -I../asf/xmega/utils/preprocessor
# INC += -I/opt/avr_v3.4.5/avr/include

CFLAGS0 = -g -Wall -O3 -ffreestanding -fshort-enums --std=gnu99 -DBOARD=USER_BOARD -DIOPORT_XMEGA_COMPAT -DCONFIG_NVM_IGNORE_XMEGA_A3_D3_REVB_ERRATA
CFLAGS = $(CFLAGS0) -pedantic
LDFLAGS = -Wl,-Map,$(PRG).map
#PRG = motorFirmware
#LDFLAGS = -Wl,-Map,$(PRG).map,-section-start=.BOOT=0x10F00

.SUFFIXES: .s .bin .out .hex .srec


all: main.hex

%.o: ../src/%.c ../inc/global.h
	avr-gcc -mmcu=atxmega64a3 $(CFLAGS) $(INC) -o $@ -c $<

ccp.o: ../asf/xmega/drivers/cpu/ccp.s
	avr-gcc -mmcu=atxmega64a3 $(CFLAGS) $(INC) -x assembler-with-cpp -o $@ -c $<

nvm_asm.o: ../asf/xmega/drivers/nvm/nvm_asm.s
	avr-gcc -mmcu=atxmega64a3 $(CFLAGS) $(INC) -x assembler-with-cpp -o $@ -c $<

tc.o: ../asf/xmega/drivers/tc/tc.c
	avr-gcc -mmcu=atxmega64a3 $(CFLAGS) $(INC) -o $@ -c $<

usart.o: ../asf/xmega/drivers/usart/usart.c
	avr-gcc -mmcu=atxmega64a3 $(CFLAGS) $(INC) -o $@ -c $<

usart_xmegae.o: ../asf/xmega/drivers/usart/usart_xmegae.c
	avr-gcc -mmcu=atxmega64a3 $(CFLAGS) $(INC) -o $@ -c $<

spi_xmega_int.o: ../asf/xmega/drivers/spi/spi.c
	avr-gcc -mmcu=atxmega64a3 $(CFLAGS) $(INC) -o $@ -c $<

spi_xmega_ext.o: ../asf/common/services/spi/xmega_spi/spi_master.c
	avr-gcc -mmcu=atxmega64a3 $(CFLAGS) $(INC) -o $@ -c $<

xcl.o: ../asf/xmega/drivers/xcl/xcl.c
	avr-gcc -mmcu=atxmega64a3 $(CFLAGS) $(INC) -o $@ -c $<

qdec.o: ../asf/xmega/drivers/qdec/qdec.c
	avr-gcc -mmcu=atxmega64a3 $(CFLAGS) $(INC) -o $@ -c $<

ioport_compat.o: ../asf/common/services/ioport/xmega/ioport_compat.c
	avr-gcc -mmcu=atxmega64a3 $(CFLAGS) $(INC) -o $@ -c $<

sysclk.o: ../asf/common/services/clock/xmega/sysclk.c
	avr-gcc -mmcu=atxmega64a3 $(CFLAGS) $(INC) -o $@ -c $<

sleepmgr.o: ../asf/common/services/sleepmgr/xmega/sleepmgr.c
	avr-gcc -mmcu=atxmega64a3 $(CFLAGS) $(INC) -o $@ -c $<

# the following 3 targets use nvm.h that uses unsupport unnamed structs/unions, to prevent warnings they are compiled without pedantic flag
safety.o: ../src/safety.c ../inc/global.h
	avr-gcc -mmcu=atxmega64a3 $(CFLAGS0) -fms-extensions $(INC) -o $@ -c $<

adc_aau.o: ../asf/xmega/drivers/adc/xmega_aau/adc_aau.c
	avr-gcc -mmcu=atxmega64a3 $(CFLAGS0) $(INC) -o $@ -c $<

adc.o: ../asf/xmega/drivers/adc/adc.c
	avr-gcc -mmcu=atxmega64a3 $(CFLAGS0) $(INC) -o $@ -c $<


main.elf: adc.o adc_aau.o ccp.o communication.o encoder.o ioport_compat.o nvm_asm.o pid.o pwm.o genericFilter2nd.o qdec.o safety.o scheduler.o  sleepmgr.o spi_xmega_ext.o spi_xmega_int.o spiMaster.o sysclk.o tc.o usart.o xmega.o main.o 
	avr-gcc -mmcu=atxmega64a3 $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LDLIBS)

main.hex: main.elf
	avr-objcopy -O ihex $< $@

# Fuse byte 5
# See datasheet section 4.16.5, page 32 doc8077.pdf Atmel AVR XMEGA A Manual
# Configures BOD operation in active mode,
# BOD in Active mode[1:0] = 10 = BOD enabled continuously 
# EESAVE = 1 = EEPROM is erased during chip erase 
# BOD level[2:0] = 010 = 2.9v
# FUSE Byte 5 = 11101010 = 0xEA

# white Atmel programmer
atmelice ice: main.hex
	avrdude -p x64a3 -P usb -e -U flash:w:main.hex -U fuse5:w:0xEA:m -c atmelice_pdi


# dark blue Atmel programmer
avrisp isp: main.hex
	avrdude -p x64a3 -P usb -e -U flash:w:main.hex -U fuse5:w:0xEA:m -c avrispmkII


../linux/resetBoard:
	make  --no-print-directory -C ../linux resetBoard

xboot: ../linux/resetBoard main.hex
	@killall peripherals 2>/dev/null || true
	# other then the arduino protocol, the avr109 does not activate the DTR again 
	@../linux/resetBoard -P /dev/ttyUSB0
	avrdude -e -p atxmega64a3 -P /dev/ttyUSB0 -c avr109 -b 115200 -U flash:w:main.hex

xbootall: ../linux/resetBoard main.hex
	@killall peripherals 2>/dev/null || true
	# other then the arduino protocol, the avr109 does not activate the DTR again 
	@../linux/resetBoard -P /dev/ttyUSB0
	@../linux/resetBoard -P /dev/ttyUSB1
	@../linux/resetBoard -P /dev/ttyUSB2
	@../linux/resetBoard -P /dev/ttyUSB3
	@../linux/resetBoard -P /dev/ttyUSB4
	avrdude -e -p atxmega64a3 -P /dev/ttyUSB0 -c avr109 -b 115200 -U flash:w:main.hex
	avrdude -e -p atxmega64a3 -P /dev/ttyUSB1 -c avr109 -b 115200 -U flash:w:main.hex
	avrdude -e -p atxmega64a3 -P /dev/ttyUSB2 -c avr109 -b 115200 -U flash:w:main.hex
	avrdude -e -p atxmega64a3 -P /dev/ttyUSB3 -c avr109 -b 115200 -U flash:w:main.hex
	avrdude -e -p atxmega64a3 -P /dev/ttyUSB4 -c avr109 -b 115200 -U flash:w:main.hex

xboot1: ../linux/resetBoard main.hex
	@killall peripherals 2>/dev/null || true
	# other then the arduino protocol, the avr109 does not activate the DTR again 
	@../linux/resetBoard -P /dev/ttyMotor1
	avrdude -e -p atxmega64a3 -P /dev/ttyMotor1 -c avr109 -b 115200 -U flash:w:main.hex

xboot2: ../linux/resetBoard main.hex
	@killall peripherals 2>/dev/null || true
	# other then the arduino protocol, the avr109 does not activate the DTR again 
	@../linux/resetBoard -P /dev/ttyMotor2
	avrdude -e -p atxmega64a3 -P /dev/ttyMotor2 -c avr109 -b 115200 -U flash:w:main.hex

xboot3: ../linux/resetBoard main.hex
	@killall peripherals 2>/dev/null || true
	# other then the arduino protocol, the avr109 does not activate the DTR again 
	@../linux/resetBoard -P /dev/ttyMotor3
	avrdude -e -p atxmega64a3 -P /dev/ttyMotor3 -c avr109 -b 115200 -U flash:w:main.hex



usbasb: main.hex
	echo "will not work because usbasp does not support PDI required to program xmega"
	avrdude -p atxmega64a3 -P usb -c usbasp -U flash:w:main.hex:i

flash: xboot
flash1: xboot1
flash2: xboot2
flash3: xboot3

clean:
	@sync
	rm -f *~ *.o *.elf *.hex
	make --no-print-directory -C ../linux clean

.PHONY: all clean flash

.SECONDARY:


