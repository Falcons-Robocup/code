# Copyright 2015 Andre Pool
# Licensed under the Apache License version 2.0
# you may not use this file except in compliance with this License
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

all: atmelice

noBootLoader:
	make -C ../build ice
	sleep 1

force: noBootLoader atmelice

# white Atmel programmer
atmelice: xboot-boot.hex
	avrdude -p atxmega64a3 -P usb -c atmelice_pdi -U boot:w:xboot-boot.hex -U fuse2:w:0xBF:m

# dark blue Atmel programmer
avrisp: xboot-boot.hex
	avrdude -p atxmega64a3 -P usb -c avrispmkII -U boot:w:xboot-boot.hex -U fuse2:w:0xBF:m

.PHONY: all atmelice avrisp

.SECONDARY:


