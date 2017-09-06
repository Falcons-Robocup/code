# Motor control software Atmel Xmega 64a3 for turtle5k robots

On the long run this software and boards will be replaced by an ARM based board. 

Andre Pool  
August 2015 

The software is used to:

* wheel motors (phase 1)
* ball handlers (phase 2)

The used motors are DC brushed type.  
The wheel motor has a quadrature encoder.  
The ball handler has a tago pulse.  

The robot also has an IO board.  
A stripped and modified version of this software can be used for the IO boards.  
(because the IO is different routed and that would result in quite some #ifdef in the code)  

## more information
http://timmel.no-ip.biz:8000/wiki/MotionFirmware

## todo
* the adc values are not always correctly e.g. battery voltage sometimes returns value of < 10v while connected 20v PSU
* when running wheel motor on full speed, abort application, timeout occurs, the motor stop directly, but sometimes it stop slowly "uitrollen"
* replace board rx buffer size calculation on linux side by usage of provided rx buffer space + feedbackId + history
* on to boards (Prabhu and robot3), the bootloader did not work anymore, might have to do with low voltage
* current measured through drv8301 inaccurate
* time used during calculation is lower before second direction change and even more lower before first direction change

## directory structure
* asf   : provided code from Atmel xdk-asf-3.25.0
* build : location for Makefile to build xmega binary (.elf) and flash (.hex) in board with the use of the already programmed bootloader
* inc   : location of include *.h files
* linux : location for the linux motorTest application to be able to control the board from linux through usbSerial
* src   : location of source *.c files
* xboot : location of the bootloader (requires an Atmel programmer e.g. atmel ice)

## compile/build
to be able to build please you need the avr-gcc compiler  
The .deb version of avr-gcc results in a linking error that section .BOOT overlaps with section .data  
you need the Atmel avr-gcc instead  
checkout the ../../scripts/setup.sh  
and search for avr-gcc  

## flash/download
to be able to flash te board, please install avrdude  
sudo apt-get install -y avrdude 
