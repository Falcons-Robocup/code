# xboot-boot.hex created from git version

Andre Pool  
September 2015

## used this commit
commit f36431725919f9b16baef7b0b3a383fefa7cc99c
Author: Alex Forencich <alex@alexforencich.com>
Date:   Sun Oct 11 15:22:44 2015 -0700

    Add config file for atxmega32a4u

git clone https://github.com/alexforencich/xboot.git

cd xboot

make ~/falcons/code/peripherals/xmegaMotor/xboot/x64a3.conf.mk

this should create the hex file

ls -tlr xboot-boot.hex
-rw-rw-r-- 1 andre andre 9676 jan 24 15:50 xboot-boot.hex

connect the Atmel ice programmer to the board  
"notch on ice programmer connector pointing to xmega chip"

type:  
avrdude -p atxmega64a3 -P usb -c atmelice_pdi -U boot:w:xboot-boot.hex -U fuse2:w:0xBF:m

or just

make program

after programming or power cycle the board, the yellow led on the rj45 should blink 5 times
in about 500ms to show the bootloader is waiting for avrdude.

(if the motor firmware is already in the board, the green led will blink after 500 ms and the
yellow led will be off, except when there is an error)

when the xboot-boot.hex is programmed in the board, new application firmware can be dowloaded
with avrdude through the usb serial converter with:

avrdude -e -p atxmega64a3 -P /dev/ttyUSB0 -c avr109 -b 115200 -U flash:w:main.hex

avrdude toggles the DTR pin of the usb serial converter to reset the board and restart xboot.

On the modified board, the usb serial converter DTR pin is connected with a capacitor of 100nF to
the RESET/PDI clock pin of the xmega. A pull up resistor between 3v3 and RESET/PDI asures the
capacitor is reloaded after a DTR toggle.

To test if the bootloader is programmed toggle the DTR pin by typing the following command:
cd ~/falcons/code/peripherals/xmegaMotor/linux
make reset

Or shortly connect pin 5 (RESET/PDI clock) and pin 6 (GND) of the ICE programming connector.

