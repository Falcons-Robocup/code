Prerequisite software (sudo apt-get install):
	gcc-avr
	binutils-avr
	srecord
	avr-libc
	avrdude

To build the software:
	cd ~/falcons/code/peripherals/ioBoard
	mkdir build
	cd build
	cmake ..
	make

To upload the software (with AVRISP mkII):
	cd ~/falcons/code/peripherals/ioBoard/build
	sudo make upload
