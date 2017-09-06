#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h> // needed for memset

_Bool doSend( int tty_fd, unsigned char sendVal, unsigned char expectedVal ) {
	write(tty_fd, &sendVal, 1);
	usleep(100000);
	unsigned char receiveVal = '0';
	read(tty_fd, &receiveVal, 1);
	if( receiveVal != expectedVal ) {
		printf("ERROR got return value 0x%02x but expected 0x%02x when writing 0x%02x\n", receiveVal, expectedVal, sendVal);
		return 1;
	}
	return 0;
}

int main(int argc, char** argv) {
	_Bool configure = 0;
	_Bool fast = 0;
	_Bool reset = 0;

	int opt = 0;
	char device[256] = "/dev/ttyUSB0";

	while( (opt = getopt(argc, argv, "cd:fhr") ) != -1 ) {
		switch(opt) {
		case 'c':
			configure = 1;
			break;
		case 'd':
			sprintf(device, "/dev/ttyUSB%d", atoi(optarg) );
		break;
	 		case 'f':
			fast = 1;
			break;
		case 'h':
			printf("options:\n");
			printf("  -c calibrate\n");
			printf("  -f print fast\n");
			printf("  -h this help\n");
			printf("  -r reset to factory defaults (afterwards calibration required)\n");
			return 0;
			break;
		case 'r':
			reset = 1;
			break;
		}
	}

	printf("control the cmps10 serial compass\n");
	struct termios tio;
	int tty_fd;

	unsigned char sendVal = 0;
	unsigned char receiveVal = '0';

	memset(&tio, 0, sizeof(tio));
	tio.c_iflag = 0;
	tio.c_oflag = 0;
	tio.c_cflag = CS8 | CSTOPB | CREAD | CLOCAL; // 8n2, see termios.h for more information
	tio.c_lflag = 0;
	tio.c_cc[VMIN] = 1;
	tio.c_cc[VTIME] = 5;

	printf("using device: %s\n", device);
	tty_fd = open(device, O_RDWR | O_NONBLOCK);
	// O_NONBLOCK is posix-specified name for O_NDELAY
	// O_NOCTTY If set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
	cfsetospeed(&tio, B9600);
	cfsetispeed(&tio, B9600);

	tcsetattr(tty_fd, TCSANOW, &tio);

	// use software version to determine if compass is connected
	if( doSend( tty_fd, 0x11, 0x0f ) ) { printf("ERROR after software version check, probably compass not available\n"); return 1; }

	if( reset ) {
		printf( "compass device reset to default values\n");
		if( doSend( tty_fd, 0x6a, 0x55 ) ) { printf("ERROR during reset to defaults\n"); return 1; }
		if( doSend( tty_fd, 0x7c, 0x55 ) ) { printf("ERROR during reset to defaults\n"); return 1; }
		if( doSend( tty_fd, 0x81, 0x55 ) ) { printf("ERROR during reset to defaults\n"); return 1; }
	}

	if (configure) {
		if( doSend( tty_fd, 0x31, 0x55 ) ) { printf("ERROR during configure\n"); return 1; }
		if( doSend( tty_fd, 0x45, 0x55 ) ) { printf("ERROR during configure\n"); return 1; }
		if( doSend( tty_fd, 0x5a, 0x55 ) ) { printf("ERROR during configure\n"); return 1; }

		printf("calibrate compass device, put shooter to north\n");
		usleep(5000000);
		if( doSend( tty_fd, 0x5e, 0x55 ) ) { printf("ERROR during configure\n"); return 1; }
		printf("put shooter to 90 degrees, red led should be active now\n");
		usleep(5000000);
		if( doSend( tty_fd, 0x5e, 0x55 ) ) { printf("ERROR during configure\n"); return 1; }
		printf("put shooter to 180 degrees\n");
		usleep(5000000);
		if( doSend( tty_fd, 0x5e, 0x55 ) ) { printf("ERROR during configure\n"); return 1; }
		printf("put shooter to 270 degrees\n");
		usleep(5000000);
		if( doSend( tty_fd, 0x5e, 0x55 ) ) { printf("ERROR during configure\n"); return 1; }
		printf("calibration done\n");
	}

	sendVal = 0x12; // write value 0x12 to serial port to get 8 bit compass value
	while( 1 ) {
		write(tty_fd, &sendVal, 1);
		if (read(tty_fd, &receiveVal, 1) > 0) {
			printf( "value 0x%02x (%3d) = %3.0f degrees\n", receiveVal, receiveVal, 360.0*receiveVal/256.0);
		}
		if( ! fast ) { usleep( 100000 ); }
	}

	close(tty_fd);
}
