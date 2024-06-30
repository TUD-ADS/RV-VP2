/*
 * magic.hpp
 *
 *  Created on: Oct 19, 2022
 *      Author: dwd
 */

#include "network_io.hpp"
#include <fcntl.h>
#include <inttypes.h>
#include <cstdio>
#include <errno.h>	// errno
#include <string.h>
#include <termios.h> // Contains POSIX terminal control definitions

bool setBaudrate(int handle, unsigned baudrate) {
	struct termios tty;
	if(tcgetattr(handle, &tty) != 0) {
		printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
		return false;
	}
	if(cfsetspeed(&tty, baudrate) != 0) {
		printf("Error %i from setting baudrate: %s\n", errno, strerror(errno));
		printf("... is the device a tty?\n");
		return false;
	}
	// Save tty settings, also checking for error
	if (tcsetattr(handle, TCSANOW, &tty) != 0) {
		printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
		return false;
	}
	return true;
}

bool setTTYRawmode(int handle) {

	if(tcflush(handle, TCIOFLUSH) != 0) {
		printf("Error %i flushing handle: %s\n", errno, strerror(errno));
	}

	struct termios tty;
	if(tcgetattr(handle, &tty) != 0) {
		printf("Error %i getting tcgetattr: %s\n", errno, strerror(errno));
		return false;
	}

	cfmakeraw(&tty);

	/* in _noncanonical mode_
	  MIN == 0, TIME > 0 (read with timeout)
	  TIME specifies the limit for a timer in tenths of a
	  second.  The timer is started when read(2) is called.
	  read(2) returns either when at least one byte of data is
	  available, or when the timer expires.  If the timer
	  expires without any input becoming available, read(2)
	  returns 0.  If data is already available at the time of
	  the call to read(2), the call behaves as though the data
	  was received immediately after the call.
	*/
	tty.c_cc[VMIN] = 0;
	tty.c_cc[VTIME] = 10;

	// Save tty settings, also checking for error
	if (tcsetattr(handle, TCSANOW, &tty) != 0) {
		printf("Error %i applying raw mode TTY from tcsetattr: %s\n", errno, strerror(errno));
		return false;
	}

	return true;
}
