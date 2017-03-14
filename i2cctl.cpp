// implementation of the i2c controller
//
// basically stolen/guided from sparkfun (great company)
// https://learn.sparkfun.com/tutorials/programming-the-pcduino/i2c-communications
// http://www.frank-buss.de/io-expander/linux-i2c-test.c
//
// Modified to be not stolen
//
// by Mark Hill

#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <sstream>

#include "i2cctl.h"

#ifdef RELEASE
int debug = 0;
#else
int debug = 1;
#endif

using namespace std;


// sorry for the global. Forgive me for I know not what I do
// defaults to 1 because lets face it, thats normal
uint8_t _bus = 1;
int _i2cFile = -1000;

// this variable is set to 1 when the i2c device is being used to do a read or write operation
// if this weren't set, programs running on a separate thread trying to write to i2c devices
//   could potentially interfere with and corrupt other concurrent operations
// 1 = locked, 0 = open
int _lock = 0;

// this is a wait function and returns once the lock is removed to allow the i2c function to access the device
void getLock() {
	while(_lock) {
		
	}
	_lock = 1;
}

// this frees the lock to allow another function to use the i2c device
void releaseLock() {
	_lock = 0;
}


// initializes the _i2cFile variable which makes the i2c bus avaliable for reading and writing
// no need for this to be run outside this file, so its private.  User access can be provided by
//   running the i2cSetBus function, which will call this function
// returns the i2c file descriptor
int i2cInit() {
	// this disrupts access to the i2c device, so obviously it sole access via the lock
	getLock();

	if (_i2cFile < 0) {
		char i2cBusName[12];
		sprintf(i2cBusName, "/dev/i2c-%d", _bus);

		_i2cFile = open(i2cBusName, O_RDWR);
		
		// this means an error has occured
		if (_i2cFile < 0) {
			if (debug == 1) {
				printf("Error opening i2c file: %d\nIn function i2cInit in i2cctl.cpp\n", _i2cFile);
			}
		}
	}

	releaseLock();
	return _i2cFile;
}




// sets the i2c device address and also configures the i2c device to take 10 bit or 8 bit addresses
// returns 0 for success and something else for error
int i2cSetAddress(uint16_t address) {
	// in case the bus was never set, this ensures the i2c device is always initialized
	i2cInit();
	
	// needs lock access because this will disrupt concurrent operations
	getLock();

	// set ten bit address mode
	uint8_t isTenBit = (address - 128 > 0) ? 1 : 0;
	int set10Bit = ioctl(_i2cFile, I2C_TENBIT, isTenBit);
	
	// this means an error has occured
	if (set10Bit < 0) {
		if (debug == 1) {
			printf("Failed to set 10 bit mode with error code %d in i2cctl.cpp\n", set10Bit);
		}

		releaseLock();
		return set10Bit;
	}

	// set the address of the slave to talk to 
	int setSlave = ioctl(_i2cFile, I2C_SLAVE, address);
	
	// this means an error has occured
	if (setSlave < 0) {
		if (debug == 1) {
			printf("Failed to set slave address to %x with error %d in i2cctl.cpp\n", address, setSlave);
		}

		releaseLock();
		return setSlave;
	}	

	releaseLock();
	return 0;
}




// set the bus used for i2c communication
bool i2cSetBus(uint8_t bus) {
	// obviously needs the lock so it gets sole access to the i2c device
	getLock();

	_bus = bus;

	// finished with the lock before most functions, and i2cInit needs the lock, so it gets released
	releaseLock();
	
	// reinitialize the i2c when the desired bus changes
	i2cInit();

	// idc that this is unnecessary, I made it bool so itll return bool because I like bool
	return (_bus == bus && _i2cFile >= 0);
}




// single or multiple byte read
uint32_t i2cRead(uint16_t address, uint8_t reg[], uint8_t numRegisters) {
	// set address of i2c device and then check if it failed
	if (i2cSetAddress(address) != 0) {
		if (debug == 1) {
			printf("Failed to set device address %x in i2cctl.cpp\n", address); 
		}
		return 0;
	}
	
	// loops through all the registers and sets the data returned from the read operation on each register
	//   to the correct position in the 'result' integer
	// also needs the lock during this loop
	getLock();
	uint32_t result = 0;
	for (int regIndex = 0; regIndex < numRegisters; regIndex++) {
		// sets the curReg temporary variable
		uint8_t curReg = reg[regIndex];

		// write the address of the register to be read (the first step in the i2c operation is actually to 
		//   write to the slave the value of the i2c register you want to read from) and then check if it failed
		if (write(_i2cFile, &curReg, 1) < 0) {
			if (debug == 1) {
				printf("Failed to set device register %x at address %x in i2cctl.cpp\n", curReg, address);
			}

			releaseLock();
			return 0;
		}
	
		// if you got this far, its time to actually perform a read and then, as you can guess, check if it failed
		uint32_t data = 0;
		if (read(_i2cFile, &data, 1) < 0) {
			if (debug == 1) {
				printf("Failed to read from register %x at address %x in i2cctl.cpp\n", curReg, address);
			}

			releaseLock();
			return 0;
		}

		// offset the bits in the data variable so that it can be addedd to 'result'
		int offsets = numRegisters - (regIndex + 1);
		data <<= offsets * 8;
		
		result += data;
	}

	releaseLock();
	return result;
}




// single or multiple byte write
int i2cWrite(uint16_t address, uint8_t reg[], uint8_t numRegisters,  uint32_t value) {	
	// set address of i2c device and then check if it failed
	if (i2cSetAddress(address) != 0) {
		if (debug == 1) {
			printf("Failed to set device address %x in i2cctl.cpp\n", address); 
		}
		return 0;
	}
	
	// loops through all the registers and sets them to the correct position in the 'result' integer
	// also needs the lock during this loop
	getLock();

	for (int regIndex = 0; regIndex < numRegisters; regIndex++) {
		// retrieves the byte to be written from within the value variable and puts it in a temporary variable
		uint32_t mask = 0x000000ff;
		int offsets = numRegisters - (regIndex + 1);
		uint8_t writeValue = (value << offsets * 8) & mask;
		
		printf("writing value %x obtained from a mask of %x on the original value %x\n", writeValue, mask, value);

		// sets the curReg temporary variable
		uint8_t curReg = reg[regIndex];

		uint8_t data[2] = {curReg, writeValue};

		// write the address of the register to be written (the first step in the i2c operation is actually to 
		//   write to the slave the value of the i2c register you want to write to), then write the byte from 
		//   the variable 'writeValue', then check if the whole operation failed
		if (write(_i2cFile, data, 2) < 0) {
			if (debug == 1) {
				printf("Failed to write %x to device register %x at address %x in i2cctl.cpp\n", writeValue, curReg, address);
			}

			releaseLock();
			return 0;
		}
	}

	releaseLock();
	return 1;
}


	







