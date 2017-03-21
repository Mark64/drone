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
#include <stdio.h>
#include <sstream>
#include <pthread.h>

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

// this mutex is used to protect the i2c device from being used to do a read or write operations
//   simulateously on another thread
// if this weren't used, programs running on a separate thread trying to write to i2c devices
//   could potentially interfere with and corrupt other concurrent operations
// the other _mutex_created variable allows the mutex to be initialized without a discrete initialize function
// 1 = created, 0 = not created
pthread_mutex_t _lock;
int _mutex_created = 0;

// this is a wait function and returns once the lock is removed to allow the i2c function to access the device
void getLock() {
	if (!_mutex_created) {
		pthread_mutex_init(&_lock, NULL);
		_mutex_created = 1;
	}
	pthread_mutex_lock(&_lock);
}

// this frees the lock to allow another function to use the i2c device
void releaseLock() {
	pthread_mutex_unlock(&_lock);
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




// closes the i2c file
void i2cClose() {
	// gets the lock before it murders the i2c file
	// this seems vaguely poetic
	getLock();

	// closes the i2c file
	close(_i2cFile);
	_i2cFile = -1000;

	// releases the lock for the now useless i2c file
	releaseLock();
	pthread_mutex_destroy(&_lock);
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

// Slightly more efficient but highly specific version of i2cRead for dealing 
//   with sampling from the accelerometer and gyro or any other device with
//   16 bit words split across a high and low register
int i2cMultiWordRead(uint16_t address, uint8_t reg[], uint8_t numRegisters, uint32_t *readResults) {
	int numSamples = 1;
	// set address of i2c device and then check if it failed
	if (i2cSetAddress(address) != 0) {
		if (debug == 1) {
			printf("Failed to set device address %x in i2cctl.cpp\n", address); 
		}
		return -1;
	}
	
	// lock is needed for the rest of the function since reads and writes will
	//   take place
	getLock();
	
	// outer loop deals with the whole word being read
	// it loops 'numRegisters/2' times, so reads 2 bytes per value
	// result is placed into the index of 'readResults' that corresponds to the 'readIndex' variable
	for (int readIndex = 0; readIndex < numRegisters/2; readIndex++) {
		
		// this variable will store the result of the read operation
		uint32_t result = 0;

		// now loop through twice to read from the two registers that make up the 
		//   16 bit value to be placed into the variable 'result' above
		for (int regIndex = 0; regIndex < 2; regIndex++) {
			
			// sets the curReg temporary variable based on the current readIndex and regIndex
			uint8_t curReg = reg[regIndex + (2*readIndex)];
	
			// if you got this far, its time to actually perform a read and then, as you can guess, check if it failed
			// in this version of i2cRead, we actually want to sample multiple times prior to determining and
			//   returning a value
			// by returning the average of n='numSuccesses' samples, the data will be less prone to
			//   outliers (Yes, I did pay attention in your class Mrs. Romero and I know the mean is
			//   not very resistant, but its the most efficient way to remove outliers in terms of the
			//   amount of computing involved)
			uint32_t sampleSum = 0;
			// this is used as a way to deal with potential one time errors in reading
			uint8_t numSuccesses = 0;
			for (int n = 0; n < numSamples; n++) {
				
				// write the address of the register to be read (the first step in the i2c operation is actually to 
				//   write to the slave the value of the i2c register you want to read from) and then check if it failed
				if (write(_i2cFile, &curReg, 1) < 0) {
					if (debug == 1) {
						printf("Failed to set device register %x at address %x in i2cctl.cpp\n", curReg, address);
					}
				}
				// this means register address setting was successful
				// this is where the data is actually read and the success is checked
				else {
					uint32_t readData = 0;
					if (read(_i2cFile, &readData, 1) < 0) {
						if (debug == 1) {
							printf("Failed to read from register %x at address %x in i2cctl.cpp\n", curReg, address);
						}
	
						printf("failed to read %d sample from register %x\n", n, curReg);
					}
				
					// if the read was actually successful, then increment the successes
					// no use in adding junk data, so thats why both are in the else
					else {
						sampleSum += readData;
						numSuccesses++;
					}
				}
			}
			// get the average which will become the value used
			uint32_t data = sampleSum/numSuccesses;
			// offset the bits in the data variable so that it can be addedd to 'result'
			int offsets = 2 - (regIndex + 1);
			data <<= offsets * 8;
			
			result += data;
		}
		readResults[readIndex] = result;
	}

	// lock done
	releaseLock();
	return 0;
}



// single or multiple byte write
int i2cWrite(uint16_t address, uint8_t reg[], uint8_t numRegisters,  uint32_t value) {	
	// set address of i2c device and then check if it failed
	if (i2cSetAddress(address) != 0) {
		if (debug == 1) {
			printf("Failed to set device address %x in i2cctl.cpp\n", address); 
		}
		return -1;
	}
	
	// loops through all the registers and sets them to the correct position in the 'result' integer
	// also needs the lock during this loop
	getLock();

	for (int regIndex = 0; regIndex < numRegisters; regIndex++) {
		// retrieves the byte to be written from within the value variable and puts it in a temporary variable
		uint32_t mask = 0x000000ff;
		int offsets = numRegisters - (regIndex + 1);
		uint8_t writeValue = (value >> offsets * 8) & mask;
		
		//printf("writing value %x obtained from a mask of %x on the original value %x\n", writeValue, mask, value);

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
			return -1;
		}
	}

	releaseLock();
	return 0;
}


	







