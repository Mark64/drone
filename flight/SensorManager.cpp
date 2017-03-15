// implementation of the sensor manager header
//
// this is one of the few files that may have to be rewritten for each new drone
// configuration values taken from the data sheet located at
// http://www.st.com/content/ccc/resource/technical/document/datasheet/a3/f5/4f/ae/8e/44/41/d7/DM00133076.pdf/files/DM00133076.pdf/jcr:content/translations/en.DM00133076.pdf
//
// by Mark Hill
#include<iostream>
#include<stdio.h>

#include "SensorManager.h"
#include "i2cctl.h"


using namespace std;


uint8_t accelAddress = 0x6b;
uint8_t gyroAddress = 0x6b;

// this value is 0 when sensors have not been initialzed and is set to
//   1 by the initialize sensors function, indicating sensors are configured
uint8_t _sensorsAvailable = 0;

void initializeSensors() {
	//
	// accelerometer section
	//
	uint8_t accelConfigRegisters[1] = {0x10};
	// sets the accelerometer to 1.6kHz mode with a range of +-16g
	uint8_t accelConfig = 0x84;
	int accelSuccess = i2cWrite(accelAddress, accelConfigRegisters, 1, accelConfig);
	
	if (accelSuccess != 0) {
		printf("Failed to set i2c configuration for accelerometer\n");
	}

	//
	// gyroscope section
	//
	uint8_t gyroConfigRegisters[1] = {0x11};
	// sets the gyroscope to 1.6kHz mode with a scale of 2000 degrees per second
	uint8_t gyroConfig = 0x8c;
	int gyroSuccess = i2cWrite(gyroAddress, gyroConfigRegisters, 1, gyroConfig);

	if (gyroSuccess != 0) {
		printf("Failed to set i2c configuration for gyroscope\n");
	}

	_sensorsAvailable = 1;
}




// values are usually returned in two's complement
// this returns the signed value from the raw two's complement input
int16_t signedValue16bit(uint32_t twoComplement) {
	// maximum positive value used for comparison
	uint32_t maxPos = 32767;
	// maximum value the number can take based on the number of bits
	// in this case, we are using 16 bit values
	uint32_t maxNeg = 65536;
	
	// the value to be returned, assuming it is positive
	int16_t result = twoComplement;
	if (twoComplement > maxPos) {
		result = -1 * (maxNeg - result);
	}

	return result;
}

Vec3double accelerationVector() {	
	// initializes the sensors if they have not been already
	if (!_sensorsAvailable) {
		initializeSensors();
	}
	// the hard-coded register addresses for the accelerometer
	uint8_t accelXRegisters[2] = {0x29, 0x28};
	uint8_t accelYRegisters[2] = {0x2b, 0x2a};
	uint8_t accelZRegisters[2] = {0x2d, 0x2c};
	
	// retrieve the raw sensor data from the registers
	uint32_t rawXaccel = i2cRead(accelAddress, accelXRegisters, 2);
	uint32_t rawYaccel = i2cRead(accelAddress, accelYRegisters, 2);
	uint32_t rawZaccel = i2cRead(accelAddress, accelZRegisters, 2);

	// proccess the sensor data and turn it into a user-friendly acceleration value
	int xaccelInt = signedValue16bit(rawXaccel);
	int yaccelInt = signedValue16bit(rawYaccel);
	int zaccelInt = signedValue16bit(rawZaccel);

	// convert the signed integer form into double form based on the configured accelerometer range
	// scale is given as the +- range for the accelerometer
	int scale = 16;

	// yeah so about the 127...
	//   I got that value because when I arranged the chip in an almost perfectly flat manner, the Z
	//   acceleration was ~127.  There was not a single bit of information about that in the datasheet
	//   so it could very well be wrong, but it seems to work experimentally
	double xaccel = (double) xaccelInt / (double) scale / 127;
	double yaccel = (double) yaccelInt / (double) scale / 127;
	double zaccel = (double) zaccelInt / (double) scale / 127;

	// create an even more user-friendly acceleration vector
	Vec3double acceleration = Vec3double(xaccel, yaccel, zaccel);

	return acceleration;
}




// compass heading retrieval
double compassHeading() { 
	return 0;
}




// gets the altitude relative to the take-off height
double barometerAltitude() {
	return 0;
}




