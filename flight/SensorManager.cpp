// implementation of the sensor manager header
//
// this is one of the few files that may have to be rewritten for each new drone
// configuration values taken from the data sheet located at the following URLs
//
// accelerometer and gyroscope
// http://www.st.com/content/ccc/resource/technical/document/datasheet/a3/f5/4f/ae/8e/44/41/d7/DM00133076.pdf/files/DM00133076.pdf/jcr:content/translations/en.DM00133076.pdf
// 
// magnetometer
//
//
// barometer
// 
//
// by Mark Hill
#include<iostream>
#include<stdio.h>
#include<unistd.h>

#include "SensorManager.h"
#include "i2cctl.h"


using namespace std;


uint8_t accelAddress = 0x6b;
uint8_t gyroAddress = 0x6b;
uint8_t magAddress = 0x0e;
uint8_t barometerAddress= 0x77;

// this value is 0 when sensors have not been initialzed and is set to
//   1 by the initialize sensors function, indicating sensors are configured
uint8_t _sensorsAvailable = 0;

void initializeSensors() {
	//
	// accelerometer section
	//
		
	// enabled auto increment on the register addresses
	uint8_t autoIncrementRegister[1] = {0x12};
	uint8_t autoIncrement = 0x04;
	int incrementSuccess = i2cWrite(accelAddress, autoIncrementRegister, 1, autoIncrement, AUTO_INCREMENT_DISABLED);
	
	if (incrementSuccess != 0) {
		printf("Failed to set auto increment for accelerometer\n");
	}
	
	uint8_t accelConfigRegisters[1] = {0x10};
	// sets the accelerometer to 1.6kHz mode with a range of +-16g
	uint8_t accelConfig = 0x84;
	int accelSuccess = i2cWrite(accelAddress, accelConfigRegisters, 1, accelConfig, AUTO_INCREMENT_DISABLED);
	
	if (accelSuccess != 0) {
		printf("Failed to set i2c configuration for accelerometer\n");
	}

	//
	// gyroscope section
	//
	uint8_t gyroConfigRegister[1] = {0x11};
	// sets the gyroscope to 1.6kHz mode with a scale of 2000 degrees per second
	uint8_t gyroConfig = 0x8c;
	int gyroSuccess = i2cWrite(gyroAddress, gyroConfigRegister, 1, gyroConfig, AUTO_INCREMENT_DISABLED);

	// sets the gyroscope high pass filter on and sets the filter cutoff frequency
	//   to 2.07Hz with rounding enabled
	gyroConfigRegister[0] = 0x16;
	gyroConfig = 0x64;
	gyroSuccess |= i2cWrite(gyroAddress, gyroConfigRegister, 1, gyroConfig, AUTO_INCREMENT_DISABLED);

	if (gyroSuccess != 0) {
		printf("Failed to set i2c configuration for gyroscope\n");
	}
	
	//
	// magnetometer section
	//
	//uint8_t magConfigRegister[1] = {0x00};
	//

	usleep(500);
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


// gets the linear acceleration from the gyroscope
Vec3double accelerationVector() {	
	// initializes the sensors if they have not been already
	if (!_sensorsAvailable) {
		initializeSensors();
	}

	// the hard-coded register addresses for the accelerometer (L,H,L,H,L,H) (x,y,z)
	uint8_t accelRegisters[6] = {0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d};
	
	// retrieve the raw sensor data from the registers
	uint32_t accelValues[3] = {0,0,0};
	int success = i2cWordRead(accelAddress, accelRegisters, 6, accelValues, WORD_16_BIT, AUTO_INCREMENT_ENABLED);
	if (success == -1) {
		printf("Reading Values from accelerometer failed in Sensor Manager\n");
	}

	// proccess the sensor data and turn it into a user-friendly acceleration value
	int xaccelInt = signedValue16bit(accelValues[0]);
	int yaccelInt = signedValue16bit(accelValues[1]);
	int zaccelInt = signedValue16bit(accelValues[2]);
	
	// convert the signed integer form into double form based on the configured accelerometer range
	// scale is given as the +- g range for the accelerometer
	// with a +- 16g scale and a 16 bit output integer, the raw value should be 
	//   divided by 2^(bits - log2(range)) which comes out to be 2^(15 - log2(16))
	//   which equals 2^11 = 2048
	// why 15? well remember, the raw 16 bit register output has to be converted to a signed value
	//   two's complement effectively uses a bit to encode sign, so we lost a bit for the total amount
	// for the sake of efficiency, this value is hardcoded, but it is important to know how it was 
	//   determined for future adaptation
	int divisor = 2048;

	double xaccel = (double) xaccelInt / (double) divisor;
	double yaccel = (double) yaccelInt / (double) divisor;
	double zaccel = (double) zaccelInt / (double) divisor;

	// create an even more user-friendly acceleration vector
	Vec3double acceleration = Vec3double(xaccel, yaccel, zaccel);

	return acceleration;
}


// returns the vector containing the angular rotation rate of the gyroscope
// also if you were wondering, yes I did just copy paste the acceleration function
//   and then change variable names and comments.  It's necessary boilerplate, 
//   the sensors are on the same chip, and I don't care if you know
Vec3double rotationVector() {	
	// initializes the sensors if they have not been already
	if (!_sensorsAvailable) {
		initializeSensors();
	}

	// the hard-coded register addresses for the gyroscope (L,H,L,H,L,H) (x,y,z)
	uint8_t gyroRegisters[6] = {0x22, 0x23, 0x24, 0x25, 0x26, 0x27};
	
	// retrieve the raw sensor data from the registers
	uint32_t gyroValues[3] = {0,0,0};
	int success = i2cWordRead(gyroAddress, gyroRegisters, 6, gyroValues, WORD_16_BIT, AUTO_INCREMENT_ENABLED);
	if (success == -1) {
		printf("Reading Values from gyroscope failed in Sensor Manager\n");
	}
	
	// proccess the sensor data and turn it into a user-friendly rotation value
	int xgyroInt = signedValue16bit(gyroValues[0]);
	int ygyroInt = signedValue16bit(gyroValues[1]);
	int zgyroInt = signedValue16bit(gyroValues[2]);

	// convert the signed integer form into double form based on the configured gyroscope range
	// scale is given as the +- dps range for the gyroscope
	// with a +- 2000 dps scale and a 16 bit output integer, the raw value should be 
	//   divided by 2^(bits - log2(range)) which comes out to be 2^(15 - ceil(log2(2000)))
	//   which equals 2^4 = 16
	// why 15? well remember, the raw 16 bit register output has to be converted to a signed value
	//   two's complement effectively uses a bit to encode sign, so we lost a bit for the total amount
	// for the sake of efficiency, this value is hardcoded, but it is important to know how it was 
	//   determined for future adaptation
	int divisor = 32;

	double xrotation = (double) xgyroInt / (double) divisor;
	double yrotation = (double) ygyroInt / (double) divisor;
	double zrotation = (double) zgyroInt / (double) divisor;

	// create an even more user-friendly rotation vector
	Vec3double rotation = Vec3double(xrotation, yrotation, zrotation);

	return rotation;
}



// returns the vector describing the magnetic field
// vector axises (no idea how to make axis plural) are the same as the accelerometer axises
Vec3double magneticField() { 
	return Vec3double(0,0,0);
}




// gets the altitude relative to the take-off height
double barometerAltitude() {
	return 0;
}




