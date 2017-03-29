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
#include<stdio.h>
#include<unistd.h>

#include "SensorManager.h"
#include "i2cctl.h"


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
	int incrementSuccess = i2cWrite(accelAddress, autoIncrementRegister, 1, autoIncrement, AUTO_INCREMENT_ENABLED);
	
	if (incrementSuccess != 0) {
		printf("Failed to set auto increment for accelerometer\n");
	}
	
	uint8_t accelConfigRegisters[1] = {0x10};
	// sets the accelerometer to 1.6kHz mode with a range of +-16g
	uint8_t accelConfig = 0x84;
	int accelSuccess = i2cWrite(accelAddress, accelConfigRegisters, 1, accelConfig, AUTO_INCREMENT_ENABLED);
	
	if (accelSuccess != 0) {
		printf("Failed to set i2c configuration for accelerometer\n");
	}

	//
	// gyroscope section
	//
	uint8_t gyroConfigRegister[1] = {0x11};
	// sets the gyroscope to 1.6kHz mode with a scale of 2000 degrees per second
	uint8_t gyroConfig = 0x8c;
	int gyroSuccess = i2cWrite(gyroAddress, gyroConfigRegister, 1, gyroConfig, AUTO_INCREMENT_ENABLED);

	// sets the gyroscope high pass filter on and sets the filter cutoff frequency
	//   to 2.07Hz with rounding enabled
	gyroConfigRegister[0] = 0x16;
	gyroConfig = 0x64;
	gyroSuccess |= i2cWrite(gyroAddress, gyroConfigRegister, 1, gyroConfig, AUTO_INCREMENT_ENABLED);

	if (gyroSuccess != 0) {
		printf("Failed to set i2c configuration for gyroscope\n");
	}
	
	//
	// magnetometer section
	//
	
	// annoyingly, the device must be put to sleep when changing settings
	// first, set the device to sleep
	uint8_t magConfigRegister1 = 0x10;
	uint8_t magSleepValue = 0x00;

	int magSuccess = i2cWrite(magAddress, &magConfigRegister1, 1, magSleepValue, AUTO_INCREMENT_ENABLED);

	// now set the register values
	uint8_t magConfigRegisters[2] = {0x10, 0x11};
	uint16_t magConfig = 0x0920;

	magSuccess |= i2cWrite(magAddress, magConfigRegisters, 2, magConfig, AUTO_INCREMENT_ENABLED);

	// finally, wake up the magnetometer again
	uint8_t magWakeValue = 0x09;

	magSuccess |= i2cWrite(magAddress, &magConfigRegister1, 1, magWakeValue, AUTO_INCREMENT_ENABLED);

	if (magSuccess != 0) {
		printf("Failed to set i2c configuration for magnetometer\n");
	}
	
	//
	// barometer section
	//
	


	// let power stabilize with a wait
	usleep(50);
	_sensorsAvailable = 1;
}


// set sensors to sleep mode
void deinitializeSensors() {
	//
	// accelerometer and gyroscope section
	//

	// expanded for readability, local variables for the configRegisters
	uint8_t accelConfigRegister = 0x10;
	uint8_t gyroConfigRegister = 0x11;

	uint8_t registers[] = {accelConfigRegister, gyroConfigRegister};

	// since I want to turn the sensors off, I can just write straight zeros
	uint16_t writeValue = 0x0000;

	// perform the actual write and check for errors
	int success = i2cWrite(accelAddress, registers, 2, writeValue, AUTO_INCREMENT_ENABLED);

	if (success != 0) {
		printf("Failed to deinitialize accelerometer and gyroscope sensors\n");
	}

	//
	// magnetometer section
	//
	uint8_t magConfigRegisters[2] = {0x11, 0x12};
	uint16_t magConfig = 0x0800;

	int magSuccess = i2cWrite(magAddress, magConfigRegisters, 2, magConfig, AUTO_INCREMENT_ENABLED);

	if (magSuccess != 0) {
		printf("Failed to set i2c configuration for magnetometer\n");
	}


}


// values are usually returned in two's complement
// this returns the signed value from the raw two's complement input
int32_t signedValue16bit(uint32_t _unsignedValue) {
	// maximum positive value used for comparison
	uint32_t maxPos = 32767;
	// maximum value the number can take based on the number of bits
	// in this case, I am using 16 bit values
	uint32_t maxNeg = 65536;
	
	// the value to be returned, assuming it is positive
	int result = _unsignedValue;
	if (_unsignedValue > maxPos) {
		result = -1 * (maxNeg - result);
	}

	return result;
}


// converts from a signed value into a two's complement value
uint32_t unsignedValue16bit(int _signedValue) {
	// maximum value the number can take based on the number of bits
	// in this case, I am using 16 bit values
	uint32_t maxNeg = 65536;
	
	uint32_t result = _signedValue;

	if (_signedValue < 0) {
		result = maxNeg - (-1 * _signedValue) + 1;
	}

	return result;
}

// this is a generalized function used by the acceleration vector,
//   rotation vector, and magnetometer vector functions since they have
//   similar hardware interfaces
// place registers in x, y, z order with low before high registers
// with the exception of the last argument, all arguments are for the i2c operation
//   see the documentation in i2cctl.h for information on what the values do
//   it only applies for 16 bit values right nowm set to 0 if using 32 bit or 8 bit sizes
// the last argument, divisor, is used to correct for the fact that decimal values
//   must be stored as integers by the registers, so the decimal point must be shifted
struct Vec3double threeAxisVector(uint16_t address, uint8_t *registers, uint8_t numRegisters, uint8_t bytesPerValue, uint8_t autoIncrementEnabled, uint8_t highByteFirst, double divisor) {
	if (numRegisters / bytesPerValue > 3 || numRegisters / bytesPerValue <= 0) {
		printf("Invalid register count for function threeAxisVector\n");
		return vectorFromComponents(0, 0, 0);
	}
	
	// initialize the sensors if they haven't been already
	if (!_sensorsAvailable) {
		initializeSensors();
	}
	
	// retrieves the sensor values based on the passed in arguments
	uint32_t vectorValues[numRegisters / bytesPerValue];
	int success = i2cWordRead(address, registers, numRegisters, vectorValues, bytesPerValue, highByteFirst, autoIncrementEnabled);

	// check for failure and return an empty vector if so
	if (success != 0) {
		printf("Read failed in threeAxisVector for device %x", address);
		return vectorFromComponents(0, 0, 0);
	}

	// convert the values from unsigned two's complement to a signed int
	int rawx = signedValue16bit(vectorValues[0]);
	int rawy = signedValue16bit(vectorValues[1]);
	int rawz = signedValue16bit(vectorValues[2]);

	double x = rawx / divisor;
	double y = rawy / divisor;
	double z = rawz / divisor;

	return vectorFromComponents(x, y, z);
}

// gets the linear acceleration from the gyroscope
struct Vec3double accelerationVector() {	
	// the hard-coded register addresses for the accelerometer (L,H,L,H,L,H) (x,y,z)
	uint8_t accelRegisters[6] = {0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d};
	
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

	// create an even more user-friendly acceleration vector
	struct Vec3double acceleration = threeAxisVector(accelAddress, accelRegisters, 6, WORD_16_BIT, AUTO_INCREMENT_ENABLED, LOW_BYTE_FIRST, divisor);

	return acceleration;
}



// returns the vector containing the angular rotation rate of the gyroscope
// also if you were wondering, yes I did just copy paste the acceleration function
//   and then change variable names and comments.  It's necessary boilerplate, 
//   the sensors are on the same chip, and I don't care if you know
struct Vec3double rotationVector() {	
	// the hard-coded register addresses for the gyroscope (L,H,L,H,L,H) (x,y,z)
	uint8_t gyroRegisters[6] = {0x22, 0x23, 0x24, 0x25, 0x26, 0x27};
	
	// scale is given as the +- dps range for the gyroscope
	// with a +- 2000 dps scale and a 16 bit output integer, the raw value should be 
	//   divided by 2^(bits - log2(range)) which comes out to be 2^(15 - ceil(log2(2000)))
	//   which equals 2^4 = 16
	// why 15? well remember, the raw 16 bit register output has to be converted to a signed value
	//   two's complement effectively uses a bit to encode sign, so we lost a bit for the total amount
	// for the sake of efficiency, this value is hardcoded, but it is important to know how it was 
	//   determined for future adaptation
	//
	// looking back, I have no idea how this divisor became 32 when the math says 16
	// dont ask me
	int divisor = 32;

	// create an even more user-friendly rotation vector
	struct Vec3double rotation = threeAxisVector(gyroAddress, gyroRegisters, 6, WORD_16_BIT, AUTO_INCREMENT_ENABLED, LOW_BYTE_FIRST, divisor);

	return rotation;
}


// returns the vector describing the magnetic field
// vector axises (no idea how to make axis plural) are the same as the accelerometer axises
struct Vec3double magneticField() { 
	// the hard-coded register addresses for the magnetometer (L,H,L,H,L,H) (x,y,z)
	uint8_t magRegisters[6] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
	
	// scale is given as the +- micro telsas (uT) range for the magnetometer
	// with a +- 1000uT scale and a 16 bit output integer, the raw value should be 
	//   divided by 2^(bits - log2(range)) which comes out to be 2^(15 - ceil(log2(1000)))
	//   which equals 2^5 = 32
	// however, experimentally, that gives the wrong value, but the number below does work
	//   dont ask me why, it just works
	// why 15? well remember, the raw 16 bit register output has to be converted to a signed value
	//   two's complement effectively uses a bit to encode sign, so we lost a bit for the total amount
	// for the sake of efficiency, this value is hardcoded, but it is important to know how it was 
	//   determined for future adaptation
	int divisor = 1024;

	// create an even more user-friendly magnetic field vector
	struct Vec3double magneticField = threeAxisVector(magAddress, magRegisters, 6, WORD_16_BIT, AUTO_INCREMENT_ENABLED, HIGH_BYTE_FIRST, divisor);

	return magneticField;
}




// gets the altitude relative to the take-off height
double barometerAltitude() {
	
	return 0;
}




