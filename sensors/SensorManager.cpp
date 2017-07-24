// implementation of the sensor manager header
//
// this is one of the few files that may have to be rewritten for each new system
// this imeplementation is for the cleanDrone
//
// configuration values taken from the data sheet located at the following URLs
//
// accelerometer and gyroscope
// http://www.st.com/content/ccc/resource/technical/document/datasheet/a3/f5/4f/ae/8e/44/41/d7/DM00133076.pdf/files/DM00133076.pdf/jcr:content/translations/en.DM00133076.pdf
//
// magnetometer
// http://www.nxp.com/assets/documents/data/en/data-sheets/MAG3110.pdf
//
// barometer
// https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf
//
// by Mark Hill
#include<stdio.h>
#include<unistd.h>
#include<stdint.h>
#include<math.h>

#include<Eigen/Dense>
#include<SensorManager.h>
extern "C" {
	#include<i2cctl.h>
}

using namespace Eigen;

static const uint8_t accelAddress = 0x6b;
static const uint8_t gyroAddress = 0x6b;
static const uint8_t magAddress = 0x0e;
static const uint8_t barometerAddress= 0x77;

// the barometer puts all the calculation responsibility on the user,
//   so you have to retrieve and store the calibration values to calculate the
//   pressure and temperature
static int32_t baroVals[11];
// internal function used to retrieve barometer values
void getBarometerParameters();

// this value is 0 when sensors have not been initialzed and is set to
//   1 by the initialize sensors function, indicating sensors are configured
static uint8_t _sensorsAvailable = 0;

// converts signed value into two's complement form
uint32_t unsignedValue(int _signedValue, uint8_t numBits);
// converts two's complement unsigned value into signed value
int32_t signedValue(uint32_t _unsignedValue, uint8_t numBits);

int initializeSensors() {
	// no need to run if sensors already initialized
	if (_sensorsAvailable == 1) {
		return 0;
	}

	//
	// accelerometer section
	//
	// enabled auto increment on the register addresses
	uint8_t autoIncrementData[2] = {0x12, 0x06};
	int incrementSuccess = i2c_write(accelAddress, autoIncrementData, 2);
	if (incrementSuccess != 0) {
		printf("Failed to set auto increment for accelerometer\n");
		return -1;
	}

	uint8_t data[] = {0x10, 0x6b, 0x64, 0x06, 0x80, 0x00, 0x00, 0x00, 0x80, 0x38, 0x38};
	// perform the actual write and check for errors
	int failure = i2c_write(accelAddress, data, 11);
	if (failure) {
		printf("failed to enable accelerometer and gyroscope\n");
		return -1;
	}

	//
	// magnetometer section
	//

	// annoyingly, the device must be put to sleep when changing settings
	// first, set the device to sleep
	uint8_t state[] = {0x10, 0x00};

	int magSuccess = i2c_write(magAddress, state, 2);

	// set the user offset values (experimentally determined, different for every setup
	// bits must be shifted by one because the last bit is 0 and unused
	uint16_t xOffset = 0;//(0x7f4a << 1);
	uint16_t yOffset = 0;//(0x0241 << 1);
	uint16_t zOffset = 0;//(0x0604 << 1);

	uint8_t xL = xOffset & 0xff;
	uint8_t xH = (xOffset & 0xff00) >> 8;
	uint8_t yL = yOffset & 0xff;
	uint8_t yH = (yOffset & 0xff00) >> 8;
	uint8_t zL = zOffset & 0xff;
	uint8_t zH = (zOffset & 0xff00) >> 8;

	uint8_t offsets[] = {0x09, xL, xH, yL, yH, zL, zH};
	magSuccess |= i2c_write(magAddress, offsets, 7);

	// this block was used to determine the offset values based on experimental testing
	//int value = -183;
	//printf("two complement of %d is %x\n", value, unsignedValue16bit(value));

	// now set the configuration values
	uint8_t config[] = {0x10, 0x00, 0x00};
	magSuccess |= i2c_write(magAddress, config, 3);

	// finally, wake up the magnetometer again
	state[1]  = 0x01;
	magSuccess |= i2c_write(magAddress, state, 2);

	if (magSuccess != 0) {
		printf("Failed to set i2c configuration for magnetometer\n");
		return -1;
	}

	//
	// barometer section
	//

	getBarometerParameters();

	// let power stabilize with a wait
	usleep(50);
	_sensorsAvailable = 1;

	return 0;
}


// set sensors to sleep mode
void deinitializeSensors() {
	//
	// accelerometer and gyroscope section
	//

	// expanded for readability, local variables for the configRegisters
	uint8_t data[] = {0x10, 0x0b, 0x00, 0x06, 0xc0, 0x00, 0x10, 0x80, 0x80, 0x38, 0x38};
	// perform the actual write and check for errors
	int success = i2c_write(accelAddress, data, 11);

	if (success != 0) {
		printf("Failed to deinitialize accelerometer and gyroscope sensors\n");
	}

	//
	// magnetometer section
	//
	uint8_t config[] = {0x10, 0x00};
	int magSuccess = i2c_write(magAddress, config, 2);
	if (magSuccess != 0) {
		printf("Failed to power down magnetometer\n");
	}

	//
	// barometer section
	//
	// sets the barometer to oversampling @ 8 times
	uint8_t barometerConfig[] = {0xf4, 0x00};

	int barometerSuccess = i2c_write(barometerAddress, barometerConfig, 2);

	if (barometerSuccess != 0) {
		printf("Failed to power down barometer\n");
	}

}


// values are usually returned in two's complement
// this returns the signed value from the raw two's complement input
int32_t signedValue(uint32_t _unsignedValue, uint8_t numBits) {
	// maximum positive value used for comparison
	uint32_t maxPos = pow(2, numBits - 1) - 1;
	// maximum value the number can take based on the number of bits
	// in this case, I am using 16 bit values
	uint32_t maxNeg = 2 * (maxPos + 1);

	// the value to be returned, assuming it is positive
	int result = _unsignedValue;
	if (_unsignedValue > maxPos) {
		result = -1 * (int32_t)(maxNeg - result);
	}

	return result;
}


// converts from a signed value into a two's complement value
uint32_t unsignedValue(int _signedValue, uint8_t numBits) {
	// maximum value the number can take based on the number of bits
	// in this case, I am using 16 bit values
	uint32_t maxNeg = pow(2, numBits);

	uint32_t result = _signedValue;

	if (_signedValue < 0) {
		result = maxNeg - (-1 * _signedValue) + 1;
	}

	return result;
}

// this is a generalized function used by the acceleration vector,
//   rotation vector, and magnetometer vector functions since they have
//   similar hardware interfaces
// <reg> should be the first register to read from
// this assumes that all values are linear and occur as registers right after <reg>
// this also assumes that there are 6 bytes per vector, 2 per component, in xyz order, msb first
// the last argument, divisor, is used to correct for the fact that decimal values
//   must be stored as integers by the registers, so the decimal point must be shifted
Vector3d threeAxisVector(uint16_t address, uint8_t reg, double divisor) {
	// initialize the sensors before using them
	initializeSensors();

	// retrieves the sensor values based on the passed in arguments
	uint8_t vectorValues[6];
	int failure = i2c_read(address, reg, vectorValues, 6);
	if (failure) {
		printf("Read failed in threeAxisVector for device %x\n", address);
		return Vector3d(0, 0, 0);
	}

	uint16_t x, y, z = 0;
	x = ((uint16_t)vectorValues[0] << 8) + (uint16_t)vectorValues[1];
	y = ((uint16_t)vectorValues[2] << 8) + (uint16_t)vectorValues[3];
	z = ((uint16_t)vectorValues[4] << 8) + (uint16_t)vectorValues[5];

	// convert the values from unsigned two's complement to a signed int
	int rawx = signedValue(x, 16);
	int rawy = signedValue(y, 16);
	int rawz = signedValue(z, 16);

	return Vector3d(rawx, rawy, rawz) / divisor;
}

// gets the linear acceleration from the gyroscope
Vector3d accelerationVector() {
	// convert the signed integer form into double form based on the configured accelerometer range
	// scale is given as the +- g range for the accelerometer
	// with a +- 4g scale and a 16 bit output integer, the raw value should be
	//   divided by 2^(bits - log2(range)) which comes out to be 2^(15 - log2(4))
	//   which equals 2^13 = 8192
	// why 15? well remember, the raw 16 bit register output has to be converted to a signed value
	//   two's complement effectively uses a bit to encode sign, so we lost a bit for the total amount
	// for the sake of efficiency, this value is hardcoded, but it is important to know how it was
	//   determined for future adaptation
	int divisor = 8192;

	// create an even more user-friendly acceleration vector
	Vector3d acc = threeAxisVector(accelAddress, 0x28, divisor);

	return acc;
}



// returns the vector containing the angular rotation rate of the gyroscope
// also if you were wondering, yes I did just copy paste the acceleration function
//   and then change variable names and comments.  It's necessary boilerplate,
//   the sensors are on the same chip, and I don't care if you know
Vector3d rotationVector() {
	// scale is given as the +- dps range for the gyroscope
	// with a +- 2000 dps scale and a 16 bit output integer, the raw value should be
	//   divided by 2^(bits - log2(range)) which comes out to be 2^(15 - ceil(log2(500)))
	//   which equals 2^4 = 16
	//
	// why 15? well remember, the raw 16 bit register output has to be converted to a signed value
	//   two's complement effectively uses a bit to encode sign, so we lost a bit for the total amount
	// for the sake of efficiency, this value is hardcoded, but it is important to know how it was
	//   determined for future adaptation
	//
	// looking back, I have no idea how this divisor became 32 when the math says 16
	// dont ask me
	int divisor = 64;

	// create an even more user-friendly rotation vector
	Vector3d r = threeAxisVector(gyroAddress, 0x22, divisor);

	return r;
}


// returns the vector describing the magnetic field
// vector axises (no idea how to make axis plural) are the same as the accelerometer axises
Vector3d magneticField() {
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
	int divisor = 10;

	// create an even more user-friendly magnetic field vector
	Vector3d magField = threeAxisVector(magAddress, 0x01, divisor);

	// since the magnetometer is actually mounted upside down, the z axis value must be flipped
	magField(2) *= -1;

	return magField;
}

// retrieves the default parameters for the barometer as defined in the eeprom registers and
//   stores them in the baroVals array in the order they appear on the data sheet
void getBarometerParameters() {
	// loop through the 11 values and get the register contents
	// convert to signed values if needed
	for (uint8_t i = 0; i < 11; i++) {
		// the registers come in pairs of two
		// they are MSB (most significant byte) first
		uint8_t registers[2] = {(uint8_t)(0xaa + (2 * i)), (uint8_t)(0xab + (2 * i))};
		uint8_t data[2];

		int success = i2c_read(barometerAddress, registers[0], data, 1);
		success |= i2c_read(barometerAddress, registers[1], &data[1], 1);
		uint32_t readValue = ((uint16_t)data[0] << 8) + (uint16_t)data[1];

		if (success != 0) {
			printf("reading eeprom data from registers %x and %x failed\n", registers[0], registers[1]);
		} else {
			int32_t signedReadValue;

			if (i < 3 || i > 5)
				signedReadValue = signedValue(readValue, 16);
			else
				signedReadValue = readValue;

			baroVals[i] = signedReadValue;
		}
	}
}


// helper function to get the barometer uncompensatedtemperature
uint32_t uncompensatedTemperature() {
	uint8_t temperatureConfig[] = {0xf4, 0x2e};
	int success = i2c_write(barometerAddress, temperatureConfig, 2);

	// datasheet suggests waiting 4.5 seconds for the value to be obtained
	usleep(4500);

	uint8_t dataRegisters[] = {0xf6, 0xf7};
	uint8_t data[2];

	success |= i2c_read(barometerAddress, dataRegisters[0], data, 1);
	success |= i2c_read(barometerAddress, dataRegisters[1], &data[1], 1);
	uint32_t temperature = ((uint16_t)data[0] << 8) + (uint16_t)data[1];

	if (success != 0) {
		printf("barometer temperature read failed");
	}

	return temperature;
}

// helper function to get the barometer uncompensated pressure
uint32_t uncompensatedPressure() {
	uint8_t sampleRate = 3;

	uint8_t pressureConfig[] = {0xf4, 0xf4};
	int success = i2c_write(barometerAddress, pressureConfig, 2);

	// datasheet suggests a 25.5ms waiting period for a sample rate of 3
	usleep(25500);

	uint8_t dataRegisters[] = {0xf6, 0xf7, 0xf8};
	uint8_t data[3];

	success |= i2c_read(barometerAddress, dataRegisters[0], data, 1);
	success |= i2c_read(barometerAddress, dataRegisters[1], &data[1], 1);
	success |= i2c_read(barometerAddress, dataRegisters[2], &data[2], 1);
	uint32_t pressure = (((uint32_t)data[0] << 16) + ((uint32_t)data[1] << 8) + (uint32_t)data[2]) >> (8 - sampleRate);

	if (success != 0) {
		printf("barometer pressure read failed");
		return 0;
	}

	return pressure;
}

// gets the altitude relative to the take-off height
double barometerAltitude() {
	// initialize the sensors before using data
	initializeSensors();

	uint8_t sampleRate = 3;

	uint32_t uncompPressure = uncompensatedPressure();
	uint32_t uncompTemperature = uncompensatedTemperature();

	// the following calculations are taken from the datasheet
	int32_t X1 = (uncompTemperature - baroVals[5]) * baroVals[4] / 32768;
	int32_t X2 = (baroVals[9] * 2048) / (X1 + baroVals[10]);
	int32_t B5 = X1 + X2;
	//int32_t trueTemperature = (B5 + 8) / 16;

	int32_t B6 = B5 - 4000;
	X1 = (baroVals[7] * (B6 * B6 / 4096)) / 2048;
	X2 = baroVals[1] * B6 / 2048;
	int32_t X3 = X1 + X2;
	int32_t B3 = (((baroVals[0] * 4 + X3) << sampleRate) + 2) / 4;
	X1 = baroVals[2] * B6 / 8192;
	X2 = (baroVals[6] * (B6 * B6 / 4096)) / 65536;
	X3 = (X1 + X2 + 2) / 4;
	uint32_t B4 = baroVals[3] * (uint32_t)(X3 + 32768) / 32768;
	uint32_t B7 = ((uint32_t)uncompPressure - B3) * (50000 >> sampleRate);
	int32_t p;
	if (B7 < 0x80000000) {
		p = (B7 * 2) / B4;
	}
	else {
		p = (B7 / B4) * 2;
	}

	X1 = (p / 256) * (p / 256);
	X1 = (X1 * 3038) / 65536;
	X2 = (-7357 * p) / 65536;

	int32_t pressure = p + (X1 + X2 + 3791) / 16;

	// now with the pressure and temperature calculated, we can used the formula
	//   provided in the data sheet for obtaining altitude based on the international
	//   barometric formula

	// pressure in pascals
	double seaLevel = 101325;
	double altitude = 44330 * (1 - pow(((double)pressure)/seaLevel, (1.0/5.255)));

	return altitude;
}








