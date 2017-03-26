// This contains code for collecting data from the various sensors and returning the data in a usable format
//
// In the future, I may implement a cache for data storage so that data collection rates can be independent of its use

#ifndef _SensorManager
#define _SensorManager

#include<stdint.h>

#include "Vector.h"


// this collects data from the accelerometer and returns the linear acceleration vector
// measured in units of 'g', which is acceleration at sea level or about 9.8m/s^2
Vec3double accelerationVector();

// in a similar manner to the accelerationVector() function, this 
//   collects sensors data and returns a vector, but of the angular
//   rotation rate of the gyroscope instead of the linear acceleration
// measured in units of degrees per second
Vec3double rotationVector();

// this collects data from the magnetometer and returns a vector containing 
//   the magnetic field as determined by the magnetometer 
// axises are the same as the axises of the accelerometer
// measured in units of 
Vec3double magneticField();

// returns the GPS position of the drone as a 2d coordinate position


// returns the current height of the drone in meters as measured from the barometer relative to the take off height
// reads 0 when not in flight
// measured in units of meters
double barometerAltitude();

// sets up all the sensors by writing their configuration registers and other setup as needed
void initializeSensors(); 


#endif
