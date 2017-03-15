// This contains code for collecting data from the various sensors and returning the data in a usable format
//
// In the future, I may implement a cache for data storage so that data collection rates can be independent of its use

#include "Vector.h"
#include<stdint.h>

// this collects data from the accelerometer and returns the acceleration vector
Vec3double accelerationVector();

// this collects data from the magnetometer and returns the heading of the device in degrees relative to the front forward face
double compassHeading();

// returns the GPS position of the drone as a 2d coordinate position


// returns the current height of the drone in meters as measured from the barometer relative to the take off height
// reads 0 when not in flight
double barometerAltitude();

// sets up all the sensors by writing their configuration registers and other setup as needed
void initializeSensors(); 



