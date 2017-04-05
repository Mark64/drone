// this takes all the sensor data from the sensor functions and turns it
//   into a representation of the drone's current orientation and velocity

#ifndef _Orientation
#define _Orientation

#include "Vector.h"

struct Orientation {
	// this vector contains the acceleration of the vehicle relative to the ground
	// it removes the already present acceleration from gravity
	// given in units of g (~9.8m/s^2)
	struct Vec3double acceleration;

	// this vector represents the vehicles angular rotation in units of degrees
	//   per second
	struct Vec3double angularRotation;

	// this gives the vehicle's orientation as an angle in degrees in the XY plane 
	//   relative to magnetic north
	double angleFromNorth;
	
	// this gives the vehicle's height in meters from sea level
	double altitude;
};

// calibrates the sensor values to enable correction for gravity and differences
//   in the angle between sensors on the circuit layout
void calibrateSensors();

// this function retrieves all the sensor values and performs corrections based on
//   calibration values and internal adjustments
// the completion handler is called and passed the Orientation struct once the
//   calculations finish
//   this is due to the fact that internally, this is a multithreaded process
// expected wait time is 5ms or less
void getOrientation(void (*completionHandler)(struct Orientation));



#endif
