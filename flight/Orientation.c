// implementation for the Orientation header
//
// by Mark Hill

#include "Orientation.h"

#include "SensorManager.h"

// get inclination
// get acceleration magnitude
// get average angular drift
// get baseline height
// convert vectors to 0 angle
//
// corrected acceleration
// altitude
// degrees from north
// angle from ground

// the inclination of the magnetic field in degrees from a
//   horizontal line
static double inclinationAngle = 0;
// the expected no motion value of the magnitude of the force of gravity
static double expectedAcceleration = 0;
// the expected deviation from 0 angular spin as measured by
//   the gyroscope
static struct Vec3double expectedAngularDrift;
// the starting altitude in meters
static double startingHeight = 0;


// internal functions for setting defaults

// gets the inclination of the magnetic field
//   and returns a decimal value in degrees
static void getInclination() {
	
}

// gets the average magnitude of the accelerometer
//   for use as a baseline acceleration at rest
// gives value in g's
static void getAccelerationMagnitude() {

}

// gets the expected angular drift of the gyroscope
//   to use as an expected value in calculations
// gives value in degrees per second
static void getExpectedAngularDrift() {

}

// returns the height of the device at rest
static void getBaselineHeight() {

}

// internal functions for conversion of sensor values relative
//   to other sensors (acceleration relative to ground defined by
//   magnetic field, for example)

// returns the acceleration vector adjusted for the position 
//   of ground as determined by the magnetic field vector
static struct Vec3double correctedAcceleration() {
	struct Vec3double rawAcceleration = accelerationVector();

	return rawAcceleration;
}	

// returns the height difference from the passed in height in meters
static double heightDifferenceFromStart(double baseline) {
	double currentHeight = barometerAltitude();

	return currentHeight - baseline;

}

// returns the horizonal plane angle the device is relative to
//   a ray pointing towards magnetic North
static double degreesFromNorth() {
	return 0.0;
}

// returns the vertical plane angle the bottom of the device is 
//   pointing relative to a ray pointing directly at the ground
static double degreesFromGround() {
	return 0.0;
}



// finally, the implementation of the functions defined in the header

// calibrates using the above internal functions
void calibrateSensors() {

}

// retrieves the orientation of the device and passes an Orientation
//   struct into the passed in completion handler function
void getOrientation(void (*completionHandler)(struct Orientation)) {


}







