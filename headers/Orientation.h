// this takes all the sensor data from the sensor functions and turns it
//   into a representation of the drone's current orientation and velocity

#ifndef _Orientation
#define _Orientation

#include<stdint.h>

#include "Vector.h"

struct Orientation {
	// this vector contains the acceleration of the vehicle relative to the ground
	// it removes the already present acceleration from gravity
	// given in units of g (~9.8m/s^2)
	struct Vec3double acceleration;

	// this vector represents the gravity vector relative to the system
	struct Vec3double gravity;

	// stores the heading as degrees from north
	double heading;

	// this gives the vehicle's height in meters from sea level
	double altitude;
};

// prints out all the members of the passed in orientation struct
void printOrientation(struct Orientation orientation);

// calibrates the sensor values to enable correction for gravity and differences
//   in the angle between sensors on the circuit layout
void calibrateSensors();

// calibrate sensors must be called before this function is called
//   since there is no guaruntee the calibration would finish before the system begins
//   moving
// because the time to retrieve the orientation is subject to a variable number of factors
//   which can become unpredictable as the complexity of the system increases, this semi-complex
//   function is used to simplify the process (or complicate it, depending on your religious
//   views)
// when you call this function, the internal implementation spins up a variable number of threads
//   to handle all the sensor inputs and computations, which reduces wasted CPU time and improves
//   performance
// once the implementation has determined that it has enough data to provide and updated
//   representation of the system's orientation, it calls the passed in function pointer with
//   the updated Orientation struct as the sole argument
// note that the completionHandler function is called on a different thread, so it should be
//   thread safe
// this will continue as long as the system continues to function properly, which can be undesired
//   while the system doesn't need the orientation (while landed for example)
// for this reason, you can stop the continuous updates by passing a -1 as the return value for
//   your completion handler
//   a 0 return value indicates that the orientation updates can proceed
//   a negative return value indicates the updates should stop
//   a positive return value changes the update Hz to the returned value for the
//     listener
// since multithreaded systems can be complicated (citation needed), it is recommended that
//   the completion handler be able to handle a few additional update calls
//   after passing a -1 to stop updates, though this hopefully won't ever be an issue
// the second argument is the desired update rate in Hz, which determines approximately how
//   often the completionHandler will be called
// returns 0 on success and -1 on failure
int getOrientation(int (*update)(struct Orientation), uint16_t updateRate);



#endif
