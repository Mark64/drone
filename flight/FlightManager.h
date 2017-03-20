// Started out as the simplest header file I've written
// This abstracts the motor control and allows for easy update and stabilization
// Just pass in your motion vectors and this will ensure they are carried out, as well as stabilize the position when at rest

#ifndef _FlightManager
#define _FlightManager

#include "Vector.h"

// this function is called to signal to the motor controller that the system is ready for takeoff
// it accepts 2 arguments:
//   - a double representing the rate of the takeoff on a scale of 0 - 1 with 1 being the fastest
//   - a function pointer to a completion handler function which accepts a double as its argument
//       where the double is the number of seconds it actually took to take off
void takeOff(double takeOffTime, void (*takeOffCompletionHandler)(double));

// this function is called to signal to the motor controller that the system has requested to
//   land
// it accepts 2 arguments:
//   - a double representing the rate of landing speed on a scale from 0 - 1 with 1 being the fastest
//   - a function pointer to a completion handler function which accepts a double as its argument
//      where the double represents the amount of time in seconds the landing took
void land(void (*landedCompletionHandler)(double));

// this function is called to update the linear velocity vector of the system to the supplied
//   Vec3double argument
// the vector should have components with values from 0 - 1 with 1 being approximately max speed
// it accepts 2 arguments
//   - a Vec3double vector representing the desired linear velocity 
//   - a function pointer to a completition handler that will be called once the vehicle reached
//      within 10% of the desired vector components
void setVelocityVector(Vec3double velocityVector, void (*velocityUpdateCompletionHandler));

// this function is called to update the angular velocity vector of the system to the supplied
//   Vec3double argument
// the vector should have components with values from 0 - 1 with 1 being approximately max angular speed
// it accepts 2 arguments
//   - a Vec3double vector representing the desired angular velocity 
//   - a function pointer to a completition handler that will be called once the vehicle reached
//      within 10% of the desired vector components
void setAngularVelocityVector(Vec3double angularVelocityVector, void (*angularVelocityUpdateCompletionHandler));




#endif
