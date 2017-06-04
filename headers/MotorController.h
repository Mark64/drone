// abstracts motor thrust values from the flight controller
// acts a middleman between the high level flight controller and the low level PWMController
// PWM = pulse width modulation
//
// by Mark Hill

#ifndef _MotorController
#define _MotorController

#include<stdint.h>

#include "Vector.h"

// Note: for the following vector functions, the vectors should follow
//       these conventions
// the x axis points to the front of the vehicle
// the y axis points to the left of the vehicle
// the z axis points to the top of the vehicle

// returns the current target linear motion vector
struct Vec3double getLinearMotionVector();

// sets the target linear motion vector to the supplied argument
// the MotorController implementation handles properly setting the thrust values
// the passed in vector should have a total magnitude less than 1 and greater than 0
void setLinearMotionVector(struct Vec3double targetLinearMotion);



// Important: only Z axis angular motion is supported in my implementation
//   the X and Y components are ignored and have no effect
// Instead, it is recommended that the linear velocity be set with a low z value to achieve
//   rotation in the x and y axises for the drone implementation

// returns the current target angular motion vector of the vehicle
struct Vec3double getAngularMotionVector();

// sets the target angular motion vector
// this MotorController implementation handles power allocation to each motor
// the passed in vector should have total magnitude 0 < mag < 1
void setAngularMotionVector(struct Vec3double targetAngularMotion);




// the following functions are for testing purposes and internal control
// it is recommended that the motors be controlled from the vector functions
//   instead
// implementation can be found in MotorControllerLowLevel.c

// returns the thrust percentage of the addressed motor as a value between 0 - 1 where 1 is maximum thrust
double getMotorThrustPercentage(uint8_t motorNumber);

// sets the thrust percentage for the addressed motor to the value passed to thrustPercentage where
//   thrustPercentage is a value between 0 - 1 with 1 being maximum thrust
void setMotorThrustPercentage(uint8_t motorNumber, double thrustPercentage);

// calibrates the motor to the values defined in MotorController.cpp (_maxThrust and _minThrust)
void calibrateMotor(uint8_t motorNumber);

#endif
