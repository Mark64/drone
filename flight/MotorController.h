// abstracts motor thrust values from the flight controller
// acts a middleman between the high level flight controller and the low level PWMController
// PWM = pulse width modulation
//
// by Mark Hill

#include<stdint.h>

// returns the thrust percentage of the addressed motor as a value between 0 - 1 where 1 is maximum thrust
double getMotorThrustPercentage(uint8_t motorNumber);

// sets the thrust percentage for the addressed motor to the value passed to thrustPercentage where
//   thrustPercentage is a value between 0 - 1 with 1 being maximum thrust
void setMotorThrustPercentage(uint8_t motorNumber, double thrustPercentage);
