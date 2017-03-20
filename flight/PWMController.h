// this abstracts and controls all devices attached to the PWM card
// PWM = pulse width modulation.  See google for more info
//
// by Mark Hill

#ifndef _PWMController
#define _PWMController

#include<stdint.h>

// returns the percentage the device is on as a value from 0 - 1
// specify the device by providing a value to the address variable
//   which has values from 0 - n where n is the number of supported PWM
//   devices (my PCA9685 chip has 16, so n=15)
double getDutyPercent(uint8_t address);

// sets the percentage the specified device should be on for
// pass in a value from 0 - 1 for the percentage on and a value 0 - n 
//   where n is the number of PWM devices supported for the address
void setDutyPercent(uint8_t address, double percent);


#endif
