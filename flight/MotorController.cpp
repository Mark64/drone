// implementation for the motor controller functions

#include<stdio.h>
#include<unistd.h>
#include<math.h>

#include "MotorController.h"

extern "C" {
#include "PWMController.h"
}

// the arming, minimum, and maximum thrust values
// the arming value is a PWM percentage that tells the ESC there is a proper PWM signal source attached
// there are based on the ESC and will most likely vary with different devices
#define _armingThrust 0.0600
#define _minimumThrust 0.1000 + 0.0100
#define _maximumThrust 0.6337
#define _thrustRange (_maximumThrust - _minimumThrust)

// array containing the status of each ESC's status as 'armed' (value of 1, meaning ready to 
//   spin) or 'unarmed' (value of 0, not ready to spin)
// ESC = electronic speed controller
uint8_t _armingStatus[] = {0, 0, 0, 0};


// returns the PWM device number based on the motor address, allows PWM slot and motor number to be independent
uint8_t motorAddress(uint8_t motorAddress) {
	// the hardware addresses of the PWM slot the motors are connected to
	// PWM = pulse width modulation
	const uint8_t motor1Address = 0;
	const uint8_t motor2Address = 1;
	const uint8_t motor3Address = 2;
	const uint8_t motor4Address = 3;

	// array containing the motor addresses based with the index number being the motor number
	const uint8_t motorAddresses[4] = {motor1Address, motor2Address, motor3Address, motor4Address};

	return motorAddresses[motorAddress];
}

// NOTE: There is a special case for 0 since the minimum thrust value actually is slightly above the
//   true minimum to ensure it has no problems rotating under load

// in order to use the motor, the ESC must first be armed by setting a low PWM signal
void armMotor(uint8_t motorNumber) {
	if (_armingStatus[motorNumber] == 0) {
		setDutyPercent(motorAddress(motorNumber), _armingThrust);
		usleep(50000);
		_armingStatus[motorNumber] = 1;
	}
}

// returns the thurst percentage
double getMotorThrustPercentage(uint8_t motorNumber) {
	double pwmThrustPercentage = getDutyPercent(motorAddress(motorNumber));

	double motorThrustPercentage = (pwmThrustPercentage - _minimumThrust) / (_thrustRange);
	
	if (motorThrustPercentage < 0) {
		motorThrustPercentage = 0;
	}

	return motorThrustPercentage;
}


// sets the thrust percentage
void setMotorThrustPercentage(uint8_t motorNumber, double thrustPercentage) {
	// first the ESC must be armed
	// the armMotor function takes care of checking if the ESC has been armed already
	armMotor(motorNumber);

	double pwmPercentage = thrustPercentage * _thrustRange + _minimumThrust;

	if (thrustPercentage == 0) {
		pwmPercentage = _armingThrust;
	}

	// this protects against a value too large being passed in
	if (thrustPercentage >= 1) {
		pwmPercentage = _maximumThrust;
	}

	setDutyPercent(motorAddress(motorNumber), pwmPercentage);
}

// calibrates the motors
// goes one at a time so that error tones can be easily differentiated
void calibrateMotor(uint8_t motorNumber) {
	// the maxThrottle should be a little higher than max thrust so that the maximum motor
	//   speed is not reached before 100%
	double maxThrottle = _maximumThrust + 0.0263;
	double minThrottle = 0.1000;
	
	printf("disconnect the ESCs (electronic speed controller) from power, then press enter to continue\n");
	getchar();
	
	printf("calibrating motor %d\n", motorNumber);
	printf("setting maximum throttle to %.2f%%\n", 100*maxThrottle);
	setDutyPercent(motorAddress(motorNumber), maxThrottle);

	usleep(500000);
	
	printf("\nreconnect the ESCs to power\nthen, within 1-2 seconds of the beeps, press enter to continue\n");
	getchar();

	printf("setting minimum throttle to %.2f%%\n", 100*minThrottle);
	setDutyPercent(motorAddress(motorNumber), minThrottle);
	
	usleep(1000000);
}







