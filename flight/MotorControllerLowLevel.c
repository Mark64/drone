// implementation for the lower level motor controller functions
// this handles the raw power and arming of the motors

#include<stdio.h>
#include<unistd.h>

#include "MotorController.h"
#include "PWMController.h"

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

// the hardware addresses of the PWM slot the motors are connected to
// PWM = pulse width modulation
#define _motor1Address 0
#define _motor2Address 1
#define _motor3Address 2
#define _motor4Address 3
// holds the array for motor addresses used to lookup true PWM address for each motor number
uint8_t motorAddressLookupTable[4];

// returns an array containing the motor addresses based with the index number being the motor number
uint8_t *motorAddresses() {
	motorAddressLookupTable[0] = _motor1Address;
       	motorAddressLookupTable[1] = _motor2Address;
	motorAddressLookupTable[2] = _motor3Address;
	motorAddressLookupTable[3] = _motor4Address;
	return motorAddressLookupTable;
}

// NOTE: There is a special case for 0 since the minimum thrust value actually is slightly above the
//   true minimum to ensure it has no problems rotating under load

// in order to use the motor, the ESC must first be armed by setting a low PWM signal
void armMotor(uint8_t motorNumber) {
	if (_armingStatus[motorNumber] == 0) {
		setDutyPercent(motorAddresses()[motorNumber], _armingThrust);
		usleep(50000);
		_armingStatus[motorNumber] = 1;
	}
}

// returns the thurst percentage
double getMotorThrustPercentage(uint8_t motorNumber) {
	double pwmThrustPercentage = getDutyPercent(motorAddresses()[motorNumber]);

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

	if (thrustPercentage <= 0.00001) {
		
		pwmPercentage = _armingThrust;
	}

	// this protects against a value too large being passed in
	if (thrustPercentage >= 1) {
		pwmPercentage = _maximumThrust;
	}

	setDutyPercent(motorAddresses()[motorNumber], pwmPercentage);
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
	setDutyPercent(motorAddresses()[motorNumber], maxThrottle);

	usleep(500000);
	
	printf("\nreconnect the ESCs to power\nthen, within 1-2 seconds of the beeps, press enter to continue\n");
	getchar();

	printf("setting minimum throttle to %.2f%%\n", 100*minThrottle);
	setDutyPercent(motorAddresses()[motorNumber], minThrottle);
	
	usleep(1000000);
}







