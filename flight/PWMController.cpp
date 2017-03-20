// implementation for the PWM controller
// PWM = pulse width modulation
// Using PCA9685 chip
// Datasheet: https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf
//
// by Mark Hill

#include "PWMController.h"
#include "i2cctl.h"
#include<stdio.h>
#include<stdlib.h>

// i2c address of the PWM chip
uint16_t pwmDeviceAddress = 0x40;

// flag indicating whether the PWM chip has been initialized
// this should only be set by the initializePWMController function
// 0 = uninitialized = not ready for use
// 1 = initialized = ready for use
uint8_t initialized = 0;

// on boot, the PWM device must have its configuration registers set
//   to the values specific for this application
void initializePWMController() {
	// if the chip has already been initialized, return
	if (initialized) {
		return;
	}

	// there are three registers that must be set:
	//   Mode 1 (first configuration register)
	//   Mode 2 (second configuration register)
	//   Prescale (sets the clock speed for PWM)
	// the registers must be set in the order opposite to the list above 
	
	// Prescale
	uint8_t prescaleRegister[] = {0xfe};
	uint8_t value = 0x09;
	
	int success = i2cWrite(pwmDeviceAddress, prescaleRegister, 1, value);

	for (int i = 0; i < 3 && success != 0; i++) {
		printf("retrying prescale register write in PWMController\n");
		success = i2cWrite(pwmDeviceAddress, prescaleRegister, 1, value);
	}
	
	// Mode 2
	uint8_t mode2Register[] = {0x01};
	value = 0x04;
	
	success = i2cWrite(pwmDeviceAddress, mode2Register, 1, value);

	for (int i = 0; i < 3 && success != 0; i++) {
		printf("retrying mode 2 register write in PWMController\n");
		success = i2cWrite(pwmDeviceAddress, mode2Register, 1, value);
	}

	// Mode 1
	uint8_t mode1Register[] = {0x00};
	value = 0x00;
	
	success = i2cWrite(pwmDeviceAddress, mode1Register, 1, value);

	for (int i = 0; i < 3 && success != 0; i++) {
		printf("retrying mode 1 register write in PWMController\n");
		success = i2cWrite(pwmDeviceAddress, mode1Register, 1, value);
	}
		
	// done
	initialized = 1;
}

// returns the duty cycle of the addressed PWM device as a value from 0 - 1
double getDutyPercent(uint8_t address) {
	// read the on and off values to the corresponding registers
	// the 4 * address sum is used to offset the register to read from
	//   based on which PWM pin is being addressed
	uint8_t onRegisters[] = {(uint8_t)(0x07 +(4 * address)), (uint8_t)(0x06 + (4 * address))};
	uint8_t offRegisters[] = {(uint8_t)(0x09 + (4 * address)), (uint8_t)(0x08 + (4 * address))};

	uint16_t on = i2cRead(pwmDeviceAddress, onRegisters, 2);
	uint16_t off = i2cRead(pwmDeviceAddress, offRegisters, 2);

	// get it?
	double duty = abs(off - on) / (double) 4096;

	return duty;
}

// sets the duty cycle of the addressed PWM device to the specified value between 0 - 1
void setDutyPercent(uint8_t address, double percent) {
	// the PWM chip obviously needs to be configured before values can be written
	initializePWMController();
	
	// the time the chip should wait each cycle before turning the pulse on
	uint16_t onDelay = 0;
	// the time the chip should wait each cycle before turning the pulse off
	uint16_t offDelay = (uint16_t)(percent * 4096) - 1;
	
	// because the data sheet says to subtract 1 for proper function of the device,
	//   I have to check that the offDelay did not underflow from 0 -1 and become 65535
	if (offDelay > 4096) {
		offDelay = 0;
	}
	
	//printf("Setting motor to on for %d counts out of 4095\n", offDelay - onDelay);
	// write the on and off values to the corresponding registers
	// the 4 * address sum is used to offset the register to write to
	//   based on which PWM pin should be configured
	uint8_t onRegisters[] = {(uint8_t)(0x07 +(4 * address)), (uint8_t)(0x06 + (4 * address))};
	uint8_t offRegisters[] = {(uint8_t)(0x09 + (4 * address)), (uint8_t)(0x08 + (4 * address))};

	i2cWrite(pwmDeviceAddress, onRegisters, 2, onDelay);
	i2cWrite(pwmDeviceAddress, offRegisters, 2, offDelay);
}






