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
#include<unistd.h>

// i2c address of the PWM chip
static const uint16_t pwmDeviceAddress = 0x40;

// flag indicating whether the PWM chip has been initialized
// this should only be set by the initializePWMController function
// 0 = uninitialized = not ready for use
// 1 = initialized = ready for use
static uint8_t initialized = 0;

// on boot, the PWM device must have its configuration registers set
//   to the values specific for this application
static void initializePWMController() {
	// if the chip has already been initialized, return
	if (initialized) {
		return;
	}

	// there are three registers that must be set:
	//   Mode 1 (first configuration register)
	//   Mode 2 (second configuration register)
	//   Prescale (sets the clock speed for PWM)
	// the registers can be set in any order, as long
	//   as the device is first put to sleep by setting the
	//   sleep bit on Mode 1, and ending with a write to wake
	//   the device with a final write to Mode 1
	
	// in order to set the registers, the chip must be put into sleep
	//   mode by turning off the internal oscilator
	// this is done by setting bit 4 on the mode 1 register to 1
	
	// Mode 1 (set to sleep)
	uint8_t mode1Register[] = {0x00};
	uint8_t value = 0x30;
	
	int success = i2cWrite(pwmDeviceAddress, mode1Register, 1, value, HIGH_BYTE_FIRST, AUTO_INCREMENT_ENABLED);

	for (int i = 0; i < 3 && success != 0; i++) {
		printf("retrying mode 1 sleep register write in PWMController\n");
		success = i2cWrite(pwmDeviceAddress, mode1Register, 1, value, HIGH_BYTE_FIRST, AUTO_INCREMENT_ENABLED);
	}
	

	// Prescale
	uint8_t prescaleRegister[] = {0xfe};
	value = 0x16;
	
	success = i2cWrite(pwmDeviceAddress, prescaleRegister, 1, value, HIGH_BYTE_FIRST, AUTO_INCREMENT_ENABLED);
	
	for (int i = 0; i < 3 && success != 0; i++) {
		printf("retrying prescale register write in PWMController\n");
		success = i2cWrite(pwmDeviceAddress, prescaleRegister, 1, value, HIGH_BYTE_FIRST, AUTO_INCREMENT_ENABLED);
	}
	
	// Mode 2
	uint8_t mode2Register[] = {0x01};
	value = 0x04;
	
	success = i2cWrite(pwmDeviceAddress, mode2Register, 1, value, HIGH_BYTE_FIRST, AUTO_INCREMENT_ENABLED);

	for (int i = 0; i < 3 && success != 0; i++) {
		printf("retrying mode 2 register write in PWMController\n");
		success = i2cWrite(pwmDeviceAddress, mode2Register, 1, value, HIGH_BYTE_FIRST, AUTO_INCREMENT_ENABLED);
	}

	// Mode 1 (wake from sleep)
	value = 0xa0;
	
	success = i2cWrite(pwmDeviceAddress, mode1Register, 1, value, HIGH_BYTE_FIRST, AUTO_INCREMENT_ENABLED);

	for (int i = 0; i < 3 && success != 0; i++) {
		printf("retrying mode 1 wake register write in PWMController\n");
		success = i2cWrite(pwmDeviceAddress, mode1Register, 1, value, HIGH_BYTE_FIRST, AUTO_INCREMENT_ENABLED);
	}

	// must wait 500us for the internal oscillator on the PWM chip to stabilize as
	//   per the datasheet recommendation	
	usleep(500);
	
	// done
	if (success == 0) {
		initialized = 1;
	}
	else {
		printf("failed to initialize PWM device\n");
	}
}

// sets the PWM device to sleep by turning off the internal oscillator
void deinitializePWMController() {
	// Mode 1 (set to sleep)
	uint8_t mode1Register[] = {0x00};
	uint8_t value = 0x30;
	
	int success = i2cWrite(pwmDeviceAddress, mode1Register, 1, value, HIGH_BYTE_FIRST, AUTO_INCREMENT_ENABLED);

	for (int i = 0; i < 3 && success != 0; i++) {
		printf("sleep failed, retrying mode 1 sleep register write in PWMController\n");
		success = i2cWrite(pwmDeviceAddress, mode1Register, 1, value, HIGH_BYTE_FIRST, AUTO_INCREMENT_ENABLED);
	}
}

// returns the duty cycle of the addressed PWM device as a value from 0 - 1
double getDutyPercent(uint8_t address) {
	// read the on and off values to the corresponding registers
	// the 4 * address sum is used to offset the register to read from
	//   based on which PWM pin is being addressed
	uint8_t onLowRegister = (uint8_t)(0x06 + (4 * address));
	uint8_t onHighRegister = (uint8_t)(0x07 + (4 * address));
	uint8_t offLowRegister = (uint8_t)(0x08 + (4 * address));
	uint8_t offHighRegister = (uint8_t)(0x09 + (4 * address));
	
	uint8_t registers[] = {onLowRegister, onHighRegister, offLowRegister, offHighRegister};

	uint32_t combinedDelay;
	i2cWordRead(pwmDeviceAddress, registers, 4, &combinedDelay, WORD_16_BIT, LOW_BYTE_FIRST, AUTO_INCREMENT_ENABLED);
	
	uint32_t on = (0x0000ffff & combinedDelay);
	uint32_t off = (0xffff0000 & combinedDelay);
	// get it?
	double duty = abs((int)off - (int)on) / (double) 4096;

	return duty;
}

// sets the duty cycle of the addressed PWM device to the specified value between 0 - 1
void setDutyPercent(uint8_t address, double percent) {
	// the PWM chip obviously needs to be configured before values can be written
	initializePWMController();
	
	// check the inputs and make sure they don't exceed passed in values
	if (percent > 1) {
		percent = 1;
	}
	if (percent < 0) {
			percent = 0;
	}

	// the time the chip should wait each cycle before turning the pulse on
	// this is used to prevent power surging
	uint16_t onDelay = 225 * address;
	// the time the chip should wait each cycle before turning the pulse off
	uint16_t offDelay = onDelay + (uint16_t)(percent * 4096) - 1;
	
	// in order to write value with one function call, combine values
	uint32_t combinedDelayValue = (offDelay << 16) + onDelay;

	// because the data sheet says to subtract 1 for proper function of the device,
	//   I have to check that the offDelay did not underflow from 0 -1 and become 65535
	if (offDelay > 4096) {
		offDelay = 0;
	}
	
	//printf("Setting motor to on for %d counts out of 4095\n", offDelay - onDelay);
	// write the on and off values to the corresponding registers
	// the 4 * address sum is used to offset the register to write to
	//   based on which PWM pin should be configured
	uint8_t onLowRegister = (uint8_t)(0x06 + (4 * address));
	uint8_t onHighRegister = (uint8_t)(0x07 + (4 * address));
	uint8_t offLowRegister = (uint8_t)(0x08 + (4 * address));
	uint8_t offHighRegister = (uint8_t)(0x09 + (4 * address));
	
	uint8_t registers[] = {onLowRegister, onHighRegister, offLowRegister, offHighRegister};

	i2cWrite(pwmDeviceAddress, registers, 4, combinedDelayValue, LOW_BYTE_FIRST, AUTO_INCREMENT_ENABLED);
}






