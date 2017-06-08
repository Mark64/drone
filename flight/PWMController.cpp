// implementation for the PWM controller
// PWM = pulse width modulation
// Using PCA9685 chip
// Datasheet: https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf
//
// by Mark Hill

#include<stdio.h>
#include<stdlib.h>
#include<unistd.h>

#include <PWMController.h>
#include <i2cctl.h>

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
	uint8_t data[] = {0x00, 0x30};
	
	int success = i2c_write(pwmDeviceAddress, data, 2);

	for (int i = 0; i < 3 && success != 0; i++) {
		printf("retrying mode 1 sleep register write in PWMController\n");
		success |= i2c_write(pwmDeviceAddress, data, 2);
	}
	

	// Prescale
	data[0] = 0xfe;
	data[1] = 0x16;
	
	success |= i2c_write(pwmDeviceAddress, data, 2);
	
	for (int i = 0; i < 3 && success != 0; i++) {
		printf("retrying prescale register write in PWMController\n");
		success |= i2c_write(pwmDeviceAddress, data, 2);
	}
	
	// Mode 2
	data[0] = 0x01;
	data[1] = 0x04;
	
	success |= i2c_write(pwmDeviceAddress, data, 2);

	for (int i = 0; i < 3 && success != 0; i++) {
		printf("retrying mode 2 register write in PWMController\n");
		success |= i2c_write(pwmDeviceAddress, data, 2);
	}

	// Mode 1 (wake from sleep)
	data[0] = 0x00;
	data[1] = 0xa0;
	
	success |= i2c_write(pwmDeviceAddress, data, 2);

	for (int i = 0; i < 3 && success != 0; i++) {
		printf("retrying mode 1 wake register write in PWMController\n");
		success |= i2c_write(pwmDeviceAddress, data, 2);
	}

	// must wait 500us for the internal oscillator on the PWM chip to stabilize as
	//   per the datasheet recommendation	
	usleep(500);
	
	if (success == 0)
		initialized = 1;
	else
		printf("failed to initialize PWM device\n");
}

// sets the PWM device to sleep by turning off the internal oscillator
void deinitializePWMController() {
	// Mode 1 (set to sleep)
	uint8_t data[] = {0x00, 0x30};
	
	int success = i2c_write(pwmDeviceAddress, data, 2);

	for (int i = 0; i < 3 && success != 0; i++) {
		printf("sleep failed, retrying mode 1 sleep register write in PWMController\n");
		success = i2c_write(pwmDeviceAddress, data, 2);
	}
}

// returns the duty cycle of the addressed PWM device as a value from 0 - 1
double getDutyPercent(uint8_t address) {
	// read the on and off values to the corresponding registers
	// the 4 * address sum is used to offset the register to read from
	//   based on which PWM pin is being addressed
	uint8_t onLowRegister = (uint8_t)(0x06 + (4 * address));
	uint8_t data[4];
	i2c_read(pwmDeviceAddress, onLowRegister, data, 4);
	
	uint32_t on = data[0] + (data[1] << 8);
	uint32_t off = data[2] + (data[3] << 8);
	// get it?
	double duty = abs((int)off - (int)on) / (double) 4096;

	return duty;
}

// sets the duty cycle of the addressed PWM device to the specified value between 0 - 1
void setDutyPercent(uint8_t address, double percent) {
	// the PWM chip obviously needs to be configured before values can be written
	initializePWMController();
	
	// check the inputs and make sure they don't exceed passed in values
	if (percent > 1)
		percent = 1;
	if (percent < 0)
		percent = 0;

	// the time the chip should wait each cycle before turning the pulse on
	// this is used to prevent power surging
	uint16_t onDelay = 225 * address;
	// the time the chip should wait each cycle before turning the pulse off
	uint16_t offDelay = onDelay + (uint16_t)(percent * 4096) - 1;
	// because the data sheet says to subtract 1 for proper function of the device,
	//   I have to check that the offDelay did not underflow from 0 -1 and become 65535
	if (offDelay > 4096)
		offDelay = 0;
	uint8_t onL = onDelay & 0xff;
	uint8_t onH = (onDelay & 0xff00) >> 8;
	uint8_t offL = offDelay & 0xff;
	uint8_t offH = (offDelay & 0xff00) >> 8;
	
	//printf("Setting motor to on for %d counts out of 4095\n", offDelay - onDelay);
	// write the on and off values to the corresponding registers
	// the 4 * address sum is used to offset the register to write to
	//   based on which PWM pin should be configured
	uint8_t onLowRegister = (uint8_t)(0x06 + (4 * address));
	
	uint8_t data[] = {onLowRegister, onL, onH, offL, offH};
	i2c_write(pwmDeviceAddress, data, 5);
}






