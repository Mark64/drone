// this file contains the main function for performing tests
// It should be fully independent of the main function for the main
//   program

#include<iostream>
#include<stdio.h>
#include<stdint.h>

#include "i2cctl.h"
#include "SensorManager.h"


using namespace std;


void testAccel(uint16_t address, uint8_t reg[], uint8_t numRegisters, uint32_t val) {
	uint32_t result = i2cRead(address, reg, numRegisters);
	printf("read: %x\n", result);
	int success = i2cWrite(address, reg, numRegisters, val);
	printf("write success: %d\ncurrent read: %x\n", success, i2cRead(address, reg, numRegisters));
	Vec3double acc = accelerationVector();
	acc.description();
}

int main() {
	uint8_t reg[] = {0x10};
	testAccel(0x6b, reg, 1, 0x74);


	return 0;
}
