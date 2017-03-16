// this file contains the main function for performing tests
// It should be fully independent of the main function for the main
//   program

#include<iostream>
#include<stdio.h>
#include<stdint.h>

#include "flight/i2cctl.h"
#include "flight/SensorManager.h"


using namespace std;


void testAccel() {
	Vec3double acc = accelerationVector();
	cout << "Acceleration ";
	acc.description();
}

void testGyro() {
	Vec3double gyro = rotationVector();
	cout << "Rotation ";
	gyro.description();
}

int main() {
	testAccel();
	testGyro();

	return 0;
}
