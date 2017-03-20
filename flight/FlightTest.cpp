// this file contains the main function for performing tests
// It should be fully independent of the main function for the main
//   program

#include<iostream>
#include<stdio.h>
#include<stdint.h>
#include<time.h>
#include<unistd.h>

#include "i2cctl.h"
#include "SensorManager.h"
#include "MotorController.h"

using namespace std;

// gets the linear acceleration values for each axis and prints them to standard output
void testAccel() {
	Vec3double acc = accelerationVector();
	cout << "Acceleration ";
	acc.description();
}

// gets the angular rotation values for each axis and prints them to standard output
void testGyro() {
	Vec3double gyro = rotationVector();
	cout << "Rotation ";
	gyro.description();
}

// gets the average angular rotation value for the selected axis (0=x, 1=y, 2=z)
double averageGyro(int axis) {
	double sum = 0;
	int n = 25;
	for (int i = 0; i < n; i++) {
		switch (axis) {
			case 2:
				sum += rotationVector().z;
				break;
			case 1:
				sum += rotationVector().y;
				break;
			default:
				sum += rotationVector().x;
				break;
		}
	}
	return sum/n;
}

// gets the average linear acceleration value for the selected axis (0=x, 1=y, 2=z)
double averageAcceleration(int axis) {
	double sum = 0;
	int n = 5;
	for (int i = 0; i < n; i++) {
		switch (axis) {
			case 2:
				sum += accelerationVector().z;
				break;
			case 1:
				sum += accelerationVector().y;
				break;
			default:
				sum += accelerationVector().x;
				break;
		}
	}
	return sum/n;
}

// tests the number of i2c reads per second
int readsPerSecond() {
	int count = 0;
	time_t startTime = time(NULL);
	
	//uint16_t address = 0x6b;
	//uint8_t registers[] = {0x29, 0x28, 0x2b, 0x2a, 0x2d, 0x2c};
	for (; count < 20000; count++) {
		//uint32_t results[6];
		int value = 0;
		accelerationVector();
		//i2cMultiWordRead(address, registers, 6, results);
		if (value == -1) {
			count--;
		}
	}

	time_t endTime = time(NULL);

	return  (count / difftime(endTime, startTime));
}

void testMotor(uint8_t address) {
	// scale
	int n = 10000;
	// increment value
	int a = 1;
	// increasing test
	for (int i = 0; i <= n; i += a) {
		setMotorThrustPercentage(address, i * 1/((double)n));
		//sleep(1);
	}
	// decreasing test
	for (int i = n; i >= 0; i -= a) {
		setMotorThrustPercentage(address, i * 1/((double)n));
		//sleep(1);
	}
	printf("Motor %d test completed\n", address);

}

void manualMotorTest(uint8_t address) {
	printf("\nInteractive Motor Tester\n");
	printf("Enter a thrust value between 0 - 1 with 1 being maximum thrust\n");
	while (1) {
		float percent = 0;
		scanf("%f", &percent);
		setMotorThrustPercentage(address, percent);
	}
}

int main() {
	testAccel();
	testGyro();
	//printf("%d rps\n", readsPerSecond());
	testMotor(0);
	manualMotorTest(0);
	return 0;
}
