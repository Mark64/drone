// this file contains the main function for performing tests
// It should be fully independent of the main function for the main
//   program

#include<iostream>
#include<stdio.h>
#include<string.h>
#include<stdint.h>
#include<time.h>
#include<unistd.h>
#include<pthread.h>
#include<math.h>

#include "i2cctl.h"
#include "SensorManager.h"
#include "MotorController.h"

using namespace std;

// gets the linear acceleration values for each axis and prints them to standard output
void testAccel() {
	Vec3double acc = accelerationVector();
	cout << "acceleration ";
	acc.description();
}

// gets the angular rotation values for each axis and prints them to standard output
void testGyro() {
	Vec3double gyro = rotationVector();
	cout << "rotation ";
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
	printf("beginning test on motor %d\n", address);
	printf("increasing motor speed\n");
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
	printf("decreasing motor speed\n");	
	for (int i = n; i >= 0; i -= a) {
		setMotorThrustPercentage(address, i * 1/((double)n));
		//sleep(1);
	}
	printf("Motor %d test completed\n", address);

}

void manualMotorTest(uint8_t address) {
	printf("interactive motor tester\n");
	printf("enter a thrust value between 0 - 1 with 1 being maximum thrust\n");
	while (1) {
		float percent = 0;
		scanf("%f", &percent);
		setMotorThrustPercentage(address, percent);
	}
}

void * magAccelMotorTest(void *address) {
	printf("orientation based motor tester\n");
	printf("magnitude mode\n");
	double maxAccel = 2.6;
	double minAccel = 0.0;
	double diff = maxAccel - minAccel;
	uint8_t a = (uint8_t)((intptr_t)address);
	while (1) {
		usleep(500);
		double mag = accelerationVector().magnitude();
		double thrust = powf((mag - minAccel)/diff, 2);
		//printf("%f, %f\n", mag, thrust);
		setMotorThrustPercentage(a, thrust);
	}
}

void magAccelTest() {
	while (1) {
		usleep(500000);
		printf("acceleration magnitude: %f\n", accelerationVector().magnitude());
	}
}

int main(int argc, char * argv[]) {
	
	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "a") == 0) {
			testAccel();
		}
		else if (strcmp(argv[i], "g") == 0) {
			testGyro();
		}
		else if (strcmp(argv[i], "t") == 0) {
			testMotor((long int)argv[i+2]);
		}
		else if (strcmp(argv[i], "o") == 0) {
			magAccelMotorTest(argv[i+2]);
		}
		else if (strcmp(argv[i], "i") == 0) {
			manualMotorTest((intptr_t)argv[i+2]);
			}

	}
	if (argc == 1) {
		printf("enter arguments a, g, t <num>, o <num>, i <num>\n");
	}
	
	//testAccel();
	//testGyro();
	//printf("%d rps\n", readsPerSecond());
	//testMotor(0);
	//magAccelTest();
	
	//void * (*func)(void *) = &magAccelMotorTest;
	//pthread_t accelThread;
	//uint8_t argument = 0;
	//pthread_create(&accelThread, NULL, func, &argument);
	
	//manualMotorTest(0);
	return 0;
}







