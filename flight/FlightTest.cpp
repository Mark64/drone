// this file contains the main function for performing tests
// It should be fully independent of the main function for the main
//   program

#include<iostream>
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<stdint.h>
#include<time.h>
#include<unistd.h>
#include<pthread.h>
#include<math.h>

#include "i2cctl.h"
#include "PWMController.h"
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

void interactiveMotorTest(uint8_t address) {
	printf("interactive motor tester\ntype 'x' to exit\n");
	printf("enter a thrust value between 0 - 1 with 1 being maximum thrust\n");
	while (1) {
		char percentString[100];
		scanf("%s", percentString);
		if (!strcmp(percentString, "x")) {
			setMotorThrustPercentage(address, 0);
			return;
		}
		
		double percent = atof(percentString);
		percent *= percent < 0 ? 0 : 1;
		
		while (percent > 1) {
			percent /= 10;
		}
		
		setMotorThrustPercentage(address, percent);
	}
}

void magAccelMotorTest(uint8_t address) {
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
		Vec3double acceleration = accelerationVector();
		printf("acceleration magnitude: %f\n", acceleration.magnitude());
		printf("XY angle: %f\n", acceleration.angleXYPlane());
		printf("XZ angle %f\n", acceleration.angleXZPlane());
		usleep(1000000);
	}
}

void interactivePWMTest(uint8_t address) {
	printf("interactive PWM tester\ntype 'x' to exit\n");
	printf("enter a thrust value between 0 - 1 with 1 being maximum thrust\n");
	while (1) {
		char percentString[100];
		scanf("%s", percentString);
		if (!strcmp(percentString, "x")) {
			setDutyPercent(address, 0);
			return;
		}
		
		double percent = atof(percentString);
		percent *= percent < 0 ? 0 : 1;
		
		setDutyPercent(address, percent);
	}
}

void calibrateMotors(uint8_t address[], int numMotors) {
	double maxThrottle = 0.6600;
	double minThrottle = 0.1000;
	
	for (int i = 0; i < numMotors; i++) {
		calibrateMotor(address[i]);
	}
}


void functionOverRange(void (*function)(uint8_t), uint8_t addresses[], uint8_t count) {
	for (int i = 0; i < count; i++) {
		function(addresses[i]);
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
		else if (strcmp(argv[i], "m") == 0) {
			magAccelTest();
		}
		else if (strcmp(argv[i], "t") == 0) {
			testMotor(atoi(argv[i+1]));
		}
		else if (strcmp(argv[i], "o") == 0) {
			magAccelMotorTest(atoi(argv[i+1]));
		}
		else if (strcmp(argv[i], "i") == 0) {
			interactiveMotorTest(atoi(argv[i+1]));
		}
		else if (strcmp(argv[i], "p") == 0) {
			interactivePWMTest(atoi(argv[i+1]));
		}
		else if (strcmp(argv[i], "c") == 0) {
			uint8_t motors[20];
			for (int j = i + 1; j < min(argc, 20); j++) {
				motors[j-i-1] = atoi(argv[j]);
			}
			calibrateMotors(motors, min(argc - i - 1, 20));
		}

	}
	if (argc == 1) {
		printf("enter arguments a, m, g, c, p, t <num>, o <num>, i <num>\n");
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







