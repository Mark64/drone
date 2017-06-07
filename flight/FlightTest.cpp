// this file contains the main function for performing tests
// It should be ful(1)independent of the main function for the main
//   program

#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<stdint.h>
#include<sys/time.h>
#include<unistd.h>
#include<pthread.h>
#include<math.h>

#include<Eigen/Dense>
#include<geometry.h>
#include<i2cctl.h>
#include<PWMController.h>
#include<SensorManager.h>
#include<MotorController.h>
#include<Orientation.h>
#include<FlightManager.h>

#define HEADING_COLOR "\x1B[1m" // bold
#define NORMAL_COLOR "\x1B[0m" // normal text

using namespace Eigen;


void testFlightManager() {
	if (startFlightManager()) {
		printf("failed to start flight manager\n");
	}
	while (1) {
		usleep(50000);
	}
}

int orientationCompletionHandler(struct Orientation orientation) {
	printOrientation(orientation);
	return 0;
}

static int canReturn = 0;
static int N = 1000;
static int hz = 100;
static struct Orientation totals = {};
// not actually variance, variance * (n + 1)
static struct Orientation variance = {};
static int i = 1;

Vector3d pow(Vector3d v, double e) {
	for (int i = 0; i < 3; i++) {
		v(i) = pow(v(i), e);
	}
	return v;
}

int orientationStatisticsCompletionHandler(struct Orientation orientation) {
	variance.acceleration += \
		pow(i * orientation.acceleration - totals.acceleration, 2) / (i * (i + 1));
	variance.gravity += \
		pow(i * orientation.gravity - totals.gravity, 2) / (i * (i + 1));
	variance.heading += \
		pow(i * orientation.heading - totals.heading, 2) / (i * (i + 1));
	variance.altitude += \
		pow(i * orientation.altitude - totals.altitude, 2) / (i * (i + 1));	

	totals.acceleration += orientation.acceleration;
	totals.gravity += orientation.gravity;
	totals.heading += orientation.heading;
	totals.altitude += orientation.altitude;

	i++;
	if (i == N) {
		totals.acceleration /= N;
		totals.gravity /= N;
		totals.heading /= N;
		totals.altitude /= N;

		variance.acceleration = pow(variance.acceleration / (i + 1), 0.5);
		variance.gravity = pow(variance.gravity / (i + 1), 0.5);
		variance.heading = pow(variance.heading / (i + 1), 0.5);
		variance.altitude = pow(variance.altitude / (i + 1), 0.5);

		printf("%saverages%s\n", HEADING_COLOR, NORMAL_COLOR);
		orientationCompletionHandler(totals);

		printf("%sstandard deviations%s\n", HEADING_COLOR, NORMAL_COLOR);
		orientationCompletionHandler(variance);

		canReturn = -1;
	}
	return canReturn;
}

void testOrientationStatistics() {
	printf("testing orientation - stats mode\n");
	printf("calibrating...\n");
	calibrateSensors();
	printf("begining orientation statistics printout of %i samples at %iHz\n", N, hz);
	getOrientation(&orientationStatisticsCompletionHandler, hz);
	while (!canReturn) {
		sleep(1);
	}
}

void testEmergencyStop() {
	printf("performing emergen(1)stop on motors\n");
	for (int i = 0; i < 4; i++) {
		setMotorThrustPercentage(i, 0);
	}
}

void testOrientation() {
	printf("testing orientation\n");
	printf("calibrating...\n");
	calibrateSensors();
	printf("begin printing orientation @ 1Hz\n");
	sleep(1);
	getOrientation(&orientationCompletionHandler, 1);
	while (!canReturn) {
		sleep(1);
	}
}

void testOrientationCalibration() {
	printf("testing orientation calibration\n");
	calibrateSensors();
	printf("done\n");
}

void testStaticVectorLinearMotion() {
	printf("linear motion vector tester\n");
	printf("enter linear motion vector components a x.xx, y.(1)z.(2)then press enter\n");

	for (int i = 0; i < 3; i++) {
		//scanf("%f", vectorComponents);
	}

	Vector3d target = Vector3d(0.0, 0.0, 0.17);
	setLinearMotionVector(target);

	char *f = NULL;
	scanf("%s", f);
	target = Vector3d(0, 0, 0);
	setLinearMotionVector(target);
}

void testStaticVectorAngularMotion() {
	printf("angular motion vector tester\n");
	printf("enter angular motion vector component z.(2)then press enter\n");

	long double vectorComponents[1];
	for (int i = 0; i < 1; i++) {
		scanf("%Lf", vectorComponents);
	}

	Vector3d target = Vector3d(0, 0, -0.2);

	setAngularMotionVector(target);
}

// gets the linear acceleration values for each axis and prints them to standard output
void testAccel() {
	Vector3d acc = accelerationVector();
	printVector(acc, "acceleration");
}

// gets the angular rotation values for each axis and prints them to standard output
void testGyro() {
	Vector3d gyro = rotationVector();
	printVector(gyro, "rotation");
}

// gets the average angular rotation value for the selected axis (0=x, 1=y, 2=z)
double averageGyro(int axis) {
	double sum = 0;
	int n = 25;
	for (int i = 0; i < n; i++) {
		switch (axis) {
			case 2:
				sum += rotationVector()(2);
				break;
			case 1:
				sum += rotationVector()(1);
				break;
			default:
				sum += rotationVector()(0);
				break;
		}
	}
	return sum/n;
}

// gets the average linear acceleration value for the selected axis (0=x, 1=y, 2=z)
void averageAcceleration() {
	double sum = 0;
	int n = 50000;
	for (int i = 0; i < n; i++) {
		Vector3d a = accelerationVector();
		sum += a.norm();
	}
	printf("average acceleration %f\n", sum/n);
}

// tests the number of i2c reads per second
void readsPerSecond() {
	int count = 0;
	struct timeval startTime, endTime;

	gettimeofday(&startTime, NULL);

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

	gettimeofday(&endTime, NULL);

	double diffTime = (double)((endTime.tv_sec * 1000000 + endTime.tv_usec) - (startTime.tv_sec * 1000000 + startTime.tv_usec)) / 1000000;

	printf("%.2f reads per second\n", (double)(count) / diffTime);
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
		Vector3d acc = accelerationVector();
		double mag = acc.norm();
		double thrust = pow((mag - minAccel)/diff, 2);
		//printf("%f, %f\n", mag, thrust);
		setMotorThrustPercentage(a, thrust);
	}
}

void sensorTest() {
	while (1) {
		Vector3d acceleration = accelerationVector();
		Vector3d rotation = rotationVector();
		Vector3d magField = magneticField();

		printVector(acceleration, "acceleration");

		printVector(rotation, "rotation");

		printVector(magField, "magnetic field");

		printf("altitude %.3f\n\n", barometerAltitude());

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
	for (int i = 0; i < numMotors; i++) {
		calibrateMotor(address[i]);
	}
}


void functionOverRange(void (*function)(uint8_t), uint8_t addresses[], uint8_t count) {
	for (int i = 0; i < count; i++) {
		function(addresses[i]);
	}
}

void testMagnetometer() {
		Vector3d magField = magneticField();
		printVector(magField, "magnetic field");
//		usleep(1000000);
}

void averageMagneticField() {
	double x = 0, y = 0, z = 0;
	int n = 10000;
	for (int i = 0; i < n; i++) {
		Vector3d mag = magneticField();
		x += mag(0);
		y += mag(1);
		z += mag(2);
	}
	
	Vector3d mag = Vector3d(x, y, z);
	mag /= n;
	
	printf("average components:\n  x: %f\n  y: %f\n  z: %f\n", x, y, z);
}

void testBarometer() {
	double altitude = 0;
	int n = 1;
	for (int i = 0; i < n; i++) {
		altitude += barometerAltitude();
	}
	altitude /= n;
	printf("altitude %.3f meters\n", altitude);
}

int main(int argc, char * argv[]) {

	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "a") == 0) {
			testAccel();
		}
		else if (strcmp(argv[i], "g") == 0) {
			testGyro();
		}
		else if (strcmp(argv[i], "s") == 0) {
			sensorTest();
		}
		else if (strcmp(argv[i], "m") == 0) {
			testMagnetometer();
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
			int minArguments = argc - 20 > 0 ? 20 : argc;
			for (int j = i + 1; j < minArguments; j++) {
				motors[j-i-1] = atoi(argv[j]);
			}
			calibrateMotors(motors, minArguments - 1 - i);
		}
		else if (strcmp(argv[i], "r") == 0) {
			readsPerSecond();
		}
		else if (strcmp(argv[i], "b") == 0) {
			testBarometer();
		}
		else if (strcmp(argv[i], "slv") == 0) {
			testStaticVectorLinearMotion();
		}
		else if (strcmp(argv[i], "sav") == 0) {
			testStaticVectorAngularMotion();
		}
		else if (strcmp(argv[i], "am") == 0) {
			averageMagneticField();
		}
		else if (strcmp(argv[i], "aa") == 0) {
			averageAcceleration();
		}
		else if (strcmp(argv[i], "oc") == 0) {
			testOrientationCalibration();
		}
		else if (strcmp(argv[i], "oo") == 0) {
			testOrientation();
		}
		else if (strcmp(argv[i], "x") == 0) {
			testEmergencyStop();
		}
		else if (strcmp(argv[i], "os") == 0) {
			testOrientationStatistics();
		}
		else if (strcmp(argv[i], "fm") == 0) {
			testFlightManager();
		}

	}
	if (argc == 1) {
		printf("enter arguments fm, os, x, aa, oo, am, sav, slv, r, m, a, s, g, c, p, t <num>, o <num>, i <num>\n");
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







