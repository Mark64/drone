// implementation for the Orientation header
//
// by Mark Hill

#include<stdint.h>
#include<stdio.h>
#include<math.h>
#include<stdlib.h>
#include<unistd.h>
#include<time.h>
#include<pthread.h>

#include<SensorManager.h>
#include<Orientation.h>
#include<Eigen/Dense>
#include<geometry.h>

using namespace Eigen;
using namespace std;

////////////////////////
// calibration values //
////////////////////////

// the expected no motion value of the vector representing the force of gravity
static Vector3d _init_gravity;
// the magnitude of the static gravity vector
static double _init_gravity_length = 0;

// the average angular error, which is subtracted from any samples
static Vector3d _angular_drift;

// the inclination of the magnetic field in degrees below the
//   horizontal line
static double _inclinationAngle = 0;
// the magnitude of the magnetic field
// it makes more sense to use only the mag. and inclination instead of
//   a vector here since the magnitude and inclination are constant
//   unlike the rest of the components
static double _magneticFieldMagnitude = 0;

////////////////////////


////////////////////////
//      constants     //
////////////////////////

// this defines the "numVectors" value to be used during calibration
static const uint16_t _init_samples = 500;

// this defines the "numVectors" value to be used during update computation
// hopefully reduces noise
static const uint16_t _update_samples = 2;

// this variable is a flag indicating whether updates should
//   be run or that the system should be put to sleep
// 0 means stop, positive values mean go
static int volatile shouldUpdate = 0;
// these following values indicate the frequency with which
//   to run their corresponding functions
// values are in Hz
static const uint16_t accelerationUpdateFrequency = 400;
static const uint16_t headingUpdateFrequency = 30;
static const uint16_t altitudeUpdateFrequency = 30;

// values used in exponentially weighted moving average filter
static const double smoothing = 0.8;

// values used in sensor fusion
static const double gyroTrust = 0.9;

/////////////////////////


////////////////////////
//    working data    //
////////////////////////

// this struct contains the angular position of the device
// it is computed through a combination of the magnetometer
//   position and the integrated angle of the gyroscope
static struct Orientation _currentOrientation = {
	.acceleration = Vector3d(0, 0, 0),
	.gravity = Vector3d(0, 0, 0), 
	.heading = 0,
	.altitude = 0,
};
static struct Orientation _previousOrientation = {
	.acceleration = Vector3d(0, 0, 0),
	.gravity = Vector3d(0, 0, 0), 
	.heading = 0,
	.altitude = 0,
};


// this is the mutex lock for use when reading from or writing to
//   the Orientation structures
static pthread_mutex_t _orientationMutex;
static int _mutex_created = 0;

////////////////////////


////////////////////////
//     functions      //
////////////////////////

static void getAcceleration();
static double getAltitude();
static double degreesFromNorth();



// taken from i2cctl, gets the mutex lock to make
//   the _currentOrienation object accessible
static void getLock() {
	if (!_mutex_created) {
		pthread_mutex_init(&_orientationMutex, NULL);
		_mutex_created = 1;
	}
	pthread_mutex_lock(&_orientationMutex);
}

// taken from i2cctl, frees the mutex lock
static void releaseLock() {
	if (_mutex_created) {
		pthread_mutex_unlock(&_orientationMutex);
	}
}

// helper functions

// creates 'numVectors' vectors using the function passed in and returns a vector with
//   components that are averages of the 'numVectors' vectors
static Vector3d averageVector(Vector3d (*creation)(), uint16_t numVectors) {
	// stores the vectors created by the creationFunction
	Vector3d vectorArray[numVectors];

	// loop through 'numVectors' times, create a vector, and add it to the array
	for (int i = 0; i < numVectors; i++) {
		vectorArray[i] = creation();
	}

	// the vector in which to store the average components
	Vector3d average = Vector3d(0, 0, 0);

	// store the sums of the components in average
	for (int i = 0; i < numVectors; i++) {
		for (int j = 0; j < 3; j++) {
			average[j] += vectorArray[i][j];
		}
	}

	// divide the totals by the count
	average /= numVectors;

	return average;
}


// internal functions for setting defaults


// gets the average vector of the accelerometer
//   for use as a baseline acceleration at rest
// gives component values in g's
static void calibrateAccelerometer() {
	// gets the mutex lock for writing
	getLock();
	_init_gravity = averageVector(&accelerationVector, _init_samples);
	_init_gravity_length = _init_gravity.norm();
	_currentOrientation.gravity = _init_gravity;
	printVector(_init_gravity, "initial gravity");
	// done writing
	releaseLock();
}

// gets the expected angular drift (offset error)
static void calibrateGyroscope() {
	getLock();
	_angular_drift = averageVector(&rotationVector, _init_samples);
	printVector(_angular_drift, "angular drift");
	releaseLock();
}

// gets the inclination of the magnetic field
//   and returns a decimal value in degrees
// DEPENDS: calibrateAccelerometer
static void calibrateMagnetometer() {
	// retrieve the magnetic field average vector and its angle from horizontal
	Vector3d meanField = averageVector(&magneticField, _init_samples);

	// gets the mutex lock for writing
	getLock();

	// gets the inclination
	_inclinationAngle = angle_between(meanField, _init_gravity) - 90;
	printf("inclination is %f degrees below horizontal\n", _inclinationAngle);

	// done writing
	releaseLock();
}

// calibrates using the above internal functions
void calibrateSensors() {
	calibrateAccelerometer();
	calibrateGyroscope();
	calibrateMagnetometer();

	// populates the current orientation object
	getAcceleration();
	degreesFromNorth();
	getAltitude();
}



// functions for updating values

// returns the horizonal plane angle the device is relative to
//   a ray pointing towards magnetic North
static double degreesFromNorth() {
	Vector3d magField = averageVector(&magneticField, _update_samples);

	// this is actually not the correct way to get north
	getLock();
	_previousOrientation.heading = _currentOrientation.heading;
	_currentOrientation.heading = smoothing * angleXY(magField) + \
				      (1 - smoothing) * _previousOrientation.heading;
	releaseLock();

	return _currentOrientation.heading;
}

// returns the difference in the current time in seconds as compared to
//   the argument, which should be a pointer to a double containing the current
//   time in seconds
// for use only with the getAngularPosition function
double deltaTime(double *prev) {
	struct timespec currentTime;
	int failed = clock_gettime(CLOCK_MONOTONIC, &currentTime);
	if (failed) {
		printf("getting clock time failed \n");
		return 0;
	}

	double nanoSecondTime = currentTime.tv_sec + \
				(double)(currentTime.tv_nsec)/1000000000.0;
	double deltaTime = nanoSecondTime - *prev;

	// which means that this is the first time this function has been called,
	//   so there was really no comparison value in the first place
	// have to include floating point tolerance when dealing with small
	// values
	if (*prev <= 0.00001) {
		*prev = nanoSecondTime;
		return 0;
	}

	*prev = nanoSecondTime;
	return deltaTime;
}

// this variable stores the time the angularPosition was last updated in nanoseconds
// this is done because the rotation angle must be found using a vector
double angUpdateTime = 0;

// uses the gyroscope to obtain the current angular position
static Vector3d angPosGyro() {
	Vector3d rotation = averageVector(&rotationVector, \
						   _update_samples) - _angular_drift;

	getLock();
	double dt = deltaTime(&angUpdateTime);
	Vector3d gravity = _currentOrientation.gravity;
	releaseLock();

	rotation *= dt;
	AngleAxisd roll = AngleAxisd(rotation(0), Vector3d::UnitX());
	AngleAxisd pitch = AngleAxisd(rotation(1), Vector3d::UnitY());
	AngleAxisd yaw = AngleAxisd(rotation(2), Vector3d::UnitZ());
	AngleAxisd rMatrix;
	rMatrix = roll * pitch * yaw;

	return rMatrix * gravity;
}

// gets the current angular position relative to a ray pointing towards the ground
// uses sensor fusion of the accelerometer and gyro to compute the angular position
// expects a vector containing the accelerometer vector as the input
void getAngularPosition(Vector3d accel) {
	// uses the gyro and accelerometer obtained position values in combination
	Vector3d accelPos = _init_gravity_length * accel / accel.norm();
	Vector3d gyroPos = angPosGyro();
	double xPos = gyroPos(0) * gyroTrust + accelPos(0) * (1 - gyroTrust);
	double yPos = gyroPos(1) * gyroTrust + accelPos(1) * (1 - gyroTrust);
	double zPos = gyroPos(2) * gyroTrust + accelPos(2) * (1 - gyroTrust);

	double mag = Vector3d(xPos, yPos, zPos).norm();
	double magCoeff = _init_gravity_length / mag;

	Vector3d angPos = magCoeff * Vector3d(xPos, yPos, zPos);

	// updates the angular position
	getLock();
	_previousOrientation.gravity = _currentOrientation.gravity;
	_currentOrientation.gravity = smoothing * angPos + (1 - smoothing) * _previousOrientation.gravity;
	releaseLock();
}

// returns the acceleration vector adjusted for the position of ground
void getAcceleration() {

	// retrieve the acceleration value from the sensors
	Vector3d rawAcceleration = averageVector(&accelerationVector, _update_samples);
	// compute the angular position to obtain the gravity vector used later
	getAngularPosition(rawAcceleration);
	
	// creates a vector pointing in the direction of gravity with the magnitude measuring
	//   in the system's stationary state
	// needs the mutex lock

	Vector3d acc = rawAcceleration - _currentOrientation.gravity;

	getLock();
	// updates the internal orientation struct
	_previousOrientation.acceleration = _currentOrientation.acceleration;
	_currentOrientation.acceleration = smoothing * acc + (1 - smoothing) * _previousOrientation.acceleration;
	releaseLock();
}

// yes, I know this seems redunant, but it allow for easier modification
static double getAltitude() {
	double altitude = barometerAltitude();

	// updates the internal orientation struct
	getLock();
	_previousOrientation.altitude = _currentOrientation.altitude;
	_currentOrientation.altitude = smoothing * altitude + \
				(1 - smoothing) * _previousOrientation.altitude;
	releaseLock();

	return altitude;
}


// this function is executed infinitely while shouldUpdate
//   is a positive value
// setting shouldUpdate to 0 causes this function to put its
//   thread into a sleep state, waking up every few seconds to
//   check the shouldUpdate flag
// updates the acceleration and gravity member of the struct input
static void *updateAcceleration(void *input) {
	// this is the pointer the orientation struct which
	//   this function will record updates to
	struct Orientation *orientation = (struct Orientation *)(input);
	// stores the time the function should sleep between
	//   updates in microseconds
	uint32_t period = 1000000 / accelerationUpdateFrequency;
	double lastUpdateTime = 0;

	while (1) {
		while (!shouldUpdate) {
			// sleeps for 1.5 seconds
			usleep(1500000);
		}

		uint32_t sleepTime = period - deltaTime(&lastUpdateTime) * 1000000;
		if (sleepTime > 0)
			usleep(sleepTime);
		
		getAcceleration();
	}
	return NULL;
}


// this function is executed infinitely while shouldUpdate
//   is a positive value
// setting shouldUpdate to 0 causes this function to put its
//   thread into a sleep state, waking up every few seconds to
//   check the shouldUpdate flag
// updates the heading member of the struct input
static void *updateHeading(void *input) {
	// stores the time the function should sleep between
	//   updates in microseconds
	uint32_t period = 1000000 / accelerationUpdateFrequency;
	double lastUpdateTime = 0;

	while (1) {
		while (!shouldUpdate) {
			// sleeps for 1.5 seconds
			usleep(1500000);
		}

		uint32_t sleepTime = period - deltaTime(&lastUpdateTime) * 1000000;
		if (sleepTime > 0)
			usleep(sleepTime);
		
		degreesFromNorth();
	}
	return NULL;
}



// this function is executed infinitely while shouldUpdate
//   is a positive value
// setting shouldUpdate to 0 causes this function to put its
//   thread into a sleep state, waking up every few seconds to
//   check the shouldUpdate flag
// updates the altitude member of the struct input
static void *updateAltitude(void *input) {
	// stores the time the function should sleep between
	//   updates in microseconds
	uint32_t period = 1000000 / accelerationUpdateFrequency;
	double lastUpdateTime = 0;

	while (1) {
		while (!shouldUpdate) {
			// sleeps for 1.5 seconds
			usleep(1500000);
		}

		uint32_t sleepTime = period - deltaTime(&lastUpdateTime) * 1000000;
		if (sleepTime > 0)
			usleep(sleepTime);
		
		getAltitude();
	}
	return NULL;
}


// whenever a call is made to getOrienation, information
//   specific to each listener is passed into the
//   function as arguments
// the getOrientation function creates a struct which is
//   passed to the updateOrientation function
// this allows for numerous listeners
struct Observer {
	uint16_t frequency;
	int (*completionHandler)(struct Orientation);
};


// stores the thread pointers
// only one thread per Orientation struct member is used, which
//   allows for multiple listener functions without wasting resources
static pthread_t _accelerationThread = 0;
static pthread_t _altitudeThread = 0;
static pthread_t _headingThread = 0;
// stores the current number of listeners as an integer
static uint16_t _numListeners = 0;
// stores the max allowed number of listeners
static const uint16_t _maxListeners = 10;

// creates the threads for the orientation struct member
//   update functions
// if the threads already exist, simply returns 0 and acts
//   like it did something
// returns 0 on success and -1 on failure
int createStructMemberUpdateThreads() {
	int failure = initializeSensors();
	calibrateSensors();

	if (_accelerationThread == 0) {
		failure |= pthread_create(&_accelerationThread, NULL, &updateAcceleration, NULL);
	}
	if (_altitudeThread == 0) {
		failure |= pthread_create(&_altitudeThread, NULL, &updateAltitude, NULL);
	}
	if (_headingThread == 0) {
		failure |= pthread_create(&_headingThread, NULL, &updateHeading, NULL);
	}

	if (failure) {
		printf("failed to create struct member update threads\n");
		return -1;
	}

	return 0;
}

// gracefully exits (hopefully) or kills the orientation
//   struct member update threads
// if the threads are already dead, or if there are other
// active listeners, simply returns 0 and acts like it did something
// returns 0 on success and -1 on failure
int killStructMemberUpdateThreads() {
	if (_numListeners > 0) {
		return 0;
	}
	printf("last listener exited\nterminated orientation update threads\n");
	int returnValue = 0;
	if (_accelerationThread != 0) {
		int failure = pthread_cancel(_accelerationThread);
		if (!failure) {
			_accelerationThread = 0;
		}
		else {
			printf("failed to cancel acceleration thread\n");
			returnValue = -1;
		}
	}
	if (_altitudeThread != 0) {
		int failure = pthread_cancel(_altitudeThread);
		if (!failure) {
			_altitudeThread = 0;
		}
		else {
			printf("failed to cancel altitude thread\n");
			returnValue = -1;
		}
	}
	if (_headingThread != 0) {
		int failure = pthread_cancel(_headingThread);
		if (!failure) {
			_headingThread = 0;
		}
		else {
			printf("failed to cancel heading thread\n");
			returnValue = -1;
		}
	}
	
	deinitializeSensors();

	return returnValue;
}

// handles calling the completion handler of the getOrientation
//   function, along with update sleep mode
// when the completionHandler requests termination,
//   checks the number of other listeners and if it is the
//   last listener, terminates the orienation struct member
//   update threads
// the argument must be an OrienationUpdateInformation struct
//   containing the information to be used on this thread
void *updateOrientation(void *input) {
	struct Observer threadInfo = *((struct Observer *)(input));
	free(input);
	// increment the number of listeners
	_numListeners++;

	// creates the threads if they haven't been created
	int failure = createStructMemberUpdateThreads();
	if (failure) {
		printf("update listener terminated due thread spawn error\n");
		_numListeners--;
		killStructMemberUpdateThreads();
		pthread_exit(NULL);
	}

	// waits the desired interval, then passes the current
	//   orientation struct to the completion handler
	// if the completion handler requests termination,
	//   terminates on the next loop
	uint32_t waitTimeMicroseconds = 1000000/threadInfo.frequency;
	double lastUpdateTime = 0;
	deltaTime(&lastUpdateTime);
	int completion = 0;

	while (1) {
		getLock();
		completion = threadInfo.completionHandler(_currentOrientation);
		releaseLock();

		if (completion < 0) {
			printf("exit requested\nending listener thread\n");
			_numListeners--;
			killStructMemberUpdateThreads();
			pthread_exit(NULL);
		}
		else if (completion > 0) {
			threadInfo.frequency = completion;
			waitTimeMicroseconds = 1000000/completion;
		}

		uint32_t sleepTimeMicro = waitTimeMicroseconds - (deltaTime(&lastUpdateTime) * 1000000);
		if (sleepTimeMicro > 0)
			usleep(sleepTimeMicro);
	}

	return NULL;
}

// retrieves the orientation of the device and passes an Orientation
//   struct into the passed in completion handler function
// returns 0 on success and -1 on failure
int getOrientation(int (*completion)(struct Orientation), uint16_t updateRate) {
	// if listener count is exceed, return failure code
	if (_numListeners >= _maxListeners) {
		printf("call to getOrientation exited due to listener count exceeding maximum allowed count\n");
		return -1;
	}
	// set the update information struct
	struct Observer *threadInfo = \
			(struct Observer *)(malloc(sizeof(struct Observer)));
	threadInfo->frequency = updateRate;
	threadInfo->completionHandler = completion;
	// set the shouldUpdate flag
	shouldUpdate = 1;

	// spin up thread for orientation updates
	pthread_t orienatationThread;
	int failure = pthread_create(&orienatationThread, NULL, \
				     &updateOrientation, threadInfo);
	if (failure) {
		printf("failed to create orientation thread\n");
		return -1;
	}

	return 0;
}

#define ACCELERATION_COLOR "\x1B[1m" // bold
#define GRAVITY_COLOR "\x1B[1m" // bold
#define HEADING_COLOR "\x1B[1m" // bold
#define ALTITUDE_COLOR "\x1B[1m" // bold
#define NORMAL_COLOR "\x1B[0m" // normal text color

// prints out the orientation struct
void printOrientation(struct Orientation o) {
	char a[50], g[50];
	sprintf(a, "\n%sacceleration%s", ACCELERATION_COLOR, NORMAL_COLOR);
 	sprintf(g, "%sgravity%s", GRAVITY_COLOR, NORMAL_COLOR);	
	printVector(o.acceleration, a);
	printVector(o.gravity, g);
	printf("%sdegrees from North%s\n %f\n", HEADING_COLOR, NORMAL_COLOR, o.heading);
	printf("%saltitude%s\n %f\n", ALTITUDE_COLOR, NORMAL_COLOR, o.altitude);
}






