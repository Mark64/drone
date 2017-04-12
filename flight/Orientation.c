// implementation for the Orientation header
//
// by Mark Hill

#include "Orientation.h"

#include<stdint.h>
#include<stdio.h>
#include<stdlib.h>
#include<unistd.h>
#include<time.h>
#include<pthread.h>

#include "SensorManager.h"

// get inclination
// get acceleration magnitude
// get average angular drift
// get baseline height
// convert vectors to 0 angle
//
// corrected acceleration
// altitude
// degrees from north
// angle from ground

// calibration values

// the expected no motion value of the vector representing the force of gravity
static struct Vec3double _stationaryAcceleration;

// the expected deviation from 0 angular spin as measured by
//   the gyroscope
static struct Vec3double _angularDrift;

// the inclination of the magnetic field in degrees from a
//   horizontal line
static double _inclinationAngle = 0;
// the magnitude of the magnetic field
// it makes more sense to use only the mag. and inclination instead of
//   a vector here since the magnitude and inclination are constant
//   unlike the rest of the components
static double _magneticFieldMagnitude = 0;

// this defines the "numVectors" value to be used during calibration
static const uint16_t _calibrationVectorCount = 3000;

// this defines the "numVectors" value to be used during update computation
// hopefully reduces noise
static const uint16_t _updateVectorCount = 3;

// this variable is a flag indicating whether updates should
//   be run or that the system should be put to sleep
// 0 means stop, positive values mean go
static int shouldUpdate = 0;
// these following values indicate the frequency with which
//   to run their corresponding functions
// values are in Hz
static const uint16_t accelerationUpdateFrequency = 800;
static const uint16_t angularPositionUpdateFrequency = 1600;
static const uint16_t altitudeUpdateFrequency = 2;


// stores the current orientation

// this struct contains the angular position of the device
// it is computed through a combination of the magnetometer
//   position and the integrated angle of the gyroscope
static struct Orientation _currentOrientation = {{0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, 0};
// this is the mutex lock for use when reading from or writing to
//   the _currentOrientation structure
static pthread_mutex_t _currentOrientationMutex;
static int _mutex_created = 0;

// taken from i2cctl, gets the mutex lock to make
//   the _currentOrienation object accessible
static void getLock() {
	if (!_mutex_created) {
		pthread_mutex_init(&_currentOrientationMutex, NULL);
		_mutex_created = 1;
	}
	pthread_mutex_lock(&_currentOrientationMutex);
}

// taken from i2cctl, frees the mutex lock
static void releaseLock() {
	if (_mutex_created) {
		pthread_mutex_unlock(&_currentOrientationMutex);
	}
}

// helper functions

// creates 'numVectors' vectors using the function passed in and returns a vector with
//   components that are averages of the 'numVectors' vectors
static struct Vec3double averageVector(struct Vec3double (*creationFunction)(), uint16_t numVectors) {
	// stores the vectors created by the creationFunction
	struct Vec3double vectorArray[numVectors];

	// loop through 'numVectors' times, create a vector, and add it to the array
	for (int i = 0; i < numVectors; i++) {
		vectorArray[i] = creationFunction();
	}

	// the vector in which to store the average components
	struct Vec3double average = vectorFromComponents(0, 0, 0);

	// store the sums of the components in average
	for (int i = 0; i < numVectors; i++) {
		average.x += vectorArray[i].x;
		average.y += vectorArray[i].y;
		average.z += vectorArray[i].z;
	}

	// divide the totals by the count
	average.x /= numVectors;
	average.y /= numVectors;
	average.z /= numVectors;

	return average;
}


// internal functions for setting defaults


// gets the average vector of the accelerometer
//   for use as a baseline acceleration at rest
// gives component values in g's
static void calibrateAccelerometer() {
	// gets the mutex lock for writing
	getLock();

	_stationaryAcceleration = averageVector(&accelerationVector, _calibrationVectorCount);
	// forces computation of the computed struct members
	magnitude(&_stationaryAcceleration);
	angleXZPlane(&_stationaryAcceleration);
	angleYZPlane(&_stationaryAcceleration);
	angleXYPlane(&_stationaryAcceleration);

	// done writing
	releaseLock();
}

// gets the inclination of the magnetic field
//   and returns a decimal value in degrees
// DEPENDS: calibrateAccelerometer
static void calibrateMagnetometer() {
	// retrieve the magnetic field average vector and its angle from horizontal
	struct Vec3double averageMagneticField = averageVector(&magneticField, _calibrationVectorCount);
	
	// gets the mutex lock for writing
	getLock();

	// gets the inclination
	_inclinationAngle = 90 - angleBetweenVectors(&_stationaryAcceleration, &averageMagneticField);
	// gets the magnitude of the field
	_magneticFieldMagnitude = magnitude(&averageMagneticField);

	printf("inclination is %f degrees above horizontal\n", _inclinationAngle);

	// done writing
	releaseLock();
}

// gets the expected angular drift of the gyroscope
//   to use as an expected value in calculations
// gives value in degrees per second
// retrieves the current angular position relative to ground
// DEPENDS: calibrateAccelerometer, calibrateMagnetometer
static void calibrateGyroscope() {
	// the angular drift has the unique property that, when
	//   the device is assumed to be motionless, the average
	//   rotational vector is equal to the angular drift since
	//   the average vector should be <0, 0, 0>
	// uses 2x vectors because I want double the data
	uint16_t numVectors = 2 * _calibrationVectorCount;
	_angularDrift = averageVector(&rotationVector, numVectors);
	printf("angular drift ");
	printVector(&_angularDrift);
	
	// gets the mutex lock for writing
	getLock();

	// notice that the z component is set to 0.  That value isn't determined by
	//   integration of the gyroscope, but instead by the actual reading
	//   from the magnetometer, so it can be set to 0 during calibration
	// the 90 - x is used to convert the angles from a horizontal axis representation into a -z axis representation
	_currentOrientation.angularPosition = vectorFromComponents(_stationaryAcceleration.angleYZ - 90, _stationaryAcceleration.angleXZ - 90, 0);
	
	printf("angular position ");
	printVector(&_currentOrientation.angularPosition);
	// releases the mutex lock
	releaseLock();
}

// calibrates using the above internal functions
void calibrateSensors() {
	// calculates the magnetic field inclination and stores it in
	//   the static variable inclination
	calibrateAccelerometer();
	calibrateMagnetometer();
	calibrateGyroscope();
}



// functions for updating values

// returns the horizonal plane angle the device is relative to
//   a ray pointing towards magnetic North
static double degreesFromNorth() {
	struct Vec3double magField = averageVector(&magneticField, _updateVectorCount);
	
	// this is actually not the correct way to get north
	return angleXYPlane(&magField);
}

// this variable stores the time the angularPosition was last updated in microseconds
// this is done because the rotation angle must be found using a vector
double lastUpdateTime = 0;
// returns the difference in the current time as compared to the variable
//   lastUpdateTime
// for use only with the getAngularPosition function
double deltaTime() {
	struct timespec currentTime;
	int failed = clock_gettime(CLOCK_MONOTONIC, &currentTime);
	if (failed) {
		printf("getting clock time failed \n");
		return 0;
	}

	double nanoSecondTime = currentTime.tv_sec + (double)(currentTime.tv_nsec)/1000000000.0;
	double deltaTime = nanoSecondTime - lastUpdateTime;
	
	// which means that this is the first time this function has been called,
	//   so there was really no comparison value in the first place
	// have to include floating point tolerance when dealing with small
	// values
	if (lastUpdateTime <= 0.00001) {
		lastUpdateTime = nanoSecondTime;
		return 0;
	}	

	lastUpdateTime = nanoSecondTime;
	return deltaTime;
}
// gets the standing angular position
// has the assumption that the device is relatively motionless
// can be at any angle though since it uses the
//   accelerometer to determine this value
static struct Vec3double getAngularPosition() {
	struct Vec3double rotation = averageVector(&rotationVector, _updateVectorCount);

	double rotationVelocityXZ = rotation.y - _angularDrift.y;
	double rotationVelocityYZ = rotation.x - _angularDrift.x;
	double rotationVelocityXY = rotation.z - _angularDrift.z;

	// gets the mutex lock for reading
	getLock();
	
	double dT = deltaTime();
	
	double angleXZ = rotationVelocityXZ * dT + _currentOrientation.angularPosition.y;
	double angleYZ = rotationVelocityYZ * dT + _currentOrientation.angularPosition.x;
	// the XY angle is also updated separately by degreesFromNorth(). so it is usually
	//   more accurate than the other angles
	double angleXY = rotationVelocityXY * dT + _currentOrientation.angularPosition.z;

	struct Vec3double currentAngularPosition = vectorFromComponents(angleYZ, angleXZ, angleXY);

	
	// updates the angular position
	_currentOrientation.angularPosition = currentAngularPosition;
	releaseLock();

	return currentAngularPosition;
}

// returns the acceleration vector adjusted for the position of ground
static struct Vec3double correctedAcceleration() {
	// retrieve the acceleration value from the sensors
	struct Vec3double rawAcceleration = averageVector(&accelerationVector, _updateVectorCount);
	// creates a vector pointing in the direction of gravity with the magnitude measuring
	//   in the system's stationary state
	// x - 90 is used because the angles need to be converted back into horizontal axis representation
	// needs the mutex lock
	getLock();
	double angleXZ = _currentOrientation.angularPosition.y + 90;
	double angleYZ = _currentOrientation.angularPosition.x + 90;
	double magnitude = _stationaryAcceleration.magnitude;
	struct Vec3double gravity = vectorFromAnglesAndMagnitude(angleXZ, angleYZ, magnitude);
	
	struct Vec3double acceleration = vectorFromSubtractingVectors(&rawAcceleration, &gravity);
	
	// updates the internal orientation struct
	_currentOrientation.acceleration = acceleration;
	releaseLock();

	return acceleration;
}	

// yes, I know this seems redunant, but it allow for easier modification
static double getAltitude() {
	double altitude = barometerAltitude();

	// updates the internal orientation struct
	getLock();
	_currentOrientation.altitude = altitude;
	releaseLock();

	return altitude;
}


// this function is executed infinitely while shouldUpdate
//   is a positive value
// setting shouldUpdate to 0 causes this function to put its
//   thread into a sleep state, waking up every few seconds to
//   check the shouldUpdate flag
// expects an Orientation struct pointer as the input
// returns NULL when it does return
// updates the acceleration member of the struct input
static void *updateAcceleration(void *input) {
	if (input == NULL) {
		printf("invalid input to updateAcceleration\n");
		return NULL;
	}
	
	// this is the pointer the orientation struct which
	//   this function will record updates to
	struct Orientation *orientation = (struct Orientation *)(input);
	// stores the time the function should sleep between
	//   updates in microseconds
	uint16_t sleepTime = (uint16_t)(1000000.0/(double)(accelerationUpdateFrequency));

	while (1) {
		while (!shouldUpdate) {
			// sleeps for 1.5 seconds
			usleep(1500000);
		}
		
		orientation->acceleration = correctedAcceleration();
		
		// sleep to prevent CPU waste since the
		//   sensors only have limited update frequency
		usleep(sleepTime);
	}
	return NULL;
}

// this function is executed infinitely while shouldUpdate
//   is a positive value
// setting shouldUpdate to 0 causes this function to put its
//   thread into a sleep state, waking up every few seconds to
//   check the shouldUpdate flag
// expects an Orientation struct pointer as the input
// returns NULL when it does return
// updates the angular position member of the struct input
static void *updateAngularPosition(void *input) {
	if (input == NULL) {
		printf("invalid input to updateAngularPosition\n");
		return NULL;
	}
	
	// this is the pointer the orientation struct which
	//   this function will record updates to
	struct Orientation *orientation = (struct Orientation *)(input);
	// stores the time the function should sleep between
	//   updates in microseconds
	uint16_t sleepTime = (uint16_t)(1000000.0/(double)(angularPositionUpdateFrequency));

	while (1) {
		while (!shouldUpdate) {
			// sleeps for 1.5 seconds
			usleep(1500000);
		}
		
		orientation->angularPosition = getAngularPosition();
		
		// sleep to prevent CPU waste since the
		//   sensors only have limited update frequency
		usleep(sleepTime);
	}
	return NULL;
}

// this function is executed infinitely while shouldUpdate
//   is a positive value
// setting shouldUpdate to 0 causes this function to put its
//   thread into a sleep state, waking up every few seconds to
//   check the shouldUpdate flag
// expects an Orientation struct pointer as the input
// returns NULL when it does return
// updates the altitude member of the struct input
static void *updateAltitude(void *input) {
	if (input == NULL) {
		printf("invalid input to updateAltitude\n");
		return NULL;
	}
	
	// this is the pointer the orientation struct which
	//   this function will record updates to
	struct Orientation *orientation = (struct Orientation *)(input);
	// stores the time the function should sleep between
	//   updates in microseconds
	uint16_t sleepTime = (uint16_t)(1000000.0/(double)(altitudeUpdateFrequency));

	while (1) {
		while (!shouldUpdate) {
			// sleeps for 1.5 seconds
			usleep(1500000);
		}
		
		orientation->altitude = getAltitude();
		
		// sleep to prevent CPU waste since the
		//   sensors only have limited update frequency
		usleep(sleepTime);
	}
	return NULL;
}


// whenever a call is made to getOrienation, information 
//   specific to each listener is passed into the 
//   function as arguments
// the getOrientation function creates a struct which is
//   passed to the updateOrientation function
// this allows for numerous listeners
struct OrientationUpdateInformation {
	uint16_t desiredOrientationUpdateRate;
	int (*completionHandler)(struct Orientation);
};


// common orientation structure to be used by the member update functions as
//   the sole place to store updates
static struct Orientation _commonOrientation = {{0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, 0};
// stores the thread pointers
// only one thread per Orientation struct member is used, which
//   allows for multiple listener functions without wasting resources
static pthread_t _accelerationThread = 0;
static pthread_t _angularPositionThread = 0;
static pthread_t _altitudeThread = 0;
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
	int failure = 0;
	if (_accelerationThread == 0) {
		failure |= pthread_create(&_accelerationThread, NULL, &updateAcceleration, (void *)&_commonOrientation);
	}
	if (_angularPositionThread == 0) {
		failure |= pthread_create(&_angularPositionThread, NULL, &updateAngularPosition, (void *)&_commonOrientation);
	}
	if (_altitudeThread == 0) {
		failure |= pthread_create(&_altitudeThread, NULL, &updateAltitude, (void *)&_commonOrientation);
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
	if (_angularPositionThread == 0) {
		int failure = pthread_cancel(_angularPositionThread);
		if (!failure) {
			_angularPositionThread = 0;
		}
		else {
			printf("failed to cancel angular position thread\n");
			returnValue = -1;
		}
	}
	if (_altitudeThread == 0) {
		int failure = pthread_cancel(_altitudeThread);
		if (!failure) {
			_altitudeThread = 0;
		}
		else {
			printf("failed to cancel altitude thread\n");
			returnValue = -1;
		}
	}
	return 0;
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
	struct OrientationUpdateInformation threadInfo = *((struct OrientationUpdateInformation *)(input));
	free(input);
	// increment the number of listeners
	_numListeners++;

	// creates the threads if they haven't been created
	int failure = createStructMemberUpdateThreads();
	if (failure) {
		printf("update listener terminated due thread creation error\n");
		_numListeners--;
		killStructMemberUpdateThreads();
		pthread_exit(NULL);
	}

	// waits the desired interval, then passes the current
	//   orientation struct to the completion handler
	// if the completion handler requests termination,
	//   terminates on the next loop
	uint32_t waitTimeMicroseconds = (uint32_t)(1000000.0/(double)(threadInfo.desiredOrientationUpdateRate)); 
	// exit = 0 means don't exit, exit = 1 means do exit
	int exit = 0;
	while (1) {
		if (exit) {
			printf("exit requested. Ending listener thread\n");
			_numListeners--;
			killStructMemberUpdateThreads();
			pthread_exit(NULL);
		}

		threadInfo.completionHandler(_commonOrientation);

		usleep(waitTimeMicroseconds);
	}

	return NULL;
}

// retrieves the orientation of the device and passes an Orientation
//   struct into the passed in completion handler function
// returns 0 on success and -1 on failure
int getOrientation(int (*completionHandler)(struct Orientation), uint16_t updateRate) {
	// if listener count is exceed, return failure code
	if (_numListeners >= _maxListeners) {
		printf("call to getOrientation exited due to listener count exceeding maximum allowed count\n");
		return -1;
	}
	// set the update information struct
	struct OrientationUpdateInformation *threadInfo = (struct OrientationUpdateInformation *)(malloc(sizeof(struct OrientationUpdateInformation)));
	threadInfo->desiredOrientationUpdateRate = updateRate;
       	threadInfo->completionHandler = completionHandler;
	// set the shouldUpdate flag
	shouldUpdate = 1;
	
	// spin up thread for orientation updates
	pthread_t orienatationThread;
	int failure = pthread_create(&orienatationThread, NULL, &updateOrientation, threadInfo);
	if (failure) {
		printf("failed to create orientation thread\n");
		return -1;
	}

	return 0;
}







