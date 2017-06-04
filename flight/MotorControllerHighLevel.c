// implementation for the high level, vector based MotorController functions
// abstacts away the motor count and exact position of the motors
//
// by Mark Hill
#include<stdio.h>
#include<math.h>
#include<stdint.h>

#include <MotorController.h>
#include <Vector.h>

// when setting the current motion vector, this array contains the thrust value for each motor located
//   at the index equal to the motor number
static double _motorThrustValues[] = {0, 0, 0, 0};
static const uint8_t _motorCount = 4;

// stores the target vectors
static struct Vec3double _targetLinearVector = {0, 0, 0, 0, 0, 0};
static struct Vec3double _targetAngularVector = {0, 0, 0, 0, 0, 0};

// these arrays contain the motor number(s) for each corresponding motion axis
static const uint8_t _linearPosXMotors[] = {2};
static const uint8_t _linearPosXMotorCount = 1;
static const uint8_t _linearNegXMotors[] = {0};
static const uint8_t _linearNegXMotorCount = 1;
static const uint8_t _linearPosYMotors[] = {3};
static const uint8_t _linearPosYMotorCount = 1;
static const uint8_t _linearNegYMotors[] = {1};
static const uint8_t _linearNegYMotorCount = 1;
static const uint8_t _linearPosZMotors[] = {0, 1, 2, 3};
static const uint8_t _linearPosZMotorCount = 4;
// no negative linear Z because the motors are not able to be reversed with my ESCs (electronic speed controllers)

static const uint8_t _angularPosZMotors[] = {1, 3};
static const uint8_t _angularPosZMotorCount = 2;
static const uint8_t _angularNegZMotors[] = {0, 2};
static const uint8_t _angularNegZMotorCount = 2;

// implemented below
// sets the motor thrust values based on the target vectors
static void updateMotion();



// returns the current linear motion vector
struct Vec3double getLinearMotionVector() {
	return _targetLinearVector;
}

// returns the current angular motion vector
struct Vec3double getAngularMotionVector() {
	return _targetAngularVector;
}

// sets the target linear motion vector
void setLinearMotionVector(struct Vec3double targetLinearMotion) {
	if (targetLinearMotion.magnitude > 1) {
		printf("error when setting linear motion vector. magnitude of supplied vector is greater than 1\n");
		return;
	}
	if (targetLinearMotion.z < 0) {
		printf("target linear motion vector z component is less than 0. Impossible to perform\n");
		return;
	}

	_targetLinearVector = targetLinearMotion;

	updateMotion();
}

// sets the target angular motion vector
void setAngularMotionVector(struct Vec3double targetAngularMotion) {
	if (targetAngularMotion.magnitude > 1) {
		printf("magnitude of angular vector too large (mag > 1)\n");
		return;
	}

	_targetAngularVector = targetAngularMotion;

	updateMotion();
}




// updates the motion of the vehicle based on the linear and angular motion vectors
//   stored (look up for the variables)
static void updateMotion() {

	// zero out the thrust values at the beginning so that thrusts by axis can be added
	for (int i = 0; i < _motorCount; i++) {
		_motorThrustValues[i] = 0;
	}

	// since x^2 + y^2 + z^2 <= 1 (as verified by the magnitude check), adding squared values
	//   rather than plain values will never give a thrust per motor of greater than 1
	double xSquare = pow(_targetLinearVector.x, 2);
	double ySquare = pow(_targetLinearVector.y, 2);
	double zSquare = pow(_targetLinearVector.z, 2);


	// linear section

	// set the x values
	const uint8_t *xMotors = _targetLinearVector.x < 0 ? _linearNegXMotors : _linearPosXMotors;
	uint8_t xMotorCount = _targetLinearVector.x < 0 ? _linearNegXMotorCount : _linearPosXMotorCount;
	for (int i = 0; i < xMotorCount; i++) {
		_motorThrustValues[xMotors[i]] += xSquare;
	}

	// set the y values
	const uint8_t *yMotors = _targetLinearVector.y < 0 ? _linearNegYMotors : _linearPosYMotors;
	uint8_t yMotorCount = _targetLinearVector.y < 0 ? _linearNegYMotorCount : _linearPosYMotorCount;
	for (int i = 0; i < yMotorCount; i++) {
		_motorThrustValues[yMotors[i]] += ySquare;
	}

	// set the z values
	const uint8_t *zMotors = _linearPosZMotors;
	uint8_t zMotorCount = _linearPosZMotorCount;
	for (int i = 0; i < zMotorCount; i++) {
		_motorThrustValues[zMotors[i]] += zSquare;
		printf("%d %f\n", zMotors[i], zSquare);
	}


	// angular section

	double zSquareAngular = pow(_targetAngularVector.z, 2);

	// set the z axis spin
	// this is done by maintaining total additive thrust, but reducing the thrust for the
	//   pair of motors generating balancing spin and increasing thrust for motors with
	//   resultant spin in the desired direction
	const uint8_t *desiredSpinMotors = _targetAngularVector.z < 0 ? _angularNegZMotors : _angularPosZMotors;
	uint8_t desiredMotorCount = _targetAngularVector.z < 0 ? _angularNegZMotorCount : _angularPosZMotorCount;
	const uint8_t *undesiredSpinMotors = _targetAngularVector.z < 0 ? _angularPosZMotors : _angularNegZMotors;
	uint8_t undesiredMotorCount = _targetAngularVector.z < 0 ? _angularPosZMotorCount : _angularNegZMotorCount;

	// since the angular setting is performed after the linear setting, the maximum increase and decrease
	//   must be determined to protect against a value >1 being set
	double maxIncrease = 1;
	double maxDecrease = undesiredSpinMotors[0];

	for (int i = 0; i < desiredMotorCount; i++) {
		double possibleIncrease = 1 - _motorThrustValues[desiredSpinMotors[i]];
		maxIncrease = maxIncrease > possibleIncrease ? possibleIncrease : maxIncrease;
	}

	for (int i = 1; i < undesiredMotorCount; i++) {
		double possibleDecrease = _motorThrustValues[undesiredSpinMotors[i]];
		maxDecrease = maxDecrease > possibleDecrease ? possibleDecrease : maxDecrease;
	}

	// with the max decrease and increase determined, now correct the motor values
	for (int i = 0; i < desiredMotorCount; i++) {
		_motorThrustValues[desiredSpinMotors[i]] += maxIncrease * zSquareAngular;
	}

	for (int i = 0; i < undesiredMotorCount; i++) {
		_motorThrustValues[undesiredSpinMotors[i]] -= maxDecrease * zSquareAngular;
	}
	printf("%f %f\n", maxDecrease, maxIncrease);


	// thrust section

	// now actually set the thrust values
	// bounds verification is done at the low level implementation
	for (int i = 0; i < _motorCount; i++) {
		// take the square root of this value since it is a sum of squares
		double thrust = pow(_motorThrustValues[i], 0.5);
		thrust = thrust > 1 ? 1 : thrust;
		thrust = thrust < 0 ? 0 : thrust;
		printf("thrust %d %.9f\n", i, thrust);

		setMotorThrustPercentage(i, thrust);
	}
}









