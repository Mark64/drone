// implementation for vector.h
//
// by Mark Hill
//
// Yes, this is terrible. No, I'm not gonna use a math library for software for an embedded device

#include"Vector.h"

#include<math.h>
#include<stdio.h>

// VERY IMPORTANT NOTE
//
// +Z points toward the sky
//
// +X points towards the front of the device
//
// +Y can be found with the right hand rule but it points towards the left, which makes sense because I'm left handed


// default function for creating Vec3double struct from components
struct Vec3double vectorFromComponents(double _x, double _y, double _z) {
	struct Vec3double vector = {_x, _y, _z, 0, 0, 0, 0};

	return vector;
}


// creates a vector and calculates the values of its components based on the angles and magnitude passed in
struct Vec3double vectorFromAnglesAndMagnitude(double angleXZ, double angleYZ, double magnitude) {
	double x = cos(radiansFromDegrees(angleXZ)) * magnitude;
	double y = cos(radiansFromDegrees(angleYZ)) * magnitude;
	double z = sin(radiansFromDegrees(angleXZ)) * magnitude;
	
	struct Vec3double vector = {x, y, z, 0, 0, 0, 0};

	return vector;
}

// basic vector subtraction
struct Vec3double vectorFromSubtractingVectors(struct Vec3double *vector1, struct Vec3double *vector2) {
	double x = vector1->x - vector2->x;
	double y = vector1->y - vector2->y;
	double z = vector1->z - vector2->z;

	return vectorFromComponents(x, y, z);
}


// returns the magnitude of the vector
double magnitude(struct Vec3double *vector) {
	vector->magnitude = pow((vector->x * vector->x) + (vector->y * vector->y) + (vector->z * vector->z), 0.5);
	return vector->magnitude;
}

double angleOfTangent(double numerator, double denominator) {
	// using atan (inverse tangent) to determine the angle
	//   of course, don't divide by zero
	double tan = M_PI/2;
	if (denominator != 0) {
		tan = numerator / denominator;
	}
	double angle = atan(tan);
	
	// however, tan inverse only has a range of +-pi/2
	// to correct for this, if statements must be used
	// to see why this is necessary and why M_PI was used, consult a unit circle
	if (angle > 0 && denominator < 0) {
		angle -= M_PI;
	}
	else if (angle < 0 && denominator < 0) {
		angle += M_PI;
	}

	return degreesFromRadians(angle);
}

// returns the xy plane angle
double angleXYPlane(struct Vec3double *vector) {
	vector->angleXY = angleOfTangent(vector->y, vector->x);
	return vector->angleXY;
}

// returns the xz plane angle
double angleXZPlane(struct Vec3double *vector) {
	vector->angleXZ = angleOfTangent(vector->z, vector->x);
	return vector->angleXZ;
}

// returns the yz plane angle, copied from xz
double angleYZPlane(struct Vec3double *vector) {
	vector->angleYZ = angleOfTangent(vector->z, vector->y);;
	return vector->angleYZ;
}

// returns the horizontal - z angle
double angleFromHorizontal(struct Vec3double *vector) {
	// using atan (inverse tangent) to determine the angle
	//   of course, don't divide by zero
	
	// getting the horizontal magnitude
	double horizMagnitude = pow(pow(vector->x, 2) + pow(vector->y, 2), 0.5);

	double tan = M_PI/2;
	if (horizMagnitude != 0) {
		tan = vector->z / horizMagnitude;
	}
	double angle = atan(tan);
	
	double degrees = degreesFromRadians(angle);
	return degrees;

}

// returns the angle between two vectors
double angleBetweenVectors(struct Vec3double *vector1, struct Vec3double *vector2) {
	double numerator = (vector1->x * vector2->x) + (vector1->y * vector2->y) + (vector1->z * vector2->z);
	double denominator = magnitude(vector1) * magnitude(vector2);
	return degreesFromRadians(acos(numerator / denominator));
}

// useful function for debugging
// returns a description of the Vector's components
void printVector(struct Vec3double *vector) {
	printf("vector\n magnitude: %f\n  x: %.3f\n  y: %f\n  z: %f\n  XZ angle: %f\n  XY angle: %f\n", magnitude(vector), vector->x, vector->y, vector->z, angleXZPlane(vector), angleXYPlane(vector));
}


// private function for converting radians to degrees
inline double degreesFromRadians(double radians) {
	return ((180.0 / M_PI) * radians);
}

// private function for converting degrees to radians
inline double radiansFromDegrees(double degrees) {
	return ((M_PI / 180.0) * degrees);
}


