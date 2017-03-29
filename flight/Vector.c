// implementation for vector.h
//
// by Mark Hill
//
// Yes, this is terrible. No, I'm not gonna use a math library for software for an embedded device

#include<math.h>
#include<stdio.h>

#include"Vector.h"

// VERY IMPORTANT NOTE
//
// +Z points toward the sky
//
// +X points towards the front of the device
//
// +Y can be found with the right hand rule but it points towards the left, which makes sense because I'm left handed


// default function for creating Vec3double struct from components
struct Vec3double vectorFromComponents(double _x, double _y, double _z) {
	struct Vec3double vector = {_x, _y, _z, 0, 0, 0};

	return vector;
}


// creates a vector and calculates the values of its components based on the angles and magnitude passed in
struct Vec3double vectorFromAnglesAndMagnitude(double angleXY, double angleXZ, double magnitude) {
	double x = cos(radiansFromDegrees(angleXZ)) * magnitude;
	double y = sin(radiansFromDegrees(angleXY)) * magnitude;
	double z = sin(radiansFromDegrees(angleXZ)) * magnitude;
	
	struct Vec3double vector = {x, y, z, 0, 0, 0};

	return vector;
}

// returns the magnitude of the vector
double magnitude(struct Vec3double *vector) {
	vector->magnitude = pow((pow(vector->x, 2) + pow(vector->y, 2) + pow(vector->z, 2)), 0.5);
	return vector->magnitude;
}

// returns the xy plane angle
double angleXYPlane(struct Vec3double *vector) {
	// using atan (inverse tangent) to determine the angle
	double tan = vector->y / vector->x;
	double angle = atan(tan);
	
	// however, tan inverse only has a range of +-pi/2
	// to correct for this, if statements must be used
	// to see why this is necessary and why M_PI was used, consult a unit circle
	if (angle > 0 && vector->x < 0) {
		angle += M_PI;
	}
	else if (angle < 0 && vector->x < 0) {
		angle -= M_PI;
	}
	
	vector->angleXY = degreesFromRadians(angle);
	return vector->angleXY;
}

// returns the xz plane angle
double angleXZPlane(struct Vec3double *vector) {
	// using atan (inverse tangent) to determine the angle
	double tan = vector->z / vector->x;
	double angle = atan(tan);
	
	// however, tan inverse only has a range of +-pi/2
	// to correct for this, if statements must be used
	// to see why this is necessary and why M_PI was used, consult a unit circle
	if (angle > 0 && vector->x < 0) {
		angle += M_PI;
	}
	else if (angle < 0 && vector->x < 0) {
		angle -= M_PI;
	}

	vector->angleXZ = degreesFromRadians(angle);
	return vector->angleXZ;
}

// useful function for debugging
// returns a description of the Vector's components
void printVector(struct Vec3double *vector) {
	printf("vector\n magnitude: %f\n  x: %.3f\n  y: %f\n  z: %f\n", magnitude(vector), vector->x, vector->y, vector->z);
}


// private function for converting radians to degrees
inline double degreesFromRadians(double radians) {
	return ((180.0 / M_PI) * radians);
}

// private function for converting degrees to radians
inline double radiansFromDegrees(double degrees) {
	return ((M_PI / 180.0) * degrees);
}


