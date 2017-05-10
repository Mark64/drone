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

// returns the magnitude of a vector based on the passed in componenets
double magnitude(double _x, double _y, double _z) {
	return pow((_x * _x) + (_y * _y) + (_z * _z), 0.5);
}


// default function for creating Vec3double struct from components
struct Vec3double vectorFromComponents(double _x, double _y, double _z) {
	double mag = magnitude(_x, _y, _z);
	double alpha, beta, gamma = 0;
	if (mag > 0) {
		alpha = _x / mag;
		beta = _y / mag;
		gamma = _z / mag;
	}
	struct Vec3double vector = {_x, _y, _z, alpha, beta, gamma, mag};

	return vector;
}


// creates a vector and calculates the values of its components based on the angles and magnitude passed in
struct Vec3double vectorFromCosAndMagnitude(double alpha, double beta, \
					       double gamma, double magnitude) {
	double x = alpha * magnitude;
	double y = beta * magnitude;
	double z = gamma * magnitude;

	struct Vec3double vector = {x, y, z, alpha, beta, gamma, magnitude};

	return vector;
}

// basic vector subtraction
struct Vec3double vectorFromSubtractingVectors(struct Vec3double *v1, \
					       struct Vec3double *v2) {
	double x = v1->x - v2->x;
	double y = v1->y - v2->y;
	double z = v1->z - v2->z;

	return vectorFromComponents(x, y, z);
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
double angleBetweenVectors(struct Vec3double *v1, struct Vec3double *v2) {
	double numerator = (v1->x * v2->x) + (v1->y * v2->y) + (v1->z * v2->z);
	double denominator = v1->magnitude * v2->magnitude;
	return degreesFromRadians(acos(numerator / denominator));
}

// useful function for debugging
// returns a description of the Vector's components
void printVector(struct Vec3double vector, const char *title) {
	printf("%s\n magnitude: %f\n  x: %.3f\n  y: %f\n  z: %f\n  alpha: %f\n  beta: %f\n  gamma: %f\n", \
	       title, vector.magnitude, vector.x, vector.y, vector.z, \
	       angleFromCos(vector.alpha), angleFromCos(vector.beta), \
	       angleFromCos(vector.gamma));
	//printf("%s\n magnitude: %f\n  x: %.3f\n  y: %f\n  z: %f\n  alpha: %f\n  beta: %f\n  gamma: %f\n", \
	       title, vector.magnitude, vector.x, vector.y, vector.z, \
	       vector.alpha, vector.beta, vector.gamma);
}

// returns xy angle
double angleXY(struct Vec3double *v) {
	double horizMag = pow(v->x * v->x + v->y * v->y, 0.5);
	return angleFromCos(v->x / horizMag) * (v->y > 0 ? 1 : -1);
}

// returns degrees from cosine input
double angleFromCos(double cosine) {
	return degreesFromRadians(acos(cosine));
}

// private function for converting radians to degrees
inline double degreesFromRadians(double radians) {
	return ((180.0 / M_PI) * radians);
}

// private function for converting degrees to radians
inline double radiansFromDegrees(double degrees) {
	return ((M_PI / 180.0) * degrees);
}


