// implementation for vector.h
//
// by Mark Hill
//
// Yes, this is terrible. No, I'm not gonna use a math library for software for an embedded device

#include<math.h>
#include<iostream>
#include<sstream>

#include"Vector.h"

// VERY IMPORTANT NOTE
//
// +Z points toward the sky
//
// +X points towards the front of the device
//
// +Y can be found with the right hand rule but it points towards the left, which makes sense because I'm left handed


// default constructor to create a vector from components
Vec3double :: Vec3double(double _x, double _y, double _z) {
	this->x = _x;
	this->y = _y;
	this->z = _z;
}


// creates a vector and calculates the values of its components based on the angles and magnitude passed in
Vec3double Vec3double :: vectorFromAnglesAndMagnitude(double angleFlat, double angleVertical, double magnitude) {
	double x = cos(radiansFromDegrees(angleFlat)) * magnitude;
	double y = sin(radiansFromDegrees(angleFlat)) * magnitude;
	double z = sin(radiansFromDegrees(angleVertical)) * magnitude;
	return Vec3double(x, y, z);
}

// returns the magnitude of the vector
double Vec3double :: magnitude() {
	double magnitude = powf((powf(this->x, 2) + powf(this->y, 2) + powf(this->z, 2)), 0.5);
	return magnitude;
}

// useful function for debugging
// returns a description of the Vector's components
void Vec3double :: description() {
	std::cout << "vector\nx: " << this->x << "\ny: " << this->y << "\nz: " << this->z << "\n";
}


// private function for converting radians to degrees
inline double degreesFromRadians(double radians) {
	return ((180.0 / M_PI) * radians);
}

// private function for converting degrees to radians
inline double radiansFromDegrees(double degrees) {
	return ((M_PI / 180.0) * degrees);
}


