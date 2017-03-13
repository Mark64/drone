// implementation for vector.h
//
// by Mark Hill
//
// Yes, this is terrible. No, I'm not gonna use a math library for software for an embedded device

#include<cmath>
#include<iostream>

#include"vector.h"

// VERY IMPORTANT NOTE
//
// LIKE SERIOUSLY DONT SCREW THIS UP
//
// +Z points toward the sky
//
// +X points towards the front of the device
//
// +Y can be found with the right hand rule but it points towards the left, which makes sense because I'm left handed


// I need radians but I use degrees.  Sue me



Vec3double :: Vec3double(double _x, double _y, double _z) {
	this->x = _x;
	this->y = _y;
	this->z = _z;
}


Vec3double Vec3double::  vectorFromAnglesAndMagnitude(double angleFlat, double angleVertical, double magnitude) {
	double x = cos(radiansFromDegrees(angleFlat)) * magnitude;
	double y = sin(radiansFromDegrees(angleFlat)) * magnitude;
	double z = sin(radiansFromDegrees(angleVertical)) * magnitude;
	return Vec3double(x, y, z);
}




inline double degreesFromRadians(double radians) {
	return ((180.0 / M_PI) * radians);
}


inline double radiansFromDegrees(double degrees) {
	return ((M_PI / 180.0) * degrees);
}


