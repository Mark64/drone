// header file containing common linear algebra geometry inlines
//
// by Mark Hill
#ifndef _geometry
#define _geometry

#include<Eigen/Dense>
#include<math.h>
#include<stdio.h>

using namespace Eigen;

// returns the angle between two 3d vectors in degrees
inline double angle_between(Vector3d a, Vector3d b) {
	double alen = a.norm();
	double blen = b.norm();

	if (alen != 0 && blen != 0)
		return (180 / M_PI) * acos(a.dot(b) / (alen * blen));
	else
		return 0;
}

// returns the angle of the vector projected onto the xy plane
inline double angleXY(Vector3d a) {
	if (a(0) != 0)
		return (180 / M_PI) * atan(a(1) / a(0));
	else if (a(1) != 0)
		90 * a(1) / abs(a(1));
	else
		return 0;
}

// prints a string representation of a vector to the console
inline void printVector(Vector3d a, const char str[]) {
	double mag = a.norm();
	printf("%s\n x: %f\n y: %f\n z: %f\n |v|: %f\n a: %f\n b: %f\n g: %f\n", \
			str, a(0), a(1), a(2), mag, a(0) / mag, a(1) / mag, a(2) / mag);
}






#endif
