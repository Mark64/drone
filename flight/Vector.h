// Vector wrapper
// I know it sucks, but I don't have the patience to learn an entire math library to get stupid 3d vector
//
// Plus most of them are for graphics libraries or research development and that honestly too much
//


//
// VERY IMPORTANT NOTE
//
// LIKE SERIOUSLY DONT SCREW THIS UP
//
// +Z points toward the sky
//
// +X points towards the front of the device
//
// +Y can be found with the right hand rule but it points towards the left, which makes sense because I'm left handed

#ifndef _Vector
#define _Vector

#include<string>


class Vec3double {

	public:
	
	// variables to store the vector components
	double x;
	double y;
	double z;
	
	// Methods
	//
	// create a vector based on components
	Vec3double(double _x, double _y, double _z);

	// create a vector based on angles and magnitude
	//   the first angle is the angle in the circle whose plane is parellel to the ground (like compass direction) 
	//   the second angle is the angle in the circle whose plane is perpendicular to the ground (like up and down)
	//   the magnitude is between 0 and 1 for a vector used for the motor controller, can be a value in another range for a different use
	static Vec3double vectorFromAnglesAndMagnitude(double angleFlat, double angleVertical, double magnitude);
	

	// returns a string description of the vector
	// useful for debuging
	void description();
};


// I need radians but all I have are these degrees
inline double radiansFromDegrees(double degrees);

// I want my degrees back
inline double degreesFromRadians(double radians);


#endif
