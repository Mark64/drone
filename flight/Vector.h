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


struct Vec3double {
	// variables to store the vector components
	double x;
	double y;
	double z;

	// variables that take longer to compute and so are set on demand
	// call the appropriate getter to have these generated
	double angleXY;
	double angleXZ;
	double magnitude;
};



// create a vector based on components
struct Vec3double vectorFromComponents(double _x, double _y, double _z);

// create a vector based on angles and magnitude
//   the first angle is the angle in the xy plane relative to the +x axis using right hand rule for rotation
//   the second angle is the angle in the xz plane relative to the +x axis using right hand rule for rotation
//   for more information on the angles, see the comments 
//   the magnitude is between 0 and 1 for a vector used for the motor controller, can be a value in another range for a different use
struct Vec3double vectorFromAnglesAndMagnitude(double angleXY, double angleXZ, double magnitude);


// gets the magnitude of the vector based on the components and sets the magnitude
//   member of the Vec3double struct
double magnitude(struct Vec3double *vector);

// returns the xy plane angle and sets the corresponding struct member
// this is the angle relative to the +x axis rotated counterclockwise about
//   the z axis
// see the wikipedia page on the right hand rule
// a vector pointing parallel to the +x axis will have an angle of 0 here
//   and a vector pointing parallel to the +y axis will have an angle of 
//   90 degrees
// returns the angle in degrees
double angleXYPlane(struct Vec3double *vector);

// returns the xz plane angle and sets the corresponding struct member
// see the comments above for the 'angleXYPlane()' function for more details
// on the right hand rule
// returns the angle in degrees relative to the +x axis rotated about the -y axis
double angleXZPlane(struct Vec3double *vector);

// returns the angle between the horizontal component (magnitude of x and y)
//   and the z component
// result is the angle in degrees
double angleFromHoritzonal(struct Vec3double *vector);

// returns a string description of the vector
// useful for debuging
void printVector(struct Vec3double *vector);



// I need radians but all I have are these degrees
double radiansFromDegrees(double degrees);

// I want my degrees back
double degreesFromRadians(double radians);


#endif
