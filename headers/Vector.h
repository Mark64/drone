// Vector wrapper
// I know it sucks, but I don't have the patience to learn an entire math library to get a simple 3d vector
//
// Plus most of them are for graphics libraries or research development and thats too much overhead
//


//
// VERY IMPORTANT NOTE
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

	double alpha;
	double beta;
	double gamma;
	double magnitude;
};


// creates a vector based on components
struct Vec3double vectorFromComponents(double x, double y, double z);

// creates a vector based on angles and magnitude
//   the magnitude is between 0 and 1 for a vector used for the motor controller, can be a value in another range for a different use
struct Vec3double vectorFromCosAndMagnitude(double alpha, double beta, double gamma, double magnitude);

// creates a vector by subtracting the two vector arguments
struct Vec3double vectorFromSubtractingVectors(struct Vec3double *vector1, struct Vec3double *vector2);

// returns the angle between the horizontal component (magnitude of x and y)
//   and the z component
// result is the angle in degrees
double angleFromHorizontal(struct Vec3double *vector) __attribute__ ((deprecated));

// returns the angle in the XY plane
double angleXY(struct Vec3double *vector);

// returns an angle in degrees based on the passed in cosine
double angleFromCos(double cosine);

// returns the angle between two vectors
// uses the formula cos(theta) = (u*v)/(||u||*||v||)
double angleBetweenVectors(struct Vec3double *vector1, struct Vec3double *vector2);

// prints a string description of the vector
// useful for debuging
void printVector(struct Vec3double vector, const char *title);



// converts the input from degrees to radians
double radiansFromDegrees(double degrees);

// converts the input from radians to degrees
double degreesFromRadians(double radians);


#endif
