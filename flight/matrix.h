// matrix code, because I insist on reinventing the wheel
//
// by Mark Hill
#ifndef __MATRIX
#define __MATRIX

// a matrix struct
struct matrix3 {
	double elements[3][3];
	unsigned char rows;
	unsigned char columns;
};

// matrices should be created with the following initializer
struct matrix3 matrix3x3();


// returns a 3x3 rotation matrix with the given angles
struct matrix3 rotationMatrix3d(double roll, double pitch, double yaw);


struct Vec3double;

// multiplies a vector by a matrix and returns a vector
struct Vec3double transformVectorByMatrix(struct Vec3double vector, struct matrix3 matrix);




#endif
