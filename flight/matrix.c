// matrix implementation
//
// by mark hill

#include<math.h>
#include<stdio.h>

#include <matrix.h>
#include <Vector.h>


struct matrix3 matrix3x3() {
	double elements[3][3] = {{0}};
	struct matrix3 m = {{{0}}, 3, 3,};
	return m;
}


struct matrix3 rotationMatrix3d(double roll, double pitch, double yaw) {
	struct matrix3 m = {{{
		cos(yaw)*cos(pitch),
		cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*cos(roll),
		sin(yaw)*sin(roll) + cos(yaw)*sin(pitch)*cos(roll)
	}, {
		sin(yaw)*sin(pitch),
		cos(yaw)*cos(roll) + sin(yaw)*sin(pitch)*sin(roll),
		sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll)
	}, {
		-1 * sin(pitch),
		cos(pitch)*sin(roll),
		cos(pitch)*cos(roll)},
	}, 3, 3};
	return m;
}


struct Vec3double transformVectorByMatrix(struct Vec3double v, struct matrix3 m) {
	double currentComponents[3] = {v.x, v.y, v.z};
	double transformedComponents[3] = {0};
	for (int i = 0; i < m.rows; i++) {
		for (int j = 0; j < m.columns; j++) {
			transformedComponents[i] += currentComponents[j] * m.elements[i][j];
		}
	}

	return vectorFromComponents(currentComponents[0], currentComponents[1], currentComponents[2]);
}



