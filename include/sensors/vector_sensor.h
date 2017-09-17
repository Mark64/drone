// vector-based sensor definitions
// supports only up to 3 dimensions
// Mark Hill
#ifndef __vector_sensor_h
#define __vector_sensor_h

#include<device_manager.h>
#include<Eigen/Dense>

using namespace Eigen;


enum dr_axis {
	X = 0b1,
	Y = 0b10,
	Z = 0b100,
//	W = 0b1000, //unimplemented
}

struct dr_dev;

/**
 * represents a sensor that measures a vector
 * define the type of sensor using the appriopriate enum value in dr_dev
 *
 * @dev					the underlying device
 * @read_vector			measures the sensor vector <count> times and places the data in <data>
 * 						@data:		array where sensor data is placed
 * 						@count:		the number of times to measure a vector from the sensor (min size of data)
 *						@return:	the number of vector values that could actually be read
 * @axes				axes the device is capable of measuring
 * 							bitwise ORed
 * @position			note: unimplemented; position relative to origin where the
 * 							sensor is physically mounted
 */
struct dr_vector_sensor {
	struct dr_dev dev;
	int (*read_vector)(Vector3d *data, int count);
	enum dr_axis axes;
	Vector3d position;
};


/**
 * called to register sensor with the runtime
 * device should already be on and ready to return vector data
 */
int register_vector_sensor(struct dr_vector_sensor *sens);

/**
 * removes sens from the runtime
 * note: does not call unregister_device()
 */
void unregister_vector_sensor(struct dr_vector_sensor *sens);

/**
 * reads from all registered vector sensors of <type> to get vector data
 * returns the greatest number of reads possible, even if only one sensors's data was used in that read
 * see @read_vector in dr_vector_sensor for info on the rest of the params and return type
 * @sens				null if read_vector() should read from all devices, otherwise reads only from sens
 * @return				same as @read_vector in dr_vector_sensor, but also returns -1 if sens is not null
 * 							and not a registered vector sensor
 */
int read_vector(enum dr_dev_type type, struct dr_vector_sensor *sens, Vector3d *data, int count);

/**
 * returns the number of registered vector sensors of type
 * @type				the type of sensor to tally
 */
int num_vector_sensors(enum dr_dev_type type);

/**
 * places registered dr_vector_sensor pointers of type <type> in data
 * @type				the type of sensor requested
 * @data				the buffer in which to place the dr_vector_sensor pointers
 * @count				the number of pointers to place in data
 * @return				0 if successful
 * 						-1 if data was too small
 */
int vector_sensors(enum dr_dev_type type, struct dr_vector_sensor **data, int count);




