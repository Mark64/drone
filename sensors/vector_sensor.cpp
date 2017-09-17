// implementation of vector sensor
// Mark Hill

#include<vector_sensor.h>
#include<Eigen/Dense>
#include<pthread.h>

using namespace Eigen;


// 2d array of vector sensors organized by dr_dev_type
static dr_vector_sensor* vector_sensor_collection[DEV_TYPE_COUNT] = NULL;
// internal locking vector sensor collection reads and vector sensor registration
static pthread_mutex_t readlock, writelock = NULL;





