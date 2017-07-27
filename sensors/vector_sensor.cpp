// implementation of vector sensor
// Mark Hill

#include<vector_sensor.h>
#include<Eigen/Dense>
#include<pthread.h>

using namespace Eigen;


static pthread_mutex_t readlock, writelock;




