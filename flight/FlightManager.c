// controls the flight operations at a high level
// takes input vectors and attempts to set the device's velocity to math the input vectors
// uses data from the avionics library (Orientation) to calculate and improve results

#include<stdint.h>
#include<unistd.h>

#include "FlightManager.h"
#include "Orientation.h"
#include "MotorController.h"


// the following values are used by setting them to the desired hold values when the corresponding 
//   OrientationHold bit is set
// they also have an accompanying tolerance specified as the absolute value of the tolerance (+/- value)
const double linearTolerance = 0.08; // g's (~9.8m/s^2)
const double angularTolerance = 3; // degrees (inaccurate measurement subject to drift)
const double angularZTolerance = 1; // degrees (this from the compass, so more accurate)
const double altitudeTolerance = 0.5; // meters

struct Orientation desiredOrientation;











