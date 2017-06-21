// controls the flight operations at a high level
// takes input vectors and attempts to set the device's velocity to math the input vectors
// uses data from the avionics library (Orientation) to calculate and improve results

#include<stdint.h>
#include<unistd.h>

#include <FlightManager.h>
#include <Orientation.h>
#include <MotorController.h>


struct Orientation desiredOrientation = {};
struct Orientation currentOrientation = {};

// flag that indicates whether drone is in flight
static int inFlight = 0;


// completion handler for orientation updates
int orientationUpdate(struct Orientation orientation) {
	printOrientation(orientation);

	return (inFlight ? 500 : 1);
}


// starts the flight manager thread
int startFlightManager() {
	getOrientation(&orientationUpdate, 1);

	return 0;
}







