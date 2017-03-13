// Finally, some real code
// 
// This abstracts the motor control and allows for easy update and stabilization
// 
// Just pass in your motion vectors and this will ensure they are carried out, as well as stabilize the position when at rest



class MotorController {

	public:
	// takes off after the delay has passed (delay in units of seconds)
	void takeOff(double secondsToDelay);
	

};
