add_library(avionics SHARED i2cctl SensorManager PWMController)

add_library(navigation SHARED MotorControllerLowLevel MotorControllerHighLevel Orientation FlightManager)
target_link_libraries(navigation avionics rt)







