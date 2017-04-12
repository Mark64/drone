add_library(computation SHARED Vector)

add_library(avionics SHARED i2cctl SensorManager PWMController)
target_link_libraries(avionics computation)

add_library(navigation SHARED MotorControllerLowLevel MotorControllerHighLevel Orientation)
target_link_libraries(navigation avionics rt)







