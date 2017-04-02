add_library(computation SHARED Vector)

add_library(avionics SHARED i2cctl SensorManager PWMController)
target_link_libraries(avionics computation)

add_library(motion SHARED MotorControllerLowLevel MotorControllerHighLevel)
target_link_libraries(motion avionics)







