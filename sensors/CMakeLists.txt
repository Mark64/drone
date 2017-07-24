set(SOURCES mpu6050.cpp SensorManager.cpp)
get_filename_component(LIBNAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
add_library(${LIBNAME} STATIC "${SOURCES}")
