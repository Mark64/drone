set(SOURCES MotorControllerHighLevel.cpp MotorControllerLowLevel.cpp)
get_filename_component(LIBNAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
add_library(${LIBNAME} STATIC "${SOURCES}")
