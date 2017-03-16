# by Mark Hill
# using tutorial from https://www.johnlamp.net/cmake-tutorial-4-libraries-and-subdirectories.html

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# flight controller test section
project("Remote Drone")

# stop the stupid insource builds
if ( ${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR} )
	message(FATAL_ERROR "In-source builds not allowed, use the Makefile wrapper")
endif()


include_directories("${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/flight")

add_subdirectory(flight)

add_executable(Test flight/FlightTest.cpp)
target_link_libraries(Test flight)

set(CONFIGURED_ONCE TRUE CACHE INTERNAL "Apparently this shows CMake has been configured once ????")
