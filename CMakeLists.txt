# by Mark Hill
# using tutorial from https://www.johnlamp.net/cmake-tutorial-4-libraries-and-subdirectories.html

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# flight controller test section
project("Remote Drone")
set(TEST_EXECUTABLE_NAME rocketTest)
set(TEST_MAIN_FILE flight/FlightTest.c)

# language settings
#   enable C99 mode
macro(use_c11)
	if (CMAKE_VERSION VERSION_LESS "3.1")
		if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
			set(CMAKE_C_FLAGS "--std=gnu11 -pedantic ${CMAKE_C_FLAGS}")
		endif()
	else()
		set (CMAKE_C_STANDARD 11)
	endif()
endmacro(use_c11)

use_c11()

# stop the stupid insource builds
if ( ${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR} )
	message(FATAL_ERROR "In-source builds not allowed, use the Makefile wrapper")
endif()

# optimization flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")

include_directories("${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/headers")

# runs CMake on the subdirectory flight
add_subdirectory(flight)

# create executable
add_executable(${TEST_EXECUTABLE_NAME} ${TEST_MAIN_FILE})

# these libraries are built from this project's source tree and allow the project to be easily extended
target_link_libraries(${TEST_EXECUTABLE_NAME} avionics computation navigation)
# these are system libraries that must be linked
target_link_libraries(${TEST_EXECUTABLE_NAME} m)

# threading section
find_package(Threads REQUIRED)
if(THEADS_HAVE_PTHREAD_ARG)
	target_compile_options(PUBLIC ${TEST_EXECUTABLE_NAME} "-pthread")
endif()
if (CMAKE_THREAD_LIBS_INIT)
	target_link_libraries(${TEST_EXECUTABLE_NAME} ${CMAKE_THREAD_LIBS_INIT})
endif()


set(CONFIGURED_ONCE TRUE CACHE INTERNAL "Apparently this shows CMake has been configured once ????")
