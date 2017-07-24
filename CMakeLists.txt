# by Mark Hill
# using tutorial from https://www.johnlamp.net/cmake-tutorial-4-libraries-and-subdirectories.html

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# flight controller test section
project("Remote Drone")
set(TEST_EXECUTABLE_NAME droneTest)
set(TEST_MAIN_FILE flight/FlightTest.cpp)

# language settings
#   enable C11 mode
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

function(prepend prefix inlist outlist)
	set(templist "")
	foreach(x ${inlist})
		list(APPEND templist "${prefix}/${x}")
	endforeach(x)
	set(${outlist} "${outlist}" "${templist}" PARENT_SCOPE)
endfunction(prepend)

# optimization flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2 -g")

set(SUBDIRS drivers flight motion orientation sensors)
set(INCLUDE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(EIGEN_INCLUDE ${INCLUDE_ROOT}/eigen)
prepend(${INCLUDE_ROOT} "${SUBDIRS}" SUBDIR_INCLUDE)
set(PROJECT_LIBS ${SUBDIRS} m rt)

message(${SUBDIR_INCLUDE})
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${INCLUDE_ROOT} ${EIGEN_INCLUDE} ${SUBDIR_INCLUDE})

# runs CMake on the subdirectories
foreach(dir ${SUBDIRS})
	add_subdirectory(${dir})
endforeach(dir)

# create executable
add_executable(${TEST_EXECUTABLE_NAME} ${TEST_MAIN_FILE})
target_link_libraries(${TEST_EXECUTABLE_NAME} ${PROJECT_LIBS})

# threading section
find_package(Threads REQUIRED)
if(THEADS_HAVE_PTHREAD_ARG)
	target_compile_options(PUBLIC ${TEST_EXECUTABLE_NAME} "-pthread")
endif()
if (CMAKE_THREAD_LIBS_INIT)
	target_link_libraries(${TEST_EXECUTABLE_NAME} ${CMAKE_THREAD_LIBS_INIT})
endif()


set(CONFIGURED_ONCE TRUE CACHE INTERNAL "")
