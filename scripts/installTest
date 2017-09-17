#!/bin/bash

DEVICE_NAME=rocket
EXECUTABLE=rocketTest
BUILDDIR=build_rpi0
LIBRARIES=lib*.so

# this script copies and installs the software to the drone
printf "transfering libraries and executables to system\n"
scp ./$BUILDDIR/$EXECUTABLE ./$BUILDDIR/flight/lib*.so ./$BUILDDIR/flight/lib*.a $DEVICE_NAME:~
printf "installing files\n"
ssh $DEVICE_NAME \
	"sudo cp $EXECUTABLE /usr/local/bin; \
	 sudo cp $LIBRARIES /usr/lib;        \
	 rm $EXECUTABLE $LIBRARIES;          "
printf "done\n"



