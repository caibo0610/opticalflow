CMAKE_MINIMUM_REQUIRED(VERSION 3.4)

#SET BIN
SET(BIN opiticalFlow)

PROJECT(${BIN})

#c++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -ffast-math -use_fast_math")

set(SRCS
    opticflow_daemon.c
	uav_of.c
	uav_of_fc.c
	camera.h
	opticflow.h
	shmdata.h
	type.h
	uav_of.h
	uav_ov7251.h
	uav_ov7251.cpp
    )

ADD_EXECUTABLE(${BIN} ${SRCS})
