cmake_minimum_required(VERSION 3.8)
project(dump1090_sdrplus)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        src/data.c
        src/data.h
        src/devices.c
        src/devices.h
        src/dump1090.c
        src/dump1090.h
        src/netpro.c
        src/netpro.h
        src/stdafx.h
        README.md
        TODO)

add_executable(dump1090_sdrplus ${SOURCE_FILES})