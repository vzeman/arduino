cmake_minimum_required(VERSION 3.29)
project(heating_control)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)

add_executable(heating_control
    heating-control.ino
    secrets.h
        heating-control.ino)
