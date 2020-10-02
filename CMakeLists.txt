cmake_minimum_required(VERSION 3.15)
project(3355_TP4)

set(CMAKE_CXX_STANDARD 14)

add_executable(3355_TP4 main.cpp basic.hpp image.hpp object.hpp object.cpp parser.hpp parser.cpp raytracer.hpp raytracer.cpp scene.hpp extras.cpp extras.h)
