#This module turns on c++11 compilation assuming GNU GCC as compiler for now.
#It sets -std=c++11 flag on CMAKE_CXX_FLAGS.

#TODO: make this work with other compilers

cmake_minimum_required(VERSION 2.8.12)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)

if(NOT COMPILER_SUPPORTS_CXX11)
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler. Cmake will exit now.")
else(NOT COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif(NOT COMPILER_SUPPORTS_CXX11)
