#This function sets up the default cmake build type.
#BUILD_TYPE: one of the cmake build types, e.g.: Debug, Release...

#cmake_minimum_required(VERSION 2.8.12)

function(set_default_build_type BUILD_TYPE)
    if(NOT CMAKE_BUILD_TYPE)
        set (CMAKE_BUILD_TYPE ${BUILD_TYPE})
        message(STATUS "Setting build type to ${BUILD_TYPE}.")
    else(NOT CMAKE_BUILD_TYPE)
        message(STATUS "Build type set to ${CMAKE_BUILD_TYPE} from command line.")
    endif(NOT CMAKE_BUILD_TYPE)
endfunction()
