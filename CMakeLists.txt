#Project name
PROJECT(BayesNetwork)

# CMake required version must be >= 2.6
cmake_minimum_required(VERSION 2.6)

# Find DisCODe installation
FIND_PACKAGE(DisCODe REQUIRED)

LINK_DIRECTORIES(${CMAKE_SOURCE_DIR}/lib/SMILE)
SET(SMILE_LIBS smile smilearn)

# OpenCV library
FIND_PACKAGE( OpenCV REQUIRED )

# PCL library
FIND_PACKAGE(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
MESSAGE("PCL include: ${PCL_INCLUDE_DIRS}")
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Use CMake macros provided with DisCODe
set(CMAKE_MODULE_PATH ${DisCODe_DIR} ${CMAKE_MODULE_PATH})
include(DisCODeMacros)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

# C++11 support
include(C++11Compiling)

# Testing
include(GoogleMock)
set(CMAKE_AUTOMOC ON)

# Set default install prefix to dist folder
IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  SET(CMAKE_INSTALL_PREFIX
      ${CMAKE_SOURCE_DIR}/dist CACHE PATH "${CMAKE_PROJECT_NAME} install prefix" FORCE
  )
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(test)

enable_testing()
