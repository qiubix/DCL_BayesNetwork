# Include the directory itself as a path to include directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Find OpenCV library files
#FIND_PACKAGE( OpenCV REQUIRED )

# Create a variable containing all .cpp files:
FILE(GLOB files *.cpp)

SET(GCC_COVERAGE_COMPILE_FLAGS "-fprofile-arcs -ftest-coverage")
SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}" )

# Create an executable file from sources:
add_library(BayesNetworkSink SHARED ${files})
target_link_libraries(BayesNetworkSink ${DisCODe_LIBRARIES} ${Boost_LIBRARIES} BayesNetworkTypes)

INSTALL_COMPONENT(BayesNetworkSink)
