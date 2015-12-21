# Include the directory itself as a path to include directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Create a variable containing all .cpp files:
FILE(GLOB files *.cpp)

SET(GCC_COVERAGE_COMPILE_FLAGS "-fprofile-arcs -ftest-coverage")
SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}" )

# Create an executable file from sources:
add_library(procNetworkBuilder SHARED ${files})
target_link_libraries(procNetworkBuilder BayesNetworkTypes ${DisCODe_LIBRARIES} ${OpenCV_LIBS} ${SMILE_LIBS} ${Boost_LIBRARIES} ${PCL_IO_LIBRARIES})

INSTALL_COMPONENT(procNetworkBuilder)
