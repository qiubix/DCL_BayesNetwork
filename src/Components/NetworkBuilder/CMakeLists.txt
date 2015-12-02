# Include the directory itself as a path to include directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Create a variable containing all .cpp files:
FILE(GLOB files *.cpp)

coveralls_turn_on_coverage()
coveralls_setup(
        "${files}"
        ${COVERALLS_UPLOAD}
        "${PROJECT_SOURCE_DIR}/cmake")

# Create an executable file from sources:
add_library(procNetworkBuilder SHARED ${files} OctreeContainers.hpp)
target_link_libraries(procNetworkBuilder ${DisCODe_LIBRARIES} ${OpenCV_LIBS} ${SMILE_LIBS} ${Boost_LIBRARIES} ${PCL_IO_LIBRARIES})

INSTALL_COMPONENT(procNetworkBuilder)
