# Include the directory itself as a path to include directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src)

# Create a variable containing all .cpp files:
FILE(GLOB files *.cpp)

# Create an executable file from sources:
add_library(OctreeBuilder SHARED ${files})
target_link_libraries(OctreeBuilder OctreeTypes ${DisCODe_LIBRARIES} ${PCL_IO_LIBRARIES})

INSTALL_COMPONENT(OctreeBuilder)
