# Include the directory itself as a path to include directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Create a variable containing all .cpp files:
FILE(GLOB files *.cpp)

# Create an executable file from sources:
add_library(procPointCloudIndexer SHARED ${files})
target_link_libraries(procPointCloudIndexer ${DisCODe_LIBRARIES} ${PCL_LIBS} ${Boost_LIBRARIES})

INSTALL_COMPONENT(procPointCloudIndexer)
