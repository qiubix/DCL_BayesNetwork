# Include the directory itself as a path to include directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Create a variable containing all .cpp files:
FILE(GLOB files *.cpp)

# Create an executable file from sources:
add_library(procSIFTFeatureMatcher SHARED ${files})
target_link_libraries(procSIFTFeatureMatcher ${DisCODe_LIBRARIES} ${SMILE_LIBS} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

INSTALL_COMPONENT(procSIFTFeatureMatcher)
