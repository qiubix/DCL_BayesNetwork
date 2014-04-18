# Include the directory itself as a path to include directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Create a variable containing all .cpp files:
FILE(GLOB files *.cpp)

# Create an executable file from sources:
add_library(procSOMEvaluation SHARED ${files})
target_link_libraries(procSOMEvaluation ${DisCODe_LIBRARIES} ${OpenCV_LIBS} ${SMILE_LIBS})

INSTALL_COMPONENT(procSOMEvaluation)
