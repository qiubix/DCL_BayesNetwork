# Include the directory itself as a path to include directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Create a variable containing all .cpp files:
FILE(GLOB files *.cpp)

# Create an executable file from sources:
add_library(procSOMSimpleEvaluation SHARED ${files})
target_link_libraries(procSOMSimpleEvaluation ${DisCODe_LIBRARIES} ${OpenCV_LIBS} ${SMILE_LIBS})

INSTALL_COMPONENT(procSOMSimpleEvaluation)
