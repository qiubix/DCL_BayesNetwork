set(CMAKE_INCLUDE_CURRENT_DIR ON)
FILE(GLOB files *.cpp)
add_library(TestUtils SHARED ${files})
target_link_libraries(TestUtils BayesNetworkTypes)
