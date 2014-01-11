#set(GMOCK_DIR $ENV{GMOCK_ROOT})
#
#add_subdirectory(${GMOCK_DIR} ${CMAKE_BINARY_DIR}/gmock)
#set(CMAKE_AUTOMOC ON)
#
#include_directories(SYSTEM ${GMOCK_DIR}/gtest/include ${GMOCK_DIR}/include)
#
#set(CREATE_NETWORK_SRC CreateNetworkTest.cpp)
#
#add_executable(CreateNetworkTest ${CREATE_NETWORK_SRC})
#
#target_link_libraries(CreateNetworkTest procCreateNetwork gmock_main)
#
#install(TARGETS CreateNetworkTest DESTINATION ${CMAKE_INSTALL_PREFIX})
#
#add_test(CreateNetworkTest CreateNetworkTest)

FIND_PACKAGE(Boost REQUIRED)
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} ../CreateNetwork)


set(CREATE_NETWORK_TEST_SRC CreateNetworkTest.cpp)
add_executable(CreateNetworkTest ${CREATE_NETWORK_TEST_SRC})
target_link_libraries(CreateNetworkTest procCreateNetwork ${Boost_LIBRARIES})

add_test(CreateNetworkTest CreateNetworkTest)
