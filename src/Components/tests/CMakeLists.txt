FIND_PACKAGE(Boost COMPONENTS system REQUIRED)
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} ../CreateNetwork)

set(CREATE_NETWORK_TEST_SRC CreateNetworkTest.cpp)
#add_executable(CreateNetworkTest ${CREATE_NETWORK_TEST_SRC})
#target_link_libraries(CreateNetworkTest procCreateNetwork ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${DisCODe_LIBRARIES})

#add_test(CreateNetworkTest CreateNetworkTest)
#install_component(CreateNetworkTest)
