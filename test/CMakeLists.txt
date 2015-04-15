INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

macro(create_test name src libs)
  add_executable(${name} ${src})
  target_link_libraries(${name} ${GMOCK_LIBRARIES} ${GMOCK_MAIN_LIBRARIES} ${libs})
  add_dependencies(${name} gmock)
  if(CMAKE_COMPILER_IS_GNUC OR CMAKE_COMPILER_IS_GNUCXX)
    target_link_libraries(${name} pthread)
  endif(CMAKE_COMPILER_IS_GNUC OR CMAKE_COMPILER_IS_GNUCXX)
  install(TARGETS ${name} DESTINATION ${CMAKE_INSTALL_PREFIX})
  add_test(${name} ${name})
endmacro(create_test)

set(CPT_MANAGER_TEST_SRC CPTManagerTest.cpp)
set(CPT_MANAGER_TEST_LIB procNetworkBuilder)
create_test(CPTManagerTest ${CPT_MANAGER_TEST_SRC} ${CPT_MANAGER_TEST_LIB})

set(BAYES_NETWORK_TEST_SRC BayesNetworkTest.cpp)
set(BAYES_NETWORK_TEST_LIB procNetworkBuilder)
create_test(BayesNetworkTest ${BAYES_NETWORK_TEST_SRC} ${BAYES_NETWORK_TEST_LIB})

set(OCTREE_NODE_TEST_SRC OctreeNodeTest.cpp)
set(OCTREE_NODE_TEST_LIB procNetworkBuilder)
create_test(OctreeNodeTest ${OCTREE_NODE_TEST_SRC} ${OCTREE_NODE_TEST_LIB})

set(OCTREE_TEST_SRC OctreeTest.cpp)
set(OCTREE_TEST_LIB procNetworkBuilder)
create_test(OctreeTest ${OCTREE_TEST_SRC} ${OCTREE_TEST_LIB})
file(COPY test_cloud.pcd DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

set(NETWORK_BUILDER_TEST_SRC NetworkBuilderTest.cpp)
set(NETWORK_BUILDER_TEST_LIB procNetworkBuilder)
create_test(NetworkBuilderTest ${NETWORK_BUILDER_TEST_SRC} ${NETWORK_BUILDER_TEST_LIB})
