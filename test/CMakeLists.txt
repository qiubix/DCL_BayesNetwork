INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src)

function(create_test name src libs)
  add_executable(${name} ${src})
  target_link_libraries(${name} ${GMOCK_LIBRARIES} ${GMOCK_MAIN_LIBRARIES} ${libs})
  add_dependencies(${name} gmock)
  if(CMAKE_COMPILER_IS_GNUC OR CMAKE_COMPILER_IS_GNUCXX)
    target_link_libraries(${name} pthread)
  endif(CMAKE_COMPILER_IS_GNUC OR CMAKE_COMPILER_IS_GNUCXX)
  install(TARGETS ${name} DESTINATION ${CMAKE_INSTALL_PREFIX})
  add_test(${name} ${name})
endfunction(create_test)

set(CPT_MANAGER_TEST_SRC CPTManagerTest.cpp)
set(CPT_MANAGER_TEST_LIB procNetworkBuilder)
create_test(CPTManagerTest ${CPT_MANAGER_TEST_SRC} ${CPT_MANAGER_TEST_LIB})

set(BAYES_NETWORK_TEST_SRC BayesNetworkTest.cpp)
set(BAYES_NETWORK_TEST_LIB procNetworkBuilder)
create_test(BayesNetworkTest ${BAYES_NETWORK_TEST_SRC} ${BAYES_NETWORK_TEST_LIB})

set(BAYES_NETWORK_NODE_TEST_SRC BayesNetworkNodeTest.cpp)
set(BAYES_NETWORK_NODE_TEST_LIB procNetworkBuilder)
create_test(BayesNetworkNodeTest ${BAYES_NETWORK_NODE_TEST_SRC} ${BAYES_NETWORK_NODE_TEST_LIB})

set(OCTREE_NODE_TEST_SRC OctreeNodeTest.cpp)
set(OCTREE_NODE_TEST_LIB procNetworkBuilder BayesNetworkTypes)
create_test(OctreeNodeTest ${OCTREE_NODE_TEST_SRC} "${OCTREE_NODE_TEST_LIB}")

set(OCTREE_TEST_SRC OctreeTest.cpp)
set(OCTREE_TEST_LIB BayesNetworkTypes)
create_test(OctreeTest ${OCTREE_TEST_SRC} ${OCTREE_TEST_LIB})
file(COPY test_cloud.pcd DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

set(NETWORK_BUILDER_TEST_SRC NetworkBuilderTest.cpp)
set(NETWORK_BUILDER_TEST_LIB procNetworkBuilder)
create_test(NetworkBuilderTest ${NETWORK_BUILDER_TEST_SRC} ${NETWORK_BUILDER_TEST_LIB})

set(SOM_EVALUATION_TEST_SRC SOMEvaluationTest.cpp)
set(SOM_EVALUATION_TEST_LIB procSOMEvaluation procNetworkBuilder)
create_test(SOMEvaluationTest ${SOM_EVALUATION_TEST_SRC} "${SOM_EVALUATION_TEST_LIB}")

set(SOM_EVALUATION_INTEGRATION_TEST_SRC SOMEvaluationIntegrationTest.cpp BayesNetworkGenerator.cpp)
set(SOM_EVALUATION_INTEGRATION_TEST_LIB procSOMEvaluation procNetworkBuilder)
create_test(SOMEvaluationIntegrationTest "${SOM_EVALUATION_INTEGRATION_TEST_SRC}" "${SOM_EVALUATION_INTEGRATION_TEST_LIB}")

set(OCTREE_BUILDER_TEST_SRC OctreeBuilderTest.cpp)
set(OCTREE_BUILDER_TEST_LIB OctreeBuilder BayesNetworkTypes)
create_test(OctreeBuilderTest ${OCTREE_BUILDER_TEST_SRC} "${OCTREE_BUILDER_TEST_LIB}")

set(POINT_CLOUD_INDEXER_TEST_SRC PointCloudIndexerTest.cpp)
set(POINT_CLOUD_INDEXER_TEST_LIB procPointCloudIndexer)
create_test(PointCloudIndexerTest ${POINT_CLOUD_INDEXER_TEST_SRC} "${POINT_CLOUD_INDEXER_TEST_LIB}")

set(SIFT_FEATURE_MATCHER_TEST_SRC SIFTFeatureMatcherTest.cpp)
set(SIFT_FEATURE_MATCHER_TEST_LIB procSIFTFeatureMatcher)
create_test(SIFTFeatureMatcherTest ${SIFT_FEATURE_MATCHER_TEST_SRC} "${SIFT_FEATURE_MATCHER_TEST_LIB}")
