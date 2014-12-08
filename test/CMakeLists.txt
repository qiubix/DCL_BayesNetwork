INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

# Find Boost, at least ver. 1.41
# FIND_PACKAGE(Boost 1.41.0 COMPONENTS system REQUIRED)
# include_directories(SYSTEM ${Boost_INCLUDE_DIR})
# 
# link_directories(${Boost_LIBRARY_DIRS})
# add_definitions(-DBOOST_ALL_NO_LIB)  # tell the compiler to undefine this boost macro

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

set(BAYES_NETWORK_TEST_SRC BayesNetworkTest.cpp)
set(BAYES_NETWORK_TEST_LIB procCreateNetworkWithSpacialDependencies)
create_test(BayesNetworkTest ${BAYES_NETWORK_TEST_SRC} ${BAYES_NETWORK_TEST_LIB})

set(CREATE_NETWORK_TEST_SRC CreateNetworkWithSpacialDependenciesTest.cpp)
#set(CREATE_NETWORK_TEST_LIB ${DisCODe_LIBRARIES} ${Boost_LIBRARIES} procCreateNetworkWithSpacialDependencies ${PCL_LIBRARY_DIRS})
set(CREATE_NETWORK_TEST_LIB procCreateNetworkWithSpacialDependencies)
create_test(CreateNetworkWithSpacialDependenciesTest ${CREATE_NETWORK_TEST_SRC} ${CREATE_NETWORK_TEST_LIB})
