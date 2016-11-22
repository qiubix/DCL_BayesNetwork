# Function adding googletest suite
# name - name of the test suite
# src - sources of tests
# libs - libraries to be linked
function(create_test_suite name src libs)
  add_executable(${name} ${src})
  add_dependencies(${name} gmock)
  target_link_libraries(${name} ${GMOCK_LIBRARIES} ${GMOCK_MAIN_LIBRARIES} ${libs})

  if(CMAKE_COMPILER_IS_GNUC OR CMAKE_COMPILER_IS_GNUCXX)
    target_link_libraries(${name} pthread)
  endif(CMAKE_COMPILER_IS_GNUC OR CMAKE_COMPILER_IS_GNUCXX)

  install(TARGETS ${name} DESTINATION ${CMAKE_INSTALL_PREFIX})
  add_test(${name} ${name})
endfunction()
