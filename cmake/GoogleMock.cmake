cmake_minimum_required(VERSION 2.8.12)

include(ExternalProject)

set(EXTERNAL_PROJECT_GMOCK_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/gmock)
set(EXTERNAL_PROJECT_GMOCK_SOURCE_DIR ${CMAKE_SOURCE_DIR}/ext/gmock)

ExternalProject_Add(
    gmock
    SVN_REPOSITORY http://googlemock.googlecode.com/svn/tags/release-1.7.0
    #TODO: figure out what happens if download time is longer than specified timeout
    #TIMEOUT 10

    #configure build tree
    PREFIX ${EXTERNAL_PROJECT_GMOCK_BINARY_DIR}
    DOWNLOAD_DIR ${EXTERNAL_PROJECT_GMOCK_SOURCE_DIR}
    SOURCE_DIR ${EXTERNAL_PROJECT_GMOCK_SOURCE_DIR}
    BINARY_DIR ${EXTERNAL_PROJECT_GMOCK_BINARY_DIR}
    STAMP_DIR ${EXTERNAL_PROJECT_GMOCK_BINARY_DIR}/stamp

    #configure logging
    LOG_DOWNLOAD ON
    LOG_UPDATE ON
    LOG_CONFIGURE ON
    LOG_BUILD ON

    # to prevent install step
    INSTALL_COMMAND ""
    # to prevent updating
    UPDATE_COMMAND ""
)

ExternalProject_Get_Property(gmock source_dir)
set(GMOCK_INCLUDE_DIRS ${source_dir}/include)
include_directories(${GMOCK_INCLUDE_DIRS})

ExternalProject_Get_Property(gmock binary_dir)
set(GMOCK_LIBRARIES ${binary_dir}/${CMAKE_STATIC_LIBRARY_PREFIX}gmock${CMAKE_STATIC_LIBRARY_SUFFIX})
set(GMOCK_MAIN_LIBRARIES ${binary_dir}/${CMAKE_STATIC_LIBRARY_PREFIX}gmock_main${CMAKE_STATIC_LIBRARY_SUFFIX})

set(GTEST_INCLUDE_DIRS ${source_dir}/gtest/include)
include_directories(${GTEST_INCLUDE_DIRS})

set(GTEST_LIBRARIES ${binary_dir}/gtest/${CMAKE_STATIC_LIBRARY_PREFIX}gtest${CMAKE_STATIC_LIBRARY_SUFFIX})
set(GTEST_MAIN_LIBRARIES ${binary_dir}/gtest/${CMAKE_STATIC_LIBRARY_PREFIX}gtest_main${CMAKE_STATIC_LIBRARY_SUFFIX})

enable_testing()
