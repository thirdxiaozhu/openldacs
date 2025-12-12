cmake_minimum_required(VERSION 3.22)
project(openldacs)

set(CMAKE_CXX_STANDARD 17)


set(OPENLDACS_THIRDPARTY_DIR ${CMAKE_SOURCE_DIR}/thirdparty)


set(OPENLDACS_USE_EIGEN_STR "Build Eigen for OPENLDACS")
option(OPENLDACS_USE_EIGEN ${OPENLDACS_USE_EIGEN_STR} ON)
set(OPENLDACS_USE_GTEST_STR "Build Google Test for OPENLDACS")
option(OPENLDACS_USE_GTEST ${OPENLDACS_USE_GTEST_STR} ON)

# Download and configure
if (OPENLDACS_USE_EIGEN)
  message(STATUS "Setting up Eigen ...")
  execute_process(
    COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
    OUTPUT_QUIET
    RESULT_VARIABLE result
    WORKING_DIRECTORY ${OPENLDACS_THIRDPARTY_DIR}/eigen)
  if(result)
    message(WARNING "Failed to download Eigen (${result}); disabling `OPENLDACS_USE_EIGEN`")
  endif()
  set(INSTALL_EIGEN OFF CACHE BOOL "" FORCE)
  mark_as_advanced(INSTALL_EIGEN)
  endif()

if (OPENLDACS_USE_EIGEN)
  execute_process(COMMAND ${CMAKE_COMMAND} --build .
    OUTPUT_QUIET
    RESULT_VARIABLE result
    WORKING_DIRECTORY ${OPENLDACS_THIRDPARTY_DIR}/eigen)
  if(result)
    message(WARNING "Failed to build Eigen (${result}); disabling `OPENLDACS_USE_EIGEN`")
  endif()

  add_subdirectory(${OPENLDACS_THIRDPARTY_DIR}/eigen/src EXCLUDE_FROM_ALL)
  set(EIGEN_INCLUDE_DIR ${OPENLDACS_THIRDPARTY_DIR}/eigen/src)
  include_directories(${EIGEN_INCLUDE_DIR})
endif()

# Google Test
# This follows the example in
# https://github.com/google/googletest/blob/release-1.10.0/googletest/README.md.

# Download and configure
if(OPENLDACS_USE_GTEST )
    message(STATUS "Setting up Google Test ...")
    execute_process(
        COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
        OUTPUT_QUIET
        RESULT_VARIABLE result
        WORKING_DIRECTORY ${OPENLDACS_THIRDPARTY_DIR}/googletest)
    if(result)
        message(WARNING "Failed to download Google Test (${result}); disabling `OPENLDACS_USE_GTEST`")
    endif()
    set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
    mark_as_advanced(BUILD_GMOCK)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    mark_as_advanced(INSTALL_GTEST)
endif()

# Build
if(OPENLDACS_USE_GTEST )
    execute_process(COMMAND ${CMAKE_COMMAND} --build .
        OUTPUT_QUIET
        RESULT_VARIABLE result
        WORKING_DIRECTORY ${OPENLDACS_THIRDPARTY_DIR}/googletest)
    if(result)
        message(WARNING "Failed to build Google Test (${result}); disabling `OPENLDACS_USE_GTEST`")
    endif()
endif()

# Set up the targets
if(OPENLDACS_USE_GTEST )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    add_subdirectory(
        ${OPENLDACS_THIRDPARTY_DIR}/googletest/src
        ${OPENLDACS_THIRDPARTY_DIR}/googletest/build
        EXCLUDE_FROM_ALL)
endif()
set(CMAKE_CXX_FLAGS  "${CMAKE_C_FLAGS} -fsanitize=address -g")

find_package(spdlog REQUIRED)

add_library(openldacs OBJECT)
target_include_directories(openldacs PRIVATE include)
target_link_libraries(openldacs PRIVATE spdlog::spdlog)
add_subdirectory(src)
add_subdirectory(example)
