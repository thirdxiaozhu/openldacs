cmake_minimum_required(VERSION 3.22)
project(openldacs)

set(CMAKE_CXX_STANDARD 20)

set(OPENLDACS_THIRDPARTY_DIR ${CMAKE_SOURCE_DIR}/thirdparty)

set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fsanitize=address -g")

find_package(spdlog REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options )
message(STATUS "Boost_PROGRAM_OPTIONS_LIBRARY: ${Boost_PROGRAM_OPTIONS_LIBRARY}")
message(STATUS "Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")

add_subdirectory(${OPENLDACS_THIRDPARTY_DIR}/openldacs-correct)

add_library(openldacs OBJECT)
include_directories(include ${OPENLDACS_THIRDPARTY_DIR}/openldacs-correct/include)
target_link_libraries(openldacs LINK_PUBLIC spdlog::spdlog itpp correct uhd ${Boost_PROGRAM_OPTIONS_LIBRARY})
add_subdirectory(src)
add_subdirectory(example)
add_subdirectory(tests)
