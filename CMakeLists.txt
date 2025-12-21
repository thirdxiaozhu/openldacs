cmake_minimum_required(VERSION 3.22)
project(openldacs)

set(CMAKE_CXX_STANDARD 20)

set(OPENLDACS_THIRDPARTY_DIR ${CMAKE_SOURCE_DIR}/thirdparty)

set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fsanitize=address -g")

find_package(spdlog REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(openldacs OBJECT)
target_include_directories(openldacs PRIVATE include)
target_link_libraries(openldacs PRIVATE spdlog::spdlog itpp Eigen3::Eigen)
add_subdirectory(src)
add_subdirectory(example)