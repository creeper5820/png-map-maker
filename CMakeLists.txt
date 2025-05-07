cmake_minimum_required(VERSION 3.20)
project(map-maker)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_BUILD_TYPE "Debug")

find_package(PCL REQUIRED COMPONENTS io common filters segmentation)
find_package(Eigen3 REQUIRED)
find_package(cxxopts REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} src/)

file(GLOB_RECURSE PROJECT_CC CONFIGURE_DEPENDS src/*.cc)
add_executable(
    app
    ${PROJECT_CC}
)
target_link_libraries(
    app
    ${PCL_LIBRARIES}
)
