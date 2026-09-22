cmake_minimum_required(VERSION 3.28)

project(Robot_wrapper VERSION 1.0 LANGUAGES CXX)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include_directories(include)

add_subdirectory(robots/hanwha)
add_executable(main main.cpp)
target_link_libraries(main PUBLIC hanwha_robot ik_solver)