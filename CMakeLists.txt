cmake_minimum_required(VERSION 3.28)

project(MyProject VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED NO_MODULE)

add_executable(main main.cpp)

target_include_directories(main
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(main
        PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/lib/libclink_api_rpc_client.so"
        "${CMAKE_CURRENT_SOURCE_DIR}/lib/kinematics/libkinematics_hcr_3gen.so"
        Eigen3::Eigen
)

# target_link_options(main
#         PUBLIC
#         "-Wl,-rpath,${CMAKE_CURRENT_SOURCE_DIR}/lib"
#         "-Wl,-rpath,${CMAKE_CURRENT_SOURCE_DIR}/lib/kinematics"
#         "-Wl,-rpath-link,${CMAKE_CURRENT_SOURCE_DIR}/lib"
#         "-Wl,-rpath-link,${CMAKE_CURRENT_SOURCE_DIR}/lib/kinematics"
# )

target_compile_definitions(main
        PUBLIC
        HANWHA_ROOT_PATH="${CMAKE_CURRENT_SOURCE_DIR}"
)