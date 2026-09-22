find_package(Eigen3 REQUIRED NO_MODULE)

add_library(hanwha_robot STATIC "${CMAKE_CURRENT_SOURCE_DIR}/src/hanwha_robot.cpp")
target_include_directories(hanwha_robot PUBLIC 
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        "${CMAKE_SOURCE_DIR}/include"
)

target_link_libraries(hanwha_robot
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

target_compile_definitions(hanwha_robot PUBLIC HANWHA_ROOT_PATH="${CMAKE_CURRENT_SOURCE_DIR}")

file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/lib/kinematics/libkinematics_hcr_3gen.so"
     DESTINATION "${CMAKE_BINARY_DIR}/lib/kinematics"
     FILES_MATCHING PATTERN "*.so"
)


find_package(pybind11 REQUIRED)
pybind11_add_module(hanwha_robot_py                           # 빌드 이름
        ${CMAKE_CURRENT_SOURCE_DIR}/src/pybind_module.cpp   # 소스
        # ${CMAKE_CURRENT_SOURCE_DIR}/src/hanwha_robot.cpp
)
target_link_libraries(hanwha_robot_py PUBLIC hanwha_robot)

set_target_properties(hanwha_robot_py PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/"
#     OUTPUT_NAME "hanwha_robot_py"
    PREFIX ""     
    SUFFIX ".so"
#     중요: 자기 자신 폴더($ORIGIN)를 라이브러리 탐색 경로로 설정
    INSTALL_RPATH "$ORIGIN"
    BUILD_WITH_INSTALL_RPATH TRUE
)


# add_custom_command(TARGET rt_bind POST_BUILD
#     # 1) HCR 동적 라이브러리(.so) 복사
#     COMMAND ${CMAKE_COMMAND} -E copy_if_different "${HCR_LIB_PATH}/libclink_api_rpc_client.so" "$<TARGET_FILE_DIR:rt_bind>/"
#     COMMAND ${CMAKE_COMMAND} -E copy_if_different "${HCR_KINE_LIB_PATH}/libkinematics_hcr_3gen.so" "$<TARGET_FILE_DIR:rt_bind>/"
    
#     # 2) HCR 설정 파일(config_rpc.ini) 복사
#     COMMAND ${CMAKE_COMMAND} -E copy_if_different "${HCR_SDK_DIR}/config/config_rpc.ini" "$<TARGET_FILE_DIR:rt_bind>/"
    
#     # 3) 로그가 저장될 빈 log 폴더 생성
#     COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:rt_bind>/log"
    
#     COMMENT "Packaging all dependencies and configs into rt_py directory..."
# )

add_library(ik_solver INTERFACE)
target_include_directories(ik_solver INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(ik_solver INTERFACE Eigen3::Eigen)