#include <iostream>
#include <thread>
#include <chrono>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "clink_api_rpc_system.h"
#include "clink_api_rpc.h"


const std::string ip = "192.168.100.200";
const CLINK_CBOX_MODEL	CBOX_MODEL_NAME = CLINK_CBOX_MODEL_3GEN;				// 펜턴트 버전
const CLINK_ROBOT_MODEL ROBOT_MODEL_NAME = CLINK_ROBOT_MODEL_HCR14; 			// 로봇 종류

const std::string		CLINK_CONFIG_FILE =  std::string(HANWHA_ROOT_PATH) + "/config/config_rpc.ini";
uint32_t cbox_id;																// 로봇 컨트롤 박스 id;
uint32_t robot_id;																// 로봇 id

inline double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

int main(){
    CLINK_API_RESULT err_ret_val = CLINK_API_RESULT_OK;
    err_ret_val = clink_rpc_system_cbox_connect(CLINK_CONFIG_FILE.c_str(), "192.168.100.200", &cbox_id);    

    //  제어SW 초기화
    err_ret_val = clink_rpc_gen_system_create(cbox_id, "", CBOX_MODEL_NAME);
    if (err_ret_val != CLINK_API_RESULT_OK && err_ret_val < CLINK_API_RESULT_WARNING_BEGIN) {
        std::cout << "[Error] [HanwhaRobot] 제어SW 초기화 실패: " << err_ret_val << std::endl;
        return false;
    }

    // 제어권 획득
    err_ret_val = clink_rpc_system_control_take(cbox_id);
    if (err_ret_val != CLINK_API_RESULT_OK) {
        std::cout << "[Error] [HanwhaRobot] 제어권 획득 실패" << std::endl;
        return false;
    }

    // 로봇 생성
    err_ret_val = clink_rpc_robot_create(cbox_id, ROBOT_MODEL_NAME, "", 0U, &robot_id);
    if (err_ret_val != CLINK_API_RESULT_OK) {
        std::cout << "[Error] [HanwhaRobot] 로봇 생성 실패" << std::endl;
        return false;
    }

    // EtherCAT 상태 확인 및 대기
    CLINK_ECAT_CONN_STATE ecat_stat = CLINK_ECAT_CONN_STATE_DISCONNECTED;
    clink_rpc_cbox_ecat_connection_state_get(cbox_id, &ecat_stat);
    if (CLINK_ECAT_CONN_STATE_CONNECTED != ecat_stat) {
        char_t valid_event = -1;
        clink_rpc_system_wait_event_group_subgroup(
            cbox_id,
            CLINK_EVENT_GRP_NOTIFICATION,
            CLINK_EVENT_SUBGRP_NOTIFICATION_ECAT_CONNECTED,
            1000000,
            1,
            &valid_event);
    }

    // 6. 자동 속도 조절 기능 ON
    clink_rpc_robot_motion_auto_adjust_swith_set(cbox_id, robot_id, CLINK_SWITCH_ON);

    // // --- 설정된 각속도 및 각가속도 값 출력 ---
    // // clink_rpc_robot_safety_limit_joint_speed_max_set(cbox_id, robot_id, 0, 200);
    // // clink_rpc_robot_safety_limit_joint_speed_max_set(cbox_id, robot_id, 1, 200);
    // // clink_rpc_robot_safety_limit_joint_speed_max_set(cbox_id, robot_id, 2, 200);
    // // clink_rpc_robot_safety_limit_joint_speed_max_set(cbox_id, robot_id, 3, 270);
    // // clink_rpc_robot_safety_limit_joint_speed_max_set(cbox_id, robot_id, 4, 270);
    // // clink_rpc_robot_safety_limit_joint_speed_max_set(cbox_id, robot_id, 5, 270);


    // std::cout << "\n[Info] [HanwhaRobot] --- 설정된 조인트 한계값 (Safety Limit) ---" << std::endl;


    clink_float_t p_pos_x;
    clink_float_t p_pos_y;
    clink_float_t p_pos_z;
    clink_float_t p_ort_x;
    clink_float_t p_ort_y;
    clink_float_t p_ort_z;
    bool first_run = false;
    while (1)
    {
        clink_rpc_robot_tcp_pose_actual_get(cbox_id, robot_id, &p_pos_x, &p_pos_y, &p_pos_z, &p_ort_x, &p_ort_y, &p_ort_z);

        // Eigen 변환 행렬 생성
        Eigen::Affine3d T = Eigen::Affine3d::Identity();
        T.translation() << p_pos_x, p_pos_y, p_pos_z;

        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(deg2rad(p_ort_z), Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(deg2rad(p_ort_y), Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(deg2rad(p_ort_x), Eigen::Vector3d::UnitX());

        T.linear() = R;
        Eigen::Matrix4d T_matrix = T.matrix();

        // 두 번째 루프부터는 출력물 높이(총 5줄)만큼 커서를 위로 올려 덮어씁니다.
        if (!first_run) {
            std::cout << "\033[5A"; // 5줄 위로 이동
        }
        first_run = false;

        // 1줄차: 원본 Pose 데이터
        std::cout << "POS: [" << std::fixed << std::setprecision(2)
                << p_pos_x << ", " << p_pos_y << ", " << p_pos_z << "] "
                << "ORT: [" 
                << p_ort_x << ", " << p_ort_y << ", " << p_ort_z << "]\033[K\n";

        // 2~5줄차: 4x4 행렬 출력
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                std::cout << std::setw(10) << T_matrix(i, j) << " ";
            }
            std::cout << "\033[K\n"; // \033[K : 이전 출력의 잔여 문자 삭제
        }

        std::cout << std::flush; // 버퍼 비우기

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }



        

    // for (uint32_t i = 0; i < 6; i++) {
    //     clink_float_t set_speed_max = 0.0f;
    //     clink_float_t set_acc_max = 0.0f;

    //     // 설정된 최대 속도 가져오기 (degree/s)
    //     clink_rpc_robot_safety_limit_joint_speed_max_get(cbox_id, robot_id, i, &set_speed_max);
    //     // 설정된 최대 가속도 가져오기 (degree/s^2)
    //     clink_rpc_robot_safety_limit_joint_acc_max_get(cbox_id, robot_id, i, &set_acc_max);

    //     std::cout << "[Info] Joint " << i << " | 설정 각속도: " << set_speed_max
    //          << " deg/s | 설정 각가속도: " << set_acc_max << " deg/s^2" << std::endl;
    // }

    // // 글로벌 스피드 팩터(전체 속도 비율) 출력
    // clink_float_t speed_factor = 0.0f;
    // clink_rpc_robot_speed_factor_get(cbox_id, robot_id, &speed_factor);
    // std::cout << "[Info] 현재 시스템 Speed Factor: " << speed_factor * 100.0f << " %" << std::endl;
    // std::cout << "---------------------------------------------------------\n" << std::endl;

    // std::cout << "[Info] [HanwhaRobot] 로봇 연결 및 초기화 완료: " << ip_ << std::endl;


    // CLINK_API_RESULT err_ret_val = clink_rpc_system_cbox_connect(CLINK_CONFIG_FILE.c_str(), ip_.c_str(), &cbox_id);
    if (err_ret_val != CLINK_API_RESULT_OK) {
        std::cout << "[Error] [HanwhaRobot] 컨트롤 박스 연결 실패: " << err_ret_val << std::endl;
        return false;
    }


    return 1;
}