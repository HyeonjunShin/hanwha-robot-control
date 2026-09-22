#include "hanwha_robot.h"

void check_sensing_frequency(uint32_t cbox_id, uint32_t robot_id) {
    clink_float_t px, py, pz, rx, ry, rz;
    clink_float_t prev_px = 0, prev_py = 0, prev_pz = 0;
    
    auto prev_time = std::chrono::high_resolution_clock::now();
    int sample_count = 0;

    std::cout << "[Sensing Check] 데이터 변경 주기 측정 중...\n";

    while (sample_count < 10) {
        clink_rpc_robot_flange_pose_actual_get(cbox_id, robot_id, &px, &py, &pz, &rx, &ry, &rz);

        // 데이터가 이전과 달라진 순간 (센서 값이 새로 갱신된 시점)
        if (px != prev_px || py != prev_py || pz != prev_pz) {
            auto curr_time = std::chrono::high_resolution_clock::now();
            double dt_ms = std::chrono::duration<double, std::milli>(curr_time - prev_time).count();

            if (sample_count > 0) { // 첫 샘플은 제외
                std::cout << "데이터 갱신 간격: " << dt_ms << " ms (" << 1000.0 / dt_ms << " Hz)\n";
            }

            prev_time = curr_time;
            prev_px = px; prev_py = py; prev_pz = pz;
            sample_count++;
        }
    }
}

Hanwha::Hanwha(const std::string &IP): IRobot(IP){
    err_ret_val = clink_rpc_system_cbox_connect(CLINK_CONFIG_FILE.c_str(), IP.c_str(), &cbox_id);    

    //  제어SW 초기화
    err_ret_val = clink_rpc_gen_system_create(cbox_id, "", CBOX_MODEL_NAME);
    if (err_ret_val != CLINK_API_RESULT_OK && err_ret_val < CLINK_API_RESULT_WARNING_BEGIN) {
        std::cout << "[Error] [HanwhaRobot] 제어SW 초기화 실패: " << err_ret_val << std::endl;
    }

    // 제어권 획득
    err_ret_val = clink_rpc_system_control_take(cbox_id);
    if (err_ret_val != CLINK_API_RESULT_OK) {
        std::cout << "[Error] [HanwhaRobot] 제어권 획득 실패" << std::endl;
    }

    // 로봇 생성
    err_ret_val = clink_rpc_robot_create(cbox_id, ROBOT_MODEL_NAME, "", 0U, &robot_id);
    if (err_ret_val != CLINK_API_RESULT_OK) {
        std::cout << "[Error] [HanwhaRobot] 로봇 생성 실패" << std::endl;
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
}

Hanwha::~Hanwha(){
    g_thread_running = false;
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }
}


void Hanwha::disconnect(){
    clink_rpc_robot_stop(cbox_id, robot_id, 0.5);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    clink_rpc_robot_servo_switch_set(cbox_id, robot_id, CLINK_SWITCH_OFF);
    clink_rpc_system_control_release(cbox_id);

    std::cout << "[Info] [HanwhaRobot] 로봇 연결 해제: " << IP << std::endl;
}

void Hanwha::runner_shm(std::string shm_name){
    int shm_fd = shm_open(shm_name.c_str(), O_RDWR, 0666);
    if (shm_fd == -1) { 
        perror("[C++] Fail create shared memory");
        std::_Exit(EXIT_FAILURE);
    }
    size_t shm_size = sizeof(SharedDataBuffer);

    SharedDataBuffer* shared_mem = (SharedDataBuffer*)mmap(
        0, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0
    );
    if (shared_mem == MAP_FAILED) {
        perror("[C++] Fail mmap");
        close(shm_fd);
        std::_Exit(EXIT_FAILURE);
    }
    shared_mem->status = true;
    shared_mem->sequence = 0;
    shared_mem->head = 0;

    clink_float_t p_pos_x;
    clink_float_t p_pos_y;
    clink_float_t p_pos_z;
    clink_float_t p_ort_x;
    clink_float_t p_ort_y;
    clink_float_t p_ort_z;

    while (g_thread_running) {
        clink_rpc_robot_flange_pose_actual_get(cbox_id, robot_id, &p_pos_x, &p_pos_y, &p_pos_z, &p_ort_x, &p_ort_y, &p_ort_z);
        Eigen::Affine3d T = Eigen::Affine3d::Identity();
        T.translation() << p_pos_x * 0.001, p_pos_y * 0.001, p_pos_z * 0.001;
        // std::cout << p_pos_x << ", " << p_pos_y << ", " << p_pos_z << ", " << p_ort_x << ", " << p_ort_y << ", " << p_ort_z << ", " << std::endl;

        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(deg2rad(p_ort_z), Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(deg2rad(p_ort_y), Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(deg2rad(p_ort_x), Eigen::Vector3d::UnitX());

        T.linear() = R;
        Eigen::Matrix4d T_matrix = T.matrix();

        // 3. Shared Memory 버퍼 기록
        uint32_t idx = shared_mem->head;

        shared_mem->buffer[idx].timestamp_us = get_pc_timestamp_us();

        // Row-Major 형태(1D 16개 배열)로 공유 메모리에 저장
        Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(
            shared_mem->buffer[idx].flange_tf
        ) = T_matrix;

        // 시퀀스 및 Head 인덱스 업데이트
        shared_mem->head = (idx + 1) % 100;
        shared_mem->sequence++;
        // std::cout << T_matrix << std::endl;

        // 갱신 주기를 로봇의 sensing 주기(2ms)와 맞춤
        // std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    munmap(shared_mem, shm_size);
    close(shm_fd);

}

bool Hanwha::conn_flange_shm(const std::string &shm_name){
    g_thread_running = true;
    capture_thread_ = std::thread(&Hanwha::runner_shm, this, shm_name);
    return true;
}



void Hanwha::run(){

    check_sensing_frequency(cbox_id, robot_id);

    // clink_float_t p_pos_x;
    // clink_float_t p_pos_y;
    // clink_float_t p_pos_z;
    // clink_float_t p_ort_x;
    // clink_float_t p_ort_y;
    // clink_float_t p_ort_z;
    // bool first_run = false;
    // while (1)
    // {
    //     clink_rpc_robot_flange_pose_actual_get(cbox_id, robot_id, &p_pos_x, &p_pos_y, &p_pos_z, &p_ort_x, &p_ort_y, &p_ort_z);

    //     // Eigen 변환 행렬 생성
    //     Eigen::Affine3d T = Eigen::Affine3d::Identity();
    //     T.translation() << p_pos_x, p_pos_y, p_pos_z;

    //     Eigen::Matrix3d R;
    //     R = Eigen::AngleAxisd(deg2rad(p_ort_z), Eigen::Vector3d::UnitZ())
    //     * Eigen::AngleAxisd(deg2rad(p_ort_y), Eigen::Vector3d::UnitY())
    //     * Eigen::AngleAxisd(deg2rad(p_ort_x), Eigen::Vector3d::UnitX());

    //     T.linear() = R;
    //     Eigen::Matrix4d T_matrix = T.matrix();

    //     // 두 번째 루프부터는 출력물 높이(총 5줄)만큼 커서를 위로 올려 덮어씁니다.
    //     if (!first_run) {
    //         std::cout << "\033[5A"; // 5줄 위로 이동
    //     }
    //     first_run = false;

    //     // 1줄차: 원본 Pose 데이터
    //     std::cout << "POS: [" << std::fixed << std::setprecision(4)
    //             << p_pos_x << ", " << p_pos_y << ", " << p_pos_z << "] "
    //             << "ORT: [" 
    //             << p_ort_x << ", " << p_ort_y << ", " << p_ort_z << "]\033[K\n";

    //     // 2~5줄차: 4x4 행렬 출력
    //     for (int i = 0; i < 4; ++i) {
    //         for (int j = 0; j < 4; ++j) {
    //             std::cout << std::setw(10) << T_matrix(i, j) << " ";
    //         }
    //         std::cout << "\033[K\n"; // \033[K : 이전 출력의 잔여 문자 삭제
    //     }

    //     std::cout << std::flush; // 버퍼 비우기

    //     // std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // }




}

std::array<double, 6> Hanwha::get_curr_joint_deg()
{
    std::array<double, 6> joint_deg;
 
    for (int i = 0; i < 6; ++i) {
        clink_rpc_robot_joint_angle_actual_get(cbox_id, robot_id, i, &joint_deg[i]);
    }

    return joint_deg;
}

std::array<double, 6> Hanwha::get_curr_tcp(){
    std::array<double, 6> tcp;
    clink_rpc_robot_tcp_pose_actual_get(cbox_id, robot_id, &tcp[0], &tcp[1], &tcp[2], &tcp[3], &tcp[4], &tcp[5]);
    return tcp;
}
