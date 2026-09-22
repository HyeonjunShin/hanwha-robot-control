#include "ik_solver.h"
#include "hanwha_robot.h"

int main(){
    Hanwha hanwha_obj("192.168.100.200");
    // hanwha_obj.conn_flange_shm("/flange_pose");
    Robot::RobotKinematics kinematics;

    while (1){
        std::array<double,6> angles = hanwha_obj.get_curr_joint_deg();
        std::array<double, 6> tcp = hanwha_obj.get_curr_tcp();
        Robot::FKResult fk = kinematics.computeFK(angles);

        double calc_fk[6] = {
            fk.position_mm.x(),
            fk.position_mm.y(),
            fk.position_mm.z(),
            0.0, 0.0, 0.0
        };
        std::cout << std::fixed << std::setprecision(3);

        std::cout << "\n+-------------------------------------------------------------------------+\n";
        std::cout << "|                           HANWHA ROBOT STATUS                           |\n";
        std::cout << "+---------+---------------+-----------+-----------------+-----------------+\n";
        std::cout << "| Joint   | Angle (deg)   | TCP Axis  | Robot TCP       | Calc FK (mm)    |\n";
        std::cout << "+---------+---------------+-----------+-----------------+-----------------+\n";

        const char* tcp_labels[6] = {"X (mm)", "Y (mm)", "Z (mm)", "Rx (deg)", "Ry (deg)", "Rz (deg)"};

        for (int i = 0; i < 6; ++i) {
            std::cout << "| J" << (i + 1) << "      |"
                    << std::right << std::setw(14) << angles[i] << " | "
                    << std::left  << std::setw(9)  << tcp_labels[i] << " | "
                    << std::right << std::setw(15) << tcp[i] << " | ";

            if (i < 3) {
                std::cout << std::right << std::setw(15) << calc_fk[i] << " |\n";
            } else {
                std::cout << std::right << std::setw(15) << "N/A" << " |\n";
            }
        }

        std::cout << "+---------+---------------+-----------+-----------------+-----------------+\n\n";

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    // hanwha_obj.run();   
    // hanwha_obj.disconnect();
    // // --- 설정된 각속도 및 각가속도 값 출력 ---
    // // clink_rpc_robot_safety_limit_joint_speed_max_set(cbox_id, robot_id, 0, 200);
    // // clink_rpc_robot_safety_limit_joint_speed_max_set(cbox_id, robot_id, 1, 200);
    // // clink_rpc_robot_safety_limit_joint_speed_max_set(cbox_id, robot_id, 2, 200);
    // // clink_rpc_robot_safety_limit_joint_speed_max_set(cbox_id, robot_id, 3, 270);
    // // clink_rpc_robot_safety_limit_joint_speed_max_set(cbox_id, robot_id, 4, 270);
    // // clink_rpc_robot_safety_limit_joint_speed_max_set(cbox_id, robot_id, 5, 270);


    // std::cout << "\n[Info] [HanwhaRobot] --- 설정된 조인트 한계값 (Safety Limit) ---" << std::endl;
        

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
    // if (err_ret_val != CLINK_API_RESULT_OK) {
        // std::cout << "[Error] [HanwhaRobot] 컨트롤 박스 연결 실패: " << err_ret_val << std::endl;
        // return false;
    // }

    

    return 1;
}