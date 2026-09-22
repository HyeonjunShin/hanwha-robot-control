#include <iostream>
#include <thread>
#include <chrono>
#include <sys/mman.h>
#include <fcntl.h>
#include <iomanip>
#include <thread>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "clink_api_rpc_system.h"
#include "clink_api_rpc.h"
#include "iRobot.h"


inline double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

inline long long get_pc_timestamp_us() {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
}

struct FlangeDataFrame {
    long long timestamp_us;
    // double flange_pos[6];
    double flange_tf[16]; // 4x4 matrix
};

struct SharedDataBuffer {
    bool status;
    uint8_t padding[7];
    uint64_t sequence;       
    uint64_t head;           
    FlangeDataFrame buffer[100];
};

class Hanwha : public IRobot{
private:
    CLINK_API_RESULT err_ret_val = CLINK_API_RESULT_OK;
    const CLINK_CBOX_MODEL	CBOX_MODEL_NAME = CLINK_CBOX_MODEL_3GEN;				// 펜턴트 버전
    const CLINK_ROBOT_MODEL ROBOT_MODEL_NAME = CLINK_ROBOT_MODEL_HCR14; 			// 로봇 종류
    uint32_t cbox_id;																// 로봇 컨트롤 박스 id;
    uint32_t robot_id;																// 로봇 id
    const std::string		CLINK_CONFIG_FILE =  std::string(HANWHA_ROOT_PATH) + "/config/config_rpc.ini";
    std::atomic<bool> g_thread_running{false};
    std::thread capture_thread_;
    
public:
    Hanwha(const std::string &IP);
    ~Hanwha();
    void disconnect() override;

    void run();
    bool conn_flange_shm(const std::string &shm_name) override;
    void runner_shm(std::string shm_name);
    std::array<double, 6> get_curr_joint_deg();
};