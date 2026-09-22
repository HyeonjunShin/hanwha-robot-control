#pragma once

#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <stdexcept>
#include <Eigen/Dense>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace Robot {

// FK 계산 결과 구조체
struct FKResult {
    Eigen::Matrix4d transform;   // 4x4 동차 변환 행렬
    Eigen::Vector3d position_m;  // X, Y, Z 위치 (m)
    Eigen::Vector3d position_mm; // X, Y, Z 위치 (mm) -> 펜던트 비교용
    Eigen::Matrix3d rotation;    // 3x3 회전 행렬
    Eigen::Vector3d rpy_deg;     // Roll, Pitch, Yaw 오일러 각도 (deg)
};

// IK 계산 결과 구조체
struct IKResult {
    bool success = false;
    std::array<double, 6> joint_angles_deg = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    int iterations = 0;
    double error_norm = 0.0;
};

// URDF 기반 로봇 joint 오프셋 구조체
struct JointOrigin {
    double x, y, z;
    double roll, pitch, yaw;
};

class RobotKinematics {
private:
    std::array<JointOrigin, 6> joint_origins;
    JointOrigin flange_origin;

public:
    RobotKinematics() {
        // URDF 파일의 <origin xyz="..." rpy="..."/> 값을 1:1로 매핑
        // Joint 1
        joint_origins[0] = { 0.0,     0.0, 0.0,    0.0,     0.0, 0.0 };
        // Joint 2
        joint_origins[1] = { 0.0,     0.0, 0.207,  1.5708,  0.0, 0.0 };
        // Joint 3
        joint_origins[2] = { -0.73,   0.0, 0.0,    0.0,     0.0, 0.0 };
        // Joint 4
        joint_origins[3] = { -0.5388, 0.0, 0.0,    0.0,     0.0, 0.0 };
        // Joint 5
        joint_origins[4] = { 0.0,     0.0, 0.1847, 1.5708,  0.0, 0.0 };
        // Joint 6
        joint_origins[5] = { 0.0,     0.0, 0.1512, -1.5708, 0.0, 0.0 };

        // Flange (Fixed joint)
        flange_origin = { 0.0, 0.0, 0.1325, 0.0, 0.0, 0.0 };
    }

    /**
     * @brief RPY (Roll, Pitch, Yaw - rad) 오일러 각도로부터 3x3 회전 행렬 생성
     */
    inline Eigen::Matrix3d createRotationMatrix(double r, double p, double y) const {
        Eigen::AngleAxisd rollAngle(r, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(p, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(y, Eigen::Vector3d::UnitZ());

        return (yawAngle * pitchAngle * rollAngle).matrix();
    }

    /**
     * @brief URDF joint origin 및 관절 회전 각도(deg)로 4x4 변환 행렬 생성
     */
    inline Eigen::Matrix4d getJointTransform(const JointOrigin& origin, double joint_angle_deg) const {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

        // 1. Joint Origin Transform
        T.block<3, 3>(0, 0) = createRotationMatrix(origin.roll, origin.pitch, origin.yaw);
        T(0, 3) = origin.x;
        T(1, 3) = origin.y;
        T(2, 3) = origin.z;

        // 2. Revolute Joint Rotation (Z축 회전)
        double rad = joint_angle_deg * M_PI / 180.0;
        Eigen::Matrix4d T_rot = Eigen::Matrix4d::Identity();
        T_rot(0, 0) = std::cos(rad);
        T_rot(0, 1) = -std::sin(rad);
        T_rot(1, 0) = std::sin(rad);
        T_rot(1, 1) = std::cos(rad);

        return T * T_rot;
    }

    // =================================================================
    // Forward Kinematics (FK)
    // =================================================================

    /**
     * @brief URDF 기반 FK 계산 (std::array 지원)
     */
    inline FKResult computeFK(const std::array<double, 6>& q) const {
        Eigen::Matrix4d T_total = Eigen::Matrix4d::Identity();

        // Joint 1 ~ 6 연쇄 곱 연산
        for (size_t i = 0; i < 6; ++i) {
            Eigen::Matrix4d Ti = getJointTransform(joint_origins[i], q[i]);
            T_total = T_total * Ti;
        }

        // Flange Fixed Joint 변환
        Eigen::Matrix4d T_flange = Eigen::Matrix4d::Identity();
        T_flange.block<3, 3>(0, 0) = createRotationMatrix(flange_origin.roll, flange_origin.pitch, flange_origin.yaw);
        T_flange(0, 3) = flange_origin.x;
        T_flange(1, 3) = flange_origin.y;
        T_flange(2, 3) = flange_origin.z;

        T_total = T_total * T_flange;

        FKResult result;
        result.transform   = T_total;
        result.position_m  = T_total.block<3, 1>(0, 3);
        result.position_mm = result.position_m * 1000.0;
        result.rotation    = T_total.block<3, 3>(0, 0);

        // Z-Y-X (Yaw-Pitch-Roll) 오일러 각도 추출
        Eigen::Vector3d rpy_rad = result.rotation.eulerAngles(2, 1, 0); 
        result.rpy_deg = Eigen::Vector3d(rpy_rad[2], rpy_rad[1], rpy_rad[0]) * (180.0 / M_PI);

        return result;
    }

    /**
     * @brief FK 계산 (std::vector 지원)
     */
    inline FKResult computeFK(const std::vector<double>& joint_angles_deg) const {
        if (joint_angles_deg.size() < 6) {
            throw std::invalid_argument("Joint angles vector must contain at least 6 elements.");
        }
        std::array<double, 6> arr;
        std::copy_n(joint_angles_deg.begin(), 6, arr.begin());
        return computeFK(arr);
    }

    // =================================================================
    // Inverse Kinematics (IK)
    // =================================================================

    /**
     * @brief URDF 기반 수치적 Inverse Kinematics 계산
     */
    inline IKResult computeIK(const Eigen::Matrix4d& T_target, 
                              const std::array<double, 6>& q_init_deg = {0, 0, 0, 0, 0, 0},
                              int max_iterations = 200, 
                              double tolerance = 1e-4, 
                              double damping_factor = 0.01) const {
        IKResult result;
        std::array<double, 6> q_deg = q_init_deg;

        Eigen::Vector3d p_target = T_target.block<3, 1>(0, 3);
        Eigen::Matrix3d R_target = T_target.block<3, 3>(0, 0);

        for (int iter = 0; iter < max_iterations; ++iter) {
            FKResult current_fk = computeFK(q_deg);
            
            // 위치 오차
            Eigen::Vector3d pos_error = p_target - current_fk.position_m;

            // 회전 오차
            Eigen::Matrix3d R_error = R_target * current_fk.rotation.transpose();
            Eigen::AngleAxisd angle_axis(R_error);
            Eigen::Vector3d rot_error = angle_axis.axis() * angle_axis.angle();

            Eigen::Matrix<double, 6, 1> error;
            error << pos_error, rot_error;

            if (error.norm() < tolerance) {
                result.success = true;
                result.joint_angles_deg = q_deg;
                result.iterations = iter;
                result.error_norm = error.norm();
                return result;
            }

            // 야코비안 연산 및 관절 업데이트
            Eigen::Matrix<double, 6, 6> J = computeJacobian(q_deg);
            Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();
            Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity();
            Eigen::Matrix<double, 6, 1> dq = J.transpose() * (JJt + damping_factor * damping_factor * I).ldlt().solve(error);

            for (size_t i = 0; i < 6; ++i) {
                q_deg[i] += (dq(i) * (180.0 / M_PI));
            }
        }

        result.success = false;
        result.joint_angles_deg = q_deg;
        result.iterations = max_iterations;
        result.error_norm = (T_target.block<3,1>(0,3) - computeFK(q_deg).position_m).norm();
        return result;
    }

private:
    /**
     * @brief 수치적 야코비안 계산
     */
    inline Eigen::Matrix<double, 6, 6> computeJacobian(const std::array<double, 6>& q_deg) const {
        Eigen::Matrix<double, 6, 6> J;
        const double delta = 1e-6; // [rad]

        FKResult fk_base = computeFK(q_deg);

        for (size_t i = 0; i < 6; ++i) {
            std::array<double, 6> q_pert = q_deg;
            q_pert[i] += (delta * (180.0 / M_PI));

            FKResult fk_pert = computeFK(q_pert);

            Eigen::Vector3d dpos = (fk_pert.position_m - fk_base.position_m) / delta;

            Eigen::Matrix3d dR = fk_pert.rotation * fk_base.rotation.transpose();
            Eigen::AngleAxisd angle_axis(dR);
            Eigen::Vector3d drot = (angle_axis.axis() * angle_axis.angle()) / delta;

            J.col(i) << dpos, drot;
        }

        return J;
    }
};

} // namespace Robot