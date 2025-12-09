#ifndef DIFFERENTIAL_FLATNESS_H
#define DIFFERENTIAL_FLATNESS_H

#include <Eigen/Dense>
#include <vector>

namespace quadrotor_df {

// 计算位置
Eigen::Vector3d computePosition(double t);

// 计算速度（解析导数）
Eigen::Vector3d computeVelocity(double t);

// 计算加速度（解析导数）
Eigen::Vector3d computeAcceleration(double t);

// 计算偏航角（从速度方向）
double computeYaw(const Eigen::Vector3d& vel);

// 计算姿态旋转矩阵
Eigen::Matrix3d computeRotationMatrix(
    const Eigen::Vector3d& pos,
    const Eigen::Vector3d& vel,
    const Eigen::Vector3d& acc,
    double yaw,
    double g = 9.81
);

// 旋转矩阵转四元数 [x, y, z, w]
Eigen::Vector4d rotationMatrixToQuaternion(const Eigen::Matrix3d& R);

// 归一化四元数并确保 w >= 0
Eigen::Vector4d normalizeQuaternion(const Eigen::Vector4d& q);

} // namespace quadrotor_df

#endif // DIFFERENTIAL_FLATNESS_H

