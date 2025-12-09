#include <quadrotor_df/differential_flatness.h>
#include <cmath>

namespace quadrotor_df {

Eigen::Vector3d computePosition(double t) {
    double sin_t = std::sin(t);
    double cos_t = std::cos(t);
    double denom = 1.0 + sin_t * sin_t;
    
    double x = 10.0 * cos_t / denom;
    double y = 10.0 * sin_t * cos_t / denom;
    double z = 10.0;
    
    return Eigen::Vector3d(x, y, z);
}

Eigen::Vector3d computeVelocity(double t) {
    double sin_t = std::sin(t);
    double cos_t = std::cos(t);
    double sin2_t = sin_t * sin_t;
    double cos2_t = cos_t * cos_t;
    double denom = 1.0 + sin2_t;
    double denom2 = denom * denom;
    
    // x_dot = -10*sin(t)/(sin(t)^2 + 1) - 20*sin(t)*cos(t)^2/(sin(t)^2 + 1)^2
    double x_dot = -10.0 * sin_t / denom - 20.0 * sin_t * cos2_t / denom2;
    
    // y_dot = -10*sin(t)^2/(sin(t)^2 + 1) + 10*cos(t)^2/(sin(t)^2 + 1) - 20*sin(t)^2*cos(t)^2/(sin(t)^2 + 1)^2
    double y_dot = -10.0 * sin2_t / denom + 10.0 * cos2_t / denom - 20.0 * sin2_t * cos2_t / denom2;
    
    double z_dot = 0.0;
    
    return Eigen::Vector3d(x_dot, y_dot, z_dot);
}

Eigen::Vector3d computeAcceleration(double t) {
    double sin_t = std::sin(t);
    double cos_t = std::cos(t);
    double sin2_t = sin_t * sin_t;
    double cos2_t = cos_t * cos_t;
    double sin3_t = sin2_t * sin_t;
    double cos3_t = cos2_t * cos_t;
    double denom = 1.0 + sin2_t;
    double denom2 = denom * denom;
    double denom3 = denom2 * denom;
    
    // x_ddot = -10*cos(t)/(sin(t)^2 + 1) + 60*sin(t)^2*cos(t)/(sin(t)^2 + 1)^2 
    //          - 20*cos(t)^3/(sin(t)^2 + 1)^2 + 80*sin(t)^2*cos(t)^3/(sin(t)^2 + 1)^3
    double x_ddot = -10.0 * cos_t / denom 
                    + 60.0 * sin2_t * cos_t / denom2
                    - 20.0 * cos3_t / denom2
                    + 80.0 * sin2_t * cos3_t / denom3;
    
    // y_ddot = -40*sin(t)*cos(t)/(sin(t)^2 + 1) + 60*sin(t)^3*cos(t)/(sin(t)^2 + 1)^2 
    //          - 60*sin(t)*cos(t)^3/(sin(t)^2 + 1)^2 + 80*sin(t)^3*cos(t)^3/(sin(t)^2 + 1)^3
    double y_ddot = -40.0 * sin_t * cos_t / denom
                    + 60.0 * sin3_t * cos_t / denom2
                    - 60.0 * sin_t * cos3_t / denom2
                    + 80.0 * sin3_t * cos3_t / denom3;
    
    double z_ddot = 0.0;
    
    return Eigen::Vector3d(x_ddot, y_ddot, z_ddot);
}

double computeYaw(const Eigen::Vector3d& vel) {
    return std::atan2(vel.y(), vel.x());
}

Eigen::Matrix3d computeRotationMatrix(
    const Eigen::Vector3d& pos,
    const Eigen::Vector3d& vel,
    const Eigen::Vector3d& acc,
    double yaw,
    double g) {
    
    // 前向轴（x轴，Front）：与速度方向对齐
    Eigen::Vector3d e_x;
    double vel_norm = vel.norm();
    if (vel_norm > 1e-6) {
        e_x = vel / vel_norm;
    } else {
        // 如果速度为零，使用偏航角计算
        e_x = Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0.0);
    }
    
    // 期望加速度（包含重力补偿）
    Eigen::Vector3d e_z_world(0.0, 0.0, 1.0);
    Eigen::Vector3d a_des = acc + g * e_z_world;
    
    // 上向轴（z轴，Up）：指向期望加速度方向
    Eigen::Vector3d e_z;
    double a_des_norm = a_des.norm();
    if (a_des_norm > 1e-6) {
        e_z = a_des / a_des_norm;
    } else {
        // 如果期望加速度为零，使用重力方向
        e_z = e_z_world;
    }
    
    // 左侧轴（y轴，Left）：根据右手坐标系 FLU
    // 在FLU坐标系中：e_y = e_z × e_x（右手坐标系）
    Eigen::Vector3d e_y = e_z.cross(e_x);
    double e_y_norm = e_y.norm();
    if (e_y_norm > 1e-6) {
        e_y = e_y / e_y_norm;
    } else {
        // 如果叉积为零，使用偏航角计算
        e_y = Eigen::Vector3d(-std::sin(yaw), std::cos(yaw), 0.0);
        e_y.normalize();
    }
    
    // 重新正交化：确保三个轴相互正交（右手坐标系）
    // 重新计算e_x，使其与e_y和e_z正交
    e_x = e_y.cross(e_z);
    e_x.normalize();
    // 重新计算e_y，确保右手坐标系
    e_y = e_z.cross(e_x);
    e_y.normalize();
    
    // 构造旋转矩阵：从世界坐标系到机体坐标系
    // 列向量形式：[e_x, e_y, e_z] 表示机体系各轴在世界系中的方向
    Eigen::Matrix3d R_world_to_body;
    R_world_to_body.col(0) = e_x;
    R_world_to_body.col(1) = e_y;
    R_world_to_body.col(2) = e_z;
    
    // 转置得到从世界系到机体系的旋转矩阵
    return R_world_to_body.transpose();
}

Eigen::Vector4d rotationMatrixToQuaternion(const Eigen::Matrix3d& R) {
    Eigen::Vector4d q;
    double tr = R(0, 0) + R(1, 1) + R(2, 2);
    
    if (tr > 0) {
        double S = std::sqrt(tr + 1.0) * 2.0;
        q(3) = 0.25 * S;  // w
        q(0) = (R(2, 1) - R(1, 2)) / S;  // x
        q(1) = (R(0, 2) - R(2, 0)) / S;  // y
        q(2) = (R(1, 0) - R(0, 1)) / S;  // z
    } else if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2)) {
        double S = std::sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0;
        q(3) = (R(2, 1) - R(1, 2)) / S;  // w
        q(0) = 0.25 * S;  // x
        q(1) = (R(0, 1) + R(1, 0)) / S;  // y
        q(2) = (R(0, 2) + R(2, 0)) / S;  // z
    } else if (R(1, 1) > R(2, 2)) {
        double S = std::sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0;
        q(3) = (R(0, 2) - R(2, 0)) / S;  // w
        q(0) = (R(0, 1) + R(1, 0)) / S;  // x
        q(1) = 0.25 * S;  // y
        q(2) = (R(1, 2) + R(2, 1)) / S;  // z
    } else {
        double S = std::sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0;
        q(3) = (R(1, 0) - R(0, 1)) / S;  // w
        q(0) = (R(0, 2) + R(2, 0)) / S;  // x
        q(1) = (R(1, 2) + R(2, 1)) / S;  // y
        q(2) = 0.25 * S;  // z
    }
    
    return q;
}

Eigen::Vector4d normalizeQuaternion(const Eigen::Vector4d& q) {
    Eigen::Vector4d q_normalized = q;
    double norm = q.norm();
    
    if (norm > 1e-10) {
        q_normalized = q / norm;
    }
    
    // 确保 w >= 0
    if (q_normalized(3) < 0) {
        q_normalized = -q_normalized;
    }
    
    return q_normalized;
}

} // namespace quadrotor_df

