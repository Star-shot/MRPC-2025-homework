#include <ros/ros.h>
#include <quadrotor_df/differential_flatness.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <cstdlib>

int main(int argc, char** argv) {
    ros::init(argc, argv, "compute_quaternion");
    
    // 参数设置
    const double t_start = 0.0;
    const double t_end = 2.0 * M_PI;
    const double dt = 0.02;
    const double g = 9.81;
    
    // 输出文件路径
    std::string output_file;
    if (argc > 1) {
        output_file = argv[1];
    } else {
        // 默认输出到当前目录或solutions目录
        const char* home = std::getenv("HOME");
        if (home) {
            output_file = std::string(home) + "/MRPC-2025-homework/solutions/df_quaternion.csv";
        } else {
            output_file = "df_quaternion.csv";
        }
    }
    
    // 打开输出文件
    std::ofstream outfile(output_file);
    if (!outfile.is_open()) {
        std::cerr << "Error: Cannot open output file " << output_file << std::endl;
        return -1;
    }
    
    // 写入CSV头部
    outfile << "t, x, y, z, w" << std::endl;
    
    // 计算每个时间点的四元数
    int count = 0;
    for (double t = t_start; t < t_end; t += dt) {
        // 计算位置、速度、加速度
        Eigen::Vector3d pos = quadrotor_df::computePosition(t);
        Eigen::Vector3d vel = quadrotor_df::computeVelocity(t);
        Eigen::Vector3d acc = quadrotor_df::computeAcceleration(t);
        
        // 计算偏航角
        double yaw = quadrotor_df::computeYaw(vel);
        
        // 计算旋转矩阵
        Eigen::Matrix3d R = quadrotor_df::computeRotationMatrix(pos, vel, acc, yaw, g);
        
        // 转换为四元数
        Eigen::Vector4d q = quadrotor_df::rotationMatrixToQuaternion(R);
        
        // 归一化并确保 w >= 0
        q = quadrotor_df::normalizeQuaternion(q);
        
        // 写入CSV文件
        // 格式：t, x, y, z, w
        // 时间保留2位小数，四元数保留7位小数
        outfile << std::fixed << std::setprecision(2) << t << ", "
                << std::setprecision(7) << q(0) << ", "
                << std::setprecision(7) << q(1) << ", "
                << std::setprecision(7) << q(2) << ", "
                << std::setprecision(7) << q(3) << std::endl;
        
        count++;
        
        // 每100个点输出一次进度
        if (count % 100 == 0) {
            ROS_INFO("Processed %d points, t = %.2f", count, t);
        }
    }
    
    outfile.close();
    
    ROS_INFO("Computation completed! Total points: %d", count);
    ROS_INFO("Output file: %s", output_file.c_str());
    
    return 0;
}

