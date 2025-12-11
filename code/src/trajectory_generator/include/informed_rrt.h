#ifndef _INFORMED_RRT_H_
#define _INFORMED_RRT_H_

#include <Eigen/Eigen>
#include <vector>
#include <random>
#include <memory>
#include <cmath>
#include <ros/ros.h>

class InformedRRTStar {
public:
    struct Node {
        Eigen::Vector3d pos;
        std::shared_ptr<Node> parent;
        double cost;
        
        Node(Eigen::Vector3d p) : pos(p), parent(nullptr), cost(0.0) {}
    };
    
    using NodePtr = std::shared_ptr<Node>;
    
    InformedRRTStar();
    ~InformedRRTStar() {}
    
    // 初始化地图参数
    void initMap(double resolution, Eigen::Vector3d map_lower, Eigen::Vector3d map_upper);
    
    // 设置障碍物检查函数
    void setObstacleChecker(std::function<bool(Eigen::Vector3d)> checker);
    
    // 规划路径
    bool plan(Eigen::Vector3d start, Eigen::Vector3d goal, std::vector<Eigen::Vector3d>& path);
    
    // 参数设置
    void setMaxIterations(int iter) { max_iterations_ = iter; }
    void setStepSize(double step) { step_size_ = step; }
    void setGoalBias(double bias) { goal_bias_ = bias; }
    void setSearchRadius(double radius) { search_radius_ = radius; }
    
private:
    // 在椭球内采样（Informed RRT* 核心）
    Eigen::Vector3d sampleInEllipsoid(Eigen::Vector3d start, Eigen::Vector3d goal, double c_best);
    
    // 普通随机采样
    Eigen::Vector3d sampleFree();
    
    // 找最近节点
    NodePtr findNearest(Eigen::Vector3d point);
    
    // 找邻近节点
    std::vector<NodePtr> findNear(Eigen::Vector3d point, double radius);
    
    // 扩展节点
    Eigen::Vector3d steer(Eigen::Vector3d from, Eigen::Vector3d to);
    
    // 碰撞检测
    bool isCollisionFree(Eigen::Vector3d from, Eigen::Vector3d to);
    
    // 计算代价
    double cost(NodePtr node);
    double lineCost(Eigen::Vector3d from, Eigen::Vector3d to);
    
    // 重新布线
    void rewire(NodePtr new_node, std::vector<NodePtr>& near_nodes);
    
    // 提取路径
    std::vector<Eigen::Vector3d> extractPath(NodePtr goal_node);
    
    // 参数
    double resolution_;
    Eigen::Vector3d map_lower_, map_upper_;
    int max_iterations_;
    double step_size_;
    double goal_bias_;
    double search_radius_;
    double goal_tolerance_;
    
    // 树
    std::vector<NodePtr> tree_;
    
    // 随机数生成器
    std::mt19937 rng_;
    
    // 障碍物检查函数
    std::function<bool(Eigen::Vector3d)> isOccupied_;
};

#endif

