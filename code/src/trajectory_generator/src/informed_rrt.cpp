#include "informed_rrt.h"
#include <algorithm>
#include <limits>

InformedRRTStar::InformedRRTStar() 
    : max_iterations_(5000),
      step_size_(0.5),
      goal_bias_(0.1),
      search_radius_(1.5),
      goal_tolerance_(0.5) {
    std::random_device rd;
    rng_ = std::mt19937(rd());
}

void InformedRRTStar::initMap(double resolution, Eigen::Vector3d map_lower, Eigen::Vector3d map_upper) {
    resolution_ = resolution;
    map_lower_ = map_lower;
    map_upper_ = map_upper;
}

void InformedRRTStar::setObstacleChecker(std::function<bool(Eigen::Vector3d)> checker) {
    isOccupied_ = checker;
}

Eigen::Vector3d InformedRRTStar::sampleFree() {
    std::uniform_real_distribution<double> dist_x(map_lower_(0), map_upper_(0));
    std::uniform_real_distribution<double> dist_y(map_lower_(1), map_upper_(1));
    std::uniform_real_distribution<double> dist_z(map_lower_(2), map_upper_(2));
    
    return Eigen::Vector3d(dist_x(rng_), dist_y(rng_), dist_z(rng_));
}

Eigen::Vector3d InformedRRTStar::sampleInEllipsoid(Eigen::Vector3d start, Eigen::Vector3d goal, double c_best) {
    // Informed RRT* 核心：在椭球内采样
    double c_min = (goal - start).norm();
    
    if (c_best >= std::numeric_limits<double>::max() || c_best <= c_min) {
        return sampleFree();
    }
    
    // 椭球参数
    double r1 = c_best / 2.0;  // 长轴
    double r2 = sqrt(c_best * c_best - c_min * c_min) / 2.0;  // 短轴
    
    // 椭球中心
    Eigen::Vector3d center = (start + goal) / 2.0;
    
    // 旋转矩阵：将椭球长轴对准 start->goal 方向
    Eigen::Vector3d a1 = (goal - start).normalized();
    Eigen::Vector3d a2, a3;
    
    // 构造正交基
    if (fabs(a1(0)) < 0.9) {
        a2 = Eigen::Vector3d(1, 0, 0).cross(a1).normalized();
    } else {
        a2 = Eigen::Vector3d(0, 1, 0).cross(a1).normalized();
    }
    a3 = a1.cross(a2).normalized();
    
    Eigen::Matrix3d C;
    C.col(0) = a1;
    C.col(1) = a2;
    C.col(2) = a3;
    
    // 在单位球内均匀采样
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    Eigen::Vector3d x_ball;
    do {
        x_ball = Eigen::Vector3d(dist(rng_), dist(rng_), dist(rng_));
    } while (x_ball.norm() > 1.0);
    
    // 变换到椭球
    Eigen::Vector3d L(r1, r2, r2);
    Eigen::Vector3d x_ellipse = C * L.asDiagonal() * x_ball + center;
    
    // 边界检查
    x_ellipse(0) = std::max(map_lower_(0), std::min(map_upper_(0), x_ellipse(0)));
    x_ellipse(1) = std::max(map_lower_(1), std::min(map_upper_(1), x_ellipse(1)));
    x_ellipse(2) = std::max(map_lower_(2), std::min(map_upper_(2), x_ellipse(2)));
    
    return x_ellipse;
}

InformedRRTStar::NodePtr InformedRRTStar::findNearest(Eigen::Vector3d point) {
    NodePtr nearest = nullptr;
    double min_dist = std::numeric_limits<double>::max();
    
    for (auto& node : tree_) {
        double dist = (node->pos - point).norm();
        if (dist < min_dist) {
            min_dist = dist;
            nearest = node;
        }
    }
    return nearest;
}

std::vector<InformedRRTStar::NodePtr> InformedRRTStar::findNear(Eigen::Vector3d point, double radius) {
    std::vector<NodePtr> near_nodes;
    for (auto& node : tree_) {
        if ((node->pos - point).norm() < radius) {
            near_nodes.push_back(node);
        }
    }
    return near_nodes;
}

Eigen::Vector3d InformedRRTStar::steer(Eigen::Vector3d from, Eigen::Vector3d to) {
    Eigen::Vector3d direction = to - from;
    double dist = direction.norm();
    
    if (dist <= step_size_) {
        return to;
    }
    return from + direction.normalized() * step_size_;
}

bool InformedRRTStar::isCollisionFree(Eigen::Vector3d from, Eigen::Vector3d to) {
    if (!isOccupied_) return true;
    
    Eigen::Vector3d direction = to - from;
    double dist = direction.norm();
    int n_checks = std::max(2, (int)(dist / (resolution_ * 0.5)));
    
    for (int i = 0; i <= n_checks; i++) {
        double t = (double)i / n_checks;
        Eigen::Vector3d point = from + t * direction;
        if (isOccupied_(point)) {
            return false;
        }
    }
    return true;
}

double InformedRRTStar::lineCost(Eigen::Vector3d from, Eigen::Vector3d to) {
    return (to - from).norm();
}

double InformedRRTStar::cost(NodePtr node) {
    return node->cost;
}

void InformedRRTStar::rewire(NodePtr new_node, std::vector<NodePtr>& near_nodes) {
    for (auto& near : near_nodes) {
        if (near == new_node->parent) continue;
        
        double new_cost = new_node->cost + lineCost(new_node->pos, near->pos);
        if (new_cost < near->cost && isCollisionFree(new_node->pos, near->pos)) {
            near->parent = new_node;
            near->cost = new_cost;
        }
    }
}

std::vector<Eigen::Vector3d> InformedRRTStar::extractPath(NodePtr goal_node) {
    std::vector<Eigen::Vector3d> path;
    NodePtr current = goal_node;
    
    while (current != nullptr) {
        path.push_back(current->pos);
        current = current->parent;
    }
    
    std::reverse(path.begin(), path.end());
    return path;
}

bool InformedRRTStar::plan(Eigen::Vector3d start, Eigen::Vector3d goal, std::vector<Eigen::Vector3d>& path) {
    ROS_INFO("[Informed RRT*] Planning from [%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f]",
             start(0), start(1), start(2), goal(0), goal(1), goal(2));
    
    // 清空树
    tree_.clear();
    
    // 添加起点
    NodePtr start_node = std::make_shared<Node>(start);
    start_node->cost = 0;
    tree_.push_back(start_node);
    
    // 最佳路径代价
    double c_best = std::numeric_limits<double>::max();
    NodePtr goal_node = nullptr;
    
    std::uniform_real_distribution<double> dist_01(0.0, 1.0);
    
    for (int iter = 0; iter < max_iterations_; iter++) {
        // 采样
        Eigen::Vector3d x_rand;
        if (dist_01(rng_) < goal_bias_) {
            x_rand = goal;
        } else {
            x_rand = sampleInEllipsoid(start, goal, c_best);
        }
        
        // 找最近节点
        NodePtr x_nearest = findNearest(x_rand);
        if (!x_nearest) continue;
        
        // 扩展
        Eigen::Vector3d x_new_pos = steer(x_nearest->pos, x_rand);
        
        // 碰撞检测
        if (!isCollisionFree(x_nearest->pos, x_new_pos)) continue;
        
        // 创建新节点
        NodePtr x_new = std::make_shared<Node>(x_new_pos);
        
        // 找邻近节点
        double r = std::min(search_radius_, step_size_ * 2);
        std::vector<NodePtr> near_nodes = findNear(x_new_pos, r);
        
        // 选择最优父节点
        NodePtr best_parent = x_nearest;
        double best_cost = x_nearest->cost + lineCost(x_nearest->pos, x_new_pos);
        
        for (auto& near : near_nodes) {
            double new_cost = near->cost + lineCost(near->pos, x_new_pos);
            if (new_cost < best_cost && isCollisionFree(near->pos, x_new_pos)) {
                best_parent = near;
                best_cost = new_cost;
            }
        }
        
        x_new->parent = best_parent;
        x_new->cost = best_cost;
        tree_.push_back(x_new);
        
        // 重新布线
        rewire(x_new, near_nodes);
        
        // 检查是否到达目标
        if ((x_new_pos - goal).norm() < goal_tolerance_) {
            if (isCollisionFree(x_new_pos, goal)) {
                double goal_cost = x_new->cost + lineCost(x_new_pos, goal);
                if (goal_cost < c_best) {
                    c_best = goal_cost;
                    goal_node = std::make_shared<Node>(goal);
                    goal_node->parent = x_new;
                    goal_node->cost = goal_cost;
                    ROS_INFO("[Informed RRT*] Found path! Cost: %.2f, Iteration: %d", c_best, iter);
                }
            }
        }
        
        // 进度输出
        if (iter % 1000 == 0) {
            ROS_INFO("[Informed RRT*] Iteration %d, Tree size: %zu, Best cost: %.2f",
                     iter, tree_.size(), c_best);
        }
    }
    
    if (goal_node) {
        path = extractPath(goal_node);
        ROS_INFO("[Informed RRT*] Path found with %zu waypoints, cost: %.2f",
                 path.size(), c_best);
        return true;
    }
    
    ROS_WARN("[Informed RRT*] Failed to find path after %d iterations", max_iterations_);
    return false;
}

