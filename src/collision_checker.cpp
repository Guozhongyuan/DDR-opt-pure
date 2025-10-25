#include "ddr_optimizer/collision_interface.h"
#include <cmath>
#include <limits>
#include <algorithm>
#include <queue>

namespace ddr_optimizer {

// ========== SimpleCircleObstacles实现 ==========

SimpleCircleObstacles::SimpleCircleObstacles(
    const Eigen::Vector2d& bounds_min,
    const Eigen::Vector2d& bounds_max)
    : bounds_min_(bounds_min), bounds_max_(bounds_max) {
}

void SimpleCircleObstacles::addObstacle(double x, double y, double radius) {
    obstacles_.emplace_back(x, y, radius);
}

void SimpleCircleObstacles::clearObstacles() {
    obstacles_.clear();
}

double SimpleCircleObstacles::getDistanceWithGradient(
    const Eigen::Vector2d& position,
    Eigen::Vector2d& gradient,
    double safe_distance) const {
    
    if (obstacles_.empty()) {
        gradient.setZero();
        return 1000.0;  // 无障碍物时返回大距离值
    }
    
    double min_distance = std::numeric_limits<double>::max();
    Eigen::Vector2d min_gradient = Eigen::Vector2d::Zero();
    
    // 遍历所有障碍物，找到最近的
    for (const auto& obs : obstacles_) {
        Eigen::Vector2d obs_center(obs.x(), obs.y());
        double obs_radius = obs.z();
        
        Eigen::Vector2d diff = position - obs_center;
        double dist_to_center = diff.norm();
        
        if (dist_to_center < 1e-6) {
            // 避免除零
            diff = Eigen::Vector2d(1e-6, 0);
            dist_to_center = 1e-6;
        }
        
        // 距离 = 到中心的距离 - 半径
        double distance = dist_to_center - obs_radius;
        
        if (distance < min_distance) {
            min_distance = distance;
            // 梯度指向远离障碍物的方向
            min_gradient = diff / dist_to_center;
        }
    }
    
    gradient = min_gradient;
    return min_distance - safe_distance;
}

double SimpleCircleObstacles::getDistance(const Eigen::Vector2d& position) const {
    Eigen::Vector2d dummy_grad;
    return getDistanceWithGradient(position, dummy_grad, 0.0);
}

bool SimpleCircleObstacles::isInBounds(const Eigen::Vector2d& position) const {
    return position.x() >= bounds_min_.x() && position.x() <= bounds_max_.x() &&
           position.y() >= bounds_min_.y() && position.y() <= bounds_max_.y();
}

// ========== GridMapCollisionChecker实现 ==========

GridMapCollisionChecker::GridMapCollisionChecker(
    const Eigen::Vector2d& origin,
    double resolution,
    int size_x,
    int size_y)
    : origin_(origin),
      resolution_(resolution),
      inv_resolution_(1.0 / resolution),
      size_x_(size_x),
      size_y_(size_y) {
    
    sdf_data_.resize(size_x * size_y, 1000.0);  // 初始化为大距离值
}

void GridMapCollisionChecker::setGridValue(int i, int j, double distance) {
    if (i >= 0 && i < size_x_ && j >= 0 && j < size_y_) {
        sdf_data_[i * size_y_ + j] = distance;
    }
}

void GridMapCollisionChecker::computeSDF(const std::vector<std::vector<int>>& occupancy_grid) {
    // 简单的Brushfire算法计算SDF
    const double max_dist = 1000.0;
    const int dx[] = {-1, 0, 1, 0, -1, 1, 1, -1};
    const int dy[] = {0, 1, 0, -1, 1, 1, -1, -1};
    
    // 初始化
    std::fill(sdf_data_.begin(), sdf_data_.end(), max_dist);
    std::queue<std::pair<int, int>> queue;
    
    // 找到所有障碍物格子
    for (int i = 0; i < size_x_; ++i) {
        for (int j = 0; j < size_y_; ++j) {
            if (i < static_cast<int>(occupancy_grid.size()) && 
                j < static_cast<int>(occupancy_grid[i].size()) &&
                occupancy_grid[i][j] > 0) {
                sdf_data_[i * size_y_ + j] = 0.0;
                queue.push({i, j});
            }
        }
    }
    
    // BFS传播距离
    while (!queue.empty()) {
        int x = queue.front().first;
        int y = queue.front().second;
        queue.pop();
        
        double curr_dist = sdf_data_[x * size_y_ + y];
        
        for (int k = 0; k < 8; ++k) {
            int nx = x + dx[k];
            int ny = y + dy[k];
            
            if (nx >= 0 && nx < size_x_ && ny >= 0 && ny < size_y_) {
                double edge_dist = (k < 4) ? resolution_ : resolution_ * 1.414;
                double new_dist = curr_dist + edge_dist;
                
                if (new_dist < sdf_data_[nx * size_y_ + ny]) {
                    sdf_data_[nx * size_y_ + ny] = new_dist;
                    queue.push({nx, ny});
                }
            }
        }
    }
}

double GridMapCollisionChecker::bilinearInterpolate(const Eigen::Vector2d& position) const {
    // 转换到网格坐标
    double fx = (position.x() - origin_.x()) * inv_resolution_;
    double fy = (position.y() - origin_.y()) * inv_resolution_;
    
    int x0 = static_cast<int>(std::floor(fx));
    int y0 = static_cast<int>(std::floor(fy));
    int x1 = x0 + 1;
    int y1 = y0 + 1;
    
    // 检查边界
    if (x0 < 0 || x1 >= size_x_ || y0 < 0 || y1 >= size_y_) {
        return -1.0;  // 越界返回负值（表示碰撞）
    }
    
    double dx = fx - x0;
    double dy = fy - y0;
    
    // 双线性插值
    double v00 = sdf_data_[x0 * size_y_ + y0];
    double v10 = sdf_data_[x1 * size_y_ + y0];
    double v01 = sdf_data_[x0 * size_y_ + y1];
    double v11 = sdf_data_[x1 * size_y_ + y1];
    
    double v0 = v00 * (1 - dx) + v10 * dx;
    double v1 = v01 * (1 - dx) + v11 * dx;
    
    return v0 * (1 - dy) + v1 * dy;
}

Eigen::Vector2d GridMapCollisionChecker::computeGradient(const Eigen::Vector2d& position) const {
    // 使用中心差分计算梯度
    const double h = resolution_ * 0.5;
    
    Eigen::Vector2d grad;
    grad.x() = (bilinearInterpolate(position + Eigen::Vector2d(h, 0)) -
                bilinearInterpolate(position - Eigen::Vector2d(h, 0))) / (2 * h);
    grad.y() = (bilinearInterpolate(position + Eigen::Vector2d(0, h)) -
                bilinearInterpolate(position - Eigen::Vector2d(0, h))) / (2 * h);
    
    return grad;
}

double GridMapCollisionChecker::getDistanceWithGradient(
    const Eigen::Vector2d& position,
    Eigen::Vector2d& gradient,
    double safe_distance) const {
    
    gradient = computeGradient(position);
    return bilinearInterpolate(position) - safe_distance;
}

double GridMapCollisionChecker::getDistance(const Eigen::Vector2d& position) const {
    return bilinearInterpolate(position);
}

bool GridMapCollisionChecker::isInBounds(const Eigen::Vector2d& position) const {
    double fx = (position.x() - origin_.x()) * inv_resolution_;
    double fy = (position.y() - origin_.y()) * inv_resolution_;
    
    return fx >= 0 && fx < size_x_ && fy >= 0 && fy < size_y_;
}

} // namespace ddr_optimizer

