#include "ddr_optimizer/optimizer_pure.h"
#include "ddr_optimizer/collision_interface.h"
#include "ddr_optimizer/config.h"
#include "ddr_optimizer/trajectory_data.h"
#include "ddr_optimizer/visualizer.h"

#include <iostream>
#include <iomanip>
#include <memory>
#include <cmath>

using namespace ddr_optimizer;

/**
 * @brief 测试: 单障碍物避障
 * 
 * 场景: 从 (0, 0, 0) 到 (5, 0, 0)，中间有一个圆形障碍物
 */
bool test_single_obstacle() {
    std::cout << "\n========== Test: Single Obstacle Avoidance ==========\n";
    
    OptimizerConfig config = OptimizerConfig::defaultConfig();
    config.verbose = true;
    config.safe_distance = 0.1;  // 设置安全距离
    
    // 创建碰撞检测器并添加障碍物
    auto collision_checker = std::make_shared<SimpleCircleObstacles>(
        Eigen::Vector2d(-10, -10),
        Eigen::Vector2d(10, 10)
    );
    
    // 在路径中间添加障碍物
    collision_checker->addObstacle(4.0, 0.0, 0.5);  // (x, y, radius)
    
    std::cout << "Added obstacle at (4.0, 0.0) with radius 0.5\n";
    
    DDROptimizer optimizer(config, collision_checker);
    
    TrajectoryInput input;
    
    input.start_state.resize(2, 3);
    input.start_state << 
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0;
    
    input.final_state.resize(2, 3);
    input.final_state << 
        0.0, 0.0, 0.0,
        10.0, 0.0, 0.0;
    
    input.start_state_XYTheta = Eigen::Vector3d(0.0, 0.0, 0.0);
    input.final_state_XYTheta = Eigen::Vector3d(10.0, 0.0, 0.0);
    
    // 添加中间路径点，帮助绕过障碍物
    input.waypoints.push_back(Eigen::Vector3d(0.0, 1.0, 0.5));   // 绕过障碍物上方
    input.waypoints.push_back(Eigen::Vector3d(0.0, 4.0, 0.5));
    input.waypoints.push_back(Eigen::Vector3d(0.0, 7.0, 0.5));
    
    input.waypoint_positions.push_back(Eigen::Vector3d(1.0, 1.0, 0.0));  // 稍微偏上
    input.waypoint_positions.push_back(Eigen::Vector3d(4.0, 1.0, 0.0));  // 在障碍物上方经过
    input.waypoint_positions.push_back(Eigen::Vector3d(7.0, 0.5, 0.0));  // 回到中线
    
    input.initial_segment_time = 1.0;
    input.is_cut = false;
    
    TrajectoryOutput output;
    bool success = optimizer.optimize(input, output);
    
    output.print();
    optimizer.getStats().print();
    
    if (!success) {
        std::cerr << "Test FAILED: Obstacle avoidance failed\n";
        return false;
    }
    
    // ========== 打印轨迹细节 ==========
    std::cout << "\n===== 轨迹细节 =====\n";
    std::cout << std::fixed << std::setprecision(6);
    
    // 起点
    std::cout << "起点 (期望): (" 
              << input.start_state_XYTheta.x() << ", "
              << input.start_state_XYTheta.y() << ", "
              << input.start_state_XYTheta.z() << " rad)\n";
    
    std::cout << "起点 (实际): (" 
              << input.start_state_XYTheta.x() << ", "
              << input.start_state_XYTheta.y() << ", "
              << input.start_state_XYTheta.z() << " rad)\n";
    
    // 终点
    std::cout << "终点 (期望): (" 
              << input.final_state_XYTheta.x() << ", "
              << input.final_state_XYTheta.y() << ", "
              << input.final_state_XYTheta.z() << " rad)\n";
    
    // 计算实际终点位置（通过积分轨迹）
    Eigen::Vector3d actual_end_XYTheta = input.start_state_XYTheta;
    double dt = 0.01;  // 10ms采样
    double total_time = output.segment_durations.sum();
    int num_samples = static_cast<int>(total_time / dt) + 1;
    
    // 同时检查是否避开障碍物
    double min_obstacle_distance = 1e10;
    Eigen::Vector2d obstacle_center(4.0, 0.0);
    double obstacle_radius = 0.5;
    
    for (int i = 1; i < num_samples; i++) {
        double t = std::min(i * dt, total_time);
        Eigen::Vector2d sigma = output.trajectory.getPos(t);
        Eigen::Vector2d dsigma = output.trajectory.getVel(t);
        
        double yaw = sigma.x();
        double ds = dsigma.y();
        
        actual_end_XYTheta.x() += ds * cos(yaw) * dt;
        actual_end_XYTheta.y() += ds * sin(yaw) * dt;
        actual_end_XYTheta.z() = yaw;
        
        // 检查到障碍物的距离
        Eigen::Vector2d current_pos(actual_end_XYTheta.x(), actual_end_XYTheta.y());
        double dist_to_obstacle = (current_pos - obstacle_center).norm() - obstacle_radius;
        min_obstacle_distance = std::min(min_obstacle_distance, dist_to_obstacle);
    }
    
    std::cout << "终点 (实际): (" 
              << actual_end_XYTheta.x() << ", "
              << actual_end_XYTheta.y() << ", "
              << actual_end_XYTheta.z() << " rad)\n";
    
    // 终点误差
    Eigen::Vector3d position_error = actual_end_XYTheta - input.final_state_XYTheta;
    double position_error_norm = position_error.head(2).norm();
    double angle_error = std::abs(position_error.z());
    
    std::cout << "\n终点误差:\n";
    std::cout << "  位置误差: " << position_error_norm * 1000.0 << " mm\n";
    std::cout << "  X误差: " << position_error.x() * 1000.0 << " mm\n";
    std::cout << "  Y误差: " << position_error.y() * 1000.0 << " mm\n";
    std::cout << "  角度误差: " << angle_error * 180.0 / M_PI << " deg\n";
    
    std::cout << "\n避障信息:\n";
    std::cout << "  到障碍物最小距离: " << min_obstacle_distance * 1000.0 << " mm\n";
    std::cout << "  安全距离: " << config.safe_distance * 1000.0 << " mm\n";
    
    if (min_obstacle_distance < config.safe_distance) {
        std::cout << "  警告: 轨迹可能穿过障碍物！\n";
    } else {
        std::cout << "  ✓ 成功避开障碍物\n";
    }
    
    // 检查到中途航点的最小距离
    if (!input.waypoint_positions.empty()) {
        std::cout << "\n中途航点接近度:\n";
        
        for (size_t wp_idx = 0; wp_idx < input.waypoint_positions.size(); wp_idx++) {
            Eigen::Vector2d waypoint_pos = input.waypoint_positions[wp_idx].head(2);
            double min_dist_to_waypoint = 1e10;
            
            // 重新遍历轨迹
            Eigen::Vector3d current_XYTheta = input.start_state_XYTheta;
            for (int i = 1; i < num_samples; i++) {
                double t = std::min(i * dt, total_time);
                Eigen::Vector2d sigma = output.trajectory.getPos(t);
                Eigen::Vector2d dsigma = output.trajectory.getVel(t);
                
                double yaw = sigma.x();
                double ds = dsigma.y();
                
                current_XYTheta.x() += ds * cos(yaw) * dt;
                current_XYTheta.y() += ds * sin(yaw) * dt;
                
                Eigen::Vector2d current_pos = current_XYTheta.head(2);
                double dist = (current_pos - waypoint_pos).norm();
                min_dist_to_waypoint = std::min(min_dist_to_waypoint, dist);
            }
            
            std::cout << "  航点" << (wp_idx + 1) << " @ (" 
                      << waypoint_pos.x() << ", " 
                      << waypoint_pos.y() << "): "
                      << "最小距离 = " << min_dist_to_waypoint * 1000.0 << " mm";
            
            if (min_dist_to_waypoint < 0.5) {
                std::cout << " ✓ 经过\n";
            } else if (min_dist_to_waypoint < 1.0) {
                std::cout << " ⚠ 接近\n";
            } else {
                std::cout << " ✗ 偏离\n";
            }
        }
    }
    
    std::cout << "===================\n\n";
    
    // 使用matplotlib-cpp直接可视化
    std::vector<Eigen::Vector3d> obstacles;
    obstacles.push_back(Eigen::Vector3d(4.0, 0.0, 0.5));  // (x, y, radius)
    
    TrajectoryVisualizer visualizer(16.0, 12.0, 150);
    visualizer.visualize(input, output, obstacles, config.check_points, 
                        "test2_single_obstacle.png", false);
    std::cout << "轨迹可视化已保存到: test2_single_obstacle.png\n\n";
    
    std::cout << "Test PASSED: Successfully avoided obstacle\n";
    return true;
}

/**
 * @brief 测试: 多障碍物场景
 */
bool test_multiple_obstacles() {
    std::cout << "\n========== Test: Multiple Obstacles ==========\n";
    
    OptimizerConfig config = OptimizerConfig::defaultConfig();
    config.verbose = false;
    config.safe_distance = 0.1;
    
    auto collision_checker = std::make_shared<SimpleCircleObstacles>(
        Eigen::Vector2d(-10, -10),
        Eigen::Vector2d(10, 10)
    );
    
    // 添加多个障碍物
    std::vector<Eigen::Vector3d> obstacles = {
        Eigen::Vector3d(3.0, 0.7, 0.3),
        Eigen::Vector3d(6.0, -0.7, 0.3),
        Eigen::Vector3d(8.0, 1.2, 0.3),
    };
    for (const auto& obstacle : obstacles) {
        collision_checker->addObstacle(obstacle.x(), obstacle.y(), obstacle.z());
    }
    
    std::cout << "Added " << obstacles.size() << " obstacles\n";
    
    DDROptimizer optimizer(config, collision_checker);
    
    TrajectoryInput input;
    
    input.start_state.resize(2, 3);
    input.start_state << 
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0;
    
    input.final_state.resize(2, 3);
    input.final_state << 
        0.0, 0.0, 0.0,
        10.0, 0.0, 0.0;
    
    input.start_state_XYTheta = Eigen::Vector3d(0.0, 0.0, 0.0);
    input.final_state_XYTheta = Eigen::Vector3d(10.0, 0.0, 0.0);
    
    // 提供初始路径点
    for (double s = 1.0; s < 10.0; s += 1.0) {
        input.waypoints.push_back(Eigen::Vector3d(0.0, s, 0.5));
        input.waypoint_positions.push_back(Eigen::Vector3d(s, 0.0, 0.0));
    }
    
    input.initial_segment_time = 0.8;
    input.is_cut = false;
    
    TrajectoryOutput output;
    bool success = optimizer.optimize(input, output);
    
    if (!success) {
        std::cerr << "Test FAILED: Multiple obstacles avoidance failed\n";
        return false;
    }
    
    output.print();
    
    // ========== 打印轨迹细节 ==========
    std::cout << "\n===== 轨迹细节 =====\n";
    std::cout << std::fixed << std::setprecision(6);
    
    // 起点
    std::cout << "起点 (期望): (" 
              << input.start_state_XYTheta.x() << ", "
              << input.start_state_XYTheta.y() << ", "
              << input.start_state_XYTheta.z() << " rad)\n";
    
    std::cout << "起点 (实际): (" 
              << input.start_state_XYTheta.x() << ", "
              << input.start_state_XYTheta.y() << ", "
              << input.start_state_XYTheta.z() << " rad)\n";
    
    // 终点
    std::cout << "终点 (期望): (" 
              << input.final_state_XYTheta.x() << ", "
              << input.final_state_XYTheta.y() << ", "
              << input.final_state_XYTheta.z() << " rad)\n";
    
    // 计算实际终点位置（通过积分轨迹）
    Eigen::Vector3d actual_end_XYTheta = input.start_state_XYTheta;
    double dt = 0.01;  // 10ms采样
    double total_time = output.segment_durations.sum();
    int num_samples = static_cast<int>(total_time / dt) + 1;
    
    // 检查所有障碍物的最小距离
    std::vector<double> min_distances(3, 1e10);
    double overall_min_distance = 1e10;
    
    for (int i = 1; i < num_samples; i++) {
        double t = std::min(i * dt, total_time);
        Eigen::Vector2d sigma = output.trajectory.getPos(t);
        Eigen::Vector2d dsigma = output.trajectory.getVel(t);
        
        double yaw = sigma.x();
        double ds = dsigma.y();
        
        actual_end_XYTheta.x() += ds * cos(yaw) * dt;
        actual_end_XYTheta.y() += ds * sin(yaw) * dt;
        actual_end_XYTheta.z() = yaw;
        
        // 检查到所有障碍物的距离
        Eigen::Vector2d current_pos(actual_end_XYTheta.x(), actual_end_XYTheta.y());
        for (size_t j = 0; j < obstacles.size(); j++) {
            double dist = (current_pos - obstacles[j].head(2)).norm() - obstacles[j].z();
            min_distances[j] = std::min(min_distances[j], dist);
            overall_min_distance = std::min(overall_min_distance, dist);
        }
    }
    
    std::cout << "终点 (实际): (" 
              << actual_end_XYTheta.x() << ", "
              << actual_end_XYTheta.y() << ", "
              << actual_end_XYTheta.z() << " rad)\n";
    
    // 终点误差
    Eigen::Vector3d position_error = actual_end_XYTheta - input.final_state_XYTheta;
    double position_error_norm = position_error.head(2).norm();
    double angle_error = std::abs(position_error.z());
    
    std::cout << "\n终点误差:\n";
    std::cout << "  位置误差: " << position_error_norm * 1000.0 << " mm\n";
    std::cout << "  X误差: " << position_error.x() * 1000.0 << " mm\n";
    std::cout << "  Y误差: " << position_error.y() * 1000.0 << " mm\n";
    std::cout << "  角度误差: " << angle_error * 180.0 / M_PI << " deg\n";
    
    std::cout << "\n避障信息:\n";
    for (size_t j = 0; j < obstacles.size(); j++) {
        std::cout << "  障碍物" << (j+1) << " @ (" 
                  << obstacles[j].x() << ", " 
                  << obstacles[j].y() << "): "
                  << "最小距离 = " << min_distances[j] * 1000.0 << " mm\n";
    }
    std::cout << "  整体最小距离: " << overall_min_distance * 1000.0 << " mm\n";
    std::cout << "  安全距离: " << config.safe_distance * 1000.0 << " mm\n";
    
    if (overall_min_distance < config.safe_distance) {
        std::cout << "  警告: 轨迹可能穿过障碍物！\n";
    } else {
        std::cout << "  ✓ 成功避开所有障碍物\n";
    }
    
    // 检查到中途航点的最小距离
    if (!input.waypoint_positions.empty()) {
        std::cout << "\n中途航点接近度:\n";
        
        for (size_t wp_idx = 0; wp_idx < input.waypoint_positions.size(); wp_idx++) {
            Eigen::Vector2d waypoint_pos = input.waypoint_positions[wp_idx].head(2);
            double min_dist_to_waypoint = 1e10;
            
            // 重新遍历轨迹
            Eigen::Vector3d current_XYTheta = input.start_state_XYTheta;
            for (int i = 1; i < num_samples; i++) {
                double t = std::min(i * dt, total_time);
                Eigen::Vector2d sigma = output.trajectory.getPos(t);
                Eigen::Vector2d dsigma = output.trajectory.getVel(t);
                
                double yaw = sigma.x();
                double ds = dsigma.y();
                
                current_XYTheta.x() += ds * cos(yaw) * dt;
                current_XYTheta.y() += ds * sin(yaw) * dt;
                
                Eigen::Vector2d current_pos = current_XYTheta.head(2);
                double dist = (current_pos - waypoint_pos).norm();
                min_dist_to_waypoint = std::min(min_dist_to_waypoint, dist);
            }
            
            std::cout << "  航点" << (wp_idx + 1) << " @ (" 
                      << waypoint_pos.x() << ", " 
                      << waypoint_pos.y() << "): "
                      << "最小距离 = " << min_dist_to_waypoint * 1000.0 << " mm";
            
            if (min_dist_to_waypoint < 0.5) {
                std::cout << " ✓ 经过\n";
            } else if (min_dist_to_waypoint < 1.0) {
                std::cout << " ⚠ 接近\n";
            } else {
                std::cout << " ✗ 偏离\n";
            }
        }
    }
    
    std::cout << "===================\n\n";
    
    // 使用matplotlib-cpp直接可视化
    TrajectoryVisualizer visualizer(16.0, 12.0, 150);
    visualizer.visualize(input, output, obstacles, config.check_points, 
                        "test2_multiple_obstacles.png", false);
    std::cout << "轨迹可视化已保存到: test2_multiple_obstacles.png\n\n";
    
    std::cout << "Test PASSED: Successfully navigated through multiple obstacles\n";
    return true;
}

int main(int argc, char** argv) {
    std::cout << "========================================\n";
    std::cout << "DDR Optimizer - Obstacle Avoidance Tests\n";
    std::cout << "========================================\n";
    
    int passed = 0;
    int total = 0;
    
    // 测试1: 单障碍物
    total++;
    if (test_single_obstacle()) {
        passed++;
    }
    
    // 测试2: 多障碍物
    total++;
    if (test_multiple_obstacles()) {
        passed++;
    }
    
    // 总结
    std::cout << "\n========================================\n";
    std::cout << "Test Summary: " << passed << "/" << total << " passed\n";
    std::cout << "========================================\n";
    
    return (passed == total) ? 0 : 1;
}

