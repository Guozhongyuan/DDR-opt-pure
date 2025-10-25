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
 * @brief 测试1: 简单直线轨迹优化
 * 
 * 场景: 从 (0, 0, 0) 到 (5, 0, 0)，无障碍物
 * 验证: 轨迹应该成功生成，运动学约束应该满足
 */
bool test_straight_line() {
    std::cout << "\n========== Test 1: Straight Line Trajectory ==========\n";
    
    // 创建配置
    OptimizerConfig config = OptimizerConfig::defaultConfig();
    config.verbose = true;
    
    // 创建碰撞检测器（无障碍物）
    auto collision_checker = std::make_shared<SimpleCircleObstacles>(
        Eigen::Vector2d(-10, -10),
        Eigen::Vector2d(10, 10)
    );
    
    // 创建优化器
    DDROptimizer optimizer(config, collision_checker);
    
    // 构造输入
    TrajectoryInput input;
    
    // 起点状态: [[yaw, dyaw, ddyaw], [s, ds, dds]]
    input.start_state.resize(2, 3);
    input.start_state << 
        0.0, 0.0, 0.0,  // yaw状态
        0.0, 0.0, 0.0;  // s状态
    
    // 终点状态
    input.final_state.resize(2, 3);
    input.final_state << 
        0.0, 0.0, 0.0,  // yaw状态
        5.0, 0.0, 0.0;  // s状态 (移动5米)
    
    // 笛卡尔坐标
    input.start_state_XYTheta = Eigen::Vector3d(0.0, 0.0, 0.0);
    input.final_state_XYTheta = Eigen::Vector3d(5.0, 0.0, 0.0);
    
    // 简单的中间点（可选，这里我们先不使用）
    // 为了测试，我们添加两个中间点
    input.waypoints.push_back(Eigen::Vector3d(0.0, 1.5, 0.5));  // yaw, s, time
    input.waypoints.push_back(Eigen::Vector3d(0.0, 3.5, 0.5));
    
    input.waypoint_positions.push_back(Eigen::Vector3d(1.5, 0.0, 0.0));
    input.waypoint_positions.push_back(Eigen::Vector3d(3.5, 0.0, 0.0));
    
    input.initial_segment_time = 1.0;
    input.is_cut = false;
    
    // 执行优化
    TrajectoryOutput output;
    bool success = optimizer.optimize(input, output);
    
    // 输出结果
    output.print();
    optimizer.getStats().print();
    
    if (!success) {
        std::cerr << "Test FAILED: Optimization failed\n";
        return false;
    }
    
    // 验证结果
    if (output.segment_durations.sum() <= 0) {
        std::cerr << "Test FAILED: Invalid trajectory duration\n";
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
    
    for (int i = 1; i < num_samples; i++) {
        double t = std::min(i * dt, total_time);
        Eigen::Vector2d sigma = output.trajectory.getPos(t);
        Eigen::Vector2d dsigma = output.trajectory.getVel(t);
        
        double yaw = sigma.x();
        double ds = dsigma.y();
        
        actual_end_XYTheta.x() += ds * cos(yaw) * dt;
        actual_end_XYTheta.y() += ds * sin(yaw) * dt;
        actual_end_XYTheta.z() = yaw;
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
    
    // 检查到中途航点的最小距离
    if (!input.waypoint_positions.empty()) {
        std::cout << "\n中途航点接近度:\n";
        
        for (size_t wp_idx = 0; wp_idx < input.waypoint_positions.size(); wp_idx++) {
            Eigen::Vector2d waypoint_pos = input.waypoint_positions[wp_idx].head(2);
            double min_dist_to_waypoint = 1e10;
            
            // 重新遍历轨迹，计算到该航点的最小距离
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
            
            if (min_dist_to_waypoint < 0.5) {  // 500mm以内认为经过
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
    TrajectoryVisualizer visualizer(20.0, 16.0, 150);
    visualizer.visualizeWithKinematics(input, output, std::vector<Eigen::Vector3d>(), 
                                      config.check_points, "test1_straight_line_with_kinematics.png", false);
    std::cout << "轨迹可视化（含运动学曲线）已保存到: test1_straight_line_with_kinematics.png\n\n";
    
    std::cout << "Test PASSED: Straight line trajectory generated successfully\n";
    return true;
}

/**
 * @brief 测试2: 带转向的轨迹
 * 
 * 场景: 从 (0, 0, 0) 到 (3, 3, π/2)
 */
bool test_turn_trajectory() {
    std::cout << "\n========== Test 2: Turn Trajectory ==========\n";
    
    OptimizerConfig config = OptimizerConfig::defaultConfig();
    config.verbose = false;  // 减少输出
    
    auto collision_checker = std::make_shared<SimpleCircleObstacles>(
        Eigen::Vector2d(-10, -10),
        Eigen::Vector2d(10, 10)
    );
    
    DDROptimizer optimizer(config, collision_checker);
    
    TrajectoryInput input;
    
    input.start_state.resize(2, 3);
    input.start_state << 
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0;
    
    input.final_state.resize(2, 3);
    input.final_state << 
        M_PI/2, 0.0, 0.0,  // 转90度
        4.0, 0.0, 0.0;     // 移动约4米
    
    input.start_state_XYTheta = Eigen::Vector3d(0.0, 0.0, 0.0);
    input.final_state_XYTheta = Eigen::Vector3d(3.0, 3.0, M_PI/2);
    
    // 添加一个中间点
    input.waypoints.push_back(Eigen::Vector3d(M_PI/4, 2.0, 1.0));
    input.waypoint_positions.push_back(Eigen::Vector3d(1.5, 1.5, M_PI/4));
    
    input.initial_segment_time = 1.0;
    input.is_cut = false;
    
    TrajectoryOutput output;
    bool success = optimizer.optimize(input, output);
    
    if (!success) {
        std::cerr << "Test FAILED: Turn trajectory optimization failed\n";
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
    
    for (int i = 1; i < num_samples; i++) {
        double t = std::min(i * dt, total_time);
        Eigen::Vector2d sigma = output.trajectory.getPos(t);
        Eigen::Vector2d dsigma = output.trajectory.getVel(t);
        
        double yaw = sigma.x();
        double ds = dsigma.y();
        
        actual_end_XYTheta.x() += ds * cos(yaw) * dt;
        actual_end_XYTheta.y() += ds * sin(yaw) * dt;
        actual_end_XYTheta.z() = yaw;
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
    
    // 检查到中途航点的最小距离
    if (!input.waypoint_positions.empty()) {
        std::cout << "\n中途航点接近度:\n";
        
        for (size_t wp_idx = 0; wp_idx < input.waypoint_positions.size(); wp_idx++) {
            Eigen::Vector2d waypoint_pos = input.waypoint_positions[wp_idx].head(2);
            double min_dist_to_waypoint = 1e10;
            
            // 重新遍历轨迹，计算到该航点的最小距离
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
            
            if (min_dist_to_waypoint < 0.5) {  // 500mm以内认为经过
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
    TrajectoryVisualizer visualizer(20.0, 16.0, 150);
    visualizer.visualizeWithKinematics(input, output, std::vector<Eigen::Vector3d>(), 
                                      config.check_points, "test1_turn_trajectory_with_kinematics.png", false);
    std::cout << "轨迹可视化（含运动学曲线）已保存到: test1_turn_trajectory_with_kinematics.png\n\n";
    
    std::cout << "Test PASSED: Turn trajectory generated successfully\n";
    return true;
}

int main(int argc, char** argv) {
    std::cout << "========================================\n";
    std::cout << "DDR Optimizer - Simple Trajectory Tests\n";
    std::cout << "========================================\n";
    
    int passed = 0;
    int total = 0;
    
    // 测试1: 直线轨迹
    total++;
    if (test_straight_line()) {
        passed++;
    }
    
    // 测试2: 转向轨迹
    total++;
    if (test_turn_trajectory()) {
        passed++;
    }
    
    // 总结
    std::cout << "\n========================================\n";
    std::cout << "Test Summary: " << passed << "/" << total << " passed\n";
    std::cout << "========================================\n";
    
    return (passed == total) ? 0 : 1;
}

