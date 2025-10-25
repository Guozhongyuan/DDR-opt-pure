/**
 * @file simple_example.cpp
 * @brief 简单的DDR轨迹优化使用示例
 * 
 * 本示例展示如何使用DDR优化器生成一条简单的轨迹
 */

#include "ddr_optimizer/optimizer_pure.h"
#include "ddr_optimizer/collision_interface.h"
#include <iostream>
#include <iomanip>

using namespace ddr_optimizer;

int main(int argc, char** argv) {
    std::cout << "========================================\n";
    std::cout << "DDR Optimizer - Simple Example\n";
    std::cout << "========================================\n\n";
    
    // 1. 创建配置
    std::cout << "Step 1: Creating configuration...\n";
    OptimizerConfig config = OptimizerConfig::defaultConfig();
    config.verbose = true;
    config.kinematic.max_vel = 3.0;       // 最大速度 3 m/s
    config.kinematic.max_acc = 2.0;       // 最大加速度 2 m/s²
    config.safe_distance = 0.5;           // 安全距离 0.5 m
    std::cout << "  Max velocity: " << config.kinematic.max_vel << " m/s\n";
    std::cout << "  Max acceleration: " << config.kinematic.max_acc << " m/s²\n";
    std::cout << "  Safe distance: " << config.safe_distance << " m\n\n";
    
    // 2. 创建碰撞检测器
    std::cout << "Step 2: Creating collision checker...\n";
    auto collision_checker = std::make_shared<SimpleCircleObstacles>(
        Eigen::Vector2d(-20, -20),  // 地图范围
        Eigen::Vector2d(20, 20)
    );
    
    // 添加一些障碍物
    collision_checker->addObstacle(5.0, 2.0, 0.8);   // 障碍物1
    collision_checker->addObstacle(8.0, -1.5, 0.6);  // 障碍物2
    std::cout << "  Added " << collision_checker->getObstacleCount() << " obstacles\n\n";
    
    // 3. 创建优化器
    std::cout << "Step 3: Creating optimizer...\n";
    DDROptimizer optimizer(config, collision_checker);
    std::cout << "  Optimizer created successfully\n\n";
    
    // 4. 构造输入轨迹
    std::cout << "Step 4: Constructing input trajectory...\n";
    TrajectoryInput input;
    
    // 起点: 原点，朝向0度
    input.start_state.resize(2, 3);
    input.start_state << 
        0.0, 0.0, 0.0,  // yaw状态: [yaw, dyaw, ddyaw]
        0.0, 0.0, 0.0;  // s状态: [s, ds, dds]
    input.start_state_XYTheta = Eigen::Vector3d(0.0, 0.0, 0.0);
    
    // 终点: (10, 0, 0)
    input.final_state.resize(2, 3);
    input.final_state << 
        0.0, 0.0, 0.0,
        10.0, 0.0, 0.0;  // 移动10米
    input.final_state_XYTheta = Eigen::Vector3d(10.0, 0.0, 0.0);
    
    // 添加中间路径点（帮助绕过障碍物）
    input.waypoints.push_back(Eigen::Vector3d(0.0, 3.0, 1.0));   // (yaw, s, time)
    input.waypoints.push_back(Eigen::Vector3d(0.0, 6.0, 1.0));
    
    input.waypoint_positions.push_back(Eigen::Vector3d(3.0, 1.0, 0.0));  // (x, y, theta)
    input.waypoint_positions.push_back(Eigen::Vector3d(6.0, 0.5, 0.0));
    
    input.initial_segment_time = 1.0;
    input.is_cut = false;
    
    std::cout << "  Start: (" << input.start_state_XYTheta.transpose() << ")\n";
    std::cout << "  Goal:  (" << input.final_state_XYTheta.transpose() << ")\n";
    std::cout << "  Waypoints: " << input.waypoints.size() << "\n\n";
    
    // 5. 执行优化
    std::cout << "Step 5: Running optimization...\n";
    std::cout << "========================================\n\n";
    
    TrajectoryOutput output;
    bool success = optimizer.optimize(input, output);
    
    std::cout << "\n========================================\n";
    
    // 6. 输出结果
    if (success) {
        std::cout << "\n✓ Optimization SUCCESS!\n\n";
        
        std::cout << "Results:\n";
        std::cout << "  Number of segments: " << output.num_segments << "\n";
        std::cout << "  Total duration: " << output.segment_durations.sum() << " s\n";
        std::cout << "  Computation time: " << output.total_time_ms << " ms\n";
        std::cout << "  Final position error: " << optimizer.getStats().final_position_error << " m\n";
        std::cout << "  Iterations: " << optimizer.getStats().total_iterations << "\n\n";
        
        // 采样轨迹点
        std::cout << "Sampling trajectory (every 0.5s):\n";
        std::cout << "  Time  |   X   |   Y   | Theta\n";
        std::cout << "  ------|-------|-------|-------\n";
        
        auto path = output.samplePath(0.5, config.if_standard_diff);
        for (size_t i = 0; i < path.size(); i++) {
            std::cout << "  " << std::fixed << std::setprecision(2)
                     << std::setw(4) << (i * 0.5) << "  | "
                     << std::setw(5) << path[i].x() << " | "
                     << std::setw(5) << path[i].y() << " | "
                     << std::setw(5) << path[i].z() << "\n";
        }
        std::cout << "\n";
        
        // 导出到文件
        std::string filename = "trajectory_output.txt";
        if (output.exportToFile(filename, 0.1, config.if_standard_diff)) {
            std::cout << "Trajectory exported to: " << filename << "\n";
        }
        
    } else {
        std::cout << "\n✗ Optimization FAILED!\n";
        std::cout << "Message: " << output.message << "\n\n";
    }
    
    std::cout << "\n========================================\n";
    std::cout << "Example Complete\n";
    std::cout << "========================================\n";
    
    return success ? 0 : 1;
}

