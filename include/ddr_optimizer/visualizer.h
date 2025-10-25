/**
 * @file visualizer.h
 * @brief 使用matplotlib-cpp进行轨迹可视化
 */

#ifndef DDR_OPTIMIZER_VISUALIZER_H
#define DDR_OPTIMIZER_VISUALIZER_H

#include <vector>
#include <string>
#include <Python.h>
#include <Eigen/Eigen>
#include "gcopter/trajectory.hpp"
#include "ddr_optimizer/trajectory_data.h"

// matplotlib-cpp
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

namespace ddr_optimizer {

class TrajectoryVisualizer {
public:
    /**
     * @brief 初始化matplotlib后端（使用非交互式后端Agg）
     */
    static void initBackend() {
        static bool initialized = false;
        if (!initialized) {
            // 使用Python代码设置后端
            PyRun_SimpleString("import matplotlib");
            PyRun_SimpleString("matplotlib.use('Agg')");
            initialized = true;
        }
    }
    
    /**
     * @brief 可视化轨迹输入和输出
     */
    static void visualize(const TrajectoryInput& input,
                         const TrajectoryOutput& output,
                         const std::string& title = "DDR Trajectory Optimization") {
        
        initBackend();
        plt::figure_size(1200, 800);
        plt::clf();
        
        // 采样优化后的轨迹
        std::vector<double> traj_x, traj_y;
        sampleTrajectory(output.trajectory, input.start_state_XYTheta, traj_x, traj_y);
        
        // 1. 绘制优化后的轨迹
        plt::plot(traj_x, traj_y, {{"color", "blue"}, {"linewidth", "2"}, {"label", "Optimized Trajectory"}});
        
        // 2. 绘制起点
        plt::plot(std::vector<double>{input.start_state_XYTheta.x()},
                 std::vector<double>{input.start_state_XYTheta.y()},
                 {{"marker", "o"}, {"markersize", "12"}, {"color", "green"}, {"label", "Start"}});
        
        // 3. 绘制终点
        plt::plot(std::vector<double>{input.final_state_XYTheta.x()},
                 std::vector<double>{input.final_state_XYTheta.y()},
                 {{"marker", "s"}, {"markersize", "12"}, {"color", "red"}, {"label", "Goal"}});
        
        // 4. 绘制航点
        if (!input.waypoint_positions.empty()) {
            std::vector<double> wp_x, wp_y;
            for (const auto& wp : input.waypoint_positions) {
                wp_x.push_back(wp.x());
                wp_y.push_back(wp.y());
            }
            plt::scatter(wp_x, wp_y, 80, {{"color", "orange"}, {"marker", "^"}, {"label", "Waypoints"}});
        }
        
        plt::xlabel("X [m]");
        plt::ylabel("Y [m]");
        plt::title(title);
        plt::legend();
        plt::grid(true);
        plt::axis("equal");
    }
    
    /**
     * @brief 可视化包含障碍物的场景
     */
    static void visualizeWithObstacles(const TrajectoryInput& input,
                                       const TrajectoryOutput& output,
                                       const std::vector<Eigen::Vector3d>& obstacles,
                                       const std::string& title = "DDR Trajectory with Obstacles") {
        
        initBackend();
        plt::figure_size(1200, 800);
        plt::clf();
        
        // 采样优化后的轨迹
        std::vector<double> traj_x, traj_y;
        sampleTrajectory(output.trajectory, input.start_state_XYTheta, traj_x, traj_y);
        
        // 1. 绘制障碍物
        for (const auto& obs : obstacles) {
            drawCircle(obs.x(), obs.y(), obs.z(), "red", 0.3);
        }
        
        // 2. 绘制优化后的轨迹
        plt::plot(traj_x, traj_y, {{"color", "blue"}, {"linewidth", "2.5"}, {"label", "Optimized Trajectory"}});
        
        // 3. 绘制初始路径（航点连线）
        if (!input.waypoint_positions.empty()) {
            std::vector<double> path_x, path_y;
            path_x.push_back(input.start_state_XYTheta.x());
            path_y.push_back(input.start_state_XYTheta.y());
            
            for (const auto& wp : input.waypoint_positions) {
                path_x.push_back(wp.x());
                path_y.push_back(wp.y());
            }
            
            path_x.push_back(input.final_state_XYTheta.x());
            path_y.push_back(input.final_state_XYTheta.y());
            
            plt::plot(path_x, path_y, {{"color", "gray"}, {"linestyle", "--"}, 
                                       {"linewidth", "1.5"}, {"alpha", "0.5"}, 
                                       {"label", "Initial Path"}});
        }
        
        // 4. 绘制起点
        plt::plot(std::vector<double>{input.start_state_XYTheta.x()},
                 std::vector<double>{input.start_state_XYTheta.y()},
                 {{"marker", "o"}, {"markersize", "12"}, {"color", "green"}, 
                  {"markeredgecolor", "black"}, {"markeredgewidth", "2"}, {"label", "Start"}});
        
        // 5. 绘制终点
        plt::plot(std::vector<double>{input.final_state_XYTheta.x()},
                 std::vector<double>{input.final_state_XYTheta.y()},
                 {{"marker", "s"}, {"markersize", "12"}, {"color", "red"}, 
                  {"markeredgecolor", "black"}, {"markeredgewidth", "2"}, {"label", "Goal"}});
        
        // 6. 绘制航点
        if (!input.waypoint_positions.empty()) {
            std::vector<double> wp_x, wp_y;
            for (const auto& wp : input.waypoint_positions) {
                wp_x.push_back(wp.x());
                wp_y.push_back(wp.y());
            }
            plt::scatter(wp_x, wp_y, 100, {{"color", "orange"}, {"marker", "^"}, 
                                           {"edgecolors", "black"}, {"linewidths", "1.5"},
                                           {"label", "Waypoints"}, {"zorder", "10"}});
        }
        
        plt::xlabel("X [m]");
        plt::ylabel("Y [m]");
        plt::title(title);
        plt::legend();
        plt::grid(true);
        plt::axis("equal");
    }
    
    /**
     * @brief 保存图像到文件
     */
    static void save(const std::string& filename) {
        plt::save(filename);
    }
    
    /**
     * @brief 显示图像
     */
    static void show() {
        plt::show();
    }
    
private:
    /**
     * @brief 采样轨迹点
     */
    static void sampleTrajectory(const Trajectory<5, 2>& traj,
                                const Eigen::Vector3d& start_XYTheta,
                                std::vector<double>& x_out,
                                std::vector<double>& y_out,
                                double dt = 0.01) {
        x_out.clear();
        y_out.clear();
        
        double total_time = traj.getTotalDuration();
        int num_samples = static_cast<int>(total_time / dt) + 1;
        
        Eigen::Vector3d current_XYTheta = start_XYTheta;
        x_out.push_back(current_XYTheta.x());
        y_out.push_back(current_XYTheta.y());
        
        for (int i = 1; i < num_samples; i++) {
            double t = std::min(i * dt, total_time);
            Eigen::Vector2d sigma = traj.getPos(t);
            Eigen::Vector2d dsigma = traj.getVel(t);
            
            double yaw = sigma.x();
            double ds = dsigma.y();
            
            current_XYTheta.x() += ds * cos(yaw) * dt;
            current_XYTheta.y() += ds * sin(yaw) * dt;
            
            x_out.push_back(current_XYTheta.x());
            y_out.push_back(current_XYTheta.y());
        }
    }
    
    /**
     * @brief 绘制圆形（障碍物）
     */
    static void drawCircle(double cx, double cy, double r, 
                          const std::string& color = "red", 
                          double alpha = 0.3) {
        const int n_points = 50;
        std::vector<double> x(n_points + 1), y(n_points + 1);
        
        for (int i = 0; i <= n_points; i++) {
            double theta = 2.0 * M_PI * i / n_points;
            x[i] = cx + r * cos(theta);
            y[i] = cy + r * sin(theta);
        }
        
        // 填充障碍物
        plt::fill(x, y, {{"color", color}, {"alpha", std::to_string(alpha)}});
        
        // 边界
        plt::plot(x, y, {{"color", color}, {"linewidth", "2"}});
    }
};

} // namespace ddr_optimizer

#endif // DDR_OPTIMIZER_VISUALIZER_H

