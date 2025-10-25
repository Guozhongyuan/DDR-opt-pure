/**
 * @file visualizer.h
 * @brief 基于matplotlib-cpp的轨迹可视化类
 */

#ifndef DDR_OPTIMIZER_VISUALIZER_H
#define DDR_OPTIMIZER_VISUALIZER_H

#include "matplotlibcpp.h"
#include <vector>
#include <string>
#include <Eigen/Eigen>
#include "ddr_optimizer/trajectory_data.h"
#include "gcopter/trajectory.hpp"

namespace ddr_optimizer {

class TrajectoryVisualizer {
public:
    /**
     * @brief 构造函数
     * @param figure_size 图像大小 (width, height)
     * @param dpi 图像分辨率
     */
    TrajectoryVisualizer(double figure_width = 12.0, double figure_height = 8.0, int dpi = 150);
    
    /**
     * @brief 可视化轨迹
     * @param input 轨迹输入
     * @param output 轨迹输出
     * @param obstacles 障碍物列表
     * @param check_points 车体检查点
     * @param output_file 输出文件名（可选）
     * @param show_plot 是否显示图形
     */
    void visualize(const TrajectoryInput& input,
                   const TrajectoryOutput& output,
                   const std::vector<Eigen::Vector3d>& obstacles = std::vector<Eigen::Vector3d>(),
                   const std::vector<Eigen::Vector2d>& check_points = std::vector<Eigen::Vector2d>(),
                   const std::string& output_file = "",
                   bool show_plot = true);
    
    /**
     * @brief 可视化轨迹和运动学曲线（速度、加速度、角速度）
     * @param input 轨迹输入
     * @param output 轨迹输出
     * @param obstacles 障碍物列表
     * @param check_points 车体检查点
     * @param output_file 输出文件名（可选）
     * @param show_plot 是否显示图形
     */
    void visualizeWithKinematics(const TrajectoryInput& input,
                                const TrajectoryOutput& output,
                                const std::vector<Eigen::Vector3d>& obstacles = std::vector<Eigen::Vector3d>(),
                                const std::vector<Eigen::Vector2d>& check_points = std::vector<Eigen::Vector2d>(),
                                const std::string& output_file = "",
                                bool show_plot = true);
    
    /**
     * @brief 设置图像标题
     */
    void setTitle(const std::string& title);
    
    /**
     * @brief 设置坐标轴标签
     */
    void setLabels(const std::string& xlabel, const std::string& ylabel);
    
    /**
     * @brief 保存图像到文件
     */
    void saveFigure(const std::string& filename);
    
    /**
     * @brief 显示图形
     */
    void show();

private:
    
    double figure_width_;
    double figure_height_;
    int dpi_;
    std::string title_;
    std::string xlabel_;
    std::string ylabel_;
    
    /**
     * @brief 采样轨迹点
     */
    void sampleTrajectory(const Trajectory<5, 2>& traj,
                          const Eigen::Vector3d& start_XYTheta,
                          std::vector<double>& x_coords,
                          std::vector<double>& y_coords,
                          double dt = 0.01);
    
    /**
     * @brief 绘制车体多边形（沿轨迹采样）
     */
    void drawVehiclePolygons(const Trajectory<5, 2>& traj,
                             const Eigen::Vector3d& start_XYTheta,
                             const std::vector<Eigen::Vector2d>& check_points);
    
    /**
     * @brief 采样轨迹的运动学数据（速度、加速度、角速度）
     */
    void sampleKinematicsData(const Trajectory<5, 2>& traj,
                              const Eigen::Vector3d& start_XYTheta,
                              std::vector<double>& time_points,
                              std::vector<double>& velocities,
                              std::vector<double>& accelerations,
                              std::vector<double>& angular_velocities,
                              double dt = 0.01);
    
    /**
     * @brief 绘制运动学曲线子图
     */
    void plotKinematicsSubplots(const std::vector<double>& time_points,
                               const std::vector<double>& velocities,
                               const std::vector<double>& accelerations,
                               const std::vector<double>& angular_velocities);
    
    /**
     * @brief 仅绘制轨迹图（用于子图）
     */
    void visualizeTrajectoryOnly(const TrajectoryInput& input,
                                const TrajectoryOutput& output,
                                const std::vector<Eigen::Vector3d>& obstacles,
                                const std::vector<Eigen::Vector2d>& check_points);
};

} // namespace ddr_optimizer

#endif // DDR_OPTIMIZER_VISUALIZER_H