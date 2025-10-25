#include "ddr_optimizer/visualizer.h"
#include <iostream>
#include <cmath>

namespace plt = matplotlibcpp;

namespace ddr_optimizer {

TrajectoryVisualizer::TrajectoryVisualizer(double figure_width, double figure_height, int dpi)
    : figure_width_(figure_width), figure_height_(figure_height), dpi_(dpi),
      title_(""), xlabel_("X [m]"), ylabel_("Y [m]") {
}

void TrajectoryVisualizer::visualize(const TrajectoryInput& input,
                                     const TrajectoryOutput& output,
                                     const std::vector<Eigen::Vector3d>& obstacles,
                                     const std::vector<Eigen::Vector2d>& check_points,
                                     const std::string& output_file,
                                     bool show_plot) {
    
    // 清空之前的绘图内容
    plt::clf();
    
    // 设置更大的图形尺寸
    // matplotlib-cpp的figure_size使用固定DPI=100，需要传入像素值
    plt::figure_size(static_cast<size_t>(figure_width_ * 100), static_cast<size_t>(figure_height_ * 100));
    
    // 采样轨迹数据
    std::vector<double> traj_x, traj_y;
    sampleTrajectory(output.trajectory, input.start_state_XYTheta, traj_x, traj_y);
    
    // 计算坐标轴范围
    double min_x = input.start_state_XYTheta.x();
    double max_x = input.final_state_XYTheta.x();
    double min_y = input.start_state_XYTheta.y();
    double max_y = input.final_state_XYTheta.y();
    
    // 包含轨迹点
    for (double x : traj_x) {
        min_x = std::min(min_x, x);
        max_x = std::max(max_x, x);
    }
    for (double y : traj_y) {
        min_y = std::min(min_y, y);
        max_y = std::max(max_y, y);
    }
    
    // 包含航点
    for (const auto& wp : input.waypoint_positions) {
        min_x = std::min(min_x, wp.x());
        max_x = std::max(max_x, wp.x());
        min_y = std::min(min_y, wp.y());
        max_y = std::max(max_y, wp.y());
    }
    
    // 包含障碍物
    for (const auto& obstacle : obstacles) {
        double x = obstacle.x();
        double y = obstacle.y();
        double r = obstacle.z();
        min_x = std::min(min_x, x - r);
        max_x = std::max(max_x, x + r);
        min_y = std::min(min_y, y - r);
        max_y = std::max(max_y, y + r);
    }
    
    // 添加边距
    double margin = std::max(max_x - min_x, max_y - min_y) * 0.1;
    min_x -= margin;
    max_x += margin;
    min_y -= margin;
    max_y += margin;
    
    // 绘制优化后的轨迹（蓝色线）
    plt::plot(traj_x, traj_y, "b-");
    
    // 绘制起点（绿色圆点）
    std::vector<double> start_x = {input.start_state_XYTheta.x()};
    std::vector<double> start_y = {input.start_state_XYTheta.y()};
    plt::plot(start_x, start_y, "go");
    
    // 绘制终点（红色方块）
    std::vector<double> goal_x = {input.final_state_XYTheta.x()};
    std::vector<double> goal_y = {input.final_state_XYTheta.y()};
    plt::plot(goal_x, goal_y, "rs");
    
    // 绘制航点（橙色三角形）
    if (!input.waypoint_positions.empty()) {
        std::vector<double> wp_x, wp_y;
        for (const auto& wp : input.waypoint_positions) {
            wp_x.push_back(wp.x());
            wp_y.push_back(wp.y());
        }
        plt::scatter(wp_x, wp_y, 50.0);
    }
    
    // 绘制障碍物（红色圆圈）
    for (const auto& obstacle : obstacles) {
        double x = obstacle.x();
        double y = obstacle.y();
        double r = obstacle.z();
        
        // 生成圆形数据
        std::vector<double> circle_x, circle_y;
        int num_points = 50;
        for (int i = 0; i <= num_points; i++) {
            double angle = 2.0 * M_PI * i / num_points;
            circle_x.push_back(x + r * cos(angle));
            circle_y.push_back(y + r * sin(angle));
        }
        plt::plot(circle_x, circle_y, "r-");
    }
    
    // 绘制初始路径（灰色虚线）
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
        
        plt::plot(path_x, path_y, "g--");
    }
    
    // 绘制车体多边形（沿轨迹采样）
    if (!check_points.empty()) {
        drawVehiclePolygons(output.trajectory, input.start_state_XYTheta, check_points);
    }
    
    // 设置图形属性
    if (!title_.empty()) {
        plt::title(title_);
    }
    plt::xlabel("X [m]");
    plt::ylabel("Y [m]");
    plt::grid(true);
    
    // 使用axis("equal")确保x和y轴尺度相同
    plt::axis("equal");
    
    // 保存或显示
    if (!output_file.empty()) {
        plt::save(output_file);
        std::cout << "Image saved to: " << output_file << std::endl;
    }
    
    if (show_plot) {
        plt::show();
    }
}

void TrajectoryVisualizer::setTitle(const std::string& title) {
    title_ = title;
}

void TrajectoryVisualizer::setLabels(const std::string& xlabel, const std::string& ylabel) {
    xlabel_ = xlabel;
    ylabel_ = ylabel;
}

void TrajectoryVisualizer::saveFigure(const std::string& filename) {
    plt::save(filename);
    std::cout << "Image saved to: " << filename << std::endl;
}

void TrajectoryVisualizer::show() {
    plt::show();
}

void TrajectoryVisualizer::sampleTrajectory(const Trajectory<5, 2>& traj,
                                            const Eigen::Vector3d& start_XYTheta,
                                            std::vector<double>& x_coords,
                                            std::vector<double>& y_coords,
                                            double dt) {
    x_coords.clear();
    y_coords.clear();
    
    double total_time = traj.getTotalDuration();
    int num_samples = static_cast<int>(total_time / dt) + 1;
    
    Eigen::Vector3d current_XYTheta = start_XYTheta;
    x_coords.push_back(current_XYTheta.x());
    y_coords.push_back(current_XYTheta.y());
    
    for (int i = 1; i < num_samples; i++) {
        double t = std::min(i * dt, total_time);
        Eigen::Vector2d sigma = traj.getPos(t);
        Eigen::Vector2d dsigma = traj.getVel(t);
        
        double yaw = sigma.x();
        double ds = dsigma.y();
        
        current_XYTheta.x() += ds * cos(yaw) * dt;
        current_XYTheta.y() += ds * sin(yaw) * dt;
        
        x_coords.push_back(current_XYTheta.x());
        y_coords.push_back(current_XYTheta.y());
    }
}

// 绘制车体多边形（沿轨迹采样）
void TrajectoryVisualizer::drawVehiclePolygons(const Trajectory<5, 2>& traj,
                                               const Eigen::Vector3d& start_XYTheta,
                                               const std::vector<Eigen::Vector2d>& check_points) {
    if (check_points.empty()) return;
    
    double total_time = traj.getTotalDuration();
    double dt = 0.1;  // 每0.1秒采样一次车体
    int num_samples = static_cast<int>(total_time / dt) + 1;
    
    for (int i = 0; i < num_samples; i += 1) {  // 多边形显示间隔
        double t = std::min(i * dt, total_time);
        
        // 使用与sampleTrajectory相同的方法计算当前位置
        Eigen::Vector3d current_XYTheta = start_XYTheta;
        if (t > 0) {
            // 使用轨迹积分计算当前位置
            double integration_dt = 0.01;  // 更小的积分步长
            int integration_steps = static_cast<int>(t / integration_dt);
            
            for (int j = 1; j <= integration_steps; j++) {
                double integration_t = std::min(j * integration_dt, t);
                Eigen::Vector2d sigma = traj.getPos(integration_t);
                Eigen::Vector2d dsigma = traj.getVel(integration_t);
                
                double yaw = sigma.x();
                double ds = dsigma.y();
                
                current_XYTheta.x() += ds * cos(yaw) * integration_dt;
                current_XYTheta.y() += ds * sin(yaw) * integration_dt;
                current_XYTheta.z() = yaw;  // 更新角度
            }
        }
        
        // 计算车体多边形在当前位置和角度下的坐标
        std::vector<double> poly_x, poly_y;
        for (const auto& check_point : check_points) {
            // 将车体局部坐标转换为全局坐标
            double cos_yaw = cos(current_XYTheta.z());
            double sin_yaw = sin(current_XYTheta.z());
            
            double global_x = current_XYTheta.x() + 
                check_point.x() * cos_yaw - check_point.y() * sin_yaw;
            double global_y = current_XYTheta.y() + 
                check_point.x() * sin_yaw + check_point.y() * cos_yaw;
            
            poly_x.push_back(global_x);
            poly_y.push_back(global_y);
        }
        
        // 确保多边形闭合
        if (!poly_x.empty() && !poly_y.empty()) {
            poly_x.push_back(poly_x[0]);
            poly_y.push_back(poly_y[0]);
            
            // 绘制半透明绿色车体
            std::map<std::string, std::string> vehicle_style;
            vehicle_style["color"] = "lightgreen";  // 使用浅绿色
            vehicle_style["linewidth"] = "1.0";     // 细线条
            plt::plot(poly_x, poly_y, vehicle_style);
        }
    }
}

} // namespace ddr_optimizer
