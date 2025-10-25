/**
 * @file trajectory_exporter.h
 * @brief 导出轨迹数据到文件用于可视化
 */

#ifndef DDR_OPTIMIZER_TRAJECTORY_EXPORTER_H
#define DDR_OPTIMIZER_TRAJECTORY_EXPORTER_H

#include <vector>
#include <string>
#include <fstream>
#include <iomanip>
#include <Eigen/Eigen>
#include "gcopter/trajectory.hpp"
#include "ddr_optimizer/trajectory_data.h"

namespace ddr_optimizer {

class TrajectoryExporter {
public:
    /**
     * @brief 导出轨迹数据到JSON格式文件
     */
    static bool exportToJSON(const TrajectoryInput& input,
                            const TrajectoryOutput& output,
                            const std::string& filename,
                            const std::vector<Eigen::Vector3d>& obstacles = std::vector<Eigen::Vector3d>(),
                            const std::vector<Eigen::Vector2d>& check_points = std::vector<Eigen::Vector2d>()) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Failed to open file: " << filename << std::endl;
            return false;
        }
        
        file << std::setprecision(6) << std::fixed;
        file << "{\n";
        
        // 采样轨迹
        std::vector<Eigen::Vector2d> trajectory_points;
        sampleTrajectory(output.trajectory, input.start_state_XYTheta, trajectory_points);
        
        // 输出轨迹点
        file << "  \"trajectory\": [\n";
        for (size_t i = 0; i < trajectory_points.size(); i++) {
            file << "    [" << trajectory_points[i].x() << ", " << trajectory_points[i].y() << "]";
            if (i < trajectory_points.size() - 1) file << ",";
            file << "\n";
        }
        file << "  ],\n";
        
        // 输出起点
        file << "  \"start\": [" << input.start_state_XYTheta.x() << ", " 
             << input.start_state_XYTheta.y() << "],\n";
        
        // 输出终点
        file << "  \"goal\": [" << input.final_state_XYTheta.x() << ", " 
             << input.final_state_XYTheta.y() << "],\n";
        
        // 输出航点
        file << "  \"waypoints\": [\n";
        for (size_t i = 0; i < input.waypoint_positions.size(); i++) {
            file << "    [" << input.waypoint_positions[i].x() << ", " 
                 << input.waypoint_positions[i].y() << "]";
            if (i < input.waypoint_positions.size() - 1) file << ",";
            file << "\n";
        }
        file << "  ],\n";
        
        // 输出障碍物
        file << "  \"obstacles\": [\n";
        for (size_t i = 0; i < obstacles.size(); i++) {
            file << "    {\"x\": " << obstacles[i].x() 
                 << ", \"y\": " << obstacles[i].y() 
                 << ", \"r\": " << obstacles[i].z() << "}";
            if (i < obstacles.size() - 1) file << ",";
            file << "\n";
        }
        file << "  ],\n";
        
        // 输出车体多边形（沿轨迹采样）
        if (!check_points.empty()) {
            std::vector<std::vector<Eigen::Vector2d>> vehicle_polygons;
            sampleVehiclePolygons(output.trajectory, input.start_state_XYTheta, 
                                 check_points, vehicle_polygons);
            
            file << "  \"vehicle_polygons\": [\n";
            for (size_t i = 0; i < vehicle_polygons.size(); i++) {
                file << "    [\n";
                for (size_t j = 0; j < vehicle_polygons[i].size(); j++) {
                    file << "      [" << vehicle_polygons[i][j].x() << ", " 
                         << vehicle_polygons[i][j].y() << "]";
                    if (j < vehicle_polygons[i].size() - 1) file << ",";
                    file << "\n";
                }
                file << "    ]";
                if (i < vehicle_polygons.size() - 1) file << ",";
                file << "\n";
            }
            file << "  ]\n";
        } else {
            file << "  \"vehicle_polygons\": []\n";
        }
        
        file << "}\n";
        file.close();
        
        return true;
    }
    
private:
    /**
     * @brief 采样轨迹点
     */
    static void sampleTrajectory(const Trajectory<5, 2>& traj,
                                const Eigen::Vector3d& start_XYTheta,
                                std::vector<Eigen::Vector2d>& points,
                                double dt = 0.01) {
        points.clear();
        
        double total_time = traj.getTotalDuration();
        int num_samples = static_cast<int>(total_time / dt) + 1;
        
        Eigen::Vector3d current_XYTheta = start_XYTheta;
        points.push_back(current_XYTheta.head(2));
        
        for (int i = 1; i < num_samples; i++) {
            double t = std::min(i * dt, total_time);
            Eigen::Vector2d sigma = traj.getPos(t);
            Eigen::Vector2d dsigma = traj.getVel(t);
            
            double yaw = sigma.x();
            double ds = dsigma.y();
            
            current_XYTheta.x() += ds * cos(yaw) * dt;
            current_XYTheta.y() += ds * sin(yaw) * dt;
            
            points.push_back(current_XYTheta.head(2));
        }
    }
    
    /**
     * @brief 采样车体多边形（沿轨迹）
     */
    static void sampleVehiclePolygons(const Trajectory<5, 2>& traj,
                                     const Eigen::Vector3d& start_XYTheta,
                                     const std::vector<Eigen::Vector2d>& check_points,
                                     std::vector<std::vector<Eigen::Vector2d>>& polygons,
                                     double dt = 0.1) {  // 每0.1秒采样一次车体
        polygons.clear();
        
        if (check_points.empty()) return;
        
        double total_time = traj.getTotalDuration();
        int num_samples = static_cast<int>(total_time / dt) + 1;
        
        Eigen::Vector3d current_XYTheta = start_XYTheta;
        
        for (int i = 0; i < num_samples; i++) {
            double t = std::min(i * dt, total_time);
            Eigen::Vector2d sigma = traj.getPos(t);
            Eigen::Vector2d dsigma = traj.getVel(t);
            
            double yaw = sigma.x();
            double ds = dsigma.y();
            
            if (i > 0) {
                current_XYTheta.x() += ds * cos(yaw) * dt;
                current_XYTheta.y() += ds * sin(yaw) * dt;
            }
            
            // 计算车体多边形在当前位置和角度下的坐标
            std::vector<Eigen::Vector2d> polygon;
            for (const auto& check_point : check_points) {
                // 将车体局部坐标转换为全局坐标
                double cos_yaw = cos(yaw);
                double sin_yaw = sin(yaw);
                
                double global_x = current_XYTheta.x() + 
                    check_point.x() * cos_yaw - check_point.y() * sin_yaw;
                double global_y = current_XYTheta.y() + 
                    check_point.x() * sin_yaw + check_point.y() * cos_yaw;
                
                polygon.emplace_back(global_x, global_y);
            }
            
            polygons.push_back(polygon);
        }
    }
};

} // namespace ddr_optimizer

#endif // DDR_OPTIMIZER_TRAJECTORY_EXPORTER_H

