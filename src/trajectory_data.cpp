#include "ddr_optimizer/trajectory_data.h"
#include <fstream>
#include <iostream>
#include <cmath>

namespace ddr_optimizer {

void TrajectoryOutput::getStateAt(double time,
                                  Eigen::Vector3d& XYTheta,
                                  Eigen::Vector3d& VAJ,
                                  Eigen::Vector3d& OAJ,
                                  double resolution,
                                  bool if_standard_diff) const {
    // TODO: 完整实现需要Simpson积分
    // 这是简化版本
    
    if (time > trajectory.getTotalDuration()) {
        time = trajectory.getTotalDuration();
    }
    
    Eigen::Vector2d sigma = trajectory.getPos(time);    // [yaw, s]
    Eigen::Vector2d vel = trajectory.getVel(time);      // [dyaw, ds]
    Eigen::Vector2d acc = trajectory.getAcc(time);      // [ddyaw, dds]
    Eigen::Vector2d jerk = trajectory.getJer(time);     // [dddyaw, ddds]
    
    // 简化计算（实际应该用积分）
    XYTheta = start_state_XYTheta;
    XYTheta.z() = sigma.x();  // yaw
    
    // 简单的位置估计
    double dt = time / 10.0;
    for (int i = 0; i < 10; i++) {
        double t = i * dt;
        Eigen::Vector2d pos = trajectory.getPos(t);
        Eigen::Vector2d v = trajectory.getVel(t);
        
        if (if_standard_diff) {
            XYTheta.x() += v.y() * cos(pos.x()) * dt;
            XYTheta.y() += v.y() * sin(pos.x()) * dt;
        } else {
            // 扩展模型（需要ICR参数）
            XYTheta.x() += v.y() * cos(pos.x()) * dt;
            XYTheta.y() += v.y() * sin(pos.x()) * dt;
        }
    }
    
    OAJ << vel.x(), acc.x(), jerk.x();
    VAJ << vel.y(), acc.y(), jerk.y();
}

std::vector<Eigen::Vector3d> TrajectoryOutput::samplePath(double dt, bool if_standard_diff) const {
    std::vector<Eigen::Vector3d> path;
    
    if (!success || num_segments == 0) {
        return path;
    }
    
    double total_time = trajectory.getTotalDuration();
    int num_samples = static_cast<int>(total_time / dt) + 1;
    
    Eigen::Vector3d XYTheta = start_state_XYTheta;
    path.push_back(XYTheta);
    
    for (int i = 1; i <= num_samples; i++) {
        double t = std::min(i * dt, total_time);
        
        Eigen::Vector2d sigma = trajectory.getPos(t);
        Eigen::Vector2d vel = trajectory.getVel(t);
        
        // 简化的积分（应该用Simpson法则）
        if (if_standard_diff) {
            XYTheta.x() += vel.y() * cos(sigma.x()) * dt;
            XYTheta.y() += vel.y() * sin(sigma.x()) * dt;
        } else {
            XYTheta.x() += vel.y() * cos(sigma.x()) * dt;
            XYTheta.y() += vel.y() * sin(sigma.x()) * dt;
        }
        XYTheta.z() = sigma.x();
        
        path.push_back(XYTheta);
    }
    
    return path;
}

bool TrajectoryOutput::exportToFile(const std::string& filename, double dt, bool if_standard_diff) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return false;
    }
    
    file << "# DDR Trajectory Export\n";
    file << "# Time, X, Y, Theta, V, Omega\n";
    
    auto path = samplePath(dt, if_standard_diff);
    
    double total_time = trajectory.getTotalDuration();
    for (size_t i = 0; i < path.size(); i++) {
        double t = std::min(i * dt, total_time);
        
        Eigen::Vector2d vel = trajectory.getVel(t);
        
        file << t << ", "
             << path[i].x() << ", "
             << path[i].y() << ", "
             << path[i].z() << ", "
             << vel.y() << ", "
             << vel.x() << "\n";
    }
    
    file.close();
    return true;
}

} // namespace ddr_optimizer

