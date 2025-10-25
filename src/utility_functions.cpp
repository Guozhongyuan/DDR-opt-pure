/**
 * @file utility_functions.cpp
 * @brief 优化器非模板辅助函数实现
 */

#include "ddr_optimizer/optimizer_pure.h"
#include <cmath>

namespace ddr_optimizer {

void DDROptimizer::log(const std::string& msg, bool force) {
    if (force || if_print_) {
        std::cout << msg << std::endl;
    }
}

void DDROptimizer::logInfo(const std::string& msg) {
    log("[INFO] " + msg, false);
}

void DDROptimizer::logWarn(const std::string& msg) {
    log("[WARN] " + msg, true);
}

void DDROptimizer::logError(const std::string& msg) {
    log("[ERROR] " + msg, true);
}

void DDROptimizer::getPredictedState(const double& time,
                                     Eigen::Vector3d& XYTheta,
                                     Eigen::Vector3d& VAJ,
                                     Eigen::Vector3d& OAJ) {
    double check_time = time;
    if (time > final_trajectory_.getTotalDuration()) {
        check_time = final_trajectory_.getTotalDuration();
    }
    
    XYTheta = ini_state_XYTheta_;
    
    double step = config_.traj_predict_resolution;
    double halfstep = step / 2.0;
    double step1_6 = step / 6.0;
    
    int sequence_num = floor(check_time / step);
    double left_time = check_time - sequence_num * step;
    
    Eigen::Vector2d p3, v3, a3, j3;
    p3 = final_trajectory_.getPos(0.0);
    v3 = final_trajectory_.getVel(0.0);
    
    // Simpson积分前进模拟
    for (int i = 0; i < sequence_num; i++) {
        double t1 = i * step;
        double t2 = t1 + halfstep;
        double t3 = t1 + step;
        
        Eigen::Vector2d p1 = final_trajectory_.getPos(t1);
        Eigen::Vector2d v1 = final_trajectory_.getVel(t1);
        Eigen::Vector2d p2 = final_trajectory_.getPos(t2);
        Eigen::Vector2d v2 = final_trajectory_.getVel(t2);
        p3 = final_trajectory_.getPos(t3);
        v3 = final_trajectory_.getVel(t3);
        
        double cosyaw1 = cos(p1.x());
        double sinyaw1 = sin(p1.x());
        double cosyaw2 = cos(p2.x());
        double sinyaw2 = sin(p2.x());
        double cosyaw3 = cos(p3.x());
        double sinyaw3 = sin(p3.x());
        
        if (config_.if_standard_diff) {
            XYTheta.x() += step1_6 * (v1.y() * cosyaw1 + 4 * v2.y() * cosyaw2 + v3.y() * cosyaw3);
            XYTheta.y() += step1_6 * (v1.y() * sinyaw1 + 4 * v2.y() * sinyaw2 + v3.y() * sinyaw3);
        } else {
            double icr_xv = config_.icr.xv;
            XYTheta.x() += step1_6 * ((v1.y() * cosyaw1 + v1.x() * icr_xv * sinyaw1) +
                                     4 * (v2.y() * cosyaw2 + v2.x() * icr_xv * sinyaw2) +
                                     (v3.y() * cosyaw3 + v3.x() * icr_xv * sinyaw3));
            XYTheta.y() += step1_6 * ((v1.y() * sinyaw1 - v1.x() * icr_xv * cosyaw1) +
                                     4 * (v2.y() * sinyaw2 - v2.x() * icr_xv * cosyaw2) +
                                     (v3.y() * sinyaw3 - v3.x() * icr_xv * cosyaw3));
        }
        
        XYTheta.z() = p3.x();
    }
    
    // 处理剩余时间
    if (left_time > 1e-6) {
        double t_start = sequence_num * step;
        Eigen::Vector2d p_start = final_trajectory_.getPos(t_start);
        Eigen::Vector2d v_start = final_trajectory_.getVel(t_start);
        Eigen::Vector2d p_end = final_trajectory_.getPos(check_time);
        Eigen::Vector2d v_end = final_trajectory_.getVel(check_time);
        
        double cosyaw_start = cos(p_start.x());
        double sinyaw_start = sin(p_start.x());
        double cosyaw_end = cos(p_end.x());
        double sinyaw_end = sin(p_end.x());
        
        if (config_.if_standard_diff) {
            XYTheta.x() += left_time / 2.0 * (v_start.y() * cosyaw_start + v_end.y() * cosyaw_end);
            XYTheta.y() += left_time / 2.0 * (v_start.y() * sinyaw_start + v_end.y() * sinyaw_end);
        } else {
            double icr_xv = config_.icr.xv;
            XYTheta.x() += left_time / 2.0 * ((v_start.y() * cosyaw_start + v_start.x() * icr_xv * sinyaw_start) +
                                             (v_end.y() * cosyaw_end + v_end.x() * icr_xv * sinyaw_end));
            XYTheta.y() += left_time / 2.0 * ((v_start.y() * sinyaw_start - v_start.x() * icr_xv * cosyaw_start) +
                                             (v_end.y() * sinyaw_end - v_end.x() * icr_xv * cosyaw_end));
        }
        
        XYTheta.z() = p_end.x();
    }
    
    // 获取最终速度、加速度、加加速度
    a3 = final_trajectory_.getAcc(check_time);
    j3 = final_trajectory_.getJer(check_time);
    
    VAJ << v3.y(), a3.y(), j3.y();
    OAJ << v3.x(), a3.x(), j3.x();
}

bool DDROptimizer::checkFinalCollision(const Trajectory<5, 2>& traj,
                                       const Eigen::Vector3d& start_XYTheta) {
    double total_time = traj.getTotalDuration();
    double dt = 0.01; // 10ms采样间隔
    int num_samples = static_cast<int>(total_time / dt) + 1;
    
    Eigen::Vector2d current_pos(start_XYTheta.x(), start_XYTheta.y());
    
    collision_points_.clear();
    bool has_collision = false;
    
    for (int i = 0; i < num_samples; i++) {
        double t = std::min(i * dt, total_time);
        
        Eigen::Vector2d sigma = traj.getPos(t);
        Eigen::Vector2d dsigma = traj.getVel(t);
        
        double yaw = sigma.x();
        double ds = dsigma.y();
        
        // 简化的位置计算
        current_pos += Eigen::Vector2d(ds * cos(yaw) * dt, ds * sin(yaw) * dt);
        
        // 检查碰撞
        Eigen::Matrix2d ego_R;
        ego_R << cos(yaw), -sin(yaw), sin(yaw), cos(yaw);
        
        for (const auto& cp_2d : config_.check_points) {
            Eigen::Vector2d check_pt = current_pos + ego_R * cp_2d;
            double sdf_value = collision_checker_->getDistance(check_pt);
            
            if (sdf_value < safe_dis_) {
                collision_points_.push_back(check_pt);
                has_collision = true;
            }
        }
        
        if (has_collision && !config_.if_visual_optimization) {
            return true; // 提前返回
        }
    }
    
    return has_collision;
}

} // namespace ddr_optimizer
