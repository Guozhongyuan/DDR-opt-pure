#include "ddr_optimizer/optimizer_pure.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <limits>
#include <chrono>

#define inf (1 << 30)

namespace ddr_optimizer {

DDROptimizer::DDROptimizer(const OptimizerConfig& config,
                           std::shared_ptr<ICollisionChecker> collision_checker)
    : config_(config), collision_checker_(collision_checker) {
    
    // 初始化辅助变量
    sam_num_each_part_ = 2 * config_.sparse_resolution;
    integral_chain_coeff_.resize(sam_num_each_part_ + 1);
    integral_chain_coeff_.setZero();
    
    // Simpson积分系数
    for (int i = 0; i < config_.sparse_resolution; i++) {
        integral_chain_coeff_.segment(2 * i, 3) += Eigen::Vector3d(1.0, 4.0, 1.0);
    }
    
    safe_dis_ = config_.safe_distance;
    smooth_eps_ = config_.smooth_eps;
    iter_num_ = 0;
    if_print_ = false;
    traj_num_ = 0;
    if_cut_traj_ = false;
    
    // 初始化ALM参数
    equal_lambda_ = Eigen::Vector2d::Zero();
    equal_rho_ = Eigen::Vector2d::Zero();
    
    final_integral_XY_error_ = Eigen::Vector2d::Zero();
    final_integral_XY_error_backup_ = Eigen::Vector2d::Zero();
    grad_by_tail_state_s_ = Eigen::Vector2d::Zero();
}

DDROptimizer::~DDROptimizer() {
}

void DDROptimizer::updateConfig(const OptimizerConfig& config) {
    config_ = config;
    safe_dis_ = config_.safe_distance;
    
    // 重新初始化Simpson积分系数
    sam_num_each_part_ = 2 * config_.sparse_resolution;
    integral_chain_coeff_.resize(sam_num_each_part_ + 1);
    integral_chain_coeff_.setZero();
    for (int i = 0; i < config_.sparse_resolution; i++) {
        integral_chain_coeff_.segment(2 * i, 3) += Eigen::Vector3d(1.0, 4.0, 1.0);
    }
}

void DDROptimizer::updateCollisionChecker(std::shared_ptr<ICollisionChecker> collision_checker) {
    collision_checker_ = collision_checker;
}

bool DDROptimizer::optimize(const TrajectoryInput& input, TrajectoryOutput& output) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // 验证输入
    if (!input.validate()) {
        output.success = false;
        output.message = "Invalid input";
        return false;
    }
    
    logInfo("========== Starting DDR Optimization ==========");
    
    // 获取初始状态
    if (!getInitialState(input)) {
        output.success = false;
        output.message = "Failed to get initial state";
        return false;
    }
    
    // 执行优化（包含碰撞检查和重规划）
    bool final_collision = false;
    int replan_num = 0;
    
    double start_safe_dis = safe_dis_;
    
    for (; replan_num < config_.safe_replan_max_time; replan_num++) {
        if (!runOptimization()) {
            output.success = false;
            output.message = "Optimization failed";
            return false;
        }
        
        minco_.getTrajectory(optimizer_trajectory_);
        final_collision = checkFinalCollision(optimizer_trajectory_, ini_state_XYTheta_);
        
        if (final_collision) {
            logWarn("Final collision detected, replanning...");
            config_.penalty.time_weight *= 0.75;
        } else {
            break;
        }
    }
    
    // 恢复配置
    config_.penalty.time_weight = config_.penalty.time_weight_backup;
    safe_dis_ = start_safe_dis;
    
    if (replan_num == config_.safe_replan_max_time) {
        logError("Failed to find collision-free trajectory after max replans");
        output.success = false;
        output.message = "Final trajectory has collision";
        return false;
    }
    
    // 保存结果
    final_trajectory_ = optimizer_trajectory_;
    final_ini_state_XYTheta_ = ini_state_XYTheta_;
    final_fin_state_XYTheta_ = fin_state_XYTheta_;
    
    // 填充输出
    output.trajectory = final_trajectory_;
    output.initial_trajectory = init_final_trajectory_;
    output.num_segments = traj_num_;
    output.segment_durations = final_piece_time_;
    output.control_points = final_inner_points_;
    output.start_state_XYTheta = final_ini_state_XYTheta_;
    output.final_state_XYTheta = final_fin_state_XYTheta_;
    output.success = true;
    output.message = "Optimization successful";
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
    output.total_time_ms = duration / 1000.0;
    
    stats_.success = true;
    stats_.total_time_ms = output.total_time_ms;
    stats_.message = "Optimization successful with " + std::to_string(replan_num + 1) + " attempts";
    
    logInfo("========== Optimization Complete ==========");
    logInfo("Total time: " + std::to_string(output.total_time_ms) + " ms");
    logInfo("Attempts: " + std::to_string(replan_num + 1));
    
    return true;
}

bool DDROptimizer::getInitialState(const TrajectoryInput& input) {
    if_cut_traj_ = input.is_cut;
    
    traj_num_ = input.waypoints.size() + 1;
    
    inner_points_.resize(2, traj_num_ - 1);
    for (size_t i = 0; i < input.waypoints.size(); i++) {
        inner_points_.col(i) = input.waypoints[i].head(2);  // yaw, s
    }
    
    inner_init_positions_ = input.waypoint_positions;
    inner_init_positions_.push_back(input.final_state_XYTheta);
    
    ini_state_ = input.start_state;
    fin_state_ = input.final_state;
    
    piece_time_.resize(traj_num_);
    piece_time_.setOnes();
    piece_time_ *= input.initial_segment_time;
    
    ini_state_XYTheta_ = input.start_state_XYTheta;
    fin_state_XYTheta_ = input.final_state_XYTheta;
    
    log("Initial state obtained: " + std::to_string(traj_num_) + " segments");
    
    return true;
}

bool DDROptimizer::runOptimization() {
    // 初始化ALM参数
    if (!if_cut_traj_) {
        equal_lambda_ = config_.alm.init_lambda;
        equal_rho_ = config_.alm.init_rho;
    } else {
        equal_lambda_ = config_.cut_alm.init_lambda;
        equal_rho_ = config_.cut_alm.init_rho;
    }
    
    // 设置变量数量: 2*(N-1)个内点 + 1个松弛s + N个时间
    int variable_num = 3 * traj_num_ - 1;
    
    // 设置MINCO初始条件
    minco_.setConditions(ini_state_, fin_state_, traj_num_, config_.energy_weights);
    minco_.setParameters(inner_points_, piece_time_);
    minco_.getTrajectory(init_final_trajectory_);
    
    // 构造优化变量向量
    Eigen::VectorXd x(variable_num);
    int offset = 0;
    
    // 复制内点
    memcpy(x.data() + offset, inner_points_.data(), inner_points_.size() * sizeof(double));
    offset += inner_points_.size();
    
    // 终点s分量（松弛变量）
    x[offset] = fin_state_(1, 0);
    ++offset;
    
    // 时间变量（虚拟时间）
    Eigen::Map<Eigen::VectorXd> Vt(x.data() + offset, piece_time_.size());
    realT2VirtualT(piece_time_, Vt);
    
    // ========== 第一阶段：预处理优化 ==========
    double cost;
    iter_num_ = 0;
    if_print_ = false;
    
    auto start = std::chrono::high_resolution_clock::now();
    
    // 设置预处理L-BFGS参数
    lbfgs::lbfgs_parameter_t path_lbfgs_params;
    path_lbfgs_params.mem_size = config_.path_lbfgs.mem_size;
    path_lbfgs_params.g_epsilon = config_.path_lbfgs.g_epsilon;
    path_lbfgs_params.min_step = config_.path_lbfgs.min_step;
    path_lbfgs_params.delta = config_.path_lbfgs.delta;
    path_lbfgs_params.max_iterations = config_.path_lbfgs.max_iterations;
    
    // 根据路径长度调整past参数
    if (fabs(fin_state_(1, 0)) < config_.path_lbfgs.shot_path_horizon) {
        path_lbfgs_params.past = config_.path_lbfgs.shot_path_past;
    } else {
        path_lbfgs_params.past = config_.path_lbfgs.normal_past;
    }
    
    logInfo("Starting pre-processing optimization...");
    int result = lbfgs::lbfgs_optimize(
        x, cost,
        DDROptimizer::costFunctionCallbackPath,
        NULL, NULL,
        this, path_lbfgs_params
    );
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    stats_.preprocess_time_ms = duration_us / 1000.0;
    stats_.preprocess_iterations = iter_num_;
    
    logInfo("Pre-processing: " + std::to_string(stats_.preprocess_time_ms) + " ms, " + 
           std::to_string(iter_num_) + " iterations, result=" + std::to_string(result));
    
    // 提取预处理结果
    offset = 0;
    Eigen::Map<const Eigen::MatrixXd> PathP(x.data() + offset, 2, traj_num_ - 1);
    final_inner_points_ = PathP;
    offset += 2 * (traj_num_ - 1);
    
    fin_state_(1, 0) = x[offset];
    ++offset;
    
    Eigen::Map<const Eigen::VectorXd> Patht(x.data() + offset, traj_num_);
    virtualT2RealT(Patht, final_piece_time_);
    
    minco_.setTConditions(fin_state_);
    minco_.setParameters(final_inner_points_, final_piece_time_);
    
    // ========== 第二阶段：正式优化 + ALM迭代 ==========
    logInfo("Starting main optimization with ALM...");
    iter_num_ = 0;
    start = std::chrono::high_resolution_clock::now();
    
    int alm_iteration = 0;
    const int max_alm_iterations = 20;
    
    // 设置正式优化L-BFGS参数
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs_params.mem_size = config_.lbfgs.mem_size;
    lbfgs_params.past = config_.lbfgs.past;
    lbfgs_params.g_epsilon = config_.lbfgs.g_epsilon;
    lbfgs_params.min_step = config_.lbfgs.min_step;
    lbfgs_params.delta = config_.lbfgs.delta;
    lbfgs_params.max_iterations = config_.lbfgs.max_iterations;
    
    while (true) {
        // L-BFGS优化
        result = lbfgs::lbfgs_optimize(
            x, cost,
            DDROptimizer::costFunctionCallback,
            NULL,
            DDROptimizer::earlyExit,
            this, lbfgs_params
        );
        
        // 记录ALM历史
        stats_.alm_errors.push_back(final_integral_XY_error_.norm());
        stats_.alm_costs.push_back(cost);
        
        logInfo("ALM iteration " + std::to_string(alm_iteration) + 
               ": position_error=" + std::to_string(final_integral_XY_error_.norm()) + 
               ", cost=" + std::to_string(cost));
        
        // 检查收敛
        double tolerance = if_cut_traj_ ? 
            config_.cut_alm.tolerance[0] : config_.alm.tolerance[0];
        
        if (final_integral_XY_error_.norm() < tolerance) {
            logInfo("ALM converged! Final position error: " + 
                   std::to_string(final_integral_XY_error_.norm()));
            break;
        }
        
        if (++alm_iteration >= max_alm_iterations) {
            logWarn("ALM reached max iterations (" + std::to_string(max_alm_iterations) + ")");
            break;
        }
        
        // 更新ALM参数
        if (!if_cut_traj_) {
            equal_lambda_[0] += equal_rho_[0] * final_integral_XY_error_.x();
            equal_lambda_[1] += equal_rho_[1] * final_integral_XY_error_.y();
            equal_rho_[0] = std::min((1 + config_.alm.gamma[0]) * equal_rho_[0], 
                                    config_.alm.rho_max[0]);
            equal_rho_[1] = std::min((1 + config_.alm.gamma[1]) * equal_rho_[1], 
                                    config_.alm.rho_max[1]);
        } else {
            equal_lambda_[0] += equal_rho_[0] * final_integral_XY_error_.x();
            equal_lambda_[1] += equal_rho_[1] * final_integral_XY_error_.y();
            equal_rho_[0] = std::min((1 + config_.cut_alm.gamma[0]) * equal_rho_[0], 
                                    config_.cut_alm.rho_max[0]);
            equal_rho_[1] = std::min((1 + config_.cut_alm.gamma[1]) * equal_rho_[1], 
                                    config_.cut_alm.rho_max[1]);
        }
    }
    
    end = std::chrono::high_resolution_clock::now();
    duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    stats_.optimization_time_ms = duration_us / 1000.0;
    stats_.optimization_iterations = iter_num_;
    
    // 提取最终结果
    offset = 0;
    Eigen::Map<const Eigen::MatrixXd> P(x.data() + offset, 2, traj_num_ - 1);
    final_inner_points_ = P;
    offset += 2 * (traj_num_ - 1);
    
    fin_state_(1, 0) = x[offset];
    ++offset;
    
    Eigen::Map<const Eigen::VectorXd> t(x.data() + offset, traj_num_);
    virtualT2RealT(t, final_piece_time_);
    
    minco_.setTConditions(fin_state_);
    minco_.setParameters(final_inner_points_, final_piece_time_);
    
    // 记录统计信息
    stats_.final_cost = cost;
    stats_.final_position_error = final_integral_XY_error_.norm();
    stats_.total_iterations = stats_.preprocess_iterations + stats_.optimization_iterations;
    
    logInfo("Optimization complete: " + std::to_string(stats_.optimization_time_ms) + " ms, " + 
           std::to_string(stats_.optimization_iterations) + " iterations");
    logInfo("Final cost: " + std::to_string(stats_.final_cost) + 
           ", Position error: " + std::to_string(stats_.final_position_error) + " m");
    
    return true;
}

bool DDROptimizer::checkFinalCollision(const Trajectory<5, 2>& traj,
                                       const Eigen::Vector3d& start_XYTheta) {
    // TODO: 完整实现请参考 IMPLEMENTATION_GUIDE.md
    // 这是临时的桩实现
    
    logInfo("checkFinalCollision() - stub implementation, skipping collision check");
    
    // 简单检查：采样几个点
    double total_duration = traj.getTotalDuration();
    int num_checks = 10;
    double dt = total_duration / num_checks;
    
    Eigen::Vector2d current_pos = start_XYTheta.head(2);
    
    for (int i = 0; i <= num_checks; i++) {
        double t = i * dt;
        if (t > total_duration) t = total_duration;
        
        // 简化的位置计算（这里应该使用Simpson积分）
        Eigen::Vector2d sigma = traj.getPos(t);
        
        // 检查碰撞（简化版）
        if (collision_checker_) {
            double dist = collision_checker_->getDistance(current_pos);
            if (dist < config_.final_min_safe_dis) {
                logWarn("Collision detected at t=" + std::to_string(t));
                return true;  // 有碰撞
            }
        }
        
        // 更新位置（简化计算）
        Eigen::Vector2d vel = traj.getVel(t);
        current_pos.x() += vel.y() * dt * cos(sigma.x());
        current_pos.y() += vel.y() * dt * sin(sigma.x());
    }
    
    logInfo("No collision detected (simplified check)");
    return false;  // 无碰撞
}

void DDROptimizer::log(const std::string& message, bool force) {
    if (config_.verbose || force) {
        std::cout << "[DDROptimizer] " << message << std::endl;
    }
}

void DDROptimizer::logInfo(const std::string& message) {
    log("[INFO] " + message, false);
}

void DDROptimizer::logWarn(const std::string& message) {
    std::cout << "[DDROptimizer] [WARN] " << message << std::endl;
}

void DDROptimizer::logError(const std::string& message) {
    std::cerr << "[DDROptimizer] [ERROR] " << message << std::endl;
}

} // namespace ddr_optimizer

