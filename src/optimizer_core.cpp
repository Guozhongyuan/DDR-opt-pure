/**
 * @file optimizer_core.cpp
 * @brief DDR优化器核心函数实现 - 代价函数和梯度计算
 * 
 * 本文件包含最核心的优化逻辑
 */

#include "ddr_optimizer/optimizer_pure.h"
#include <cmath>
#include <limits>

#define inf (1 << 30)

namespace ddr_optimizer {

// ========== L-BFGS回调函数实现 ==========

double DDROptimizer::costFunctionCallback(void* ptr,
                                         const Eigen::VectorXd& x,
                                         Eigen::VectorXd& g) {
    if (x.norm() > 1e4) return inf;
    
    DDROptimizer& obj = *(DDROptimizer*)ptr;
    obj.iter_num_++;
    
    g.setZero();
    
    // 从优化变量中提取参数
    int offset = 0;
    Eigen::Map<const Eigen::MatrixXd> P(x.data() + offset, 2, obj.traj_num_ - 1);
    Eigen::Map<Eigen::MatrixXd> gradP(g.data() + offset, 2, obj.traj_num_ - 1);
    offset += 2 * (obj.traj_num_ - 1);
    
    double* gradTailS = g.data() + offset;
    obj.fin_state_(1, 0) = x[offset];
    ++offset;
    
    gradP.setZero();
    obj.inner_points_ = P;
    
    Eigen::Map<const Eigen::VectorXd> t(x.data() + offset, obj.traj_num_);
    Eigen::Map<Eigen::VectorXd> gradt(g.data() + offset, obj.traj_num_);
    offset += obj.traj_num_;
    
    obj.virtualT2RealT(t, obj.piece_time_);
    gradt.setZero();
    
    // 计算能量代价
    double cost;
    obj.minco_.setTConditions(obj.fin_state_);
    obj.minco_.setParameters(obj.inner_points_, obj.piece_time_);
    obj.minco_.getEnergy(cost);
    obj.minco_.getEnergyPartialGradByCoeffs(obj.partial_grad_by_coeffs_);
    obj.minco_.getEnergyPartialGradByTimes(obj.partial_grad_by_times_);
    
    if (obj.if_print_) {
        obj.log("Energy cost: " + std::to_string(cost));
    }
    
    // 添加惩罚项
    obj.attachPenaltyFunctional(cost);
    
    if (obj.if_print_) {
        obj.log("Total cost after penalty: " + std::to_string(cost));
    }
    
    // 传播梯度
    obj.minco_.propogateArcYawLenghGrad(
        obj.partial_grad_by_coeffs_, obj.partial_grad_by_times_,
        obj.grad_by_points_, obj.grad_by_times_, obj.grad_by_tail_state_s_
    );
    
    // 添加时间代价
    cost += obj.config_.penalty.time_weight * obj.piece_time_.sum();
    if (obj.if_print_) {
        obj.log("Time cost: " + std::to_string(obj.config_.penalty.time_weight * obj.piece_time_.sum()));
    }
    
    Eigen::VectorXd rho_times(obj.grad_by_times_.size());
    obj.grad_by_times_ += obj.config_.penalty.time_weight * rho_times.setOnes();
    
    // 填充梯度
    *gradTailS = obj.grad_by_tail_state_s_.y();
    gradP = obj.grad_by_points_;
    backwardGradT(t, obj.grad_by_times_, gradt);
    
    return cost;
}

double DDROptimizer::costFunctionCallbackPath(void* ptr,
                                              const Eigen::VectorXd& x,
                                              Eigen::VectorXd& g) {
    if (x.norm() > 1e4) return inf;
    
    DDROptimizer& obj = *(DDROptimizer*)ptr;
    obj.iter_num_++;
    
    g.setZero();
    
    int offset = 0;
    Eigen::Map<const Eigen::MatrixXd> P(x.data() + offset, 2, obj.traj_num_ - 1);
    Eigen::Map<Eigen::MatrixXd> gradP(g.data() + offset, 2, obj.traj_num_ - 1);
    offset += 2 * (obj.traj_num_ - 1);
    
    double* gradTailS = g.data() + offset;
    obj.fin_state_(1, 0) = x[offset];
    ++offset;
    
    gradP.setZero();
    obj.inner_points_ = P;
    
    Eigen::Map<const Eigen::VectorXd> t(x.data() + offset, obj.traj_num_);
    Eigen::Map<Eigen::VectorXd> gradt(g.data() + offset, obj.traj_num_);
    offset += obj.traj_num_;
    
    obj.virtualT2RealT(t, obj.piece_time_);
    gradt.setZero();
    
    double cost;
    obj.minco_.setTConditions(obj.fin_state_);
    obj.minco_.setParameters(obj.inner_points_, obj.piece_time_);
    obj.minco_.getEnergy(cost);
    obj.minco_.getEnergyPartialGradByCoeffs(obj.partial_grad_by_coeffs_);
    obj.minco_.getEnergyPartialGradByTimes(obj.partial_grad_by_times_);
    
    // 预处理阶段的惩罚项
    obj.attachPenaltyFunctionalPath(cost);
    
    obj.minco_.propogateArcYawLenghGrad(
        obj.partial_grad_by_coeffs_, obj.partial_grad_by_times_,
        obj.grad_by_points_, obj.grad_by_times_, obj.grad_by_tail_state_s_
    );
    
    *gradTailS = obj.grad_by_tail_state_s_.y();
    
    cost += obj.config_.path_penalty.time_weight * obj.piece_time_.sum();
    
    Eigen::VectorXd rho_times(obj.grad_by_times_.size());
    obj.grad_by_times_ += obj.config_.path_penalty.time_weight * rho_times.setOnes();
    
    gradP = obj.grad_by_points_;
    backwardGradT(t, obj.grad_by_times_, gradt);
    
    return cost;
}

int DDROptimizer::earlyExit(void* instance,
                           const Eigen::VectorXd& x,
                           const Eigen::VectorXd& g,
                           const double fx,
                           const double step,
                           const int k,
                           const int ls) {
    DDROptimizer& obj = *(DDROptimizer*)instance;
    obj.final_integral_XY_error_backup_ = obj.final_integral_XY_error_;
    obj.collision_points_backup_ = obj.collision_points_;
    
    if (obj.config_.if_visual_optimization) {
        obj.log("Iteration " + std::to_string(k) + 
               ": cost=" + std::to_string(fx) + 
               ", step=" + std::to_string(step) +
               ", ls=" + std::to_string(ls));
    }
    
    return 0;
}

} // namespace ddr_optimizer

