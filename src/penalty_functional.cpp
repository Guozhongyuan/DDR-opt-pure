/**
 * @file penalty_functional.cpp
 * @brief 惩罚函数实现 - 约束检查和梯度计算
 * 
 * 这是优化器最核心和最复杂的部分
 */

#include "ddr_optimizer/optimizer_pure.h"
#include <cmath>
#include <limits>

namespace ddr_optimizer {

void DDROptimizer::attachPenaltyFunctional(double& cost) {
    collision_points_.clear();
    
    double ini_x = ini_state_XYTheta_.x();
    double ini_y = ini_state_XYTheta_.y();
    
    // 辅助变量
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3;
    double s1, s2, s3, s4, s5;
    Eigen::Vector2d sigma, dsigma, ddsigma, dddsigma;
    double integral_alpha, alpha, omg, omg_step;
    
    double unoccupied_average_t = piece_time_.mean();
    
    // 代价分量
    double cost_collision = 0, cost_acc = 0, cost_domega = 0;
    double cost_moment = 0, cost_mean_time = 0, cost_cen_acc = 0;
    double cost_endpoint = 0;
    
    // 违反量和惩罚
    double viola_acc, viola_alp, viola_pos, viola_mom, viola_cen_acc;
    double viola_acc_pena, viola_alp_pena, viola_pos_pena, viola_mom_pena, viola_cen_acc_pena;
    double viola_acc_pena_d, viola_alp_pena_d, viola_pos_pena_d, viola_mom_pena_d, viola_cen_acc_pena_d;
    double grad_viola_at, grad_viola_dot, grad_viola_pt, grad_viola_mt, grad_viola_cat;
    
    double viola_vel, viola_vel_pena, viola_vel_pena_d;
    double viola_omega, viola_omega_pena, viola_omega_pena_d;
    
    Eigen::Matrix2d help_L;
    Eigen::Vector2d grad_ESDF_2d;
    
    // 存储积分位置
    std::vector<Eigen::VectorXd> vec_integral_X(traj_num_);
    std::vector<Eigen::VectorXd> vec_integral_Y(traj_num_);
    std::vector<Eigen::Vector2d> vec_traj_final_XY(traj_num_ + 1);
    vec_traj_final_XY[0] = Eigen::Vector2d(ini_x, ini_y);
    
    // 存储梯度链式法则所需的中间结果
    std::vector<Eigen::MatrixXd> vec_single_X_grad_CS(traj_num_);
    std::vector<Eigen::MatrixXd> vec_single_X_grad_C_theta(traj_num_);
    std::vector<Eigen::VectorXd> vec_single_X_grad_T(traj_num_);
    std::vector<Eigen::MatrixXd> vec_single_Y_grad_CS(traj_num_);
    std::vector<Eigen::MatrixXd> vec_single_Y_grad_C_theta(traj_num_);
    std::vector<Eigen::VectorXd> vec_single_Y_grad_T(traj_num_);
    
    // 梯度累积向量
    Eigen::VectorXd vec_coeff_chain_X(traj_num_ * (sam_num_each_part_ + 1));
    vec_coeff_chain_X.setZero();
    Eigen::VectorXd vec_coeff_chain_Y(traj_num_ * (sam_num_each_part_ + 1));
    vec_coeff_chain_Y.setZero();
    
    Eigen::Vector2d current_point_XY(ini_x, ini_y);
    
    // 遍历每个轨迹段
    for (int i = 0; i < traj_num_; i++) {
        const Eigen::Matrix<double, 6, 2>& c = minco_.getCoeffs().block<6, 2>(6 * i, 0);
        double step = piece_time_[i] / config_.sparse_resolution;
        double halfstep = step / 2.0;
        double coeff_integral = piece_time_[i] / (config_.sparse_resolution * 6.0);
        
        Eigen::MatrixXd single_X_grad_CS(6, sam_num_each_part_ + 1);
        Eigen::MatrixXd single_X_grad_C_theta(6, sam_num_each_part_ + 1);
        Eigen::VectorXd single_X_grad_T(sam_num_each_part_ + 1);
        Eigen::MatrixXd single_Y_grad_CS(6, sam_num_each_part_ + 1);
        Eigen::MatrixXd single_Y_grad_C_theta(6, sam_num_each_part_ + 1);
        Eigen::VectorXd single_Y_grad_T(sam_num_each_part_ + 1);
        
        Eigen::VectorXd integral_X(config_.sparse_resolution);
        integral_X.setZero();
        Eigen::VectorXd integral_Y(config_.sparse_resolution);
        integral_Y.setZero();
        
        s1 = 0.0;
        
        // Simpson积分采样
        for (int j = 0; j <= sam_num_each_part_; j++) {
            if (j % 2 == 0) {
                // 偶数索引点
                s2 = s1 * s1;
                s3 = s2 * s1;
                s4 = s2 * s2;
                s5 = s3 * s2;
                
                beta0 << 1.0, s1, s2, s3, s4, s5;
                beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
                beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
                beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
                
                s1 += halfstep;
                integral_alpha = 1.0 / sam_num_each_part_ * j;
                alpha = 1.0 / config_.sparse_resolution * (double(j) / 2);
                omg = (j == 0 || j == sam_num_each_part_) ? 0.5 : 1.0;
                omg_step = omg * step;
                
                sigma = c.transpose() * beta0;      // [yaw, s]
                dsigma = c.transpose() * beta1;     // [dyaw, ds]
                ddsigma = c.transpose() * beta2;    // [ddyaw, dds]
                dddsigma = c.transpose() * beta3;   // [dddyaw, ddds]
                
                double cosyaw = cos(sigma.x());
                double sinyaw = sin(sigma.x());
                
                Eigen::MatrixXd grad_beta(3, 2);
                grad_beta.setZero();
                
                // Simpson积分 - 累积位置
                if (config_.if_standard_diff) {
                    if (j != 0) {
                        integral_X[j / 2 - 1] += coeff_integral * dsigma.y() * cosyaw;
                        integral_Y[j / 2 - 1] += coeff_integral * dsigma.y() * sinyaw;
                    }
                    if (j != sam_num_each_part_) {
                        integral_X[j / 2] += coeff_integral * dsigma.y() * cosyaw;
                        integral_Y[j / 2] += coeff_integral * dsigma.y() * sinyaw;
                    }
                    
                    single_X_grad_CS.col(j) = beta1 * cosyaw;
                    single_X_grad_C_theta.col(j) = -dsigma.y() * beta0 * sinyaw;
                    single_X_grad_T[j] = (ddsigma.y() * cosyaw - dsigma.y() * dsigma.x() * sinyaw) * 
                                        integral_alpha * coeff_integral + 
                                        dsigma.y() * cosyaw / (config_.sparse_resolution * 6.0);
                    
                    single_Y_grad_CS.col(j) = beta1 * sinyaw;
                    single_Y_grad_C_theta.col(j) = dsigma.y() * beta0 * cosyaw;
                    single_Y_grad_T[j] = (ddsigma.y() * sinyaw + dsigma.y() * dsigma.x() * cosyaw) * 
                                        integral_alpha * coeff_integral + 
                                        dsigma.y() * sinyaw / (config_.sparse_resolution * 6.0);
                } else {
                    // 扩展模型（带ICR）
                    Eigen::Vector3d ICR(config_.icr.yl, config_.icr.yr, config_.icr.xv);
                    if (j != 0) {
                        integral_X[j / 2 - 1] += coeff_integral * (dsigma.y() * cosyaw + dsigma.x() * ICR.z() * sinyaw);
                        integral_Y[j / 2 - 1] += coeff_integral * (dsigma.y() * sinyaw - dsigma.x() * ICR.z() * cosyaw);
                    }
                    if (j != sam_num_each_part_) {
                        integral_X[j / 2] += coeff_integral * (dsigma.y() * cosyaw + dsigma.x() * ICR.z() * sinyaw);
                        integral_Y[j / 2] += coeff_integral * (dsigma.y() * sinyaw - dsigma.x() * ICR.z() * cosyaw);
                    }
                    
                    single_X_grad_CS.col(j) = beta1 * cosyaw;
                    single_X_grad_C_theta.col(j) = beta0 * (-dsigma.y() * sinyaw + dsigma.x() * ICR.z() * cosyaw) + 
                                                   beta1 * sinyaw * ICR.z();
                    single_X_grad_T[j] = (ddsigma.y() * cosyaw - dsigma.y() * dsigma.x() * sinyaw + 
                                         ddsigma.x() * ICR.z() * sinyaw + dsigma.x() * dsigma.x() * ICR.z() * cosyaw) * 
                                        integral_alpha * coeff_integral + 
                                        (dsigma.y() * cosyaw + dsigma.x() * ICR.z() * sinyaw) / (config_.sparse_resolution * 6.0);
                    
                    single_Y_grad_CS.col(j) = beta1 * sinyaw;
                    single_Y_grad_C_theta.col(j) = beta0 * (dsigma.y() * cosyaw - dsigma.x() * ICR.z() * sinyaw) - 
                                                   beta1 * cosyaw * ICR.z();
                    single_Y_grad_T[j] = (ddsigma.y() * sinyaw + dsigma.y() * dsigma.x() * cosyaw - 
                                         ddsigma.x() * ICR.z() * cosyaw + dsigma.x() * dsigma.x() * ICR.z() * sinyaw) * 
                                        integral_alpha * coeff_integral + 
                                        (dsigma.y() * sinyaw - dsigma.x() * ICR.z() * cosyaw) / (config_.sparse_resolution * 6.0);
                }
                
                // 检查加速度约束
                viola_acc = ddsigma.y() * ddsigma.y() - config_.kinematic.max_acc * config_.kinematic.max_acc;
                if (viola_acc > 0) {
                    positiveSmoothedL1(viola_acc, viola_acc_pena, viola_acc_pena_d);
                    grad_viola_at = 2.0 * alpha * ddsigma.y() * dddsigma.y();
                    grad_beta(2, 1) += omg_step * config_.penalty.acc_weight * viola_acc_pena_d * 2.0 * ddsigma.y();
                    partial_grad_by_times_(i) += omg * config_.penalty.acc_weight * 
                                                (viola_acc_pena_d * grad_viola_at * step + viola_acc_pena / config_.sparse_resolution);
                    cost += omg_step * config_.penalty.acc_weight * viola_acc_pena;
                    cost_acc += omg_step * config_.penalty.acc_weight * viola_acc_pena;
                }
                
                // 检查角加速度约束
                viola_alp = ddsigma.x() * ddsigma.x() - config_.kinematic.max_domega * config_.kinematic.max_domega;
                if (viola_alp > 0) {
                    positiveSmoothedL1(viola_alp, viola_alp_pena, viola_alp_pena_d);
                    grad_viola_dot = 2.0 * alpha * ddsigma.x() * dddsigma.x();
                    grad_beta(2, 0) += omg_step * config_.penalty.domega_weight * viola_alp_pena_d * 2.0 * ddsigma.x();
                    partial_grad_by_times_(i) += omg * config_.penalty.domega_weight * 
                                                (viola_alp_pena_d * grad_viola_dot * step + viola_alp_pena / config_.sparse_resolution);
                    cost += omg_step * config_.penalty.domega_weight * viola_alp_pena;
                    cost_domega += omg_step * config_.penalty.domega_weight * viola_alp_pena;
                }
                
                // 检查速度-角速度约束（力矩约束）
                if (config_.kinematic.directly_constrain_v_omega) {
                    // 直接约束速度和角速度
                    viola_vel = dsigma.y() * dsigma.y() - config_.kinematic.max_vel * config_.kinematic.max_vel;
                    if (viola_vel > 0) {
                        positiveSmoothedL1(viola_vel, viola_vel_pena, viola_vel_pena_d);
                        grad_viola_pt = 2.0 * alpha * dsigma.y() * ddsigma.y();
                        grad_beta(1, 1) += omg_step * config_.penalty.moment_weight * viola_vel_pena_d * 2.0 * dsigma.y();
                        partial_grad_by_times_(i) += omg * config_.penalty.moment_weight * 
                                                    (viola_vel_pena_d * grad_viola_pt * step + viola_vel_pena / config_.sparse_resolution);
                        cost += omg_step * config_.penalty.moment_weight * viola_vel_pena;
                        cost_moment += omg_step * config_.penalty.moment_weight * viola_vel_pena;
                    }
                    
                    viola_omega = dsigma.x() * dsigma.x() - config_.kinematic.max_omega * config_.kinematic.max_omega;
                    if (viola_omega > 0) {
                        positiveSmoothedL1(viola_omega, viola_omega_pena, viola_omega_pena_d);
                        grad_viola_pt = 2.0 * alpha * dsigma.x() * ddsigma.x();
                        grad_beta(1, 0) += omg_step * config_.penalty.moment_weight * viola_omega_pena_d * 2.0 * dsigma.x();
                        partial_grad_by_times_(i) += omg * config_.penalty.moment_weight * 
                                                    (viola_omega_pena_d * grad_viola_pt * step + viola_omega_pena / config_.sparse_resolution);
                        cost += omg_step * config_.penalty.moment_weight * viola_omega_pena;
                        cost_moment += omg_step * config_.penalty.moment_weight * viola_omega_pena;
                    }
                } else {
                    // 驱动轮扭矩约束（多面体约束）
                    for (int omg_sym = -1; omg_sym <= 1; omg_sym += 2) {
                        viola_mom = omg_sym * config_.kinematic.max_vel * dsigma.x() + 
                                   config_.kinematic.max_omega * dsigma.y() - 
                                   config_.kinematic.max_vel * config_.kinematic.max_omega;
                        if (viola_mom > 0) {
                            positiveSmoothedL1(viola_mom, viola_mom_pena, viola_mom_pena_d);
                            grad_viola_mt = alpha * (omg_sym * config_.kinematic.max_vel * ddsigma.x() + 
                                                    config_.kinematic.max_omega * ddsigma.y());
                            grad_beta(1, 0) += omg_step * config_.penalty.moment_weight * viola_mom_pena_d * 
                                             omg_sym * config_.kinematic.max_vel;
                            grad_beta(1, 1) += omg_step * config_.penalty.moment_weight * viola_mom_pena_d * 
                                             config_.kinematic.max_omega;
                            partial_grad_by_times_(i) += omg * config_.penalty.moment_weight * 
                                                        (viola_mom_pena_d * grad_viola_mt * step + viola_mom_pena / config_.sparse_resolution);
                            cost += omg_step * config_.penalty.moment_weight * viola_mom_pena;
                            cost_moment += omg_step * config_.penalty.moment_weight * viola_mom_pena;
                        }
                    }
                    
                    for (int omg_sym = -1; omg_sym <= 1; omg_sym += 2) {
                        viola_mom = omg_sym * -config_.kinematic.min_vel * dsigma.x() - 
                                   config_.kinematic.max_omega * dsigma.y() + 
                                   config_.kinematic.min_vel * config_.kinematic.max_omega;
                        if (viola_mom > 0) {
                            positiveSmoothedL1(viola_mom, viola_mom_pena, viola_mom_pena_d);
                            grad_viola_mt = alpha * (omg_sym * -config_.kinematic.min_vel * ddsigma.x() - 
                                                    config_.kinematic.max_omega * ddsigma.y());
                            grad_beta(1, 0) += omg_step * config_.penalty.moment_weight * viola_mom_pena_d * 
                                             omg_sym * -config_.kinematic.min_vel;
                            grad_beta(1, 1) -= omg_step * config_.penalty.moment_weight * viola_mom_pena_d * 
                                             config_.kinematic.max_omega;
                            partial_grad_by_times_(i) += omg * config_.penalty.moment_weight * 
                                                        (viola_mom_pena_d * grad_viola_mt * step + viola_mom_pena / config_.sparse_resolution);
                            cost += omg_step * config_.penalty.moment_weight * viola_mom_pena;
                            cost_moment += omg_step * config_.penalty.moment_weight * viola_mom_pena;
                        }
                    }
                }
                
                // 检查向心加速度约束（防侧滑/翻滚）
                viola_cen_acc = dsigma.x() * dsigma.x() * dsigma.y() * dsigma.y() - 
                               config_.kinematic.max_centripetal_acc * config_.kinematic.max_centripetal_acc;
                if (viola_cen_acc > 0) {
                    positiveSmoothedL1(viola_cen_acc, viola_cen_acc_pena, viola_cen_acc_pena_d);
                    grad_viola_cat = 2.0 * alpha * (dsigma.x() * dsigma.y() * dsigma.y() * ddsigma.x() + 
                                                    dsigma.y() * dsigma.x() * dsigma.x() * ddsigma.y());
                    grad_beta(1, 0) += omg_step * config_.penalty.cen_acc_weight * viola_cen_acc_pena_d * 
                                      (2 * dsigma.x() * dsigma.y() * dsigma.y());
                    grad_beta(1, 1) += omg_step * config_.penalty.cen_acc_weight * viola_cen_acc_pena_d * 
                                      (2 * dsigma.x() * dsigma.x() * dsigma.y());
                    partial_grad_by_times_(i) += omg * config_.penalty.cen_acc_weight * 
                                                (viola_cen_acc_pena_d * grad_viola_cat * step + viola_cen_acc_pena / config_.sparse_resolution);
                    cost += omg_step * config_.penalty.cen_acc_weight * viola_cen_acc_pena;
                    cost_cen_acc += omg_step * config_.penalty.cen_acc_weight * viola_cen_acc_pena;
                }
                
                // 更新当前位置
                if (j != 0) current_point_XY += Eigen::Vector2d(integral_X[j / 2 - 1], integral_Y[j / 2 - 1]);
                
                // 碰撞检测
                Eigen::Matrix2d ego_R;
                ego_R << cosyaw, -sinyaw, sinyaw, cosyaw;
                
                bool if_collision = false;
                Eigen::Vector2d all_grad_2_pos = Eigen::Vector2d::Zero();
                
                for (const auto& cp_2d : config_.check_points) {
                    Eigen::Vector2d check_pt = current_point_XY + ego_R * cp_2d;
                    double sdf_value = collision_checker_->getDistanceWithGradient(check_pt, grad_ESDF_2d, safe_dis_);
                    viola_pos = -sdf_value + safe_dis_;
                    
                    if (viola_pos > 0.0) {
                        if_collision = true;
                        positiveSmoothedL1(viola_pos, viola_pos_pena, viola_pos_pena_d);
                        all_grad_2_pos -= omg_step * config_.penalty.collision_weight * viola_pos_pena_d * grad_ESDF_2d;
                        help_L << -sinyaw, -cosyaw, cosyaw, -sinyaw;
                        grad_viola_pt = -alpha * dsigma.x() * grad_ESDF_2d.transpose() * help_L * cp_2d;
                        
                        grad_beta(0, 0) -= omg_step * config_.penalty.collision_weight * viola_pos_pena_d * 
                                         grad_ESDF_2d.transpose() * help_L * cp_2d;
                        partial_grad_by_times_(i) += omg * config_.penalty.collision_weight * 
                                                    (viola_pos_pena_d * grad_viola_pt * step + viola_pos_pena / config_.sparse_resolution);
                        cost += omg_step * config_.penalty.collision_weight * viola_pos_pena;
                        cost_collision += omg_step * config_.penalty.collision_weight * viola_pos_pena;
                        
                        collision_points_.push_back(check_pt);
                    }
                }
                
                if (if_collision) {
                    vec_coeff_chain_X.head(i * (sam_num_each_part_ + 1) + j + 1).array() += all_grad_2_pos.x();
                    vec_coeff_chain_Y.head(i * (sam_num_each_part_ + 1) + j + 1).array() += all_grad_2_pos.y();
                }
                
                // 累积梯度
                partial_grad_by_coeffs_.block<6, 2>(i * 6, 0) += beta0 * grad_beta.row(0) + 
                                                                  beta1 * grad_beta.row(1) + 
                                                                  beta2 * grad_beta.row(2);
            } else {
                // 奇数索引点（简化处理）
                s2 = s1 * s1;
                s3 = s2 * s1;
                s4 = s2 * s2;
                s5 = s3 * s2;
                
                beta0 << 1.0, s1, s2, s3, s4, s5;
                beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
                beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
                
                s1 += halfstep;
                integral_alpha = 1.0 / sam_num_each_part_ * j;
                sigma = c.transpose() * beta0;
                dsigma = c.transpose() * beta1;
                ddsigma = c.transpose() * beta2;
                
                double cosyaw = cos(sigma.x());
                double sinyaw = sin(sigma.x());
                
                if (config_.if_standard_diff) {
                    integral_X[j / 2] += 4 * coeff_integral * dsigma.y() * cosyaw;
                    integral_Y[j / 2] += 4 * coeff_integral * dsigma.y() * sinyaw;
                    
                    single_X_grad_CS.col(j) = beta1 * cosyaw;
                    single_X_grad_C_theta.col(j) = -dsigma.y() * beta0 * sinyaw;
                    single_X_grad_T[j] = (ddsigma.y() * cosyaw - dsigma.y() * dsigma.x() * sinyaw) * 
                                        integral_alpha * coeff_integral + 
                                        dsigma.y() * cosyaw / (config_.sparse_resolution * 6.0);
                    
                    single_Y_grad_CS.col(j) = beta1 * sinyaw;
                    single_Y_grad_C_theta.col(j) = dsigma.y() * beta0 * cosyaw;
                    single_Y_grad_T[j] = (ddsigma.y() * sinyaw + dsigma.y() * dsigma.x() * cosyaw) * 
                                        integral_alpha * coeff_integral + 
                                        dsigma.y() * sinyaw / (config_.sparse_resolution * 6.0);
                } else {
                    Eigen::Vector3d ICR(config_.icr.yl, config_.icr.yr, config_.icr.xv);
                    integral_X[j / 2] += 4 * coeff_integral * (dsigma.y() * cosyaw + dsigma.x() * ICR.z() * sinyaw);
                    integral_Y[j / 2] += 4 * coeff_integral * (dsigma.y() * sinyaw - dsigma.x() * ICR.z() * cosyaw);
                    
                    single_X_grad_CS.col(j) = beta1 * cosyaw;
                    single_X_grad_C_theta.col(j) = beta0 * (-dsigma.y() * sinyaw + dsigma.x() * ICR.z() * cosyaw) + 
                                                   beta1 * sinyaw * ICR.z();
                    single_X_grad_T[j] = (ddsigma.y() * cosyaw - dsigma.y() * dsigma.x() * sinyaw + 
                                         ddsigma.x() * ICR.z() * sinyaw + dsigma.x() * dsigma.x() * ICR.z() * cosyaw) * 
                                        integral_alpha * coeff_integral + 
                                        (dsigma.y() * cosyaw + dsigma.x() * ICR.z() * sinyaw) / (config_.sparse_resolution * 6.0);
                    
                    single_Y_grad_CS.col(j) = beta1 * sinyaw;
                    single_Y_grad_C_theta.col(j) = beta0 * (dsigma.y() * cosyaw - dsigma.x() * ICR.z() * sinyaw) - 
                                                   beta1 * cosyaw * ICR.z();
                    single_Y_grad_T[j] = (ddsigma.y() * sinyaw + dsigma.y() * dsigma.x() * cosyaw - 
                                         ddsigma.x() * ICR.z() * cosyaw + dsigma.x() * dsigma.x() * ICR.z() * sinyaw) * 
                                        integral_alpha * coeff_integral + 
                                        (dsigma.y() * sinyaw - dsigma.x() * ICR.z() * cosyaw) / (config_.sparse_resolution * 6.0);
                }
            }
        }
        
        // 时间分布均衡约束
        if (piece_time_[i] < unoccupied_average_t * config_.mean_time_lower_bound) {
            double diff = piece_time_[i] - unoccupied_average_t * config_.mean_time_lower_bound;
            cost += config_.penalty.mean_time_weight * diff * diff;
            cost_mean_time += config_.penalty.mean_time_weight * diff * diff;
            partial_grad_by_times_.array() += config_.penalty.mean_time_weight * 2.0 * diff * 
                                             (-config_.mean_time_lower_bound / traj_num_);
            partial_grad_by_times_(i) += config_.penalty.mean_time_weight * 2.0 * diff;
        }
        if (piece_time_[i] > unoccupied_average_t * config_.mean_time_upper_bound) {
            double diff = piece_time_[i] - unoccupied_average_t * config_.mean_time_upper_bound;
            cost += config_.penalty.mean_time_weight * diff * diff;
            cost_mean_time += config_.penalty.mean_time_weight * diff * diff;
            partial_grad_by_times_.array() += config_.penalty.mean_time_weight * 2.0 * diff * 
                                             (-config_.mean_time_upper_bound / traj_num_);
            partial_grad_by_times_(i) += config_.penalty.mean_time_weight * 2.0 * diff;
        }
        
        // 保存结果
        vec_integral_X[i] = integral_X;
        vec_integral_Y[i] = integral_Y;
        vec_traj_final_XY[i + 1] = vec_traj_final_XY[i] + Eigen::Vector2d(integral_X.sum(), integral_Y.sum());
        vec_single_X_grad_CS[i] = single_X_grad_CS * coeff_integral;
        vec_single_X_grad_C_theta[i] = single_X_grad_C_theta * coeff_integral;
        vec_single_X_grad_T[i] = single_X_grad_T;
        vec_single_Y_grad_CS[i] = single_Y_grad_CS * coeff_integral;
        vec_single_Y_grad_C_theta[i] = single_Y_grad_C_theta * coeff_integral;
        vec_single_Y_grad_T[i] = single_Y_grad_T;
    }
    
    // 终点约束（增广拉格朗日方法）
    final_integral_XY_error_ = vec_traj_final_XY.back() - fin_state_XYTheta_.head(2);
    
    cost_endpoint = 0.5 * (equal_rho_[0] * pow(final_integral_XY_error_.x() + equal_lambda_[0] / equal_rho_[0], 2) +
                          equal_rho_[1] * pow(final_integral_XY_error_.y() + equal_lambda_[1] / equal_rho_[1], 2));
    
    if (if_print_) {
        log("Final position error: " + std::to_string(final_integral_XY_error_.norm()));
        log("Cost breakdown - Collision: " + std::to_string(cost_collision) + 
           ", Acc: " + std::to_string(cost_acc) + 
           ", Domega: " + std::to_string(cost_domega) + 
           ", Moment: " + std::to_string(cost_moment) + 
           ", Cen_acc: " + std::to_string(cost_cen_acc) + 
           ", MeanT: " + std::to_string(cost_mean_time) + 
           ", Endpoint: " + std::to_string(cost_endpoint));
    }
    
    vec_coeff_chain_X.array() += equal_rho_[0] * (final_integral_XY_error_.x() + equal_lambda_[0] / equal_rho_[0]);
    vec_coeff_chain_Y.array() += equal_rho_[1] * (final_integral_XY_error_.y() + equal_lambda_[1] / equal_rho_[1]);
    
    // 梯度传播（链式法则）
    for (int i = 0; i < traj_num_; i++) {
        Eigen::VectorXd coeff_X = vec_coeff_chain_X.segment(i * (sam_num_each_part_ + 1), sam_num_each_part_ + 1).cwiseProduct(integral_chain_coeff_);
        Eigen::VectorXd coeff_Y = vec_coeff_chain_Y.segment(i * (sam_num_each_part_ + 1), sam_num_each_part_ + 1).cwiseProduct(integral_chain_coeff_);
        
        partial_grad_by_coeffs_.block<6, 1>(i * 6, 1) += vec_single_X_grad_CS[i] * coeff_X;
        partial_grad_by_coeffs_.block<6, 1>(i * 6, 0) += vec_single_X_grad_C_theta[i] * coeff_X;
        partial_grad_by_coeffs_.block<6, 1>(i * 6, 1) += vec_single_Y_grad_CS[i] * coeff_Y;
        partial_grad_by_coeffs_.block<6, 1>(i * 6, 0) += vec_single_Y_grad_C_theta[i] * coeff_Y;
        
        partial_grad_by_times_(i) += (vec_single_X_grad_T[i].cwiseProduct(coeff_X)).sum();
        partial_grad_by_times_(i) += (vec_single_Y_grad_T[i].cwiseProduct(coeff_Y)).sum();
    }
    
    // 累加所有代价
    cost += cost_collision + cost_acc + cost_domega + cost_moment + cost_cen_acc + cost_mean_time + cost_endpoint;
}

} // namespace ddr_optimizer

