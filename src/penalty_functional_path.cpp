/**
 * @file penalty_functional_path.cpp  
 * @brief 预处理阶段的惩罚函数实现
 */

#include "ddr_optimizer/optimizer_pure.h"
#include <cmath>

namespace ddr_optimizer {

void DDROptimizer::attachPenaltyFunctionalPath(double& cost) {
    double ini_x = ini_state_XYTheta_.x();
    double ini_y = ini_state_XYTheta_.y();
    
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3;
    double s1, s2, s3, s4, s5;
    Eigen::Vector2d sigma, dsigma, ddsigma, dddsigma;
    double integral_alpha, omg;
    
    double unoccupied_average_t = piece_time_.mean();
    
    double cost_bp = 0, cost_moment = 0, cost_mean_time = 0;
    
    double viola_mom, viola_mom_pena, viola_mom_pena_d;
    
    Eigen::VectorXd integral_chain_coeff(sam_num_each_part_ + 1);
    integral_chain_coeff.setZero();
    for (int i = 0; i < config_.sparse_resolution; i++) {
        integral_chain_coeff.segment(2 * i, 3) += Eigen::Vector3d(1.0, 4.0, 1.0);
    }
    
    std::vector<Eigen::VectorXd> vec_integral_X(traj_num_);
    std::vector<Eigen::VectorXd> vec_integral_Y(traj_num_);
    std::vector<Eigen::Vector2d> vec_traj_final_XY(traj_num_ + 1);
    vec_traj_final_XY[0] = Eigen::Vector2d(ini_x, ini_y);
    
    std::vector<Eigen::MatrixXd> vec_single_X_grad_CS(traj_num_);
    std::vector<Eigen::MatrixXd> vec_single_X_grad_C_theta(traj_num_);
    std::vector<Eigen::VectorXd> vec_single_X_grad_T(traj_num_);
    std::vector<Eigen::MatrixXd> vec_single_Y_grad_CS(traj_num_);
    std::vector<Eigen::MatrixXd> vec_single_Y_grad_C_theta(traj_num_);
    std::vector<Eigen::VectorXd> vec_single_Y_grad_T(traj_num_);
    
    Eigen::VectorXd vec_coeff_chain_X(traj_num_ * (sam_num_each_part_ + 1));
    vec_coeff_chain_X.setZero();
    Eigen::VectorXd vec_coeff_chain_Y(traj_num_ * (sam_num_each_part_ + 1));
    vec_coeff_chain_Y.setZero();
    
    for (int i = 0; i < traj_num_; i++) {
        const Eigen::Matrix<double, 6, 2>& c = minco_.getCoeffs().block<6, 2>(6 * i, 0);
        double step = piece_time_[i] / config_.sparse_resolution;
        double halfstep = step / 2;
        double coeff_integral = piece_time_[i] / config_.sparse_resolution / 6;
        
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
        
        for (int j = 0; j <= sam_num_each_part_; j++) {
            if (j % 2 == 0) {
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
                omg = (j == 0 || j == sam_num_each_part_) ? 0.5 : 1.0;
                
                sigma = c.transpose() * beta0;
                dsigma = c.transpose() * beta1;
                ddsigma = c.transpose() * beta2;
                dddsigma = c.transpose() * beta3;
                
                double cosyaw = cos(sigma.x());
                double sinyaw = sin(sigma.x());
                
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
                
                // 力矩约束（简化版本）
                double alpha = 1.0 / config_.sparse_resolution * (double(j) / 2);
                Eigen::MatrixXd grad_beta(3, 2);
                grad_beta.setZero();
                
                double grad_viola_mt;
                for (int omg_sym = -1; omg_sym <= 1; omg_sym += 2) {
                    viola_mom = omg_sym * config_.kinematic.max_vel * dsigma.x() + 
                               config_.kinematic.max_omega * dsigma.y() - 
                               config_.kinematic.max_vel * config_.kinematic.max_omega;
                    if (viola_mom > 0) {
                        positiveSmoothedL1(viola_mom, viola_mom_pena, viola_mom_pena_d);
                        grad_viola_mt = alpha * (omg_sym * config_.kinematic.max_vel * ddsigma.x() + 
                                                config_.kinematic.max_omega * ddsigma.y());
                        grad_beta(1, 0) += omg * step * config_.path_penalty.moment_weight * viola_mom_pena_d * 
                                         omg_sym * config_.kinematic.max_vel;
                        grad_beta(1, 1) += omg * step * config_.path_penalty.moment_weight * viola_mom_pena_d * 
                                         config_.kinematic.max_omega;
                        partial_grad_by_times_(i) += omg * config_.path_penalty.moment_weight * 
                                                    (viola_mom_pena_d * grad_viola_mt * step + viola_mom_pena / config_.sparse_resolution);
                        cost += omg * step * config_.path_penalty.moment_weight * viola_mom_pena;
                        cost_moment += omg * step * config_.path_penalty.moment_weight * viola_mom_pena;
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
                        grad_beta(1, 0) += omg * step * config_.path_penalty.moment_weight * viola_mom_pena_d * 
                                         omg_sym * -config_.kinematic.min_vel;
                        grad_beta(1, 1) -= omg * step * config_.path_penalty.moment_weight * viola_mom_pena_d * 
                                         config_.kinematic.max_omega;
                        partial_grad_by_times_(i) += omg * config_.path_penalty.moment_weight * 
                                                    (viola_mom_pena_d * grad_viola_mt * step + viola_mom_pena / config_.sparse_resolution);
                        cost += omg * step * config_.path_penalty.moment_weight * viola_mom_pena;
                        cost_moment += omg * step * config_.path_penalty.moment_weight * viola_mom_pena;
                    }
                }
                
                // 加速度和角加速度约束
                double viola_acc = ddsigma.y() * ddsigma.y() - config_.kinematic.max_acc * config_.kinematic.max_acc;
                double viola_alp = ddsigma.x() * ddsigma.x() - config_.kinematic.max_domega * config_.kinematic.max_domega;
                double viola_acc_pena, viola_acc_pena_d, viola_alp_pena, viola_alp_pena_d;
                
                if (viola_acc > 0) {
                    positiveSmoothedL1(viola_acc, viola_acc_pena, viola_acc_pena_d);
                    double grad_viola_at = 2.0 * alpha * ddsigma.y() * dddsigma.y();
                    grad_beta(2, 1) += omg * step * config_.path_penalty.acc_weight * viola_acc_pena_d * 2.0 * ddsigma.y();
                    partial_grad_by_times_(i) += omg * config_.path_penalty.acc_weight * 
                                                (viola_acc_pena_d * grad_viola_at * step + viola_acc_pena / config_.sparse_resolution);
                    cost += omg * step * config_.path_penalty.acc_weight * viola_acc_pena;
                }
                
                if (viola_alp > 0) {
                    positiveSmoothedL1(viola_alp, viola_alp_pena, viola_alp_pena_d);
                    double grad_viola_dot = 2.0 * alpha * ddsigma.x() * dddsigma.x();
                    grad_beta(2, 0) += omg * step * config_.path_penalty.domega_weight * viola_alp_pena_d * 2.0 * ddsigma.x();
                    partial_grad_by_times_(i) += omg * config_.path_penalty.domega_weight * 
                                                (viola_alp_pena_d * grad_viola_dot * step + viola_alp_pena / config_.sparse_resolution);
                    cost += omg * step * config_.path_penalty.domega_weight * viola_alp_pena;
                }
                
                partial_grad_by_coeffs_.block<6, 2>(i * 6, 0) += beta0 * grad_beta.row(0) + 
                                                                  beta1 * grad_beta.row(1) + 
                                                                  beta2 * grad_beta.row(2);
            } else {
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
        
        vec_integral_X[i] = integral_X;
        vec_integral_Y[i] = integral_Y;
        vec_traj_final_XY[i + 1] = vec_traj_final_XY[i] + Eigen::Vector2d(integral_X.sum(), integral_Y.sum());
        vec_single_X_grad_CS[i] = single_X_grad_CS * coeff_integral;
        vec_single_X_grad_C_theta[i] = single_X_grad_C_theta * coeff_integral;
        vec_single_X_grad_T[i] = single_X_grad_T;
        vec_single_Y_grad_CS[i] = single_Y_grad_CS * coeff_integral;
        vec_single_Y_grad_C_theta[i] = single_Y_grad_C_theta * coeff_integral;
        vec_single_Y_grad_T[i] = single_Y_grad_T;
        
        // 时间分布约束
        if (piece_time_[i] < unoccupied_average_t * config_.mean_time_lower_bound) {
            double diff = piece_time_[i] - unoccupied_average_t * config_.mean_time_lower_bound;
            cost += config_.path_penalty.mean_time_weight * diff * diff;
            cost_mean_time += config_.path_penalty.mean_time_weight * diff * diff;
            partial_grad_by_times_.array() += config_.path_penalty.mean_time_weight * 2.0 * diff * 
                                             (-config_.mean_time_lower_bound / traj_num_);
            partial_grad_by_times_(i) += config_.path_penalty.mean_time_weight * 2.0 * diff;
        }
        if (piece_time_[i] > unoccupied_average_t * config_.mean_time_upper_bound) {
            double diff = piece_time_[i] - unoccupied_average_t * config_.mean_time_upper_bound;
            cost += config_.path_penalty.mean_time_weight * diff * diff;
            cost_mean_time += config_.path_penalty.mean_time_weight * diff * diff;
            partial_grad_by_times_.array() += config_.path_penalty.mean_time_weight * 2.0 * diff * 
                                             (-config_.mean_time_upper_bound / traj_num_);
            partial_grad_by_times_(i) += config_.path_penalty.mean_time_weight * 2.0 * diff;
        }
        
        // 路径相似度约束（保持与初始路径接近）
        Eigen::Vector2d inner_point_XY = vec_traj_final_XY[i + 1];
        double viola_pos = (inner_point_XY - inner_init_positions_[i].head(2)).squaredNorm();
        vec_coeff_chain_X.head((i + 1) * (sam_num_each_part_ + 1)).array() += 
            config_.path_penalty.bigpath_sdf_weight * 2.0 * (inner_point_XY.x() - inner_init_positions_[i].x());
        vec_coeff_chain_Y.head((i + 1) * (sam_num_each_part_ + 1)).array() += 
            config_.path_penalty.bigpath_sdf_weight * 2.0 * (inner_point_XY.y() - inner_init_positions_[i].y());
        cost += config_.path_penalty.bigpath_sdf_weight * viola_pos;
        cost_bp += config_.path_penalty.bigpath_sdf_weight * viola_pos;
    }
    
    if (if_print_) {
        log("Path cost breakdown - BigPath: " + std::to_string(cost_bp) + 
           ", Moment: " + std::to_string(cost_moment) + 
           ", MeanT: " + std::to_string(cost_mean_time));
    }
    
    // 梯度传播
    for (int i = 0; i < traj_num_; i++) {
        Eigen::VectorXd coeff_X = vec_coeff_chain_X.segment(i * (sam_num_each_part_ + 1), sam_num_each_part_ + 1).cwiseProduct(integral_chain_coeff);
        Eigen::VectorXd coeff_Y = vec_coeff_chain_Y.segment(i * (sam_num_each_part_ + 1), sam_num_each_part_ + 1).cwiseProduct(integral_chain_coeff);
        
        partial_grad_by_coeffs_.block<6, 1>(i * 6, 1) += vec_single_X_grad_CS[i] * coeff_X;
        partial_grad_by_coeffs_.block<6, 1>(i * 6, 0) += vec_single_X_grad_C_theta[i] * coeff_X;
        partial_grad_by_coeffs_.block<6, 1>(i * 6, 1) += vec_single_Y_grad_CS[i] * coeff_Y;
        partial_grad_by_coeffs_.block<6, 1>(i * 6, 0) += vec_single_Y_grad_C_theta[i] * coeff_Y;
        
        partial_grad_by_times_(i) += (vec_single_X_grad_T[i].cwiseProduct(coeff_X)).sum();
        partial_grad_by_times_(i) += (vec_single_Y_grad_T[i].cwiseProduct(coeff_Y)).sum();
    }
}

} // namespace ddr_optimizer

