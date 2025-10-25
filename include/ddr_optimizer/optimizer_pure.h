#ifndef DDR_OPTIMIZER_PURE_H
#define DDR_OPTIMIZER_PURE_H

#include "config.h"
#include "collision_interface.h"
#include "trajectory_data.h"
#include "gcopter/trajectory.hpp"
#include "gcopter/minco.hpp"
#include "gcopter/lbfgs.hpp"

#include <Eigen/Eigen>
#include <memory>
#include <vector>
#include <chrono>

namespace ddr_optimizer {

/**
 * @brief DDR (Differential Drive Robot) 轨迹优化器（纯净版）
 * 
 * 基于MINCO轨迹表示和L-BFGS优化的微分驱动机器人轨迹优化器
 * 完全移除了ROS依赖，可独立使用
 */
class DDROptimizer {
public:
    /**
     * @brief 构造函数
     * @param config 优化器配置
     * @param collision_checker 碰撞检测器
     */
    DDROptimizer(const OptimizerConfig& config,
                 std::shared_ptr<ICollisionChecker> collision_checker);
    
    /**
     * @brief 析构函数
     */
    ~DDROptimizer();
    
    /**
     * @brief 执行轨迹优化
     * @param input 输入轨迹数据
     * @param output 输出优化结果
     * @return 是否成功
     */
    bool optimize(const TrajectoryInput& input, TrajectoryOutput& output);
    
    /**
     * @brief 获取最终轨迹
     */
    const Trajectory<5, 2>& getTrajectory() const { return final_trajectory_; }
    
    /**
     * @brief 获取优化统计信息
     */
    const OptimizationStats& getStats() const { return stats_; }
    
    /**
     * @brief 更新配置
     */
    void updateConfig(const OptimizerConfig& config);
    
    /**
     * @brief 更新碰撞检测器
     */
    void updateCollisionChecker(std::shared_ptr<ICollisionChecker> collision_checker);
    
private:
    // 配置和依赖
    OptimizerConfig config_;
    std::shared_ptr<ICollisionChecker> collision_checker_;
    
    // 优化结果
    Trajectory<5, 2> final_trajectory_;
    Trajectory<5, 2> optimizer_trajectory_;
    Trajectory<5, 2> init_final_trajectory_;
    OptimizationStats stats_;
    
    // MINCO轨迹表示
    minco::MINCO_S3NU minco_;
    
    // 当前优化状态
    Eigen::Vector3d ini_state_XYTheta_;
    Eigen::Vector3d fin_state_XYTheta_;
    Eigen::Vector3d final_ini_state_XYTheta_;
    Eigen::Vector3d final_fin_state_XYTheta_;
    
    Eigen::VectorXd piece_time_;
    Eigen::MatrixXd inner_points_;
    Eigen::MatrixXd ini_state_;
    Eigen::MatrixXd fin_state_;
    int traj_num_;
    bool if_cut_traj_;
    
    Eigen::MatrixXd final_inner_points_;
    Eigen::VectorXd final_piece_time_;
    
    std::vector<Eigen::Vector3d> inner_init_positions_;
    
    // 梯度存储
    Eigen::Matrix2Xd grad_by_points_;
    Eigen::VectorXd grad_by_times_;
    Eigen::MatrixX2d partial_grad_by_coeffs_;
    Eigen::VectorXd partial_grad_by_times_;
    Eigen::Vector2d grad_by_tail_state_s_;
    Eigen::Vector2d final_integral_XY_error_;
    Eigen::Vector2d final_integral_XY_error_backup_;
    
    // ALM变量
    Eigen::VectorXd equal_lambda_;
    Eigen::VectorXd equal_rho_;
    
    // 碰撞点记录（用于调试）
    std::vector<Eigen::Vector2d> collision_points_;
    std::vector<Eigen::Vector2d> collision_points_backup_;
    
    // 辅助变量
    int sam_num_each_part_;
    Eigen::VectorXd integral_chain_coeff_;
    double safe_dis_;
    double smooth_eps_;
    int iter_num_;
    bool if_print_;
    
    // ========== 核心优化函数 ==========
    
    /**
     * @brief 从输入数据获取初始状态
     */
    bool getInitialState(const TrajectoryInput& input);
    
    /**
     * @brief 执行优化主循环
     */
    bool runOptimization();
    
    /**
     * @brief 最终碰撞检查
     */
    bool checkFinalCollision(const Trajectory<5, 2>& traj,
                            const Eigen::Vector3d& start_XYTheta);
    
    // ========== L-BFGS回调函数 ==========
    
    /**
     * @brief 预处理阶段的代价函数
     */
    static double costFunctionCallbackPath(void* ptr,
                                          const Eigen::VectorXd& x,
                                          Eigen::VectorXd& g);
    
    /**
     * @brief 正式优化阶段的代价函数
     */
    static double costFunctionCallback(void* ptr,
                                      const Eigen::VectorXd& x,
                                      Eigen::VectorXd& g);
    
    /**
     * @brief 早停回调函数
     */
    static int earlyExit(void* instance,
                        const Eigen::VectorXd& x,
                        const Eigen::VectorXd& g,
                        const double fx,
                        const double step,
                        const int k,
                        const int ls);
    
    // ========== 惩罚函数计算 ==========
    
    /**
     * @brief 添加预处理惩罚项
     */
    void attachPenaltyFunctionalPath(double& cost);
    
    /**
     * @brief 添加正式优化惩罚项
     */
    void attachPenaltyFunctional(double& cost);
    
    // ========== 辅助函数 ==========
    
    /**
     * @brief 平滑L1惩罚函数
     */
    inline void positiveSmoothedL1(const double& x, double& f, double& df) {
        // 平滑L1惩罚函数: f(x) = smooth_L1(max(0, x))
        // smooth_L1(x) = x^2/(2*eps) for |x| <= eps, else |x| - eps/2
        if (x <= 0.0) {
            f = 0.0;
            df = 0.0;
        } else if (x < smooth_eps_) {
            f = x * x / (2.0 * smooth_eps_);
            df = x / smooth_eps_;
        } else {
            f = x - smooth_eps_ / 2.0;
            df = 1.0;
        }
    }
    
    /**
     * @brief 实时间到虚拟时间的转换
     */
    template <typename EIGENVEC>
    inline void realT2VirtualT(const Eigen::VectorXd& RT, EIGENVEC& VT);
    
    /**
     * @brief 虚拟时间到实时间的转换
     */
    template <typename EIGENVEC>
    inline void virtualT2RealT(const EIGENVEC& VT, Eigen::VectorXd& RT);
    
    /**
     * @brief 时间梯度反向传播
     */
    template <typename EIGENVEC>
    static inline void backwardGradT(const Eigen::VectorXd& tau,
                                    const Eigen::VectorXd& gradT,
                                    EIGENVEC& gradTau);
    
    /**
     * @brief 角度归一化到[-π, π]
     */
    inline double normalizeAngle(double angle);
    
    /**
     * @brief 获取指定时间的预测状态
     */
    void getPredictedState(const double& time,
                          Eigen::Vector3d& XYTheta,
                          Eigen::Vector3d& VAJ,
                          Eigen::Vector3d& OAJ);
    
    /**
     * @brief 日志输出函数
     */
    void log(const std::string& message, bool force = false);
    void logInfo(const std::string& message);
    void logWarn(const std::string& message);
    void logError(const std::string& message);
};

// ========== 模板函数实现 ==========

template <typename EIGENVEC>
inline void DDROptimizer::realT2VirtualT(const Eigen::VectorXd& RT, EIGENVEC& VT) {
    const int sizeT = RT.size();
    VT.resize(sizeT);
    for (int i = 0; i < sizeT; ++i) {
        VT(i) = RT(i) > 1.0 ? (sqrt(2.0 * RT(i) - 1.0) - 1.0)
                            : (1.0 - sqrt(2.0 / RT(i) - 1.0));
    }
}

template <typename EIGENVEC>
inline void DDROptimizer::virtualT2RealT(const EIGENVEC& VT, Eigen::VectorXd& RT) {
    const int sizeTau = VT.size();
    RT.resize(sizeTau);
    for (int i = 0; i < sizeTau; ++i) {
        RT(i) = VT(i) > 0.0 ? ((0.5 * VT(i) + 1.0) * VT(i) + 1.0)
                            : 1.0 / ((0.5 * VT(i) - 1.0) * VT(i) + 1.0);
    }
}

template <typename EIGENVEC>
inline void DDROptimizer::backwardGradT(const Eigen::VectorXd& tau,
                                       const Eigen::VectorXd& gradT,
                                       EIGENVEC& gradTau) {
    const int sizetau = tau.size();
    gradTau.resize(sizetau);
    double gradrt2vt;
    for (int i = 0; i < sizetau; i++) {
        if (tau(i) > 0) {
            gradrt2vt = tau(i) + 1.0;
        } else {
            double denSqrt = (0.5 * tau(i) - 1.0) * tau(i) + 1.0;
            gradrt2vt = (1.0 - tau(i)) / (denSqrt * denSqrt);
        }
        gradTau(i) = gradT(i) * gradrt2vt;
    }
}

} // namespace ddr_optimizer

#endif // DDR_OPTIMIZER_PURE_H

