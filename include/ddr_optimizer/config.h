#ifndef DDR_CONFIG_H
#define DDR_CONFIG_H

#include <Eigen/Eigen>
#include <string>

namespace ddr_optimizer {

/**
 * @brief 优化器配置结构
 */
struct OptimizerConfig {
    
    // 运动学约束
    struct Kinematic {
        double max_vel = 5.0;           // 最大线速度 (m/s)
        double min_vel = -5.0;          // 最小线速度 (m/s)
        double max_acc = 5.0;           // 最大加速度 (m/s²)
        double max_omega = 1.0;         // 最大角速度 (rad/s)
        double max_domega = 50.0;       // 最大角加速度 (rad/s²)
        double max_centripetal_acc = 10000.0;  // 最大向心加速度 (m/s²)
        bool directly_constrain_v_omega = false;  // 是否直接约束速度和角速度
    } kinematic;
    
    // 惩罚权重
    struct PenaltyWeights {
        double time_weight = 50.0;              // 时间权重
        double time_weight_backup = 50.0;       // 重规划时的时间权重备份
        double acc_weight = 300.0;              // 加速度约束权重
        double domega_weight = 300.0;           // 角加速度约束权重
        double collision_weight = 500000.0;     // 碰撞约束权重
        double moment_weight = 300.0;           // 力矩约束权重
        double mean_time_weight = 300.0;        // 时间分布均衡权重
        double cen_acc_weight = 300.0;          // 向心加速度约束权重
    } penalty;
    
    // 预处理阶段惩罚权重
    struct PathPenaltyWeights {
        double time_weight = 20.0;
        double bigpath_sdf_weight = 200000.0;
        double mean_time_weight = 100.0;
        double moment_weight = 1000.0;
        double acc_weight = 100.0;
        double domega_weight = 100.0;
    } path_penalty;
    
    // L-BFGS优化参数
    struct LbfgsParams {
        int mem_size = 256;                     // 历史记录大小
        int past = 3;                           // delta收敛测试的历史步数
        double g_epsilon = 0.0;                 // 梯度收敛阈值
        double min_step = 1.0e-32;              // 最小步长
        double delta = 5.0e-4;                  // delta收敛阈值
        int max_iterations = 8000;              // 最大迭代次数
    } lbfgs;
    
    // 预处理L-BFGS参数
    struct PathLbfgsParams {
        int mem_size = 256;
        int normal_past = 2;
        int shot_path_past = 8;
        double shot_path_horizon = 0.5;
        double g_epsilon = 0.0;
        double min_step = 0.0;
        double delta = 5.0e-2;
        int max_iterations = 8000;
    } path_lbfgs;
    
    // 增广拉格朗日参数
    struct ALMParams {
        Eigen::VectorXd init_lambda;            // 初始拉格朗日乘子
        Eigen::VectorXd init_rho;               // 初始惩罚参数
        Eigen::VectorXd rho_max;                // 最大惩罚参数
        Eigen::VectorXd gamma;                  // 惩罚参数增长率
        Eigen::VectorXd tolerance;              // 收敛容差
        
        ALMParams() {
            init_lambda = Eigen::Vector2d(0.0, 0.0);
            init_rho = Eigen::Vector2d(10000.0, 10000.0);
            rho_max = Eigen::Vector2d(1.0e10, 1.0e10);
            gamma = Eigen::Vector2d(9.0, 9.0);
            tolerance = Eigen::Vector2d(0.01, 0.0);
        }
    } alm;
    
    // 截断轨迹的ALM参数
    struct CutALMParams {
        Eigen::VectorXd init_lambda;
        Eigen::VectorXd init_rho;
        Eigen::VectorXd rho_max;
        Eigen::VectorXd gamma;
        Eigen::VectorXd tolerance;
        
        CutALMParams() {
            init_lambda = Eigen::Vector2d(0.0, 0.0);
            init_rho = Eigen::Vector2d(1000.0, 1000.0);
            rho_max = Eigen::Vector2d(1.0e10, 1.0e10);
            gamma = Eigen::Vector2d(5.0, 5.0);
            tolerance = Eigen::Vector2d(0.5, 0.0);
        }
    } cut_alm;
    
    // 其他参数
    double smooth_eps = 0.01;                   // 平滑L1参数
    double safe_distance = 0.3;                 // 安全距离(米)
    int sparse_resolution = 8;                  // 采样分辨率
    double time_resolution = 0.4;               // 时间分辨率
    int min_traj_num = 3;                       // 最小轨迹段数
    
    double mean_time_lower_bound = 0.50;        // 时间下界比例
    double mean_time_upper_bound = 2.00;        // 时间上界比例
    
    Eigen::Vector2d energy_weights = Eigen::Vector2d(0.33, 1.0);  // 能量权重 [yaw_weight, s_weight]
    
    // ICR参数 (Instantaneous Center of Rotation)
    struct ICR {
        double yl = 0.0;     // 左轮y坐标
        double yr = 0.0;     // 右轮y坐标
        double xv = 0.0;     // 速度中心x坐标
    } icr;
    
    bool if_standard_diff = true;               // 是否使用标准微分驱动模型
    
    // 轨迹预测参数
    double traj_predict_resolution = 0.01;      // 轨迹预测分辨率
    
    // 最终碰撞检查参数
    double final_min_safe_dis = 0.10;           // 最终最小安全距离
    int final_safe_dis_check_num = 16;          // 最终安全距离检查分辨率
    int safe_replan_max_time = 3;               // 安全重规划最大次数
    
    // 车辆检查点（用于碰撞检测）
    std::vector<Eigen::Vector2d> check_points;  // 车体上的检查点
    
    // 调试选项
    bool verbose = false;                       // 是否输出详细信息
    bool if_visual_optimization = false;        // 是否可视化优化过程
    
    /**
     * @brief 从YAML文件加载配置
     * @param filename YAML文件路径
     * @return 配置对象
     */
    static OptimizerConfig loadFromYAML(const std::string& filename);
    
    /**
     * @brief 保存配置到YAML文件
     * @param filename YAML文件路径
     * @return 是否成功
     */
    bool saveToYAML(const std::string& filename) const;
    
    /**
     * @brief 使用默认配置
     * @return 默认配置对象
     */
    static OptimizerConfig defaultConfig();
    
    /**
     * @brief 验证配置有效性
     * @return 是否有效
     */
    bool validate() const;
};

} // namespace ddr_optimizer

#endif // DDR_CONFIG_H

