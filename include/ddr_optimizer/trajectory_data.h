#ifndef DDR_TRAJECTORY_DATA_H
#define DDR_TRAJECTORY_DATA_H

#include <Eigen/Eigen>
#include <vector>
#include <string>
#include "gcopter/trajectory.hpp"

namespace ddr_optimizer {

/**
 * @brief 轨迹优化输入数据
 */
struct TrajectoryInput {
    /**
     * @brief 起点状态 (2行3列矩阵)
     * 行: [yaw状态, s状态]
     * 列: [位置, 速度, 加速度]
     * 即: [[yaw, dyaw, ddyaw], [s, ds, dds]]
     */
    Eigen::MatrixXd start_state;  // (2, 3)
    
    /**
     * @brief 终点状态 (2行3列矩阵)
     * 格式同start_state
     */
    Eigen::MatrixXd final_state;  // (2, 3)
    
    /**
     * @brief 起点的笛卡尔坐标 [x, y, theta]
     */
    Eigen::Vector3d start_state_XYTheta;
    
    /**
     * @brief 终点的笛卡尔坐标 [x, y, theta]
     */
    Eigen::Vector3d final_state_XYTheta;
    
    /**
     * @brief 中间路径点 (来自前端路径规划)
     * 每个点包含: [yaw, s, ...]
     * 这些点用于初始化优化器
     */
    std::vector<Eigen::Vector3d> waypoints;
    
    /**
     * @brief 中间位置点的笛卡尔坐标
     * 每个点包含: [x, y, theta]
     */
    std::vector<Eigen::Vector3d> waypoint_positions;
    
    /**
     * @brief 每段轨迹的初始时间分配
     */
    double initial_segment_time = 1.0;
    
    /**
     * @brief 是否是截断的轨迹（用于在线重规划）
     */
    bool is_cut = false;
    
    /**
     * @brief 打印输入数据（用于调试）
     */
    void print() const {
        std::cout << "===== Trajectory Input =====" << std::endl;
        std::cout << "Start state:" << std::endl << start_state << std::endl;
        std::cout << "Final state:" << std::endl << final_state << std::endl;
        std::cout << "Start XYTheta: " << start_state_XYTheta.transpose() << std::endl;
        std::cout << "Final XYTheta: " << final_state_XYTheta.transpose() << std::endl;
        std::cout << "Waypoints count: " << waypoints.size() << std::endl;
        std::cout << "Is cut: " << (is_cut ? "yes" : "no") << std::endl;
    }
    
    /**
     * @brief 验证输入数据有效性
     */
    bool validate() const {
        if (start_state.rows() != 2 || start_state.cols() != 3) {
            std::cerr << "Error: start_state must be 2x3" << std::endl;
            return false;
        }
        if (final_state.rows() != 2 || final_state.cols() != 3) {
            std::cerr << "Error: final_state must be 2x3" << std::endl;
            return false;
        }
        if (waypoints.size() != waypoint_positions.size()) {
            std::cerr << "Error: waypoints and waypoint_positions size mismatch" << std::endl;
            return false;
        }
        if (initial_segment_time <= 0) {
            std::cerr << "Error: initial_segment_time must be positive" << std::endl;
            return false;
        }
        return true;
    }
};

/**
 * @brief 轨迹优化输出数据
 */
struct TrajectoryOutput {
    /**
     * @brief 优化后的轨迹
     * Trajectory<5, 2>: 5阶多项式, 2个自由度(yaw, s)
     */
    Trajectory<5, 2> trajectory;
    
    /**
     * @brief 优化前的轨迹（预处理结果）
     */
    Trajectory<5, 2> initial_trajectory;
    
    /**
     * @brief 轨迹段数
     */
    int num_segments = 0;
    
    /**
     * @brief 每段时间
     */
    Eigen::VectorXd segment_durations;
    
    /**
     * @brief 中间控制点 (2 x (N-1))
     */
    Eigen::MatrixXd control_points;
    
    /**
     * @brief 起点状态 [x, y, theta]
     */
    Eigen::Vector3d start_state_XYTheta;
    
    /**
     * @brief 终点状态 [x, y, theta]
     */
    Eigen::Vector3d final_state_XYTheta;
    
    /**
     * @brief 优化是否成功
     */
    bool success = false;
    
    /**
     * @brief 状态消息
     */
    std::string message;
    
    /**
     * @brief 总耗时（毫秒）
     */
    double total_time_ms = 0.0;
    
    /**
     * @brief 打印输出数据（用于调试）
     */
    void print() const {
        std::cout << "===== Trajectory Output =====" << std::endl;
        std::cout << "Success: " << (success ? "yes" : "no") << std::endl;
        std::cout << "Message: " << message << std::endl;
        std::cout << "Num segments: " << num_segments << std::endl;
        std::cout << "Total duration: " << segment_durations.sum() << " s" << std::endl;
        std::cout << "Total time: " << total_time_ms << " ms" << std::endl;
        std::cout << "Start XYTheta: " << start_state_XYTheta.transpose() << std::endl;
        std::cout << "Final XYTheta: " << final_state_XYTheta.transpose() << std::endl;
    }
    
    /**
     * @brief 获取指定时间的状态
     * @param time 时间
     * @param XYTheta 输出位置 [x, y, theta]
     * @param VAJ 输出线速度及其导数 [v, a, j]
     * @param OAJ 输出角速度及其导数 [omega, alpha, j_omega]
     * @param resolution Simpson积分分辨率
     */
    void getStateAt(double time,
                   Eigen::Vector3d& XYTheta,
                   Eigen::Vector3d& VAJ,
                   Eigen::Vector3d& OAJ,
                   double resolution = 0.01,
                   bool if_standard_diff = true) const;
    
    /**
     * @brief 将轨迹采样为路径点
     * @param dt 采样时间间隔
     * @return 路径点列表 [x, y, theta]
     */
    std::vector<Eigen::Vector3d> samplePath(double dt = 0.1,
                                            bool if_standard_diff = true) const;
    
    /**
     * @brief 导出轨迹到文件（用于可视化）
     * @param filename 文件名
     * @param dt 采样时间间隔
     */
    bool exportToFile(const std::string& filename, double dt = 0.01,
                     bool if_standard_diff = true) const;
};

/**
 * @brief 优化统计信息
 */
struct OptimizationStats {
    double total_time_ms = 0.0;
    double preprocess_time_ms = 0.0;
    double optimization_time_ms = 0.0;
    int total_iterations = 0;
    int preprocess_iterations = 0;
    int optimization_iterations = 0;
    double final_cost = 0.0;
    double final_position_error = 0.0;
    bool success = false;
    std::string message;
    
    // ALM收敛历史
    std::vector<double> alm_errors;
    std::vector<double> alm_costs;
    
    void print() const {
        std::cout << "===== Optimization Stats =====" << std::endl;
        std::cout << "Success: " << (success ? "yes" : "no") << std::endl;
        std::cout << "Total time: " << total_time_ms << " ms" << std::endl;
        std::cout << "  - Preprocess: " << preprocess_time_ms << " ms (" 
                  << preprocess_iterations << " iters)" << std::endl;
        std::cout << "  - Optimization: " << optimization_time_ms << " ms (" 
                  << optimization_iterations << " iters)" << std::endl;
        std::cout << "Final cost: " << final_cost << std::endl;
        std::cout << "Final position error: " << final_position_error << std::endl;
        std::cout << "Message: " << message << std::endl;
    }
};

} // namespace ddr_optimizer

#endif // DDR_TRAJECTORY_DATA_H

