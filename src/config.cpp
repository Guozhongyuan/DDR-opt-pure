#include "ddr_optimizer/config.h"
#include <fstream>
#include <iostream>
#include <sstream>

namespace ddr_optimizer {

OptimizerConfig OptimizerConfig::defaultConfig() {
    OptimizerConfig config;
    
    // 使用默认值（已在结构体中定义）
    
    // 设置默认车体检查点（四角，按顺时针顺序）
    config.check_points.clear();
    config.check_points.emplace_back(1.7, 0.5);   // 右前
    config.check_points.emplace_back(1.7, -0.5);  // 左前
    config.check_points.emplace_back(-0.3, -0.5); // 左后
    config.check_points.emplace_back(-0.3, 0.5);  // 右后
    
    return config;
}

bool OptimizerConfig::validate() const {
    bool valid = true;
    
    if (kinematic.max_vel <= 0) {
        std::cerr << "Error: max_vel must be positive" << std::endl;
        valid = false;
    }
    
    if (kinematic.max_acc <= 0) {
        std::cerr << "Error: max_acc must be positive" << std::endl;
        valid = false;
    }
    
    if (kinematic.max_omega <= 0) {
        std::cerr << "Error: max_omega must be positive" << std::endl;
        valid = false;
    }
    
    if (kinematic.max_domega <= 0) {
        std::cerr << "Error: max_domega must be positive" << std::endl;
        valid = false;
    }
    
    if (safe_distance < 0) {
        std::cerr << "Error: safe_distance must be non-negative" << std::endl;
        valid = false;
    }
    
    if (sparse_resolution <= 0) {
        std::cerr << "Error: sparse_resolution must be positive" << std::endl;
        valid = false;
    }
    
    if (lbfgs.max_iterations < 0) {
        std::cerr << "Error: max_iterations must be non-negative" << std::endl;
        valid = false;
    }
    
    if (check_points.empty()) {
        std::cerr << "Warning: no check_points defined" << std::endl;
    }
    
    return valid;
}

// 简化的YAML加载实现（不依赖yaml-cpp库）
// 仅支持简单的key: value格式
OptimizerConfig OptimizerConfig::loadFromYAML(const std::string& filename) {
    OptimizerConfig config = defaultConfig();
    
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Warning: Cannot open config file " << filename 
                  << ", using default config" << std::endl;
        return config;
    }
    
    std::string line;
    while (std::getline(file, line)) {
        // 去除注释
        size_t comment_pos = line.find('#');
        if (comment_pos != std::string::npos) {
            line = line.substr(0, comment_pos);
        }
        
        // 去除前后空格
        size_t start = line.find_first_not_of(" \t\r\n");
        if (start == std::string::npos) continue;
        size_t end = line.find_last_not_of(" \t\r\n");
        line = line.substr(start, end - start + 1);
        
        // 解析 key: value
        size_t colon_pos = line.find(':');
        if (colon_pos == std::string::npos) continue;
        
        std::string key = line.substr(0, colon_pos);
        std::string value = line.substr(colon_pos + 1);
        
        // 去除key和value的前后空格
        key = key.substr(key.find_first_not_of(" \t"));
        key = key.substr(0, key.find_last_not_of(" \t") + 1);
        value = value.substr(value.find_first_not_of(" \t"));
        value = value.substr(0, value.find_last_not_of(" \t") + 1);
        
        // 解析值
        try {
            if (key == "max_vel") config.kinematic.max_vel = std::stod(value);
            else if (key == "min_vel") config.kinematic.min_vel = std::stod(value);
            else if (key == "max_acc") config.kinematic.max_acc = std::stod(value);
            else if (key == "max_omega") config.kinematic.max_omega = std::stod(value);
            else if (key == "max_domega") config.kinematic.max_domega = std::stod(value);
            else if (key == "max_centripetal_acc") config.kinematic.max_centripetal_acc = std::stod(value);
            else if (key == "if_directly_constrain_v_omega") {
                config.kinematic.directly_constrain_v_omega = (value == "true" || value == "True" || value == "1");
            }
            else if (key == "time_weight") config.penalty.time_weight = std::stod(value);
            else if (key == "acc_weight") config.penalty.acc_weight = std::stod(value);
            else if (key == "domega_weight") config.penalty.domega_weight = std::stod(value);
            else if (key == "collision_weight") config.penalty.collision_weight = std::stod(value);
            else if (key == "moment_weight") config.penalty.moment_weight = std::stod(value);
            else if (key == "mean_time_weight") config.penalty.mean_time_weight = std::stod(value);
            else if (key == "cen_acc_weight") config.penalty.cen_acc_weight = std::stod(value);
            else if (key == "smoothingFactor") config.smooth_eps = std::stod(value);
            else if (key == "safeDis") config.safe_distance = std::stod(value);
            else if (key == "sparseResolution") config.sparse_resolution = std::stoi(value);
            else if (key == "timeResolution") config.time_resolution = std::stod(value);
            else if (key == "mintrajNum") config.min_traj_num = std::stoi(value);
            else if (key == "mean_time_lowBound") config.mean_time_lower_bound = std::stod(value);
            else if (key == "mean_time_uppBound") config.mean_time_upper_bound = std::stod(value);
            else if (key == "if_standard_diff") {
                config.if_standard_diff = (value == "true" || value == "True" || value == "1");
            }
            else if (key == "trajPredictResolution") config.traj_predict_resolution = std::stod(value);
            else if (key == "verbose") {
                config.verbose = (value == "true" || value == "True" || value == "1");
            }
        } catch (const std::exception& e) {
            std::cerr << "Warning: Failed to parse " << key << ": " << e.what() << std::endl;
        }
    }
    
    file.close();
    
    if (!config.validate()) {
        std::cerr << "Warning: Loaded config is invalid, some parameters may be incorrect" << std::endl;
    }
    
    return config;
}

bool OptimizerConfig::saveToYAML(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open file " << filename << " for writing" << std::endl;
        return false;
    }
    
    file << "# DDR Optimizer Configuration\n\n";
    
    file << "# Kinematic Constraints\n";
    file << "max_vel: " << kinematic.max_vel << "\n";
    file << "min_vel: " << kinematic.min_vel << "\n";
    file << "max_acc: " << kinematic.max_acc << "\n";
    file << "max_omega: " << kinematic.max_omega << "\n";
    file << "max_domega: " << kinematic.max_domega << "\n";
    file << "max_centripetal_acc: " << kinematic.max_centripetal_acc << "\n";
    file << "if_directly_constrain_v_omega: " << (kinematic.directly_constrain_v_omega ? "true" : "false") << "\n\n";
    
    file << "# Penalty Weights\n";
    file << "time_weight: " << penalty.time_weight << "\n";
    file << "acc_weight: " << penalty.acc_weight << "\n";
    file << "domega_weight: " << penalty.domega_weight << "\n";
    file << "collision_weight: " << penalty.collision_weight << "\n";
    file << "moment_weight: " << penalty.moment_weight << "\n";
    file << "mean_time_weight: " << penalty.mean_time_weight << "\n";
    file << "cen_acc_weight: " << penalty.cen_acc_weight << "\n\n";
    
    file << "# Other Parameters\n";
    file << "smoothingFactor: " << smooth_eps << "\n";
    file << "safeDis: " << safe_distance << "\n";
    file << "sparseResolution: " << sparse_resolution << "\n";
    file << "timeResolution: " << time_resolution << "\n";
    file << "mintrajNum: " << min_traj_num << "\n";
    file << "mean_time_lowBound: " << mean_time_lower_bound << "\n";
    file << "mean_time_uppBound: " << mean_time_upper_bound << "\n";
    file << "if_standard_diff: " << (if_standard_diff ? "true" : "false") << "\n";
    file << "trajPredictResolution: " << traj_predict_resolution << "\n";
    file << "verbose: " << (verbose ? "true" : "false") << "\n";
    
    file.close();
    return true;
}

} // namespace ddr_optimizer

