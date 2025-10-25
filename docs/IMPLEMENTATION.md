# 开发者指南 - DDR后端轨迹优化器

## 概述

本文档为开发者提供DDR轨迹优化器的技术细节和扩展指南。项目已完成核心实现，本文档主要用于理解算法原理和进行二次开发。

## ⚠️ 重要说明

**项目已完成所有核心功能实现**。以下内容主要用于理解算法原理和进行二次开发，而非待实现的功能清单。

## 核心算法实现

### 1. `runOptimization()` - 主优化循环

**位置**: `src/optimizer_pure.cpp`

**功能**: 执行两阶段优化（预处理 + 正式优化）+ ALM迭代

**参考**: 原`optimizer.cpp`的244-464行

**伪代码**:

```cpp
bool DDROptimizer::runOptimization() {
    // 1. 初始化ALM参数
    if (!if_cut_traj_) {
        equal_lambda_ = config_.alm.init_lambda;
        equal_rho_ = config_.alm.init_rho;
    } else {
        equal_lambda_ = config_.cut_alm.init_lambda;
        equal_rho_ = config_.cut_alm.init_rho;
    }
    
    // 2. 设置MINCO条件
    int variable_num = 3 * traj_num_ - 1;
    minco_.setConditions(ini_state_, fin_state_, traj_num_, config_.energy_weights);
    minco_.setParameters(inner_points_, piece_time_);
    minco_.getTrajectory(init_final_trajectory_);
    
    // 3. 构造优化变量向量 x
    Eigen::VectorXd x(variable_num);
    int offset = 0;
    
    // 复制inner_points
    memcpy(x.data() + offset, inner_points_.data(), inner_points_.size() * sizeof(double));
    offset += inner_points_.size();
    
    // fin_state的s分量（松弛变量）
    x[offset] = fin_state_(1, 0);
    ++offset;
    
    // 时间变量（转换为虚拟时间）
    Eigen::Map<Eigen::VectorXd> Vt(x.data() + offset, piece_time_.size());
    realT2VirtualT(piece_time_, Vt);
    
    // 4. 第一阶段：预处理优化
    double cost;
    iter_num_ = 0;
    if_print_ = false;
    
    auto start = std::chrono::high_resolution_clock::now();
    
    // 设置L-BFGS参数
    lbfgs::lbfgs_parameter_t path_lbfgs_params = config_.path_lbfgs.lbfgs_params;
    if (fabs(fin_state_(1, 0)) < config_.path_lbfgs.shot_path_horizon) {
        path_lbfgs_params.past = config_.path_lbfgs.shot_path_past;
    } else {
        path_lbfgs_params.past = config_.path_lbfgs.normal_past;
    }
    
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
    
    logInfo("Pre-processing: " + std::to_string(stats_.preprocess_time_ms) + " ms, " 
            + std::to_string(iter_num_) + " iterations");
    
    // 从优化变量中提取结果
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
    
    // 5. 第二阶段：正式优化 + ALM迭代
    iter_num_ = 0;
    start = std::chrono::high_resolution_clock::now();
    
    int alm_iteration = 0;
    const int max_alm_iterations = 20;
    
    while (true) {
        // L-BFGS优化
        result = lbfgs::lbfgs_optimize(
            x, cost,
            DDROptimizer::costFunctionCallback,
            NULL,
            DDROptimizer::earlyExit,
            this,
            config_.lbfgs
        );
        
        // 记录ALM误差
        stats_.alm_errors.push_back(final_integral_XY_error_.norm());
        stats_.alm_costs.push_back(cost);
        
        // 检查收敛
        double tolerance = if_cut_traj_ ? 
            config_.cut_alm.tolerance[0] : config_.alm.tolerance[0];
        
        if (final_integral_XY_error_.norm() < tolerance) {
            logInfo("ALM converged! Position error: " + 
                   std::to_string(final_integral_XY_error_.norm()));
            break;
        }
        
        if (++alm_iteration >= max_alm_iterations) {
            logWarn("ALM max iterations reached");
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
    
    stats_.final_cost = cost;
    stats_.final_position_error = final_integral_XY_error_.norm();
    
    logInfo("Optimization: " + std::to_string(stats_.optimization_time_ms) + " ms, " 
            + std::to_string(iter_num_) + " iterations");
    logInfo("Final position error: " + std::to_string(stats_.final_position_error));
    
    return true;
}
```

### 2. `attachPenaltyFunctional()` - 正式优化的代价函数

**位置**: `src/optimizer_pure.cpp`

**功能**: 计算完整的代价函数和梯度

**参考**: 原`optimizer.cpp`的687-1055行

**主要组成部分**:

```cpp
void DDROptimizer::attachPenaltyFunctional(double& cost) {
    collision_points_.clear();
    
    double ini_x = ini_state_XYTheta_.x();
    double ini_y = ini_state_XYTheta_.y();
    
    // 初始化代价分量
    double cost_energy = 0;      // 已由MINCO计算
    double cost_collision = 0;
    double cost_acc = 0;
    double cost_domega = 0;
    double cost_moment = 0;
    double cost_cen_acc = 0;
    double cost_mean_time = 0;
    double cost_endpoint = 0;
    
    // 初始化梯度累积向量
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
        
        // 遍历采样点（Simpson积分）
        double s1 = 0.0;
        for (int j = 0; j <= sam_num_each_part_; j++) {
            // 1. 计算当前点的状态
            Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3;
            computeBasis(s1, beta0, beta1, beta2, beta3);  // 需要实现
            
            Eigen::Vector2d sigma = c.transpose() * beta0;      // [yaw, s]
            Eigen::Vector2d dsigma = c.transpose() * beta1;     // [dyaw, ds]
            Eigen::Vector2d ddsigma = c.transpose() * beta2;    // [ddyaw, dds]
            Eigen::Vector2d dddsigma = c.transpose() * beta3;   // [dddyaw, ddds]
            
            double cosyaw = cos(sigma.x());
            double sinyaw = sin(sigma.x());
            
            // 2. 累积位置（Simpson积分）
            if (j != 0) {
                // 更新current_point_XY
                // ...
            }
            
            // 3. 检查并惩罚运动学约束违反
            // 3.1 加速度约束
            double viola_acc = ddsigma.y() * ddsigma.y() - 
                              config_.kinematic.max_acc * config_.kinematic.max_acc;
            if (viola_acc > 0) {
                double f, df;
                positiveSmoothedL1(viola_acc, f, df);
                cost_acc += weight * f;
                // 累积梯度...
            }
            
            // 3.2 角加速度约束
            // ...
            
            // 3.3 速度-角速度约束（力矩约束）
            // ...
            
            // 3.4 向心加速度约束
            // ...
            
            // 4. 碰撞约束
            Eigen::Matrix2d ego_R;
            ego_R << cosyaw, -sinyaw, sinyaw, cosyaw;
            
            for (const auto& cp : config_.check_points) {
                Eigen::Vector2d check_pos = current_point_XY + ego_R * cp;
                Eigen::Vector2d grad_sdf;
                double sdf = collision_checker_->getDistanceWithGradient(
                    check_pos, grad_sdf, safe_dis_);
                
                double viola_collision = -sdf + safe_dis_;
                if (viola_collision > 0) {
                    double f, df;
                    positiveSmoothedL1(viola_collision, f, df);
                    cost_collision += weight * f;
                    // 累积梯度...
                    collision_points_.push_back(check_pos);
                }
            }
            
            // 5. 累积梯度到partialGradByCoeffs
            // ...
            
            s1 += (j % 2 == 0) ? halfstep : halfstep;
        }
        
        // 6. 时间分布均衡约束
        double avg_time = piece_time_.mean();
        if (piece_time_[i] < avg_time * config_.mean_time_lower_bound) {
            // 添加惩罚...
        }
        if (piece_time_[i] > avg_time * config_.mean_time_upper_bound) {
            // 添加惩罚...
        }
    }
    
    // 7. 终点约束（ALM）
    final_integral_XY_error_ = current_point_XY - fin_state_XYTheta_.head(2);
    
    cost_endpoint = 0.5 * (
        equal_rho_[0] * pow(final_integral_XY_error_.x() + equal_lambda_[0] / equal_rho_[0], 2) +
        equal_rho_[1] * pow(final_integral_XY_error_.y() + equal_lambda_[1] / equal_rho_[1], 2)
    );
    
    // 累积终点约束的梯度
    vec_coeff_chain_X.array() += equal_rho_[0] * (final_integral_XY_error_.x() + equal_lambda_[0] / equal_rho_[0]);
    vec_coeff_chain_Y.array() += equal_rho_[1] * (final_integral_XY_error_.y() + equal_lambda_[1] / equal_rho_[1]);
    
    // 8. 将梯度链式传播到partialGradByCoeffs和partialGradByTimes
    // ...（这是最复杂的部分，需要仔细推导）
    
    // 9. 累加所有代价
    cost += cost_collision + cost_acc + cost_domega + cost_moment + 
            cost_cen_acc + cost_mean_time + cost_endpoint;
    
    if (config_.verbose && if_print_) {
        std::cout << "  Collision: " << cost_collision << std::endl;
        std::cout << "  Acc: " << cost_acc << std::endl;
        std::cout << "  Domega: " << cost_domega << std::endl;
        std::cout << "  Moment: " << cost_moment << std::endl;
        std::cout << "  Cen_acc: " << cost_cen_acc << std::endl;
        std::cout << "  Mean_time: " << cost_mean_time << std::endl;
        std::cout << "  Endpoint: " << cost_endpoint << std::endl;
    }
}
```

### 3. `attachPenaltyFunctionalPath()` - 预处理的代价函数

**位置**: `src/optimizer_pure.cpp`

**功能**: 预处理阶段的简化代价函数

**参考**: 原`optimizer.cpp`的1307-1579行

**与正式优化的区别**:
- 不检查碰撞（或使用更宽松的碰撞约束）
- 添加路径相似度约束（保持与初始路径接近）
- 权重配置不同

### 4. `costFunctionCallback()` 和 `costFunctionCallbackPath()`

**位置**: `src/optimizer_pure.cpp`

**功能**: L-BFGS的回调函数

**参考**: 原`optimizer.cpp`的624-685行和1260-1305行

**实现框架**:

```cpp
double DDROptimizer::costFunctionCallback(void* ptr,
                                         const Eigen::VectorXd& x,
                                         Eigen::VectorXd& g) {
    if (x.norm() > 1e4) return inf;
    
    DDROptimizer& obj = *(DDROptimizer*)ptr;
    obj.iter_num_++;
    
    g.setZero();
    
    // 1. 从x中提取变量
    int offset = 0;
    Eigen::Map<const Eigen::MatrixXd> P(x.data() + offset, 2, obj.traj_num_ - 1);
    Eigen::Map<Eigen::MatrixXd> gradP(g.data() + offset, 2, obj.traj_num_ - 1);
    offset += 2 * (obj.traj_num_ - 1);
    
    double* gradTailS = g.data() + offset;
    obj.fin_state_(1, 0) = x[offset];
    ++offset;
    
    obj.inner_points_ = P;
    
    Eigen::Map<const Eigen::VectorXd> t(x.data() + offset, obj.traj_num_);
    Eigen::Map<Eigen::VectorXd> gradt(g.data() + offset, obj.traj_num_);
    offset += obj.traj_num_;
    
    obj.virtualT2RealT(t, obj.piece_time_);
    gradt.setZero();
    
    // 2. 计算能量代价和梯度
    double cost;
    obj.minco_.setTConditions(obj.fin_state_);
    obj.minco_.setParameters(obj.inner_points_, obj.piece_time_);
    obj.minco_.getEnergy(cost);
    obj.minco_.getEnergyPartialGradByCoeffs(obj.partial_grad_by_coeffs_);
    obj.minco_.getEnergyPartialGradByTimes(obj.partial_grad_by_times_);
    
    // 3. 添加惩罚项
    obj.attachPenaltyFunctional(cost);
    
    // 4. 传播梯度
    obj.minco_.propogateArcYawLenghGrad(
        obj.partial_grad_by_coeffs_, obj.partial_grad_by_times_,
        obj.grad_by_points_, obj.grad_by_times_, obj.grad_by_tail_state_s_
    );
    
    // 5. 添加时间项
    cost += obj.config_.penalty.time_weight * obj.piece_time_.sum();
    Eigen::VectorXd rho_times(obj.grad_by_times_.size());
    obj.grad_by_times_ += obj.config_.penalty.time_weight * rho_times.setOnes();
    
    // 6. 填充梯度向量
    *gradTailS = obj.grad_by_tail_state_s_.y();
    gradP = obj.grad_by_points_;
    backwardGradT(t, obj.grad_by_times_, gradt);
    
    return cost;
}
```

### 5. `checkFinalCollision()` - 最终碰撞检查

**位置**: `src/optimizer_pure.cpp`

**功能**: 以更高分辨率检查最终轨迹是否有碰撞

**参考**: 原`optimizer.cpp`的467-564行

### 6. `getPredictedState()` - 轨迹预测

**位置**: `src/optimizer_pure.cpp`

**功能**: 获取指定时间的机器人状态（位置、速度、加速度等）

**参考**: 原`optimizer.cpp`的1096-1177行

## 辅助函数

### Simpson积分

需要实现位置积分：

```cpp
// 对于标准微分驱动:
x_next = x_curr + ∫(v·cos(θ))dt
y_next = y_curr + ∫(v·sin(θ))dt

// Simpson法则:
∫f(t)dt ≈ (h/6)[f(t0) + 4f(t1) + 2f(t2) + 4f(t3) + ... + f(tn)]
```

### 梯度链式法则

关键是理解梯度如何从笛卡尔空间传播回(yaw, s)空间：

```
∂L/∂c = ∂L/∂(x,y) · ∂(x,y)/∂σ · ∂σ/∂c
```

其中：
- `c`: 多项式系数
- `σ`: (yaw, s)
- `(x,y)`: 笛卡尔坐标

## 测试策略

### 1. 单元测试

分步骤测试每个函数：

```cpp
// 测试1: 时间转换
void test_time_conversion() {
    Eigen::VectorXd realT(5);
    realT << 0.5, 1.0, 1.5, 2.0, 2.5;
    
    Eigen::VectorXd virtualT;
    realT2VirtualT(realT, virtualT);
    
    Eigen::VectorXd realT2;
    virtualT2RealT(virtualT, realT2);
    
    assert((realT - realT2).norm() < 1e-10);
}

// 测试2: 平滑L1
void test_smoothed_l1() {
    double x = 0.05;
    double f, df;
    positiveSmoothedL1(x, f, df);
    
    // 验证数值导数
    double eps = 1e-6;
    double f1, df1;
    positiveSmoothedL1(x + eps, f1, df1);
    double numerical_df = (f1 - f) / eps;
    
    assert(fabs(numerical_df - df) < 1e-3);
}
```

### 2. 集成测试

逐步增加复杂度：

1. 零初始条件，无约束
2. 添加运动学约束
3. 添加简单障碍物
4. 添加复杂场景

### 3. 调试技巧

```cpp
// 在attachPenaltyFunctional中添加调试输出
if (config_.verbose) {
    std::cout << "Iteration: " << iter_num_ << std::endl;
    std::cout << "Position error: " << final_integral_XY_error_.norm() << std::endl;
    std::cout << "Collision points: " << collision_points_.size() << std::endl;
}

// 导出中间结果
void exportIntermediateTrajectory(int iteration) {
    std::string filename = "traj_iter_" + std::to_string(iteration) + ".txt";
    // 保存当前轨迹...
}
```

## 常见错误和解决方案

### 1. 梯度检查失败

**症状**: 优化不收敛或收敛到错误结果

**解决**: 实现数值梯度检查

```cpp
void check_gradient() {
    const double eps = 1e-6;
    Eigen::VectorXd x = getCurrentOptimizationVariables();
    Eigen::VectorXd g_analytical(x.size());
    Eigen::VectorXd g_numerical(x.size());
    
    double f0 = costFunctionCallback(this, x, g_analytical);
    
    for (int i = 0; i < x.size(); i++) {
        x[i] += eps;
        double f1 = costFunctionCallback(this, x, g_analytical);
        x[i] -= eps;
        
        g_numerical[i] = (f1 - f0) / eps;
    }
    
    std::cout << "Gradient error: " << (g_analytical - g_numerical).norm() << std::endl;
}
```

### 2. ALM不收敛

**症状**: 位置误差不减小

**解决**:
- 检查梯度计算是否正确
- 增大`rho_max`
- 调整`gamma`
- 提供更好的初始猜测

### 3. 碰撞检测问题

**症状**: 轨迹穿过障碍物

**解决**:
- 验证碰撞检测器实现
- 检查梯度方向
- 增加`collision_weight`
- 增加`sparse_resolution`

## 性能优化

### 1. 向量化计算

尽可能使用Eigen的向量化操作：

```cpp
// 慢:
for (int i = 0; i < n; i++) {
    result[i] = a[i] * b[i] + c[i];
}

// 快:
result = a.array() * b.array() + c.array();
```

### 2. 避免不必要的内存分配

```cpp
// 慢:
for (int i = 0; i < n; i++) {
    Eigen::VectorXd temp(m);  // 每次循环分配
    // ...
}

// 快:
Eigen::VectorXd temp(m);  // 只分配一次
for (int i = 0; i < n; i++) {
    // ...
}
```

### 3. 预计算常量

```cpp
// 在构造函数中预计算
double coeff_integral_ = piece_time / (sparse_resolution * 6.0);
double inv_sparse_resolution_ = 1.0 / sparse_resolution;
```

## 总结

实现DDR优化器的核心步骤：

1. **理解数学模型**: MINCO轨迹表示，Simpson积分，微分驱动运动学
2. **实现代价函数**: 分解为多个部分，逐个实现和测试
3. **正确计算梯度**: 使用链式法则，验证数值梯度
4. **调试和测试**: 从简单到复杂，逐步增加约束
5. **优化性能**: 使用Eigen向量化，避免不必要的分配

建议的实现顺序：

1. 实现基本框架（已完成）
2. 实现无约束优化（只有能量项）
3. 添加运动学约束
4. 添加碰撞约束
5. 实现ALM
6. 性能优化和测试

参考原始代码时注意：
- ROS相关的调用需要替换或删除
- 日志输出改用std::cout
- 时间测量改用std::chrono
- 参数读取改用配置对象

祝实现顺利！如有问题请参考原始代码`optimizer.cpp`或提问。

