# DDR后端轨迹优化器（纯净版）

## 项目简介

这是DDR-opt项目后端轨迹优化模块的纯净版本，完全移除了ROS依赖，可以作为独立的C++库使用。项目专注于微分驱动机器人（Differential Drive Robot）的轨迹规划与优化。

### 核心特性

- ✅ **无ROS依赖**: 完全独立的C++库，仅依赖Eigen3和Python
- ✅ **MINCO轨迹表示**: 基于Minimum Control轨迹优化
- ✅ **L-BFGS优化**: 高效的无约束优化算法
- ✅ **增广拉格朗日方法**: 处理终点位置约束
- ✅ **完整的运动学约束**: 速度、加速度、角速度、角加速度、向心加速度等
- ✅ **碰撞避障**: 基于SDF的碰撞检测和避障
- ✅ **灵活的配置系统**: 支持YAML配置文件
- ✅ **测试用例**: 包含简单轨迹和避障测试
- ✅ **可视化功能**: 使用matplotlib-cpp进行轨迹可视化

### 适用场景

- 微分驱动机器人（Differential Drive Robot）轨迹规划
- 离线轨迹优化
- 运动规划研究
- 机器人仿真

## 依赖安装

### 必需依赖

- **CMake** >= 3.10
- **C++14** 或更高
- **Eigen3** >= 3.3
- **Python3** >= 3.6
- **matplotlib-cpp** (已包含在项目中)

### Ubuntu系统安装

```bash
# 安装Eigen3
sudo apt-get install libeigen3-dev

# 安装Python3和matplotlib
sudo apt-get install python3 python3-matplotlib python3-numpy

# 安装其他构建工具
sudo apt-get install cmake build-essential
```

### 其他系统

对于其他Linux发行版或macOS，请使用相应的包管理器安装上述依赖。

## 编译和运行

### 编译项目

```bash
# 进入项目目录
cd DDR-opt-pure

# 创建构建目录
mkdir build && cd build

# 配置项目
cmake ..

# 编译项目
make -j4
```

编译成功后会生成：
- 静态库: `libddr_optimizer.a`
- 测试程序: `test_simple_trajectory`, `test_obstacle_avoidance`
- 示例程序: `simple_example`
- matplotlib-cpp测试程序: `test_matplotlib_cpp`

### 运行测试用例

```bash
cd build

# 运行简单轨迹测试
./test_simple_trajectory

# 运行避障测试
./test_obstacle_avoidance

# 运行示例程序
./simple_example

# 测试matplotlib-cpp功能
./test_matplotlib_cpp
```

### 使用CTest运行测试

```bash
cd build
ctest --verbose
```

### 测试效果
![绕障](assets/imgs/test2_multiple_obstacles_with_kinematics.png "绕障")
![贴边](assets/imgs/test2_near_boundary_with_kinematics.png "贴边")
