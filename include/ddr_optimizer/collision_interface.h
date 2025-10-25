#ifndef DDR_COLLISION_INTERFACE_H
#define DDR_COLLISION_INTERFACE_H

#include <Eigen/Eigen>
#include <vector>
#include <memory>

namespace ddr_optimizer {

/**
 * @brief 碰撞检测接口基类
 * 
 * 所有碰撞检测器都应该继承此接口
 */
class ICollisionChecker {
public:
    virtual ~ICollisionChecker() = default;
    
    /**
     * @brief 获取指定位置的距离值和梯度
     * @param position 查询位置 (x, y)
     * @param gradient 输出梯度 (dx, dy)
     * @param safe_distance 安全距离阈值
     * @return 有符号距离值（正值表示无碰撞，负值表示碰撞）
     */
    virtual double getDistanceWithGradient(
        const Eigen::Vector2d& position,
        Eigen::Vector2d& gradient,
        double safe_distance = 0.0
    ) const = 0;
    
    /**
     * @brief 获取指定位置的距离值
     * @param position 查询位置 (x, y)
     * @return 有符号距离值
     */
    virtual double getDistance(const Eigen::Vector2d& position) const = 0;
    
    /**
     * @brief 检查是否在有效区域内
     * @param position 查询位置 (x, y)
     * @return true表示在有效区域内
     */
    virtual bool isInBounds(const Eigen::Vector2d& position) const = 0;
};

/**
 * @brief 简单的圆形障碍物碰撞检测器
 * 
 * 用于测试和简单场景，支持多个圆形障碍物
 */
class SimpleCircleObstacles : public ICollisionChecker {
public:
    /**
     * @brief 构造函数
     * @param bounds_min 地图最小边界 (x_min, y_min)
     * @param bounds_max 地图最大边界 (x_max, y_max)
     */
    SimpleCircleObstacles(
        const Eigen::Vector2d& bounds_min = Eigen::Vector2d(-10, -10),
        const Eigen::Vector2d& bounds_max = Eigen::Vector2d(10, 10)
    );
    
    /**
     * @brief 添加圆形障碍物
     * @param x 障碍物中心x坐标
     * @param y 障碍物中心y坐标
     * @param radius 障碍物半径
     */
    void addObstacle(double x, double y, double radius);
    
    /**
     * @brief 清除所有障碍物
     */
    void clearObstacles();
    
    /**
     * @brief 获取障碍物数量
     */
    size_t getObstacleCount() const { return obstacles_.size(); }
    
    // 实现接口
    double getDistanceWithGradient(
        const Eigen::Vector2d& position,
        Eigen::Vector2d& gradient,
        double safe_distance = 0.0
    ) const override;
    
    double getDistance(const Eigen::Vector2d& position) const override;
    
    bool isInBounds(const Eigen::Vector2d& position) const override;

private:
    // 障碍物存储: (x, y, radius)
    std::vector<Eigen::Vector3d> obstacles_;
    
    // 地图边界
    Eigen::Vector2d bounds_min_;
    Eigen::Vector2d bounds_max_;
};

/**
 * @brief 网格地图碰撞检测器（用于与原始SDFmap接口兼容）
 * 
 * 使用网格存储SDF值
 */
class GridMapCollisionChecker : public ICollisionChecker {
public:
    /**
     * @brief 构造函数
     * @param origin 地图原点 (x, y)
     * @param resolution 网格分辨率(米/格)
     * @param size_x x方向格子数
     * @param size_y y方向格子数
     */
    GridMapCollisionChecker(
        const Eigen::Vector2d& origin,
        double resolution,
        int size_x,
        int size_y
    );
    
    /**
     * @brief 设置网格值
     * @param i x方向索引
     * @param j y方向索引
     * @param distance SDF距离值
     */
    void setGridValue(int i, int j, double distance);
    
    /**
     * @brief 从占据栅格地图生成SDF
     * @param occupancy_grid 占据栅格(0=free, >0=occupied)
     */
    void computeSDF(const std::vector<std::vector<int>>& occupancy_grid);
    
    // 实现接口
    double getDistanceWithGradient(
        const Eigen::Vector2d& position,
        Eigen::Vector2d& gradient,
        double safe_distance = 0.0
    ) const override;
    
    double getDistance(const Eigen::Vector2d& position) const override;
    
    bool isInBounds(const Eigen::Vector2d& position) const override;

private:
    Eigen::Vector2d origin_;
    double resolution_;
    double inv_resolution_;
    int size_x_;
    int size_y_;
    std::vector<double> sdf_data_;
    
    // 双线性插值
    double bilinearInterpolate(const Eigen::Vector2d& position) const;
    Eigen::Vector2d computeGradient(const Eigen::Vector2d& position) const;
};

} // namespace ddr_optimizer

#endif // DDR_COLLISION_INTERFACE_H

