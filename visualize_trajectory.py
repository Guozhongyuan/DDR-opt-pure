#!/usr/bin/env python3
"""
轨迹可视化脚本
用法: python3 visualize_trajectory.py <json_file> [output_image]
"""

import json
import sys
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

def visualize_trajectory(json_file, output_file=None):
    """从JSON文件读取轨迹数据并可视化"""
    
    # 读取数据
    with open(json_file, 'r') as f:
        data = json.load(f)
    
    trajectory = np.array(data['trajectory'])
    start = np.array(data['start'])
    goal = np.array(data['goal'])
    waypoints = np.array(data['waypoints']) if data['waypoints'] else np.array([])
    obstacles = data['obstacles']
    vehicle_polygons = data.get('vehicle_polygons', [])
    
    # 创建图形
    fig, ax = plt.subplots(1, 1, figsize=(12, 8))
    
    # 绘制障碍物
    for obs in obstacles:
        circle = patches.Circle((obs['x'], obs['y']), obs['r'], 
                               color='red', alpha=0.3, zorder=1)
        ax.add_patch(circle)
        circle_edge = patches.Circle((obs['x'], obs['y']), obs['r'], 
                                    color='red', fill=False, linewidth=2, zorder=1)
        ax.add_patch(circle_edge)
    
    # 绘制车体多边形（沿轨迹）
    if vehicle_polygons:
        for i, polygon in enumerate(vehicle_polygons):
            if len(polygon) >= 3:  # 确保是多边形
                polygon_array = np.array(polygon)
                # 使用半透明绿色绘制车体，每隔几个显示一个避免过于密集
                if i % 4 == 0:  # 每4个多边形显示一个
                    vehicle_patch = patches.Polygon(polygon_array, 
                                                  facecolor='lightgreen', 
                                                  alpha=0.4, 
                                                  linewidth=0.8,
                                                  edgecolor='darkgreen',
                                                  zorder=2)
                    ax.add_patch(vehicle_patch)
    
    # 绘制初始路径（航点连线）
    if len(waypoints) > 0:
        path_points = [start] + waypoints.tolist() + [goal]
        path_points = np.array(path_points)
        ax.plot(path_points[:, 0], path_points[:, 1], 
               'gray', linestyle='--', linewidth=1.5, alpha=0.5, 
               label='Initial Path', zorder=2)
    
    # 绘制优化后的轨迹
    ax.plot(trajectory[:, 0], trajectory[:, 1], 
           'b-', linewidth=2.5, label='Optimized Trajectory', zorder=3)
    
    # 绘制起点
    ax.plot(start[0], start[1], 'go', markersize=12, 
           markeredgecolor='black', markeredgewidth=2, 
           label='Start', zorder=5)
    
    # 绘制终点
    ax.plot(goal[0], goal[1], 'rs', markersize=12, 
           markeredgecolor='black', markeredgewidth=2, 
           label='Goal', zorder=5)
    
    # 绘制航点
    if len(waypoints) > 0:
        ax.scatter(waypoints[:, 0], waypoints[:, 1], 
                  c='orange', marker='^', s=100, 
                  edgecolors='black', linewidths=1.5,
                  label='Waypoints', zorder=4)
    
    # 设置图形属性
    ax.set_xlabel('X [m]', fontsize=12)
    ax.set_ylabel('Y [m]', fontsize=12)
    ax.set_title('DDR Trajectory Optimization', fontsize=14, fontweight='bold')
    ax.legend(loc='best', fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    # 保存或显示
    plt.tight_layout()
    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"图像已保存到: {output_file}")
    else:
        plt.show()
    
    plt.close()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("用法: python3 visualize_trajectory.py <json_file> [output_image]")
        sys.exit(1)
    
    json_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else None
    
    visualize_trajectory(json_file, output_file)

