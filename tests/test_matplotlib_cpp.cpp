#include "matplotlibcpp.h"
#include <vector>
#include <cmath>
#include <iostream>

namespace plt = matplotlibcpp;

int main() {
    std::cout << "测试matplotlib-cpp绘图功能..." << std::endl;
    
    // 准备数据
    std::vector<double> x(100);
    std::vector<double> y(100);
    
    // 生成正弦曲线数据
    for (int i = 0; i < 100; ++i) {
        x[i] = i * 0.1;
        y[i] = std::sin(x[i]);
    }
    
    // 绘制正弦曲线
    plt::plot(x, y, "r-");
    plt::title("正弦曲线测试");
    plt::xlabel("X轴");
    plt::ylabel("Y轴");
    plt::grid(true);
    
    // 准备散点图数据
    std::vector<double> scatter_x = {1.0, 2.0, 3.0, 4.0, 5.0};
    std::vector<double> scatter_y = {1.0, 4.0, 2.0, 5.0, 3.0};
    
    // 绘制散点图
    plt::scatter(scatter_x, scatter_y, 50.0);
    
    // 显示图形
    plt::show();
    
    std::cout << "matplotlib-cpp测试完成！" << std::endl;
    return 0;
}
