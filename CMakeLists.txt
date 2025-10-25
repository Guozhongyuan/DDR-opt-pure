cmake_minimum_required(VERSION 3.10)
project(ddr_optimizer_pure)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

# 查找Eigen3
find_package(Eigen3 REQUIRED)

# 查找Python3
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# 通过Python获取numpy包含路径
execute_process(
    COMMAND python3 -c "import numpy; print(numpy.get_include())"
    OUTPUT_VARIABLE NUMPY_INCLUDE_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# 包含目录
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
    ${Python3_INCLUDE_DIRS}  # 使用自动找到的Python包含路径
    ${NUMPY_INCLUDE_DIR}     # 使用自动找到的numpy包含路径
)

# 库源文件
set(LIB_SOURCES
    src/collision_checker.cpp
    src/config.cpp
    src/optimizer_pure.cpp
    src/trajectory_data.cpp
    src/optimizer_core.cpp
    src/penalty_functional.cpp
    src/penalty_functional_path.cpp
    src/utility_functions.cpp
    src/visualizer.cpp
)

# 创建库
add_library(ddr_optimizer STATIC ${LIB_SOURCES})
target_link_libraries(ddr_optimizer ${EIGEN3_LIBRARIES} ${Python3_LIBRARIES} dl m)

# 测试程序
add_executable(test_simple_trajectory tests/test_simple_trajectory.cpp)
target_link_libraries(test_simple_trajectory ddr_optimizer)

add_executable(test_obstacle_avoidance tests/test_obstacle_avoidance.cpp)
target_link_libraries(test_obstacle_avoidance ddr_optimizer)

# matplotlib-cpp测试程序
add_executable(test_matplotlib_cpp tests/test_matplotlib_cpp.cpp)
target_link_libraries(test_matplotlib_cpp ddr_optimizer 
    ${Python3_LIBRARIES}  # 使用自动找到的Python库
    dl m
)


# 示例程序
add_executable(simple_example examples/simple_example.cpp)
target_link_libraries(simple_example ddr_optimizer)

# 安装规则
install(TARGETS ddr_optimizer
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# 可选：添加测试
enable_testing()
add_test(NAME SimpleTrajectoryTest COMMAND test_simple_trajectory)
add_test(NAME ObstacleAvoidanceTest COMMAND test_obstacle_avoidance)

message(STATUS "DDR Optimizer Pure - Configuration Complete")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  Eigen3 include: ${EIGEN3_INCLUDE_DIRS}")

