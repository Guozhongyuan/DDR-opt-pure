#!/bin/bash
# 批量可视化所有轨迹数据
# 用法: ./visualize_all.sh

cd build

echo "开始批量生成可视化图像..."
echo "================================"

count=0
for json in *.json; do
    if [ -f "$json" ]; then
        png="${json%.json}.png"
        echo "处理: $json -> $png"
        python3 ../visualize_trajectory.py "$json" "$png"
        if [ $? -eq 0 ]; then
            count=$((count + 1))
        fi
    fi
done

echo "================================"
echo "完成！共生成 $count 个图像"
echo ""
echo "生成的图像:"
ls -lh *.png 2>/dev/null | awk '{print "  " $9 " (" $5 ")"}'

