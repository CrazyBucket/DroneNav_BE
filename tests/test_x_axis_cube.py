# tests/test_x_axis_cube.py
import json
import numpy as np
import os
import sys
import time

# 添加项目根目录到Python路径
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
sys.path.insert(0, project_root)

from services.path_planner import plan_path, is_position_safe, is_path_safe
from core.obstacle_processor import parse_obstacles

def create_test_scenario():
    """创建测试场景：在X轴路径上放置一个立方体障碍物"""
    
    # 基础场景配置
    scenario = {
        "name": "X轴立方体避障测试",
        "description": "测试无人机是否能正确避开X轴上的立方体障碍物",
        "version": "1.0",
        "obstacles": [
            {
                "id": "x_axis_blocker",
                "type": "CUBE",
                "position": {
                    "x": 4.0,  # 立方体在X轴方向上的位置（坐标中心点）
                    "y": 0.0,  # 在Y轴上与起点终点在同一直线
                    "z": 5.0   # 立方体中心高度
                },
                "feature": {
                    "size": [
                        3.0,   # X轴宽度
                        3.0,   # Y轴深度
                        10.0   # Z轴高度 - 足够高的障碍物
                    ]
                }
            }
        ]
    }
    
    return scenario

def test_x_axis_obstacle_avoidance():
    """测试X轴方向上的立方体障碍物避障能力"""
    print("\n====== X轴立方体避障测试 ======")
    
    # 创建测试场景
    scene_config = create_test_scenario()
    
    # 解析障碍物
    obstacles = parse_obstacles(scene_config)
    
    print(f"测试场景中包含 {len(obstacles)} 个障碍物:")
    for i, (pos, size) in enumerate(obstacles):
        print(f"障碍物 {i+1}: 位置={pos}, 尺寸={size}")
        # 计算障碍物的边界
        x_min, y_min, z_min = pos
        x_max = x_min + size[0]
        y_max = y_min + size[1]
        z_max = z_min + size[2]
        print(f"障碍物边界: X={x_min:.1f}~{x_max:.1f}, Y={y_min:.1f}~{y_max:.1f}, Z={z_min:.1f}~{z_max:.1f}")
    
    # 设置起点和终点
    start_pos = (0.0, 0.0, 1.0)  # 起点在原点，高度1米
    end_pos = (8.0, 0.0, 1.0)    # 终点在X轴正方向8米处，高度1米
    
    drone_size = (0.5, 0.5, 0.2)  # 小型无人机尺寸
    
    print(f"\n起点: {start_pos}")
    print(f"终点: {end_pos}")
    
    # 检查直线路径安全性
    resolution = 0.1  # 使用较小的分辨率进行精细碰撞检测
    direct_path_safe = is_path_safe(start_pos, end_pos, obstacles, drone_size, resolution)
    print(f"\n直线路径安全: {'是' if direct_path_safe else '否'}")
    
    if not direct_path_safe:
        print("直线路径穿过障碍物，应执行避障")
    
    # 执行路径规划
    start_time = time.time()
    
    path = plan_path(start_pos, end_pos, scene_config, drone_size)
    
    planning_time = time.time() - start_time
    print(f"\n路径规划耗时: {planning_time:.2f}秒")
    print(f"路径点数: {len(path)}")
    
    # 检查是否为直线路径
    direct_line = True
    max_deviation = 0
    deviated_points = []
    
    # 计算直线方程参数
    start = np.array(start_pos)
    end = np.array(end_pos)
    line_vector = end - start
    line_length = np.linalg.norm(line_vector)
    line_direction = line_vector / line_length
    
    # 检查路径点是否偏离直线
    for i, point in enumerate(path):
        point_vector = np.array(point) - start
        # 计算点在直线上的投影
        projection = np.dot(point_vector, line_direction) * line_direction
        # 计算点到直线的距离
        deviation_vector = point_vector - projection
        deviation = np.linalg.norm(deviation_vector)
        
        if deviation > 0.3:  # 如果偏离超过30cm，则不是直线
            direct_line = False
            deviated_points.append((i, point, deviation))
            if deviation > max_deviation:
                max_deviation = deviation
    
    print(f"\n路径最大偏离直线距离: {max_deviation:.2f}米")
    if not direct_line:
        print(f"发现 {len(deviated_points)} 个显著偏离直线的点:")
        for i, (idx, point, dev) in enumerate(deviated_points[:5]):
            print(f"  点 {idx}: {point}, 偏离: {dev:.2f}米")
        if len(deviated_points) > 5:
            print(f"  ... 以及另外 {len(deviated_points) - 5} 个点")
    
    # 检查路径是否穿过障碍物
    path_through_obstacle = False
    
    for i in range(len(path) - 1):
        for j in range(50):  # 在每两个路径点之间采样50个点
            t = j / 50
            sample_point = (
                path[i][0] + t * (path[i+1][0] - path[i][0]),
                path[i][1] + t * (path[i+1][1] - path[i][1]),
                path[i][2] + t * (path[i+1][2] - path[i][2])
            )
            if not is_position_safe(sample_point, obstacles, drone_size):
                path_through_obstacle = True
                print(f"警告: 路径在点 {i} 到 {i+1} 之间穿过障碍物")
                break
        if path_through_obstacle:
            break
    
    if path_through_obstacle:
        print("测试失败: 路径穿过了障碍物!")
    else:
        if direct_line and not direct_path_safe:
            print("测试失败: 路径是直线，但直线会穿过障碍物!")
        elif direct_line:
            print("测试通过: 路径是直线，可能障碍物不在路径上。")
        else:
            print("测试通过: 路径成功避开了障碍物!")
            print(f"路径最大偏离直线距离为 {max_deviation:.2f} 米，说明避障功能有效。")
    
    print("\n路径点示例:")
    num_samples = min(10, len(path))
    sample_indices = [int(i * len(path) / num_samples) for i in range(num_samples)]
    for i in sample_indices:
        print(f"点 {i}: {path[i]}")

if __name__ == "__main__":
    test_x_axis_obstacle_avoidance() 