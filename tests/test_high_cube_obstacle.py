# tests/test_high_cube_obstacle.py
import json
import numpy as np
import os
import sys

# 添加项目根目录到Python路径
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
sys.path.insert(0, project_root)

from services.path_planner import plan_path, is_position_safe
from core.obstacle_processor import parse_obstacles

def create_test_scenario():
    """创建测试场景：在路径上放置一个高立方体"""
    
    # 基础场景配置
    scenario = {
        "name": "高立方体避障测试",
        "description": "测试无人机是否能正确避开高立方体障碍物",
        "version": "1.0",
        "obstacles": [
            {
                "id": "high_cube",
                "type": "CUBE",
                "position": {
                    "x": 0,
                    "y": 4,  # 立方体在Y轴方向上的位置
                    "z": 5   # 立方体中心高度
                },
                "feature": {
                    "size": [
                        3,   # X轴宽度
                        3,   # Y轴深度
                        10   # Z轴高度 - 足够高的障碍物
                    ]
                }
            }
        ]
    }
    
    return scenario

def test_cube_obstacle_avoidance():
    """测试立方体障碍物的避障功能"""
    
    # 创建测试场景
    scene_config = create_test_scenario()
    
    # 解析障碍物
    obstacles = parse_obstacles(scene_config)
    
    print(f"测试场景中包含 {len(obstacles)} 个障碍物:")
    for i, (pos, size) in enumerate(obstacles):
        print(f"障碍物 {i+1}: 位置={pos}, 尺寸={size}")
    
    # 设置起点和终点
    start_pos = (0, 0, 1)
    end_pos = (0, 10, 1)
    
    drone_size = (0.5, 0.5, 0.2)  # 无人机尺寸
    
    # 检查直线路径安全性
    from services.path_planner import is_path_safe
    direct_path_safe = is_path_safe(start_pos, end_pos, obstacles, drone_size, 0.1)
    print(f"直线路径安全: {direct_path_safe}")
    
    # 执行路径规划
    import time
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
        print("测试失败: 路径穿过了障碍物!")
    else:
        if direct_line:
            print("测试失败: 路径是直线，没有避开障碍物!")
        else:
            print("测试通过: 路径成功避开了障碍物!")
            print(f"路径最大偏离直线距离为 {max_deviation:.2f} 米，说明避障功能有效。")
    
    print("\n路径点示例:")
    for i in range(0, len(path), max(1, len(path) // 10)):
        print(f"点 {i}: {path[i]}")
    
if __name__ == "__main__":
    test_cube_obstacle_avoidance() 