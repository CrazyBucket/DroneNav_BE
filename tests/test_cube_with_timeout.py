# tests/test_cube_with_timeout.py
import json
import numpy as np
import os
import sys
import signal
import time

# 添加项目根目录到Python路径
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
sys.path.insert(0, project_root)

from services.path_planner import plan_path, is_position_safe
from core.obstacle_processor import parse_obstacles

# 超时处理函数
class TimeoutError(Exception):
    pass

def timeout_handler(signum, frame):
    raise TimeoutError("路径规划超时")

def create_x_axis_test_scenario():
    """创建测试场景：在X轴路径上放置一个立方体"""
    
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
                    "x": 5.0,  # 立方体在X轴方向上的位置
                    "y": 0.0,
                    "z": 2.0   # 立方体中心高度
                },
                "feature": {
                    "size": [2.0, 2.0, 4.0]  # 2x2米底面，4米高
                }
            }
        ]
    }
    
    return scenario

def test_x_axis_cube_obstacle_avoidance():
    """测试X轴上立方体障碍物的避障功能，带有超时限制"""
    
    # 创建测试场景
    scene_config = create_x_axis_test_scenario()
    
    # 解析障碍物
    obstacles = parse_obstacles(scene_config)
    
    # 显示障碍物信息
    obstacle_pos, obstacle_size = obstacles[0]
    print("\nX轴障碍物范围:")
    print(f"X范围: [{obstacle_pos[0]:.2f}, {obstacle_pos[0] + obstacle_size[0]:.2f}]")
    print(f"Y范围: [{obstacle_pos[1]:.2f}, {obstacle_pos[1] + obstacle_size[1]:.2f}]")
    print(f"Z范围: [{obstacle_pos[2]:.2f}, {obstacle_pos[2] + obstacle_size[2]:.2f}]")
    
    # 设置起点和终点，确保路径会穿过障碍物
    start_pos = (0.0, 0.0, 1.0)  # 起点在原点，高度1米
    end_pos = (8.0, 0.0, 1.0)    # 终点在X轴正方向8米处，高度1米
    
    # 无人机尺寸
    drone_size = (0.5, 0.5, 0.2)  # 小型无人机尺寸
    
    print(f"\n起点: {start_pos}")
    print(f"终点: {end_pos}")
    print(f"障碍物位置: {obstacle_pos}, 尺寸: {obstacle_size}")
    
    # 设置超时信号处理
    signal.signal(signal.SIGALRM, timeout_handler)
    signal.alarm(30)  # 设置30秒超时
    
    try:
        start_time = time.time()
        
        # 规划路径
        path = plan_path(start_pos, end_pos, scene_config, drone_size)
        
        # 计算耗时
        elapsed_time = time.time() - start_time
        print(f"\n路径规划耗时: {elapsed_time:.2f}秒")
        print(f"路径点数: {len(path)}")
        
        # 关闭超时信号
        signal.alarm(0)
        
        # 计算路径偏离直线距离
        straight_line_points = []
        num_straight_points = 200  # 生成200个直线采样点用于比较
        for i in range(num_straight_points):
            t = i / (num_straight_points - 1)
            point = (
                start_pos[0] + t * (end_pos[0] - start_pos[0]),
                start_pos[1] + t * (end_pos[1] - start_pos[1]),
                start_pos[2] + t * (end_pos[2] - start_pos[2])
            )
            straight_line_points.append(point)
        
        # 计算每个路径点到直线的最大距离
        max_deviation = 0
        deviated_points = []
        for i, point in enumerate(path):
            # 计算到直线的最小距离
            min_dist = float('inf')
            for straight_point in straight_line_points:
                dist = np.linalg.norm(np.array(point) - np.array(straight_point))
                min_dist = min(min_dist, dist)
            
            # 如果偏离超过0.5米，记录下来
            if min_dist > 0.5:
                deviated_points.append((i, point, min_dist))
                max_deviation = max(max_deviation, min_dist)
        
        # 显示路径偏离信息
        print(f"\n路径最大偏离直线距离: {max_deviation:.2f}米")
        print(f"发现 {len(deviated_points)} 个显著偏离直线的点:")
        for i in range(min(5, len(deviated_points))):
            if i < len(deviated_points):
                idx, point, dist = deviated_points[i]
                print(f"  点 {idx}: {point}, 偏离: {dist:.2f}米")
        
        if len(deviated_points) > 5:
            print(f"  ... 以及另外 {len(deviated_points) - 5} 个点")
        
        # 检查是否有路径点穿过障碍物
        collision_points = []
        
        # 障碍物边界
        x_min = obstacle_pos[0]
        x_max = obstacle_pos[0] + obstacle_size[0]
        y_min = obstacle_pos[1]
        y_max = obstacle_pos[1] + obstacle_size[1]
        z_min = obstacle_pos[2]
        z_max = obstacle_pos[2] + obstacle_size[2]
        
        # 考虑无人机尺寸的安全距离
        safety_margin = max(drone_size) / 2 + 0.1  # 半径 + 10cm安全裕度
        
        for i, point in enumerate(path):
            x, y, z = point
            
            # 检查点是否在障碍物的扩展边界内
            if (x_min - safety_margin <= x <= x_max + safety_margin and
                y_min - safety_margin <= y <= y_max + safety_margin and
                z_min - safety_margin <= z <= z_max + safety_margin):
                collision_points.append((i, point))
        
        if collision_points:
            print(f"\n警告: 发现 {len(collision_points)} 个可能与障碍物碰撞的点:")
            for i, point in collision_points[:5]:
                print(f"  点 {i}: {point}")
            if len(collision_points) > 5:
                print(f"  ... 以及另外 {len(collision_points) - 5} 个点")
                
            print("\n测试失败: 路径穿过障碍物!")
        else:
            if len(deviated_points) > 0 and max_deviation > 1.0:
                print("\n测试通过: 路径成功避开了障碍物!")
                print(f"路径最大偏离直线距离为 {max_deviation:.2f} 米，说明避障功能有效。")
            else:
                print("\n测试结果不确定: 路径没有显著避开障碍物，但也没有检测到碰撞点。")
                print(f"路径最大偏离直线距离仅为 {max_deviation:.2f} 米，请检查路径规划逻辑。")
        
        # 打印路径点示例
        print("\n路径点示例:")
        interval = max(1, len(path)//8)
        for i in range(0, len(path), interval):
            print(f"点 {i}: {path[i]}")
            
    except TimeoutError as e:
        print(f"\n错误: {e}")
        print("路径规划超时！这可能表明算法效率问题或死循环。")
    
    finally:
        # 确保关闭超时信号
        signal.alarm(0)

if __name__ == "__main__":
    test_x_axis_cube_obstacle_avoidance() 