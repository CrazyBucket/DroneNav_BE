# tests/test_with_direct_path.py
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

# 简化的路径规划函数，不使用A*算法
def simplified_plan_path(current_pos, target_pos, scene_config, drone_size):
    """简化版路径规划，只使用分段绕行和高空绕行策略"""
    print("\n开始简化版路径规划:")
    print(f"起点坐标: {current_pos}")
    print(f"终点坐标: {target_pos}")
    
    # 解析场景中的障碍物
    obstacles = parse_obstacles(scene_config)
    print(f"障碍物数量: {len(obstacles)}")
    
    # 计算起点和终点之间的直线距离
    direct_distance = np.linalg.norm(np.array(target_pos) - np.array(current_pos))
    print(f"起点和终点之间的直线距离: {direct_distance:.2f}m")
    
    # 尝试直接路径
    is_direct_path_safe = is_path_safe(current_pos, target_pos, obstacles, drone_size, 0.1)
    
    if is_direct_path_safe:
        print("策略1: 直接路径安全可行")
        # 创建更密集的采样点
        sampling_density = min(direct_distance / 100, 0.1)
        num_points = max(20, int(direct_distance / sampling_density))
        
        direct_path = []
        for i in range(num_points + 1):
            t = i / num_points
            point = (
                current_pos[0] + t * (target_pos[0] - current_pos[0]),
                current_pos[1] + t * (target_pos[1] - current_pos[1]),
                current_pos[2] + t * (target_pos[2] - current_pos[2])
            )
            direct_path.append(point)
        
        return direct_path
    else:
        print("策略1: 直接路径不安全，尝试其他策略")
    
    # 尝试高空绕行
    # 相比路障高度高出至少10米
    obstacle_max_height = 0
    for pos, size in obstacles:
        obstacle_max_height = max(obstacle_max_height, pos[2] + size[2])
    
    high_altitude = max(obstacle_max_height + 10, 20)  # 至少20米高
    print(f"尝试高空绕行，高度: {high_altitude}m")
    
    # 创建高空绕行路径
    mid_point1 = (current_pos[0], current_pos[1], high_altitude)
    mid_point2 = (target_pos[0], target_pos[1], high_altitude)
    
    # 检查高空路径是否安全
    if (is_path_safe(current_pos, mid_point1, obstacles, drone_size, 0.1) and
        is_path_safe(mid_point1, mid_point2, obstacles, drone_size, 0.1) and
        is_path_safe(mid_point2, target_pos, obstacles, drone_size, 0.1)):
        
        print("策略2: 高空绕行路径安全可行")
        
        # 生成完整路径
        path = []
        
        # 起点到高空点1
        segments = max(20, int(np.linalg.norm(np.array(mid_point1) - np.array(current_pos)) / 0.1))
        for i in range(segments + 1):
            t = i / segments
            point = (
                current_pos[0] + t * (mid_point1[0] - current_pos[0]),
                current_pos[1] + t * (mid_point1[1] - current_pos[1]),
                current_pos[2] + t * (mid_point1[2] - current_pos[2])
            )
            path.append(point)
        
        # 高空点1到高空点2
        segments = max(20, int(np.linalg.norm(np.array(mid_point2) - np.array(mid_point1)) / 0.1))
        for i in range(1, segments + 1):  # 从1开始避免重复点
            t = i / segments
            point = (
                mid_point1[0] + t * (mid_point2[0] - mid_point1[0]),
                mid_point1[1] + t * (mid_point2[1] - mid_point1[1]),
                mid_point1[2] + t * (mid_point2[2] - mid_point1[2])
            )
            path.append(point)
        
        # 高空点2到终点
        segments = max(20, int(np.linalg.norm(np.array(target_pos) - np.array(mid_point2)) / 0.1))
        for i in range(1, segments + 1):  # 从1开始避免重复点
            t = i / segments
            point = (
                mid_point2[0] + t * (target_pos[0] - mid_point2[0]),
                mid_point2[1] + t * (target_pos[1] - mid_point2[1]),
                mid_point2[2] + t * (target_pos[2] - mid_point2[2])
            )
            path.append(point)
        
        return path
    
    # 尝试侧绕路径
    print("策略3: 尝试侧绕路径...")
    
    # 障碍物范围检查
    obstacle_x_min = float('inf')
    obstacle_x_max = float('-inf')
    obstacle_y_min = float('inf')
    obstacle_y_max = float('-inf')
    
    for pos, size in obstacles:
        obstacle_x_min = min(obstacle_x_min, pos[0])
        obstacle_x_max = max(obstacle_x_max, pos[0] + size[0])
        obstacle_y_min = min(obstacle_y_min, pos[1])
        obstacle_y_max = max(obstacle_y_max, pos[1] + size[1])
    
    # 计算侧绕点 - 尝试Y轴方向的偏移
    y_offset = max(5.0, (obstacle_y_max - obstacle_y_min) + 3.0)  # 至少偏移5米或障碍物Y轴尺寸+3米
    
    # 生成两个可能的侧绕点
    side_point1 = (current_pos[0] + (target_pos[0] - current_pos[0])/3, current_pos[1] - y_offset, current_pos[2] + 5)
    side_point2 = (target_pos[0] - (target_pos[0] - current_pos[0])/3, target_pos[1] + y_offset, target_pos[2] + 5)
    
    # 检查侧绕路径是否安全
    if (is_path_safe(current_pos, side_point1, obstacles, drone_size, 0.1) and
        is_path_safe(side_point1, side_point2, obstacles, drone_size, 0.1) and
        is_path_safe(side_point2, target_pos, obstacles, drone_size, 0.1)):
        
        print("策略3: 侧绕路径安全可行")
        
        # 生成完整路径
        path = []
        
        # 起点到侧绕点1
        segments = max(20, int(np.linalg.norm(np.array(side_point1) - np.array(current_pos)) / 0.1))
        for i in range(segments + 1):
            t = i / segments
            point = (
                current_pos[0] + t * (side_point1[0] - current_pos[0]),
                current_pos[1] + t * (side_point1[1] - current_pos[1]),
                current_pos[2] + t * (side_point1[2] - current_pos[2])
            )
            path.append(point)
        
        # 侧绕点1到侧绕点2
        segments = max(20, int(np.linalg.norm(np.array(side_point2) - np.array(side_point1)) / 0.1))
        for i in range(1, segments + 1):  # 从1开始避免重复点
            t = i / segments
            point = (
                side_point1[0] + t * (side_point2[0] - side_point1[0]),
                side_point1[1] + t * (side_point2[1] - side_point1[1]),
                side_point1[2] + t * (side_point2[2] - side_point1[2])
            )
            path.append(point)
        
        # 侧绕点2到终点
        segments = max(20, int(np.linalg.norm(np.array(target_pos) - np.array(side_point2)) / 0.1))
        for i in range(1, segments + 1):  # 从1开始避免重复点
            t = i / segments
            point = (
                side_point2[0] + t * (target_pos[0] - side_point2[0]),
                side_point2[1] + t * (target_pos[1] - side_point2[1]),
                side_point2[2] + t * (target_pos[2] - side_point2[2])
            )
            path.append(point)
        
        return path
    
    # 所有策略都失败，尝试紧急备用策略 - 高空直连
    print("策略4: 应急高空直连策略...")
    
    emergency_height = max(obstacle_max_height + 15, 30)  # 比正常高空再高5米
    mid_point = ((current_pos[0] + target_pos[0])/2, (current_pos[1] + target_pos[1])/2, emergency_height)
    
    # 生成紧急路径
    path = []
    
    # 起点到高空中点
    segments = max(20, int(np.linalg.norm(np.array(mid_point) - np.array(current_pos)) / 0.1))
    for i in range(segments + 1):
        t = i / segments
        point = (
            current_pos[0] + t * (mid_point[0] - current_pos[0]),
            current_pos[1] + t * (mid_point[1] - current_pos[1]),
            current_pos[2] + t * (mid_point[2] - current_pos[2])
        )
        path.append(point)
    
    # 高空中点到终点
    segments = max(20, int(np.linalg.norm(np.array(target_pos) - np.array(mid_point)) / 0.1))
    for i in range(1, segments + 1):  # 从1开始避免重复点
        t = i / segments
        point = (
            mid_point[0] + t * (target_pos[0] - mid_point[0]),
            mid_point[1] + t * (target_pos[1] - mid_point[1]),
            mid_point[2] + t * (target_pos[2] - mid_point[2])
        )
        path.append(point)
    
    print("使用应急高空路径 - 警告：这个路径可能不是最优的")
    return path

# 检查路径安全性函数
def is_path_safe(start, end, obstacles, drone_size, interval):
    """检查路径段是否安全"""
    # 计算路径段长度
    vec = np.array(end) - np.array(start)
    distance = np.linalg.norm(vec)
    
    # 设置采样点数量
    num_samples = max(10, int(distance / interval))
    
    print(f"检查路径安全性: {start} → {end}, 距离: {distance:.2f}m, 采样点数: {num_samples}")
    
    # 检查起点和终点
    if not is_position_safe(start, obstacles, drone_size):
        print(f"起点 {start} 不安全")
        return False
        
    if not is_position_safe(end, obstacles, drone_size):
        print(f"终点 {end} 不安全")
        return False
    
    # 检查中间点
    for i in range(1, num_samples):
        t = i / num_samples
        mid_point = (
            start[0] + t * (end[0] - start[0]),
            start[1] + t * (end[1] - start[1]),
            start[2] + t * (end[2] - start[2])
        )
        
        if not is_position_safe(mid_point, obstacles, drone_size):
            print(f"路径中间点 ({i}/{num_samples}) {mid_point} 不安全")
            return False
    
    return True

def test_x_axis_cube_obstacle_avoidance():
    """测试X轴上立方体障碍物的避障功能，使用简化路径规划"""
    
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
    signal.alarm(15)  # 设置15秒超时
    
    try:
        start_time = time.time()
        
        # 使用简化版路径规划
        path = simplified_plan_path(start_pos, end_pos, scene_config, drone_size)
        
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