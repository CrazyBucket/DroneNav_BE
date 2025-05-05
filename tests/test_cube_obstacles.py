# tests/test_cube_obstacles.py
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

def test_cube_obstacle_avoidance():
    """测试立方体障碍物的避障功能"""
    
    # 加载测试场景
    with open("scenarios/presets/test.json", "r") as f:
        scene_config = json.load(f)
    
    # 解析障碍物
    obstacles = parse_obstacles(scene_config)
    
    # 打印解析结果
    print(f"\n测试场景中包含 {len(obstacles)} 个障碍物:")
    for i, (pos, size) in enumerate(obstacles):
        print(f"障碍物 {i+1}: 位置={pos}, 尺寸={size}")
    
    # 定义起点和终点，确保它们之间有障碍物
    # 在X轴上的测试 - 应该避开 x_axis_blocker_1
    start_pos_x = (-2.0, 0.0, 3.0)
    end_pos_x = (15.0, 0.0, 3.0)
    
    drone_size = (0.5, 0.5, 0.2)  # 小型无人机尺寸
    
    print("\n测试X轴避障:")
    print(f"起点: {start_pos_x}")
    print(f"终点: {end_pos_x}")
    
    # 规划路径
    path_x = plan_path(
        current_pos=start_pos_x,
        target_pos=end_pos_x,
        scene_config=scene_config,
        drone_size=drone_size,
        grid_resolution=0.3  # 使用更高精度
    )
    
    # 检查路径是否存在
    assert path_x is not None and len(path_x) > 0, "规划X轴路径失败"
    print(f"X轴路径规划成功，路径点数: {len(path_x)}")
    
    # 检查路径是否安全 - 确保没有点穿过障碍物
    for i, point in enumerate(path_x):
        assert is_position_safe(point, obstacles, drone_size), f"路径点 {i}: {point} 不安全！"
    
    # 验证路径是否真的避开了障碍物
    x_axis_obs = obstacles[0]  # x_axis_blocker_1
    obstacle_x_min = x_axis_obs[0][0]
    obstacle_x_max = x_axis_obs[0][0] + x_axis_obs[1][0]
    obstacle_y_min = x_axis_obs[0][1]
    obstacle_y_max = x_axis_obs[0][1] + x_axis_obs[1][1]
    obstacle_z_min = x_axis_obs[0][2]
    obstacle_z_max = x_axis_obs[0][2] + x_axis_obs[1][2]
    
    # 为了视觉化路径绕开障碍物的方式，打印路径点
    print("\n路径点详情:")
    print(f"障碍物 X范围: [{obstacle_x_min:.2f}, {obstacle_x_max:.2f}]")
    print(f"障碍物 Y范围: [{obstacle_y_min:.2f}, {obstacle_y_max:.2f}]")
    print(f"障碍物 Z范围: [{obstacle_z_min:.2f}, {obstacle_z_max:.2f}]")
    
    # 只打印关键点，节约输出
    key_points = [0, len(path_x)//4, len(path_x)//2, 3*len(path_x)//4, len(path_x)-1]
    for i in key_points:
        point = path_x[i]
        print(f"点 {i}: {point}")
        
        # 检查点是否在障碍物内部
        in_obstacle = (
            obstacle_x_min <= point[0] <= obstacle_x_max and
            obstacle_y_min <= point[1] <= obstacle_y_max and
            obstacle_z_min <= point[2] <= obstacle_z_max
        )
        assert not in_obstacle, f"路径点 {i}: {point} 穿过障碍物！"

    # 检查相邻点的间距，确保路径平滑
    max_distance = 0
    for i in range(1, len(path_x)):
        distance = np.linalg.norm(np.array(path_x[i]) - np.array(path_x[i-1]))
        max_distance = max(max_distance, distance)
    
    print(f"最大相邻点间距: {max_distance:.2f}m")
    assert max_distance < 1.0, f"路径点间距过大: {max_distance}m > 1.0m"
    
    print("测试通过：路径成功避开障碍物！")

if __name__ == "__main__":
    test_cube_obstacle_avoidance() 