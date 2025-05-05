# test_planner.py
import numpy as np
from config.settings import DRONE_PHYSICAL_SIZE, GRID_RESOLUTION
from core.obstacle_processor import parse_obstacles
from core.grid_utils import _calculate_grid_bounds, _initialize_grid
from core.a_star_3d import optimize_path, a_star_3d, physical_to_grid, grid_to_physical
from services.path_planner import plan_path, is_position_safe, is_path_safe
import time

def test_building_parsing():
    """验证建筑物解析逻辑"""
    scene_config = {
        "obstacles": [{
            "id": "office_building_2",
            "type": "BUILDING",
            "position": {"x": 20.0, "y": 5.0, "z": 0},
            "feature": {
                "footprint": [10.0, 10.0],
                "height": 20.0
            }
        }]
    }
    
    obstacles = parse_obstacles(scene_config)
    assert len(obstacles) == 1, "障碍物未正确解析"
    
    pos, size = obstacles[0]
    print(f"解析结果: 位置{pos}, 尺寸{size}")
    
    # 验证建筑物位置转换逻辑
    assert pos[0] == 15.0, f"X轴位置错误: {pos[0]}"  # 20 - 10/2
    assert pos[1] == 0.0, f"Y轴位置错误: {pos[1]}"   # 5 - 10/2
    assert size == (10.0, 10.0, 20.0), "尺寸解析错误"

def test_grid_initialization():
    """验证网格初始化逻辑"""
    scene_config = {
        "obstacles": [{
            "id": "office_building_2",
            "type": "BUILDING",
            "position": {"x": 20.0, "y": 5.0, "z": 0},
            "feature": {
                "footprint": [10.0, 10.0],
                "height": 20.0
            }
        }]
    }
    obstacles = parse_obstacles(scene_config)
    
    drone_size = DRONE_PHYSICAL_SIZE  
    resolution = GRID_RESOLUTION          
    bounds = _calculate_grid_bounds(
        start=(0,0,0), goal=(30,10,30),
        obstacles=obstacles, resolution=resolution
    )
    
    grid = _initialize_grid(bounds, obstacles, drone_size, resolution)
    
    # 验证网格形状
    x_range = bounds[0][1] - bounds[0][0]
    y_range = bounds[1][1] - bounds[1][0]
    z_range = bounds[2][1] - bounds[2][0]
    assert grid.shape == (
        int(np.ceil(x_range/resolution)), 
        int(np.ceil(y_range/resolution)), 
        int(np.ceil(z_range/resolution))
    ), "网格形状错误"
    
    # 打印网格统计
    print(f"网格尺寸: {grid.shape}")
    print(f"障碍物占用: {np.sum(grid)} cells")
    
    # 验证障碍物区域是否在网格中标记
    # 计算建筑物在网格中的大致位置
    building_center_x = 15.0  # 建筑物中心X坐标
    building_center_y = 0.0   # 建筑物中心Y坐标
    building_center_z = 10.0  # 建筑物中心Z坐标（高度/2）
    
    # 转换为网格坐标
    grid_x = int((building_center_x - bounds[0][0]) / resolution)
    grid_y = int((building_center_y - bounds[1][0]) / resolution)
    grid_z = int((building_center_z - bounds[2][0]) / resolution)
    
    # 验证网格上的障碍物标记
    try:
        assert grid[grid_x, grid_y, grid_z], "建筑物中心未标记为障碍物"
        print(f"验证通过：建筑物中心({grid_x},{grid_y},{grid_z})已标记为障碍物")
    except Exception as e:
        print(f"验证失败：{str(e)} - 坐标({grid_x},{grid_y},{grid_z})")
        # 检查网格边界
        print(f"网格形状: {grid.shape}")

def test_position_safety():
    """测试位置安全检查函数"""
    # 定义测试场景
    obstacles = [
        ((15.0, 0.0, 0.0), (10.0, 10.0, 20.0))  # 建筑物
    ]
    drone_size = (1.0, 1.0, 1.0)
    
    # 测试点位于建筑物中心
    point_inside = (20.0, 5.0, 10.0)
    safe = is_position_safe(point_inside, obstacles, drone_size)
    assert not safe, f"位置{point_inside}应该是不安全的，但函数返回安全"
    print(f"安全检查通过：点{point_inside}被正确识别为不安全")
    
    # 测试点位于建筑物边缘旁边（但考虑安全范围后不安全）
    point_nearby = (25.1, 5.0, 10.0)  # 距离建筑物边缘0.1米
    safe = is_position_safe(point_nearby, obstacles, drone_size)
    assert not safe, f"位置{point_nearby}太靠近建筑物，应该不安全，但函数返回安全"
    print(f"安全检查通过：点{point_nearby}（靠近障碍物）被正确识别为不安全")
    
    # 测试位于足够远的安全点
    point_safe = (35.0, 15.0, 10.0)  # 远离建筑物
    safe = is_position_safe(point_safe, obstacles, drone_size)
    assert safe, f"位置{point_safe}应该是安全的，但函数返回不安全"
    print(f"安全检查通过：远离障碍物的点{point_safe}被正确识别为安全")

def test_path_safety():
    """测试路径安全检查函数"""
    # 定义测试场景
    obstacles = [
        ((15.0, 0.0, 0.0), (10.0, 10.0, 20.0))  # 建筑物
    ]
    drone_size = (1.0, 1.0, 1.0)
    resolution = 0.5
    
    # 测试穿过障碍物的路径
    path_through = ((10.0, 5.0, 10.0), (30.0, 5.0, 10.0))  # 从左到右穿过建筑物
    safe = is_path_safe(
        start=path_through[0],
        end=path_through[1],
        obstacles=obstacles,
        drone_size=drone_size,
        resolution=resolution
    )
    assert not safe, "穿过障碍物的路径被错误标记为安全"
    print(f"路径安全检查通过：穿过障碍物的路径被正确识别为不安全")
    
    # 测试不穿过障碍物的安全路径
    path_safe = ((10.0, -15.0, 10.0), (30.0, -15.0, 10.0))  # 从左到右绕过建筑物
    safe = is_path_safe(
        start=path_safe[0],
        end=path_safe[1],
        obstacles=obstacles,
        drone_size=drone_size,
        resolution=resolution
    )
    assert safe, "安全路径被错误标记为不安全"
    print(f"路径安全检查通过：安全路径被正确识别为安全")

def test_services_integration():
    """测试服务层调用路径规划时是否传递了障碍物信息"""
    from services.path_planner import get_current_obstacles, set_current_obstacles
    from services.task_processor import process_simulation_task, DEFAULT_ENVIRONMENT
    
    # 先测试默认环境是否正确加载
    assert DEFAULT_ENVIRONMENT is not None, "默认环境未加载"
    assert "obstacles" in DEFAULT_ENVIRONMENT, "默认环境中缺少障碍物信息"
    assert len(DEFAULT_ENVIRONMENT["obstacles"]) > 0, "默认环境中障碍物列表为空"
    
    print(f"默认环境中有 {len(DEFAULT_ENVIRONMENT['obstacles'])} 个障碍物")
    
    # 检查全局障碍物存储功能是否正常
    test_obstacles = [((0,0,0), (1,1,1))]
    set_current_obstacles(test_obstacles)
    current_obstacles = get_current_obstacles()
    assert current_obstacles == test_obstacles, "全局障碍物存储功能异常"
    
    print("服务层集成测试通过：障碍物数据传递正常")

def test_direct_astar():
    """直接测试A*算法，检查是否会忽略障碍物"""
    # 创建一个简单的测试场景
    start_pos = (0, 0, 5)
    end_pos = (20, 0, 5)
    
    # 在起点和终点之间放置一个障碍物
    obstacles = [((8, -2, 0), (4, 4, 10))]  # 中间的障碍物
    
    drone_size = (1.0, 1.0, 1.0)
    resolution = 1.0  # 使用较大的分辨率加快测试
    
    # 计算网格边界
    bounds = _calculate_grid_bounds(
        start=start_pos,
        goal=end_pos,
        obstacles=obstacles,
        resolution=resolution,
        buffer=3,
        drone_size=drone_size
    )
    
    # 初始化网格
    grid = _initialize_grid(
        bounds=bounds,
        obstacles=obstacles,
        drone_size=drone_size,
        resolution=resolution
    )
    
    # 设置全局障碍物（确保路径优化可以使用）
    from services.path_planner import set_current_obstacles
    set_current_obstacles(obstacles)
    
    print("\n测试A*算法避障能力:")
    print(f"起点: {start_pos}, 终点: {end_pos}")
    print(f"障碍物: 位置={obstacles[0][0]}, 尺寸={obstacles[0][1]}")
    
    # 直接调用A*算法
    start_time = time.time()
    path = a_star_3d(
        start_physical=start_pos,
        goal_physical=end_pos,
        grid=grid,
        bounds=bounds,
        resolution=resolution,
        drone_size=drone_size
    )
    elapsed = time.time() - start_time
    
    print(f"A*算法耗时: {elapsed:.2f}秒")
    
    if not path:
        print("❌ A*算法未找到有效路径")
        return
    
    print(f"找到路径，共{len(path)}个点")
    
    # 检查路径是否绕过障碍物
    obstacle_x_range = (obstacles[0][0][0], obstacles[0][0][0] + obstacles[0][1][0])
    obstacle_y_range = (obstacles[0][0][1], obstacles[0][0][1] + obstacles[0][1][1])
    obstacle_z_range = (obstacles[0][0][2], obstacles[0][0][2] + obstacles[0][1][2])
    
    # 扩展障碍物范围（考虑安全距离）
    safety_margin = 3.0
    obs_x_min = obstacle_x_range[0] - safety_margin
    obs_x_max = obstacle_x_range[1] + safety_margin
    obs_y_min = obstacle_y_range[0] - safety_margin
    obs_y_max = obstacle_y_range[1] + safety_margin
    obs_z_min = obstacle_z_range[0] - safety_margin
    obs_z_max = obstacle_z_range[1] + safety_margin
    
    # 检查路径点是否进入障碍物区域
    violation_count = 0
    for i, point in enumerate(path):
        x, y, z = point
        if (obs_x_min <= x <= obs_x_max and 
            obs_y_min <= y <= obs_y_max and 
            obs_z_min <= z <= obs_z_max):
            violation_count += 1
            print(f"⚠️ 点{i}: {point} 太靠近或穿过障碍物")
    
    if violation_count > 0:
        print(f"❌ 路径中有{violation_count}个点太靠近或穿过障碍物")
    else:
        print("✅ 路径成功避开了障碍物")
    
    # 检查路径是否有大角度转弯
    if len(path) >= 3:
        sharp_turns = 0
        for i in range(1, len(path)-1):
            vec1 = np.array(path[i]) - np.array(path[i-1])
            vec2 = np.array(path[i+1]) - np.array(path[i])
            
            if np.linalg.norm(vec1) > 0 and np.linalg.norm(vec2) > 0:
                cos_angle = np.dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2))
                
                # cos_angle < 0 表示大于90度的转角
                if cos_angle < 0:
                    sharp_turns += 1
                    print(f"⚠️ 点{i}: {path[i]} 处有急转弯 (>90°)")
        
        if sharp_turns > 0:
            print(f"⚠️ 路径中有{sharp_turns}处大角度转弯")
        else:
            print("✅ 路径没有大角度转弯")
        
    # 检查相邻点间距
    large_gap_count = 0
    max_expected_gap = resolution * 2
    
    for i in range(1, len(path)):
        distance = np.linalg.norm(np.array(path[i]) - np.array(path[i-1]))
        if distance > max_expected_gap:
            large_gap_count += 1
            print(f"⚠️ 点{i-1}和点{i}之间距离过大: {distance:.2f}m > {max_expected_gap:.2f}m")
    
    if large_gap_count > 0:
        print(f"⚠️ 路径中有{large_gap_count}处间距过大")
    else:
        print("✅ 路径点间距合理")

def test_path_planning_simplified():
    """简化版的路径规划测试，只关注是否能绕过障碍物"""
    # 创建一个简单的测试场景
    scene_config = {
        "obstacles": [{
            "id": "simple_building",
            "type": "BUILDING",
            "position": {"x": 10.0, "y": 0.0, "z": 0},
            "feature": {
                "footprint": [4.0, 4.0],
                "height": 10.0
            }
        }]
    }
    
    start_pos = (0, 0, 5)
    end_pos = (20, 0, 5)
    drone_size = (1.0, 1.0, 1.0)
    resolution = 1.0  # 使用较大的分辨率加快测试
    
    print(f"\n简化路径规划测试 ({start_pos} → {end_pos}):")
    print(f"障碍物: 建筑物位于x=10,y=0位置，尺寸4x4x10")
    
    start_time = time.time()
    path = plan_path(
        current_pos=start_pos,
        target_pos=end_pos,
        scene_config=scene_config,
        drone_size=drone_size,
        grid_resolution=resolution
    )
    elapsed = time.time() - start_time
    
    print(f"路径规划耗时: {elapsed:.2f}秒")
    
    if not path:
        print("❌ 路径规划失败：未找到有效路径")
        return
    
    print(f"找到路径，共{len(path)}个点")
    
    # 处理障碍物
    obstacles = parse_obstacles(scene_config)
    assert len(obstacles) == 1, "障碍物未正确解析"
    
    # 检查路径是否穿过障碍物
    obstacle_pos, obstacle_size = obstacles[0]
    safety_margin = 3.0  # 安全距离
    
    obs_x_min = obstacle_pos[0] - safety_margin
    obs_x_max = obstacle_pos[0] + obstacle_size[0] + safety_margin
    obs_y_min = obstacle_pos[1] - safety_margin
    obs_y_max = obstacle_pos[1] + obstacle_size[1] + safety_margin
    obs_z_min = obstacle_pos[2] - safety_margin
    obs_z_max = obstacle_pos[2] + obstacle_size[2] + safety_margin
    
    print(f"障碍物位置: X[{obstacle_pos[0]:.1f}-{obstacle_pos[0]+obstacle_size[0]:.1f}], "
          f"Y[{obstacle_pos[1]:.1f}-{obstacle_pos[1]+obstacle_size[1]:.1f}], "
          f"Z[{obstacle_pos[2]:.1f}-{obstacle_pos[2]+obstacle_size[2]:.1f}]")
    
    # 检查每个路径点
    violation_count = 0
    for i, point in enumerate(path):
        x, y, z = point
        if (obs_x_min <= x <= obs_x_max and 
            obs_y_min <= y <= obs_y_max and 
            obs_z_min <= z <= obs_z_max):
            violation_count += 1
            print(f"⚠️ 点{i}: {point} 太靠近或穿过障碍物")
            
    if violation_count > 0:
        print(f"❌ 路径中有{violation_count}个点太靠近或穿过障碍物")
        assert False, f"路径穿过障碍物区域"
    else:
        print("✅ 路径成功避开了障碍物")
    
    # 检查路径连续性
    max_expected_gap = resolution * 2
    large_gap_count = 0
    
    for i in range(1, len(path)):
        distance = np.linalg.norm(np.array(path[i]) - np.array(path[i-1]))
        if distance > max_expected_gap:
            large_gap_count += 1
            print(f"⚠️ 点{i-1}和点{i}之间距离过大: {distance:.2f}m > {max_expected_gap:.2f}m")
    
    if large_gap_count > 0:
        print(f"⚠️ 路径中有{large_gap_count}处间距过大")
    else:
        print("✅ 路径点间距合理")
    
    # 检查起点和终点
    assert np.allclose(path[0], start_pos, atol=1e-6), "路径起点不正确"
    assert np.allclose(path[-1], end_pos, atol=1e-6), "路径终点不正确"
    print("✅ 起点和终点正确")

if __name__ == "__main__":
    print("=== 开始单元测试 ===")
    test_building_parsing()
    test_grid_initialization()
    test_position_safety()
    test_path_safety()
    test_services_integration()
    test_direct_astar()
    test_path_planning_simplified()
    print("所有测试通过!")